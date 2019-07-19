/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * process.cpp - Process object
 */

#include "process.h"

#include <algorithm>
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <list>
#include <signal.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include <libcamera/event_notifier.h>

#include "log.h"
#include "utils.h"

/**
 * \file process.h
 * \brief Process object
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Process)

/**
 * \class ProcessManager
 * \brief Manager of processes
 *
 * The ProcessManager singleton keeps track of all created Process instances,
 * and manages the signal handling involved in terminating processes.
 */
class ProcessManager
{
public:
	void registerProcess(Process *proc);

	static ProcessManager *instance();

	int writePipe() const;

	const struct sigaction &oldsa() const;

private:
	void sighandler(EventNotifier *notifier);
	ProcessManager();
	~ProcessManager();

	std::list<Process *> processes_;

	struct sigaction oldsa_;
	EventNotifier *sigEvent_;
	int pipe_[2];
};

namespace {

void sigact(int signal, siginfo_t *info, void *ucontext)
{
	char data = 0;
	/* We're in a signal handler so we can't log any message,
	 * and we need to continue anyway. */
	(void)write(ProcessManager::instance()->writePipe(), &data, sizeof(data));

	const struct sigaction &oldsa = ProcessManager::instance()->oldsa();
	if (oldsa.sa_flags & SA_SIGINFO) {
		oldsa.sa_sigaction(signal, info, ucontext);
	} else {
		if (oldsa.sa_handler != SIG_IGN && oldsa.sa_handler != SIG_DFL)
			oldsa.sa_handler(signal);
	}
}

} /* namespace */

void ProcessManager::sighandler(EventNotifier *notifier)
{
	char data;
	ssize_t ret = read(pipe_[0], &data, sizeof(data));
	if (ret < 0) {
		LOG(Process, Error)
			<< "Failed to read byte from signal handler pipe";
		return;
	}

	for (auto it = processes_.begin(); it != processes_.end(); ) {
		Process *process = *it;

		int wstatus;
		pid_t pid = waitpid(process->pid_, &wstatus, WNOHANG);
		if (process->pid_ != pid) {
			++it;
			continue;
		}

		it = processes_.erase(it);
		process->died(wstatus);
	}
}

/**
 * \brief Register process with process manager
 * \param[in] proc Process to register
 *
 * This method registers the \a proc with the process manager. It
 * shall be called by the parent process after successfully forking, in
 * order to let the parent signal process termination.
 */
void ProcessManager::registerProcess(Process *proc)
{
	processes_.push_back(proc);
}

ProcessManager::ProcessManager()
{
	sigaction(SIGCHLD, NULL, &oldsa_);

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_sigaction = &sigact;
	memcpy(&sa.sa_mask, &oldsa_.sa_mask, sizeof(sa.sa_mask));
	sigaddset(&sa.sa_mask, SIGCHLD);
	sa.sa_flags = oldsa_.sa_flags | SA_SIGINFO;

	sigaction(SIGCHLD, &sa, NULL);

	if (pipe2(pipe_, O_CLOEXEC | O_DIRECT | O_NONBLOCK))
		LOG(Process, Fatal)
			<< "Failed to initialize pipe for signal handling";
	sigEvent_ = new EventNotifier(pipe_[0], EventNotifier::Read);
	sigEvent_->activated.connect(this, &ProcessManager::sighandler);
}

ProcessManager::~ProcessManager()
{
	sigaction(SIGCHLD, &oldsa_, NULL);
	delete sigEvent_;
	close(pipe_[0]);
	close(pipe_[1]);
}

/**
 * \brief Retrieve the Process manager instance
 *
 * The ProcessManager is a singleton and can't be constructed manually. This
 * method shall instead be used to retrieve the single global instance of the
 * manager.
 *
 * \return The Process manager instance
 */
ProcessManager *ProcessManager::instance()
{
	static ProcessManager processManager;
	return &processManager;
}

/**
 * \brief Retrieve the Process manager's write pipe
 *
 * This method is meant only to be used by the static signal handler.
 *
 * \return Pipe for writing
 */
int ProcessManager::writePipe() const
{
	return pipe_[1];
}

/**
 * \brief Retrive the old signal action data
 *
 * This method is meant only to be used by the static signal handler.
 *
 * \return The old signal action data
 */
const struct sigaction &ProcessManager::oldsa() const
{
	return oldsa_;
}


/**
 * \class Process
 * \brief Process object
 *
 * The Process class models a process, and simplifies spawning new processes
 * and monitoring the exiting of a process.
 */

/**
 * \enum Process::ExitStatus
 * \brief Exit status of process
 * \var Process::NotExited
 * The process hasn't exited yet
 * \var Process::NormalExit
 * The process exited normally, either via exit() or returning from main
 * \var Process::SignalExit
 * The process was terminated by a signal (this includes crashing)
 */

Process::Process()
	: pid_(-1), running_(false), exitStatus_(NotExited), exitCode_(0)
{
}

Process::~Process()
{
	kill();
	/* \todo wait for child process to exit */
}

/**
 * \brief Fork and exec a process, and close fds
 * \param[in] path Path to executable
 * \param[in] args Arguments to pass to executable (optional)
 * \param[in] fds Vector of file descriptors to keep open (optional)
 *
 * Fork a process, and exec the executable specified by path. Prior to
 * exec'ing, but after forking, all file descriptors except for those
 * specified in fds will be closed.
 *
 * All indexes of args will be incremented by 1 before being fed to exec(),
 * so args[0] should not need to be equal to path.
 *
 * \return Zero on successful fork, exec, and closing the file descriptors,
 * or a negative error code otherwise
 */
int Process::start(const std::string &path,
		   const std::vector<std::string> &args,
		   const std::vector<int> &fds)
{
	int ret;

	if (running_)
		return 0;

	int childPid = fork();
	if (childPid == -1) {
		ret = -errno;
		LOG(Process, Error) << "Failed to fork: " << strerror(-ret);
		return ret;
	} else if (childPid) {
		pid_ = childPid;
		ProcessManager::instance()->registerProcess(this);

		running_ = true;

		return 0;
	} else {
		if (isolate())
			_exit(EXIT_FAILURE);

		closeAllFdsExcept(fds);

		unsetenv("LIBCAMERA_LOG_FILE");

		const char **argv = new const char *[args.size() + 2];
		unsigned int len = args.size();
		argv[0] = path.c_str();
		for (unsigned int i = 0; i < len; i++)
			argv[i+1] = args[i].c_str();
		argv[len+1] = nullptr;

		execv(path.c_str(), (char **)argv);

		exit(EXIT_FAILURE);
	}
}

void Process::closeAllFdsExcept(const std::vector<int> &fds)
{
	std::vector<int> v(fds);
	sort(v.begin(), v.end());

	DIR *dir = opendir("/proc/self/fd");
	if (!dir)
		return;

	int dfd = dirfd(dir);

	struct dirent *ent;
	while ((ent = readdir(dir)) != nullptr) {
		char *endp;
		int fd = strtoul(ent->d_name, &endp, 10);
		if (*endp)
			continue;

		if (fd >= 0 && fd != dfd &&
		    !std::binary_search(v.begin(), v.end(), fd))
			close(fd);
	}

	closedir(dir);
}

int Process::isolate()
{
	int ret = unshare(CLONE_NEWUSER | CLONE_NEWNET);
	if (ret) {
		ret = -errno;
		LOG(Process, Error) << "Failed to unshare execution context: "
				    << strerror(-ret);
		return ret;
	}

	return 0;
}

/**
 * \brief SIGCHLD handler
 * \param[in] wstatus The status as output by waitpid()
 *
 * This method is called when the process associated with Process terminates.
 * It emits the Process::finished signal.
 */
void Process::died(int wstatus)
{
	running_ = false;
	exitStatus_ = WIFEXITED(wstatus) ? NormalExit : SignalExit;
	exitCode_ = exitStatus_ == NormalExit ? WEXITSTATUS(wstatus) : -1;

	finished.emit(this, exitStatus_, exitCode_);
}

/**
 * \fn Process::exitStatus()
 * \brief Retrieve the exit status of the process
 *
 * Return the exit status of the process, that is, whether the process
 * has exited via exit() or returning from main, or if the process was
 * terminated by a signal.
 *
 * \sa ExitStatus
 *
 * \return The process exit status
 */

/**
 * \fn Process::exitCode()
 * \brief Retrieve the exit code of the process
 *
 * This method is only valid if exitStatus() returned NormalExit.
 *
 * \return Exit code
 */

/**
 * \var Process::finished
 *
 * Signal that is emitted when the process is confirmed to have terminated.
 */

/**
 * \brief Kill the process
 *
 * Sends SIGKILL to the process.
 */
void Process::kill()
{
	::kill(pid_, SIGKILL);
}

} /* namespace libcamera */
