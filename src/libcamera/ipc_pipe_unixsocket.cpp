/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm IPC module using unix socket
 */

#include "libcamera/internal/ipc_pipe_unixsocket.h"

#include <unistd.h>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/mutex.h>
#include <libcamera/base/thread.h>

#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_unixsocket.h"
#include "libcamera/internal/process.h"

using namespace std::chrono_literals;

namespace libcamera {

LOG_DECLARE_CATEGORY(IPCPipe)

class IPCUnixSocketWrapper : Thread
{
public:
	IPCUnixSocketWrapper(Signal<const IPCMessage &> *recv)
		: recv_(recv), ready_(false), sendSyncPending_(false),
		  sendSyncCookie_(0)
	{
		start();
	}

	~IPCUnixSocketWrapper()
	{
		exit();
		wait();
	}

	void run() override
	{
		/*
		 * IPC socket construction and connection to its readyRead
		 * signal has to be done from the IPC thread so that the
		 * relevant Object instances (EventNotifier, slot) are bound to
		 * its context.
		 */
		init();
		exec();
		deinit();
	}

	int fd() { return fd_.get(); }
	int sendSync(const IPCMessage &in, IPCMessage *out);
	int sendAsync(const IPCMessage &data);
	bool waitReady();

private:
	void init();
	void deinit();
	void readyRead();

	UniqueFD fd_;
	Signal<const IPCMessage &> *recv_;
	ConditionVariable cv_;
	Mutex mutex_;
	bool ready_;
	bool sendSyncPending_;
	uint32_t sendSyncCookie_;
	IPCUnixSocket::Payload *sendSyncResponse_;

	/* Socket shall be constructed and destructed from IPC thread context */
	std::unique_ptr<IPCUnixSocket> socket_;
};

int IPCUnixSocketWrapper::sendSync(const IPCMessage &in, IPCMessage *out)
{
	int ret;
	IPCUnixSocket::Payload response;

	mutex_.lock();
	ASSERT(!sendSyncPending_);
	sendSyncPending_ = true;
	sendSyncCookie_ = in.header().cookie;
	sendSyncResponse_ = &response;
	mutex_.unlock();

	ret = socket_->send(in.payload());
	if (ret) {
		LOG(IPCPipe, Error) << "Failed to send sync message";
		goto cleanup;
	}

	bool complete;
	{
		MutexLocker locker(mutex_);
		auto syncComplete = ([&]() {
			return sendSyncPending_ == false;
		});
		complete = cv_.wait_for(locker, 1000ms, syncComplete);
	}

	if (!complete) {
		LOG(IPCPipe, Error) << "Timeout sending sync message";
		ret = -ETIMEDOUT;
		goto cleanup;
	}

	if (out)
		*out = IPCMessage(response);

	return 0;

cleanup:
	mutex_.lock();
	sendSyncPending_ = false;
	mutex_.unlock();

	return ret;
}

int IPCUnixSocketWrapper::sendAsync(const IPCMessage &data)
{
	int ret;
	ret = socket_->send(data.payload());
	if (ret)
		LOG(IPCPipe, Error) << "Failed to send sync message";
	return ret;
}

bool IPCUnixSocketWrapper::waitReady()
{
	bool ready;
	{
		MutexLocker locker(mutex_);
		auto isReady = ([&]() {
			return ready_;
		});
		ready = cv_.wait_for(locker, 1000ms, isReady);
	}

	return ready;
}

void IPCUnixSocketWrapper::init()
{
	/* Init is to be done from the IPC thread context */
	ASSERT(Thread::current() == this);

	socket_ = std::make_unique<IPCUnixSocket>();
	fd_ = socket_->create();
	if (!fd_.isValid()) {
		LOG(IPCPipe, Error) << "Failed to create socket";
		return;
	}

	socket_->readyRead.connect(this, &IPCUnixSocketWrapper::readyRead);

	mutex_.lock();
	ready_ = true;
	mutex_.unlock();
	cv_.notify_one();
}

void IPCUnixSocketWrapper::deinit()
{
	/* Deinit is to be done from the IPC thread context */
	ASSERT(Thread::current() == this);

	socket_->readyRead.disconnect(this);
	socket_.reset();

	mutex_.lock();
	ready_ = false;
	mutex_.unlock();
}

void IPCUnixSocketWrapper::readyRead()
{
	IPCUnixSocket::Payload payload;
	int ret = socket_->receive(&payload);
	if (ret) {
		LOG(IPCPipe, Error) << "Receive message failed" << ret;
		return;
	}

	if (payload.data.size() < sizeof(IPCMessage::Header)) {
		LOG(IPCPipe, Error) << "Not enough data received";
		return;
	}

	const IPCMessage::Header *header =
		reinterpret_cast<IPCMessage::Header *>(payload.data.data());
	bool syncComplete = false;
	mutex_.lock();
	if (sendSyncPending_ && sendSyncCookie_ == header->cookie) {
		syncComplete = true;
		sendSyncPending_ = false;
		*sendSyncResponse_ = std::move(payload);
	}
	mutex_.unlock();

	if (syncComplete) {
		cv_.notify_one();
		return;
	}

	/* Received unexpected data, this means it's a call from the IPA. */
	IPCMessage ipcMessage(payload);
	recv_->emit(ipcMessage);
}

IPCPipeUnixSocket::IPCPipeUnixSocket(const char *ipaModulePath,
				     const char *ipaProxyWorkerPath)
	: IPCPipe()
{
	socketWrap_ = std::make_unique<IPCUnixSocketWrapper>(&recv);
	if (!socketWrap_->waitReady()) {
		LOG(IPCPipe, Error) << "Failed to create socket";
		return;
	}
	int fd = socketWrap_->fd();

	std::vector<int> fds;
	std::vector<std::string> args;
	args.push_back(ipaModulePath);
	args.push_back(std::to_string(fd));
	fds.push_back(fd);

	/* Share stdout and stderr with the proxy for logging purpose */
	fds.push_back(STDOUT_FILENO);
	fds.push_back(STDERR_FILENO);

	proc_ = std::make_unique<Process>();
	int ret = proc_->start(ipaProxyWorkerPath, args, fds);
	if (ret) {
		LOG(IPCPipe, Error)
			<< "Failed to start proxy worker process";
		return;
	}

	connected_ = true;
}

IPCPipeUnixSocket::~IPCPipeUnixSocket()
{
}

int IPCPipeUnixSocket::sendSync(const IPCMessage &in, IPCMessage *out)
{
	return socketWrap_->sendSync(in, out);
}

int IPCPipeUnixSocket::sendAsync(const IPCMessage &data)
{
	return socketWrap_->sendAsync(data);
}

} /* namespace libcamera */
