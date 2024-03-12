/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * vimc_ipa_proxy_worker.cpp - Image Processing Algorithm proxy worker for vimc
 *
 * This file is auto-generated. Do not edit.
 */

#include <algorithm>
#include <iostream>
#include <sys/types.h>
#include <tuple>
#include <unistd.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/vimc_ipa_interface.h>
#include <libcamera/ipa/vimc_ipa_serializer.h>
#include <libcamera/logging.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/unique_fd.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_data_serializer.h"
#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_pipe_unixsocket.h"
#include "libcamera/internal/ipc_unixsocket.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(IPAProxyVimcWorker)
using namespace ipa;
using namespace vimc;


class IPAProxyVimcWorker
{
public:
	IPAProxyVimcWorker()
		: ipa_(nullptr),
		  controlSerializer_(ControlSerializer::Role::Worker),
		  exit_(false) {}

	~IPAProxyVimcWorker() {}

	void readyRead()
	{
		IPCUnixSocket::Payload _message;
		int _retRecv = socket_.receive(&_message);
		if (_retRecv) {
			LOG(IPAProxyVimcWorker, Error)
				<< "Receive message failed: " << _retRecv;
			return;
		}

		IPCMessage _ipcMessage(_message);

		_VimcCmd _cmd = static_cast<_VimcCmd>(_ipcMessage.header().cmd);

		switch (_cmd) {
		case _VimcCmd::Exit: {
			exit_ = true;
			break;
		}


		case _VimcCmd::Init: {
		                
                	[[maybe_unused]] const size_t settingsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t codeBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t inFlagsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 8);

                	const size_t settingsStart = 12;
                	const size_t codeStart = settingsStart + settingsBufSize;
                	const size_t inFlagsStart = codeStart + codeBufSize;


                	IPASettings settings =
                        IPADataSerializer<IPASettings>::deserialize(
                        	_ipcMessage.data().cbegin() + settingsStart,
                        	_ipcMessage.data().cbegin() + settingsStart + settingsBufSize);

                	IPAOperationCode code =
                        static_cast<ipa::vimc::IPAOperationCode>(IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + codeStart,
                        	_ipcMessage.data().cbegin() + codeStart + codeBufSize));

                	Flags<ipa::vimc::TestFlag> inFlags =
                        IPADataSerializer<Flags<ipa::vimc::TestFlag>>::deserialize(
                        	_ipcMessage.data().cbegin() + inFlagsStart,
                        	_ipcMessage.data().cend());


			Flags<ipa::vimc::TestFlag> outFlags;

			int32_t _callRet =ipa_->init(settings, code, inFlags, &outFlags);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
                	static_assert(sizeof(Flags<ipa::vimc::TestFlag>) <= 4);
                	std::vector<uint8_t> outFlagsBuf;
                	std::tie(outFlagsBuf, std::ignore) =
                		IPADataSerializer<Flags<ipa::vimc::TestFlag>>::serialize(outFlags);
                	_response.data().insert(_response.data().end(), outFlagsBuf.begin(), outFlagsBuf.end());
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyVimcWorker, Error)
					<< "Reply to init() failed: " << _ret;
			}
			LOG(IPAProxyVimcWorker, Debug) << "Done replying to init()";
			break;
		}

		case _VimcCmd::Configure: {
			controlSerializer_.reset();
		                
                	[[maybe_unused]] const size_t sensorInfoBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t streamConfigBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t entityControlsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 8);

                	const size_t sensorInfoStart = 12;
                	const size_t streamConfigStart = sensorInfoStart + sensorInfoBufSize;
                	const size_t entityControlsStart = streamConfigStart + streamConfigBufSize;


                	IPACameraSensorInfo sensorInfo =
                        IPADataSerializer<IPACameraSensorInfo>::deserialize(
                        	_ipcMessage.data().cbegin() + sensorInfoStart,
                        	_ipcMessage.data().cbegin() + sensorInfoStart + sensorInfoBufSize);

                	std::map<uint32_t, libcamera::IPAStream> streamConfig =
                        IPADataSerializer<std::map<uint32_t, libcamera::IPAStream>>::deserialize(
                        	_ipcMessage.data().cbegin() + streamConfigStart,
                        	_ipcMessage.data().cbegin() + streamConfigStart + streamConfigBufSize);

                	std::map<uint32_t, libcamera::ControlInfoMap> entityControls =
                        IPADataSerializer<std::map<uint32_t, libcamera::ControlInfoMap>>::deserialize(
                        	_ipcMessage.data().cbegin() + entityControlsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);


			int32_t _callRet =ipa_->configure(sensorInfo, streamConfig, entityControls);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyVimcWorker, Error)
					<< "Reply to configure() failed: " << _ret;
			}
			LOG(IPAProxyVimcWorker, Debug) << "Done replying to configure()";
			break;
		}

		case _VimcCmd::Start: {
		                




			int32_t _callRet =ipa_->start();

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyVimcWorker, Error)
					<< "Reply to start() failed: " << _ret;
			}
			LOG(IPAProxyVimcWorker, Debug) << "Done replying to start()";
			break;
		}

		case _VimcCmd::Stop: {
		                



ipa_->stop();

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyVimcWorker, Error)
					<< "Reply to stop() failed: " << _ret;
			}
			LOG(IPAProxyVimcWorker, Debug) << "Done replying to stop()";
			break;
		}

		case _VimcCmd::MapBuffers: {
		                

                	const size_t buffersStart = 0;

                	const size_t buffersFdStart = 0;

                	std::vector<libcamera::IPABuffer> buffers =
                        IPADataSerializer<std::vector<libcamera::IPABuffer>>::deserialize(
                        	_ipcMessage.data().cbegin() + buffersStart,
                        	_ipcMessage.data().cend(),
                        	_ipcMessage.fds().cbegin() + buffersFdStart,
                        	_ipcMessage.fds().cend());

ipa_->mapBuffers(buffers);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyVimcWorker, Error)
					<< "Reply to mapBuffers() failed: " << _ret;
			}
			LOG(IPAProxyVimcWorker, Debug) << "Done replying to mapBuffers()";
			break;
		}

		case _VimcCmd::UnmapBuffers: {
		                

                	const size_t idsStart = 0;


                	std::vector<uint32_t> ids =
                        IPADataSerializer<std::vector<uint32_t>>::deserialize(
                        	_ipcMessage.data().cbegin() + idsStart,
                        	_ipcMessage.data().cend());

ipa_->unmapBuffers(ids);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyVimcWorker, Error)
					<< "Reply to unmapBuffers() failed: " << _ret;
			}
			LOG(IPAProxyVimcWorker, Debug) << "Done replying to unmapBuffers()";
			break;
		}

		case _VimcCmd::QueueRequest: {
		                
                	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t controlsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);

                	const size_t frameStart = 8;
                	const size_t controlsStart = frameStart + frameBufSize;


                	uint32_t frame =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + frameStart,
                        	_ipcMessage.data().cbegin() + frameStart + frameBufSize);

                	ControlList controls =
                        IPADataSerializer<ControlList>::deserialize(
                        	_ipcMessage.data().cbegin() + controlsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);

ipa_->queueRequest(frame, controls);

			break;
		}

		case _VimcCmd::FillParamsBuffer: {
		                
                	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t bufferIdBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);

                	const size_t frameStart = 8;
                	const size_t bufferIdStart = frameStart + frameBufSize;


                	uint32_t frame =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + frameStart,
                        	_ipcMessage.data().cbegin() + frameStart + frameBufSize);

                	uint32_t bufferId =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + bufferIdStart,
                        	_ipcMessage.data().cend());

ipa_->fillParamsBuffer(frame, bufferId);

			break;
		}

		default:
			LOG(IPAProxyVimcWorker, Error) << "Unknown command " << _ipcMessage.header().cmd;
		}
	}

	int init(std::unique_ptr<IPAModule> &ipam, UniqueFD socketfd)
	{
		if (socket_.bind(std::move(socketfd)) < 0) {
			LOG(IPAProxyVimcWorker, Error)
				<< "IPC socket binding failed";
			return EXIT_FAILURE;
		}
		socket_.readyRead.connect(this, &IPAProxyVimcWorker::readyRead);

		ipa_ = dynamic_cast<IPAVimcInterface *>(ipam->createInterface());
		if (!ipa_) {
			LOG(IPAProxyVimcWorker, Error)
				<< "Failed to create IPA interface instance";
			return EXIT_FAILURE;
		}

		ipa_->paramsBufferReady.connect(this, &IPAProxyVimcWorker::paramsBufferReady);
		return 0;
	}

	void run()
	{
		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();
		while (!exit_)
			dispatcher->processEvents();
	}

	void cleanup()
	{
		delete ipa_;
		socket_.close();
	}

private:


        void paramsBufferReady(
        	const uint32_t bufferId,
        	const Flags<ipa::vimc::TestFlag> flags)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_VimcEventCmd::ParamsBufferReady),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> bufferIdBuf;
	std::tie(bufferIdBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(bufferId
);
	static_assert(sizeof(Flags<ipa::vimc::TestFlag>) <= 4);
	std::vector<uint8_t> flagsBuf;
	std::tie(flagsBuf, std::ignore) =
		IPADataSerializer<Flags<ipa::vimc::TestFlag>>::serialize(flags);
	appendPOD<uint32_t>(_message.data(), bufferIdBuf.size());
	appendPOD<uint32_t>(_message.data(), flagsBuf.size());
	_message.data().insert(_message.data().end(), bufferIdBuf.begin(), bufferIdBuf.end());
	_message.data().insert(_message.data().end(), flagsBuf.begin(), flagsBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyVimcWorker, Error)
				<< "Sending event paramsBufferReady() failed: " << _ret;

		LOG(IPAProxyVimcWorker, Debug) << "paramsBufferReady done";
	}


	IPAVimcInterface *ipa_;
	IPCUnixSocket socket_;

	ControlSerializer controlSerializer_;

	bool exit_;
};

int main(int argc, char **argv)
{
	/* Uncomment this for debugging. */
#if 0
	std::string logPath = "/tmp/libcamera.worker." +
			      std::to_string(getpid()) + ".log";
	logSetFile(logPath.c_str());
#endif

	if (argc < 3) {
		LOG(IPAProxyVimcWorker, Error)
			<< "Tried to start worker with no args: "
			<< "expected <path to IPA so> <fd to bind unix socket>";
		return EXIT_FAILURE;
	}

	UniqueFD fd(std::stoi(argv[2]));
	LOG(IPAProxyVimcWorker, Info)
		<< "Starting worker for IPA module " << argv[1]
		<< " with IPC fd = " << fd.get();

	std::unique_ptr<IPAModule> ipam = std::make_unique<IPAModule>(argv[1]);
	if (!ipam->isValid() || !ipam->load()) {
		LOG(IPAProxyVimcWorker, Error)
			<< "IPAModule " << argv[1] << " isn't valid";
		return EXIT_FAILURE;
	}

	/*
	 * Shutdown of proxy worker can be pre-empted by events like
	 * SIGINT/SIGTERM, even before the pipeline handler can request
	 * shutdown. Hence, assign a new gid to prevent signals on the
	 * application being delivered to the proxy.
	 */
	if (setpgid(0, 0) < 0) {
		int err = errno;
		LOG(IPAProxyVimcWorker, Warning)
			<< "Failed to set new gid: " << strerror(err);
	}

	IPAProxyVimcWorker proxyWorker;
	int ret = proxyWorker.init(ipam, std::move(fd));
	if (ret < 0) {
		LOG(IPAProxyVimcWorker, Error)
			<< "Failed to initialize proxy worker";
		return ret;
	}

	LOG(IPAProxyVimcWorker, Debug) << "Proxy worker successfully initialized";

	proxyWorker.run();

	proxyWorker.cleanup();

	return 0;
}