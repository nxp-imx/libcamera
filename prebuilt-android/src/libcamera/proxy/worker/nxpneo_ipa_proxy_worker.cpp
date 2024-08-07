/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy worker for nxpneo
 *
 * This file is auto-generated. Do not edit.
 */

#include <algorithm>
#include <iostream>
#include <sys/types.h>
#include <tuple>
#include <unistd.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/nxpneo_ipa_interface.h>
#include <libcamera/ipa/nxpneo_ipa_serializer.h>
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

LOG_DEFINE_CATEGORY(IPAProxyNxpNeoWorker)
using namespace ipa;
using namespace nxpneo;


class IPAProxyNxpNeoWorker
{
public:
	IPAProxyNxpNeoWorker()
		: ipa_(nullptr),
		  controlSerializer_(ControlSerializer::Role::Worker),
		  exit_(false) {}

	~IPAProxyNxpNeoWorker() {}

	void readyRead()
	{
		IPCUnixSocket::Payload _message;
		int _retRecv = socket_.receive(&_message);
		if (_retRecv) {
			LOG(IPAProxyNxpNeoWorker, Error)
				<< "Receive message failed: " << _retRecv;
			return;
		}

		IPCMessage _ipcMessage(_message);

		_NxpNeoCmd _cmd = static_cast<_NxpNeoCmd>(_ipcMessage.header().cmd);

		switch (_cmd) {
		case _NxpNeoCmd::Exit: {
			exit_ = true;
			break;
		}


		case _NxpNeoCmd::Init: {
		                
                	[[maybe_unused]] const size_t settingsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t hwRevisionBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t sensorInfoBufSize = readPOD<uint32_t>(_ipcMessage.data(), 8);
                	[[maybe_unused]] const size_t sensorControlsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 12);

                	const size_t settingsStart = 16;
                	const size_t hwRevisionStart = settingsStart + settingsBufSize;
                	const size_t sensorInfoStart = hwRevisionStart + hwRevisionBufSize;
                	const size_t sensorControlsStart = sensorInfoStart + sensorInfoBufSize;


                	IPASettings settings =
                        IPADataSerializer<IPASettings>::deserialize(
                        	_ipcMessage.data().cbegin() + settingsStart,
                        	_ipcMessage.data().cbegin() + settingsStart + settingsBufSize);

                	uint32_t hwRevision =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + hwRevisionStart,
                        	_ipcMessage.data().cbegin() + hwRevisionStart + hwRevisionBufSize);

                	IPACameraSensorInfo sensorInfo =
                        IPADataSerializer<IPACameraSensorInfo>::deserialize(
                        	_ipcMessage.data().cbegin() + sensorInfoStart,
                        	_ipcMessage.data().cbegin() + sensorInfoStart + sensorInfoBufSize);

                	ControlInfoMap sensorControls =
                        IPADataSerializer<ControlInfoMap>::deserialize(
                        	_ipcMessage.data().cbegin() + sensorControlsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);


			ControlInfoMap ipaControls;

			SensorConfig sensorConfig;

			int32_t _callRet =ipa_->init(settings, hwRevision, sensorInfo, sensorControls, &ipaControls, &sensorConfig);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
                	std::vector<uint8_t> ipaControlsBuf;
                	std::tie(ipaControlsBuf, std::ignore) =
                		IPADataSerializer<ControlInfoMap>::serialize(ipaControls
                , &controlSerializer_);
                	std::vector<uint8_t> sensorConfigBuf;
                	std::tie(sensorConfigBuf, std::ignore) =
                		IPADataSerializer<SensorConfig>::serialize(sensorConfig
                );
                	appendPOD<uint32_t>(_response.data(), ipaControlsBuf.size());
                	appendPOD<uint32_t>(_response.data(), sensorConfigBuf.size());
                	_response.data().insert(_response.data().end(), ipaControlsBuf.begin(), ipaControlsBuf.end());
                	_response.data().insert(_response.data().end(), sensorConfigBuf.begin(), sensorConfigBuf.end());
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyNxpNeoWorker, Error)
					<< "Reply to init() failed: " << _ret;
			}
			LOG(IPAProxyNxpNeoWorker, Debug) << "Done replying to init()";
			break;
		}

		case _NxpNeoCmd::Start: {
		                




			int32_t _callRet =ipa_->start();

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyNxpNeoWorker, Error)
					<< "Reply to start() failed: " << _ret;
			}
			LOG(IPAProxyNxpNeoWorker, Debug) << "Done replying to start()";
			break;
		}

		case _NxpNeoCmd::Stop: {
		                



ipa_->stop();

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyNxpNeoWorker, Error)
					<< "Reply to stop() failed: " << _ret;
			}
			LOG(IPAProxyNxpNeoWorker, Debug) << "Done replying to stop()";
			break;
		}

		case _NxpNeoCmd::Configure: {
			controlSerializer_.reset();
		                
                	[[maybe_unused]] const size_t configInfoBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t streamConfigBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);

                	const size_t configInfoStart = 8;
                	const size_t streamConfigStart = configInfoStart + configInfoBufSize;


                	IPAConfigInfo configInfo =
                        IPADataSerializer<IPAConfigInfo>::deserialize(
                        	_ipcMessage.data().cbegin() + configInfoStart,
                        	_ipcMessage.data().cbegin() + configInfoStart + configInfoBufSize,
                        	&controlSerializer_);

                	std::map<uint32_t, libcamera::IPAStream> streamConfig =
                        IPADataSerializer<std::map<uint32_t, libcamera::IPAStream>>::deserialize(
                        	_ipcMessage.data().cbegin() + streamConfigStart,
                        	_ipcMessage.data().cend());


			ControlInfoMap ipaControls;

			int32_t _callRet =ipa_->configure(configInfo, streamConfig, &ipaControls);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
                	std::vector<uint8_t> ipaControlsBuf;
                	std::tie(ipaControlsBuf, std::ignore) =
                		IPADataSerializer<ControlInfoMap>::serialize(ipaControls
                , &controlSerializer_);
                	_response.data().insert(_response.data().end(), ipaControlsBuf.begin(), ipaControlsBuf.end());
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyNxpNeoWorker, Error)
					<< "Reply to configure() failed: " << _ret;
			}
			LOG(IPAProxyNxpNeoWorker, Debug) << "Done replying to configure()";
			break;
		}

		case _NxpNeoCmd::MapBuffers: {
		                

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
				LOG(IPAProxyNxpNeoWorker, Error)
					<< "Reply to mapBuffers() failed: " << _ret;
			}
			LOG(IPAProxyNxpNeoWorker, Debug) << "Done replying to mapBuffers()";
			break;
		}

		case _NxpNeoCmd::UnmapBuffers: {
		                

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
				LOG(IPAProxyNxpNeoWorker, Error)
					<< "Reply to unmapBuffers() failed: " << _ret;
			}
			LOG(IPAProxyNxpNeoWorker, Debug) << "Done replying to unmapBuffers()";
			break;
		}

		case _NxpNeoCmd::QueueRequest: {
		                
                	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t reqControlsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);

                	const size_t frameStart = 8;
                	const size_t reqControlsStart = frameStart + frameBufSize;


                	uint32_t frame =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + frameStart,
                        	_ipcMessage.data().cbegin() + frameStart + frameBufSize);

                	ControlList reqControls =
                        IPADataSerializer<ControlList>::deserialize(
                        	_ipcMessage.data().cbegin() + reqControlsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);

ipa_->queueRequest(frame, reqControls);

			break;
		}

		case _NxpNeoCmd::FillParamsBuffer: {
		                
                	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t paramsbufferIdBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t rawBufferIdBufSize = readPOD<uint32_t>(_ipcMessage.data(), 8);

                	const size_t frameStart = 12;
                	const size_t paramsbufferIdStart = frameStart + frameBufSize;
                	const size_t rawBufferIdStart = paramsbufferIdStart + paramsbufferIdBufSize;


                	uint32_t frame =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + frameStart,
                        	_ipcMessage.data().cbegin() + frameStart + frameBufSize);

                	uint32_t paramsbufferId =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + paramsbufferIdStart,
                        	_ipcMessage.data().cbegin() + paramsbufferIdStart + paramsbufferIdBufSize);

                	uint32_t rawBufferId =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + rawBufferIdStart,
                        	_ipcMessage.data().cend());

ipa_->fillParamsBuffer(frame, paramsbufferId, rawBufferId);

			break;
		}

		case _NxpNeoCmd::ProcessStatsBuffer: {
		                
                	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t bufferIdBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t sensorControlsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 8);

                	const size_t frameStart = 12;
                	const size_t bufferIdStart = frameStart + frameBufSize;
                	const size_t sensorControlsStart = bufferIdStart + bufferIdBufSize;


                	uint32_t frame =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + frameStart,
                        	_ipcMessage.data().cbegin() + frameStart + frameBufSize);

                	uint32_t bufferId =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + bufferIdStart,
                        	_ipcMessage.data().cbegin() + bufferIdStart + bufferIdBufSize);

                	ControlList sensorControls =
                        IPADataSerializer<ControlList>::deserialize(
                        	_ipcMessage.data().cbegin() + sensorControlsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);

ipa_->processStatsBuffer(frame, bufferId, sensorControls);

			break;
		}

		default:
			LOG(IPAProxyNxpNeoWorker, Error) << "Unknown command " << _ipcMessage.header().cmd;
		}
	}

	int init(std::unique_ptr<IPAModule> &ipam, UniqueFD socketfd)
	{
		if (socket_.bind(std::move(socketfd)) < 0) {
			LOG(IPAProxyNxpNeoWorker, Error)
				<< "IPC socket binding failed";
			return EXIT_FAILURE;
		}
		socket_.readyRead.connect(this, &IPAProxyNxpNeoWorker::readyRead);

		ipa_ = dynamic_cast<IPANxpNeoInterface *>(ipam->createInterface());
		if (!ipa_) {
			LOG(IPAProxyNxpNeoWorker, Error)
				<< "Failed to create IPA interface instance";
			return EXIT_FAILURE;
		}

		ipa_->paramsBufferReady.connect(this, &IPAProxyNxpNeoWorker::paramsBufferReady);
		ipa_->setSensorControls.connect(this, &IPAProxyNxpNeoWorker::setSensorControls);
		ipa_->metadataReady.connect(this, &IPAProxyNxpNeoWorker::metadataReady);
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
        	const uint32_t frame)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_NxpNeoEventCmd::ParamsBufferReady),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	_message.data().insert(_message.data().end(), frameBuf.begin(), frameBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyNxpNeoWorker, Error)
				<< "Sending event paramsBufferReady() failed: " << _ret;

		LOG(IPAProxyNxpNeoWorker, Debug) << "paramsBufferReady done";
	}

        void setSensorControls(
        	const uint32_t frame,
        	const ControlList &sensorControls)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_NxpNeoEventCmd::SetSensorControls),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(sensorControls
, &controlSerializer_);
	appendPOD<uint32_t>(_message.data(), frameBuf.size());
	appendPOD<uint32_t>(_message.data(), sensorControlsBuf.size());
	_message.data().insert(_message.data().end(), frameBuf.begin(), frameBuf.end());
	_message.data().insert(_message.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyNxpNeoWorker, Error)
				<< "Sending event setSensorControls() failed: " << _ret;

		LOG(IPAProxyNxpNeoWorker, Debug) << "setSensorControls done";
	}

        void metadataReady(
        	const uint32_t frame,
        	const ControlList &metadata)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_NxpNeoEventCmd::MetadataReady),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> metadataBuf;
	std::tie(metadataBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(metadata
, &controlSerializer_);
	appendPOD<uint32_t>(_message.data(), frameBuf.size());
	appendPOD<uint32_t>(_message.data(), metadataBuf.size());
	_message.data().insert(_message.data().end(), frameBuf.begin(), frameBuf.end());
	_message.data().insert(_message.data().end(), metadataBuf.begin(), metadataBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyNxpNeoWorker, Error)
				<< "Sending event metadataReady() failed: " << _ret;

		LOG(IPAProxyNxpNeoWorker, Debug) << "metadataReady done";
	}


	IPANxpNeoInterface *ipa_;
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
		LOG(IPAProxyNxpNeoWorker, Error)
			<< "Tried to start worker with no args: "
			<< "expected <path to IPA so> <fd to bind unix socket>";
		return EXIT_FAILURE;
	}

	UniqueFD fd(std::stoi(argv[2]));
	LOG(IPAProxyNxpNeoWorker, Info)
		<< "Starting worker for IPA module " << argv[1]
		<< " with IPC fd = " << fd.get();

	std::unique_ptr<IPAModule> ipam = std::make_unique<IPAModule>(argv[1]);
	if (!ipam->isValid() || !ipam->load()) {
		LOG(IPAProxyNxpNeoWorker, Error)
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
		LOG(IPAProxyNxpNeoWorker, Warning)
			<< "Failed to set new gid: " << strerror(err);
	}

	IPAProxyNxpNeoWorker proxyWorker;
	int ret = proxyWorker.init(ipam, std::move(fd));
	if (ret < 0) {
		LOG(IPAProxyNxpNeoWorker, Error)
			<< "Failed to initialize proxy worker";
		return ret;
	}

	LOG(IPAProxyNxpNeoWorker, Debug) << "Proxy worker successfully initialized";

	proxyWorker.run();

	proxyWorker.cleanup();

	return 0;
}