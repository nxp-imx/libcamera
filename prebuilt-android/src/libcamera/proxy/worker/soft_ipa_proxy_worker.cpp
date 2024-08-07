/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy worker for soft
 *
 * This file is auto-generated. Do not edit.
 */

#include <algorithm>
#include <iostream>
#include <sys/types.h>
#include <tuple>
#include <unistd.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/soft_ipa_interface.h>
#include <libcamera/ipa/soft_ipa_serializer.h>
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

LOG_DEFINE_CATEGORY(IPAProxySoftWorker)
using namespace ipa;
using namespace soft;


class IPAProxySoftWorker
{
public:
	IPAProxySoftWorker()
		: ipa_(nullptr),
		  controlSerializer_(ControlSerializer::Role::Worker),
		  exit_(false) {}

	~IPAProxySoftWorker() {}

	void readyRead()
	{
		IPCUnixSocket::Payload _message;
		int _retRecv = socket_.receive(&_message);
		if (_retRecv) {
			LOG(IPAProxySoftWorker, Error)
				<< "Receive message failed: " << _retRecv;
			return;
		}

		IPCMessage _ipcMessage(_message);

		_SoftCmd _cmd = static_cast<_SoftCmd>(_ipcMessage.header().cmd);

		switch (_cmd) {
		case _SoftCmd::Exit: {
			exit_ = true;
			break;
		}


		case _SoftCmd::Init: {
		                
                	[[maybe_unused]] const size_t settingsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t fdStatsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t fdStatsFdsSize = readPOD<uint32_t>(_ipcMessage.data(), 8);
                	[[maybe_unused]] const size_t fdParamsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 12);
                	[[maybe_unused]] const size_t fdParamsFdsSize = readPOD<uint32_t>(_ipcMessage.data(), 16);
                	[[maybe_unused]] const size_t sensorInfoBufSize = readPOD<uint32_t>(_ipcMessage.data(), 20);
                	[[maybe_unused]] const size_t sensorControlsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 24);

                	const size_t settingsStart = 28;
                	const size_t fdStatsStart = settingsStart + settingsBufSize;
                	const size_t fdParamsStart = fdStatsStart + fdStatsBufSize;
                	const size_t sensorInfoStart = fdParamsStart + fdParamsBufSize;
                	const size_t sensorControlsStart = sensorInfoStart + sensorInfoBufSize;

                	const size_t fdStatsFdStart = 0;
                	const size_t fdParamsFdStart = fdStatsFdStart + fdStatsFdsSize;

                	IPASettings settings =
                        IPADataSerializer<IPASettings>::deserialize(
                        	_ipcMessage.data().cbegin() + settingsStart,
                        	_ipcMessage.data().cbegin() + settingsStart + settingsBufSize);

                	SharedFD fdStats =
                        IPADataSerializer<SharedFD>::deserialize(
                        	_ipcMessage.data().cbegin() + fdStatsStart,
                        	_ipcMessage.data().cbegin() + fdStatsStart + fdStatsBufSize,
                        	_ipcMessage.fds().cbegin() + fdStatsFdStart,
                        	_ipcMessage.fds().cbegin() + fdStatsFdStart + fdStatsFdsSize);

                	SharedFD fdParams =
                        IPADataSerializer<SharedFD>::deserialize(
                        	_ipcMessage.data().cbegin() + fdParamsStart,
                        	_ipcMessage.data().cbegin() + fdParamsStart + fdParamsBufSize,
                        	_ipcMessage.fds().cbegin() + fdParamsFdStart,
                        	_ipcMessage.fds().cbegin() + fdParamsFdStart + fdParamsFdsSize);

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

			bool ccmEnabled;

			int32_t _callRet =ipa_->init(settings, fdStats, fdParams, sensorInfo, sensorControls, &ipaControls, &ccmEnabled);

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
                	std::vector<uint8_t> ccmEnabledBuf;
                	std::tie(ccmEnabledBuf, std::ignore) =
                		IPADataSerializer<bool>::serialize(ccmEnabled
                );
                	appendPOD<uint32_t>(_response.data(), ipaControlsBuf.size());
                	appendPOD<uint32_t>(_response.data(), ccmEnabledBuf.size());
                	_response.data().insert(_response.data().end(), ipaControlsBuf.begin(), ipaControlsBuf.end());
                	_response.data().insert(_response.data().end(), ccmEnabledBuf.begin(), ccmEnabledBuf.end());
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxySoftWorker, Error)
					<< "Reply to init() failed: " << _ret;
			}
			LOG(IPAProxySoftWorker, Debug) << "Done replying to init()";
			break;
		}

		case _SoftCmd::Start: {
		                




			int32_t _callRet =ipa_->start();

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxySoftWorker, Error)
					<< "Reply to start() failed: " << _ret;
			}
			LOG(IPAProxySoftWorker, Debug) << "Done replying to start()";
			break;
		}

		case _SoftCmd::Stop: {
		                



ipa_->stop();

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxySoftWorker, Error)
					<< "Reply to stop() failed: " << _ret;
			}
			LOG(IPAProxySoftWorker, Debug) << "Done replying to stop()";
			break;
		}

		case _SoftCmd::Configure: {
			controlSerializer_.reset();
		                

                	const size_t configInfoStart = 0;


                	IPAConfigInfo configInfo =
                        IPADataSerializer<IPAConfigInfo>::deserialize(
                        	_ipcMessage.data().cbegin() + configInfoStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);


			int32_t _callRet =ipa_->configure(configInfo);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxySoftWorker, Error)
					<< "Reply to configure() failed: " << _ret;
			}
			LOG(IPAProxySoftWorker, Debug) << "Done replying to configure()";
			break;
		}

		case _SoftCmd::QueueRequest: {
		                
                	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t sensorControlsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);

                	const size_t frameStart = 8;
                	const size_t sensorControlsStart = frameStart + frameBufSize;


                	uint32_t frame =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + frameStart,
                        	_ipcMessage.data().cbegin() + frameStart + frameBufSize);

                	ControlList sensorControls =
                        IPADataSerializer<ControlList>::deserialize(
                        	_ipcMessage.data().cbegin() + sensorControlsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);

ipa_->queueRequest(frame, sensorControls);

			break;
		}

		case _SoftCmd::ComputeParams: {
		                

                	const size_t frameStart = 0;


                	uint32_t frame =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + frameStart,
                        	_ipcMessage.data().cend());

ipa_->computeParams(frame);

			break;
		}

		case _SoftCmd::ProcessStats: {
		                
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

ipa_->processStats(frame, bufferId, sensorControls);

			break;
		}

		default:
			LOG(IPAProxySoftWorker, Error) << "Unknown command " << _ipcMessage.header().cmd;
		}
	}

	int init(std::unique_ptr<IPAModule> &ipam, UniqueFD socketfd)
	{
		if (socket_.bind(std::move(socketfd)) < 0) {
			LOG(IPAProxySoftWorker, Error)
				<< "IPC socket binding failed";
			return EXIT_FAILURE;
		}
		socket_.readyRead.connect(this, &IPAProxySoftWorker::readyRead);

		ipa_ = dynamic_cast<IPASoftInterface *>(ipam->createInterface());
		if (!ipa_) {
			LOG(IPAProxySoftWorker, Error)
				<< "Failed to create IPA interface instance";
			return EXIT_FAILURE;
		}

		ipa_->setSensorControls.connect(this, &IPAProxySoftWorker::setSensorControls);
		ipa_->setIspParams.connect(this, &IPAProxySoftWorker::setIspParams);
		ipa_->metadataReady.connect(this, &IPAProxySoftWorker::metadataReady);
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


        void setSensorControls(
        	const ControlList &sensorControls)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_SoftEventCmd::SetSensorControls),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(sensorControls
, &controlSerializer_);
	_message.data().insert(_message.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxySoftWorker, Error)
				<< "Sending event setSensorControls() failed: " << _ret;

		LOG(IPAProxySoftWorker, Debug) << "setSensorControls done";
	}

        void setIspParams()
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_SoftEventCmd::SetIspParams),
			0
		};
		IPCMessage _message(header);

		

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxySoftWorker, Error)
				<< "Sending event setIspParams() failed: " << _ret;

		LOG(IPAProxySoftWorker, Debug) << "setIspParams done";
	}

        void metadataReady(
        	const uint32_t frame,
        	const ControlList &metadata)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_SoftEventCmd::MetadataReady),
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
			LOG(IPAProxySoftWorker, Error)
				<< "Sending event metadataReady() failed: " << _ret;

		LOG(IPAProxySoftWorker, Debug) << "metadataReady done";
	}


	IPASoftInterface *ipa_;
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
		LOG(IPAProxySoftWorker, Error)
			<< "Tried to start worker with no args: "
			<< "expected <path to IPA so> <fd to bind unix socket>";
		return EXIT_FAILURE;
	}

	UniqueFD fd(std::stoi(argv[2]));
	LOG(IPAProxySoftWorker, Info)
		<< "Starting worker for IPA module " << argv[1]
		<< " with IPC fd = " << fd.get();

	std::unique_ptr<IPAModule> ipam = std::make_unique<IPAModule>(argv[1]);
	if (!ipam->isValid() || !ipam->load()) {
		LOG(IPAProxySoftWorker, Error)
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
		LOG(IPAProxySoftWorker, Warning)
			<< "Failed to set new gid: " << strerror(err);
	}

	IPAProxySoftWorker proxyWorker;
	int ret = proxyWorker.init(ipam, std::move(fd));
	if (ret < 0) {
		LOG(IPAProxySoftWorker, Error)
			<< "Failed to initialize proxy worker";
		return ret;
	}

	LOG(IPAProxySoftWorker, Debug) << "Proxy worker successfully initialized";

	proxyWorker.run();

	proxyWorker.cleanup();

	return 0;
}