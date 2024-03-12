/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * vimc_ipa_proxy.cpp - Image Processing Algorithm proxy for vimc
 *
 * This file is auto-generated. Do not edit.
 */

#include <libcamera/ipa/vimc_ipa_proxy.h>

#include <memory>
#include <string>
#include <vector>

#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/vimc_ipa_interface.h>
#include <libcamera/ipa/vimc_ipa_serializer.h>

#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>

#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_data_serializer.h"
#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_pipe_unixsocket.h"
#include "libcamera/internal/ipc_unixsocket.h"
#include "libcamera/internal/process.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPAProxy)

namespace ipa {

namespace vimc {


IPAProxyVimc::IPAProxyVimc(IPAModule *ipam, bool isolate)
	: IPAProxy(ipam), isolate_(isolate),
	  controlSerializer_(ControlSerializer::Role::Proxy), seq_(0)
{
	LOG(IPAProxy, Debug)
		<< "initializing vimc proxy: loading IPA from "
		<< ipam->path();

	if (isolate_) {
		const std::string proxyWorkerPath = resolvePath("vimc_ipa_proxy");
		if (proxyWorkerPath.empty()) {
			LOG(IPAProxy, Error)
				<< "Failed to get proxy worker path";
			return;
		}

		ipc_ = std::make_unique<IPCPipeUnixSocket>(ipam->path().c_str(),
							   proxyWorkerPath.c_str());
		if (!ipc_->isConnected()) {
			LOG(IPAProxy, Error) << "Failed to create IPCPipe";
			return;
		}

		ipc_->recv.connect(this, &IPAProxyVimc::recvMessage);

		valid_ = true;
		return;
	}

	if (!ipam->load())
		return;

	IPAInterface *ipai = ipam->createInterface();
	if (!ipai) {
		LOG(IPAProxy, Error)
			<< "Failed to create IPA context for " << ipam->path();
		return;
	}

	ipa_ = std::unique_ptr<IPAVimcInterface>(static_cast<IPAVimcInterface *>(ipai));
	proxy_.setIPA(ipa_.get());


	ipa_->paramsBufferReady.connect(this, &IPAProxyVimc::paramsBufferReadyThread);

	valid_ = true;
}

IPAProxyVimc::~IPAProxyVimc()
{
	if (isolate_) {
		IPCMessage::Header header =
			{ static_cast<uint32_t>(_VimcCmd::Exit), seq_++ };
		IPCMessage msg(header);
		ipc_->sendAsync(msg);
	}
}


void IPAProxyVimc::recvMessage(const IPCMessage &data)
{
	size_t dataSize = data.data().size();
	_VimcEventCmd _cmd = static_cast<_VimcEventCmd>(data.header().cmd);

	switch (_cmd) {
	case _VimcEventCmd::ParamsBufferReady: {
		paramsBufferReadyIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	default:
		LOG(IPAProxy, Error) << "Unknown command " << static_cast<uint32_t>(_cmd);
	}
}


int32_t IPAProxyVimc::init(
	const IPASettings &settings,
	const IPAOperationCode code,
	const Flags<ipa::vimc::TestFlag> inFlags,
	Flags<ipa::vimc::TestFlag> *outFlags)
{
	if (isolate_)
		return initIPC(settings, code, inFlags, outFlags);
	else
		return initThread(settings, code, inFlags, outFlags);
}

int32_t IPAProxyVimc::initThread(
	const IPASettings &settings,
	const IPAOperationCode code,
	const Flags<ipa::vimc::TestFlag> inFlags,
	Flags<ipa::vimc::TestFlag> *outFlags)
{
	int32_t _ret = ipa_->init(settings, code, inFlags, outFlags);

	proxy_.moveToThread(&thread_);

	return _ret;
}

int32_t IPAProxyVimc::initIPC(
	const IPASettings &settings,
	const IPAOperationCode code,
	const Flags<ipa::vimc::TestFlag> inFlags,
	Flags<ipa::vimc::TestFlag> *outFlags)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_VimcCmd::Init), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> settingsBuf;
	std::tie(settingsBuf, std::ignore) =
		IPADataSerializer<IPASettings>::serialize(settings
);
	static_assert(sizeof(ipa::vimc::IPAOperationCode) <= 4);
	std::vector<uint8_t> codeBuf;
	std::tie(codeBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(static_cast<uint32_t>(code));
	static_assert(sizeof(Flags<ipa::vimc::TestFlag>) <= 4);
	std::vector<uint8_t> inFlagsBuf;
	std::tie(inFlagsBuf, std::ignore) =
		IPADataSerializer<Flags<ipa::vimc::TestFlag>>::serialize(inFlags);
	appendPOD<uint32_t>(_ipcInputBuf.data(), settingsBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), codeBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), inFlagsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), settingsBuf.begin(), settingsBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), codeBuf.begin(), codeBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), inFlagsBuf.begin(), inFlagsBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call init";
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);



	const size_t outFlagsStart = 4;


	if (outFlags) {
                *outFlags =
                IPADataSerializer<Flags<ipa::vimc::TestFlag>>::deserialize(
                	_ipcOutputBuf.data().cbegin() + outFlagsStart,
                	_ipcOutputBuf.data().cend());
	}


	return _retValue;

}


int32_t IPAProxyVimc::configure(
	const IPACameraSensorInfo &sensorInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	const std::map<uint32_t, libcamera::ControlInfoMap> &entityControls)
{
	if (isolate_)
		return configureIPC(sensorInfo, streamConfig, entityControls);
	else
		return configureThread(sensorInfo, streamConfig, entityControls);
}

int32_t IPAProxyVimc::configureThread(
	const IPACameraSensorInfo &sensorInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	const std::map<uint32_t, libcamera::ControlInfoMap> &entityControls)
{
	return ipa_->configure(sensorInfo, streamConfig, entityControls);

}

int32_t IPAProxyVimc::configureIPC(
	const IPACameraSensorInfo &sensorInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	const std::map<uint32_t, libcamera::ControlInfoMap> &entityControls)
{
	controlSerializer_.reset();
	IPCMessage::Header _header = { static_cast<uint32_t>(_VimcCmd::Configure), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> sensorInfoBuf;
	std::tie(sensorInfoBuf, std::ignore) =
		IPADataSerializer<IPACameraSensorInfo>::serialize(sensorInfo
);
	std::vector<uint8_t> streamConfigBuf;
	std::tie(streamConfigBuf, std::ignore) =
		IPADataSerializer<std::map<uint32_t, libcamera::IPAStream>>::serialize(streamConfig
);
	std::vector<uint8_t> entityControlsBuf;
	std::tie(entityControlsBuf, std::ignore) =
		IPADataSerializer<std::map<uint32_t, libcamera::ControlInfoMap>>::serialize(entityControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorInfoBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), streamConfigBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), entityControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorInfoBuf.begin(), sensorInfoBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), streamConfigBuf.begin(), streamConfigBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), entityControlsBuf.begin(), entityControlsBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call configure";
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);






	return _retValue;

}


int32_t IPAProxyVimc::start()
{
	if (isolate_)
		return startIPC();
	else
		return startThread();
}

int32_t IPAProxyVimc::startThread()
{
	state_ = ProxyRunning;
	thread_.start();

	return proxy_.invokeMethod(&ThreadProxy::start, ConnectionTypeBlocking);
}

int32_t IPAProxyVimc::startIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_VimcCmd::Start), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;




	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call start";
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);






	return _retValue;

}


void IPAProxyVimc::stop()
{
	if (isolate_)
		stopIPC();
	else
		stopThread();
}

void IPAProxyVimc::stopThread()
{
	ASSERT(state_ != ProxyStopping);
	if (state_ != ProxyRunning)
		return;

	state_ = ProxyStopping;

	proxy_.invokeMethod(&ThreadProxy::stop, ConnectionTypeBlocking);

	thread_.exit();
	thread_.wait();

	Thread::current()->dispatchMessages(Message::Type::InvokeMessage);

	state_ = ProxyStopped;
}

void IPAProxyVimc::stopIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_VimcCmd::Stop), seq_++ };
	IPCMessage _ipcInputBuf(_header);




	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call stop";
		return;
	}
}


void IPAProxyVimc::mapBuffers(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	if (isolate_)
		mapBuffersIPC(buffers);
	else
		mapBuffersThread(buffers);
}

void IPAProxyVimc::mapBuffersThread(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	ipa_->mapBuffers(buffers);

}

void IPAProxyVimc::mapBuffersIPC(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_VimcCmd::MapBuffers), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> buffersBuf;
	std::vector<SharedFD> buffersFds;
	std::tie(buffersBuf, buffersFds) =
		IPADataSerializer<std::vector<libcamera::IPABuffer>>::serialize(buffers
);
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), buffersBuf.begin(), buffersBuf.end());
	_ipcInputBuf.fds().insert(_ipcInputBuf.fds().end(), buffersFds.begin(), buffersFds.end());


	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call mapBuffers";
		return;
	}
}


void IPAProxyVimc::unmapBuffers(
	const std::vector<uint32_t> &ids)
{
	if (isolate_)
		unmapBuffersIPC(ids);
	else
		unmapBuffersThread(ids);
}

void IPAProxyVimc::unmapBuffersThread(
	const std::vector<uint32_t> &ids)
{
	ipa_->unmapBuffers(ids);

}

void IPAProxyVimc::unmapBuffersIPC(
	const std::vector<uint32_t> &ids)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_VimcCmd::UnmapBuffers), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> idsBuf;
	std::tie(idsBuf, std::ignore) =
		IPADataSerializer<std::vector<uint32_t>>::serialize(ids
);
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), idsBuf.begin(), idsBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call unmapBuffers";
		return;
	}
}


void IPAProxyVimc::queueRequest(
	const uint32_t frame,
	const ControlList &controls)
{
	if (isolate_)
		queueRequestIPC(frame, controls);
	else
		queueRequestThread(frame, controls);
}

void IPAProxyVimc::queueRequestThread(
	const uint32_t frame,
	const ControlList &controls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::queueRequest, ConnectionTypeQueued,frame, controls);
}

void IPAProxyVimc::queueRequestIPC(
	const uint32_t frame,
	const ControlList &controls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_VimcCmd::QueueRequest), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> controlsBuf;
	std::tie(controlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(controls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), controlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), controlsBuf.begin(), controlsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call queueRequest";
		return;
	}
}


void IPAProxyVimc::fillParamsBuffer(
	const uint32_t frame,
	const uint32_t bufferId)
{
	if (isolate_)
		fillParamsBufferIPC(frame, bufferId);
	else
		fillParamsBufferThread(frame, bufferId);
}

void IPAProxyVimc::fillParamsBufferThread(
	const uint32_t frame,
	const uint32_t bufferId)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::fillParamsBuffer, ConnectionTypeQueued,frame, bufferId);
}

void IPAProxyVimc::fillParamsBufferIPC(
	const uint32_t frame,
	const uint32_t bufferId)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_VimcCmd::FillParamsBuffer), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> bufferIdBuf;
	std::tie(bufferIdBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(bufferId
);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), bufferIdBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), bufferIdBuf.begin(), bufferIdBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call fillParamsBuffer";
		return;
	}
}




void IPAProxyVimc::paramsBufferReadyThread(
	const uint32_t bufferId,
	const Flags<ipa::vimc::TestFlag> flags)
{
	ASSERT(state_ != ProxyStopped);
	paramsBufferReady.emit(bufferId, flags);
}

void IPAProxyVimc::paramsBufferReadyIPC(
	std::vector<uint8_t>::const_iterator data,
	size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{
	uint32_t bufferId;
	Flags<ipa::vimc::TestFlag> flags;

	[[maybe_unused]] const size_t bufferIdBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t flagsBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t bufferIdStart = 8;
	const size_t flagsStart = bufferIdStart + bufferIdBufSize;


	bufferId =
        IPADataSerializer<uint32_t>::deserialize(
        	data + bufferIdStart,
        	data + bufferIdStart + bufferIdBufSize);

	flags =
        IPADataSerializer<Flags<ipa::vimc::TestFlag>>::deserialize(
        	data + flagsStart,
        	data + flagsStart + flagsBufSize);

	paramsBufferReady.emit(bufferId, flags);
}


} /* namespace vimc */

} /* namespace ipa */

} /* namespace libcamera */