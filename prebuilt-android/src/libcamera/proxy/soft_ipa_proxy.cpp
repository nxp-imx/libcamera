/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy for soft
 *
 * This file is auto-generated. Do not edit.
 */

#include <libcamera/ipa/soft_ipa_proxy.h>

#include <memory>
#include <string>
#include <vector>

#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/soft_ipa_interface.h>
#include <libcamera/ipa/soft_ipa_serializer.h>

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

namespace soft {


IPAProxySoft::IPAProxySoft(IPAModule *ipam, bool isolate)
	: IPAProxy(ipam), isolate_(isolate),
	  controlSerializer_(ControlSerializer::Role::Proxy), seq_(0)
{
	LOG(IPAProxy, Debug)
		<< "initializing soft proxy: loading IPA from "
		<< ipam->path();

	if (isolate_) {
		const std::string proxyWorkerPath = resolvePath("soft_ipa_proxy");
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

		ipc_->recv.connect(this, &IPAProxySoft::recvMessage);

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

	ipa_ = std::unique_ptr<IPASoftInterface>(static_cast<IPASoftInterface *>(ipai));
	proxy_.setIPA(ipa_.get());


	ipa_->setSensorControls.connect(this, &IPAProxySoft::setSensorControlsThread);
	ipa_->setIspParams.connect(this, &IPAProxySoft::setIspParamsThread);
	ipa_->metadataReady.connect(this, &IPAProxySoft::metadataReadyThread);

	valid_ = true;
}

IPAProxySoft::~IPAProxySoft()
{
	if (isolate_) {
		IPCMessage::Header header =
			{ static_cast<uint32_t>(_SoftCmd::Exit), seq_++ };
		IPCMessage msg(header);
		ipc_->sendAsync(msg);
	}
}


void IPAProxySoft::recvMessage(const IPCMessage &data)
{
	size_t dataSize = data.data().size();
	_SoftEventCmd _cmd = static_cast<_SoftEventCmd>(data.header().cmd);

	switch (_cmd) {
	case _SoftEventCmd::SetSensorControls: {
		setSensorControlsIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	case _SoftEventCmd::SetIspParams: {
		setIspParamsIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	case _SoftEventCmd::MetadataReady: {
		metadataReadyIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	default:
		LOG(IPAProxy, Error) << "Unknown command " << static_cast<uint32_t>(_cmd);
	}
}


int32_t IPAProxySoft::init(
	const IPASettings &settings,
	const SharedFD &fdStats,
	const SharedFD &fdParams,
	const IPACameraSensorInfo &sensorInfo,
	const ControlInfoMap &sensorControls,
	ControlInfoMap *ipaControls,
	bool *ccmEnabled)
{
	if (isolate_)
		return initIPC(settings, fdStats, fdParams, sensorInfo, sensorControls, ipaControls, ccmEnabled);
	else
		return initThread(settings, fdStats, fdParams, sensorInfo, sensorControls, ipaControls, ccmEnabled);
}

int32_t IPAProxySoft::initThread(
	const IPASettings &settings,
	const SharedFD &fdStats,
	const SharedFD &fdParams,
	const IPACameraSensorInfo &sensorInfo,
	const ControlInfoMap &sensorControls,
	ControlInfoMap *ipaControls,
	bool *ccmEnabled)
{
	int32_t _ret = ipa_->init(settings, fdStats, fdParams, sensorInfo, sensorControls, ipaControls, ccmEnabled);

	proxy_.moveToThread(&thread_);

	return _ret;
}

int32_t IPAProxySoft::initIPC(
	const IPASettings &settings,
	const SharedFD &fdStats,
	const SharedFD &fdParams,
	const IPACameraSensorInfo &sensorInfo,
	const ControlInfoMap &sensorControls,
	ControlInfoMap *ipaControls,
	bool *ccmEnabled)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_SoftCmd::Init), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> settingsBuf;
	std::tie(settingsBuf, std::ignore) =
		IPADataSerializer<IPASettings>::serialize(settings
);
	std::vector<uint8_t> fdStatsBuf;
	std::vector<SharedFD> fdStatsFds;
	std::tie(fdStatsBuf, fdStatsFds) =
		IPADataSerializer<SharedFD>::serialize(fdStats
);
	std::vector<uint8_t> fdParamsBuf;
	std::vector<SharedFD> fdParamsFds;
	std::tie(fdParamsBuf, fdParamsFds) =
		IPADataSerializer<SharedFD>::serialize(fdParams
);
	std::vector<uint8_t> sensorInfoBuf;
	std::tie(sensorInfoBuf, std::ignore) =
		IPADataSerializer<IPACameraSensorInfo>::serialize(sensorInfo
);
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlInfoMap>::serialize(sensorControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), settingsBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), fdStatsBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), fdStatsFds.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), fdParamsBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), fdParamsFds.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorInfoBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), settingsBuf.begin(), settingsBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), fdStatsBuf.begin(), fdStatsBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), fdParamsBuf.begin(), fdParamsBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorInfoBuf.begin(), sensorInfoBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());
	_ipcInputBuf.fds().insert(_ipcInputBuf.fds().end(), fdStatsFds.begin(), fdStatsFds.end());
	_ipcInputBuf.fds().insert(_ipcInputBuf.fds().end(), fdParamsFds.begin(), fdParamsFds.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call init: " << _ret;
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);


	[[maybe_unused]] const size_t ipaControlsBufSize = readPOD<uint32_t>(_ipcOutputBuf.data(), 4);
	[[maybe_unused]] const size_t ccmEnabledBufSize = readPOD<uint32_t>(_ipcOutputBuf.data(), 8);

	const size_t ipaControlsStart = 12;
	const size_t ccmEnabledStart = ipaControlsStart + ipaControlsBufSize;


	if (ipaControls) {
                *ipaControls =
                IPADataSerializer<ControlInfoMap>::deserialize(
                	_ipcOutputBuf.data().cbegin() + ipaControlsStart,
                	_ipcOutputBuf.data().cbegin() + ipaControlsStart + ipaControlsBufSize,
                	&controlSerializer_);
	}

	if (ccmEnabled) {
                *ccmEnabled =
                IPADataSerializer<bool>::deserialize(
                	_ipcOutputBuf.data().cbegin() + ccmEnabledStart,
                	_ipcOutputBuf.data().cend());
	}


	return _retValue;

}


int32_t IPAProxySoft::start()
{
	if (isolate_)
		return startIPC();
	else
		return startThread();
}

int32_t IPAProxySoft::startThread()
{
	state_ = ProxyRunning;
	thread_.start();

	return proxy_.invokeMethod(&ThreadProxy::start, ConnectionTypeBlocking);
}

int32_t IPAProxySoft::startIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_SoftCmd::Start), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;




	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call start: " << _ret;
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);






	return _retValue;

}


void IPAProxySoft::stop()
{
	if (isolate_)
		return stopIPC();
	else
		return stopThread();
}

void IPAProxySoft::stopThread()
{
	ASSERT(state_ != ProxyStopping);
	if (state_ != ProxyRunning)
		return;

	state_ = ProxyStopping;

	proxy_.invokeMethod(&ThreadProxy::stop, ConnectionTypeBlocking);

	thread_.exit();
	thread_.wait();

	Thread::current()->dispatchMessages(Message::Type::InvokeMessage, this);

	state_ = ProxyStopped;
}

void IPAProxySoft::stopIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_SoftCmd::Stop), seq_++ };
	IPCMessage _ipcInputBuf(_header);




	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call stop: " << _ret;
		return;
	}
}


int32_t IPAProxySoft::configure(
	const IPAConfigInfo &configInfo)
{
	if (isolate_)
		return configureIPC(configInfo);
	else
		return configureThread(configInfo);
}

int32_t IPAProxySoft::configureThread(
	const IPAConfigInfo &configInfo)
{
	return ipa_->configure(configInfo);

}

int32_t IPAProxySoft::configureIPC(
	const IPAConfigInfo &configInfo)
{
	controlSerializer_.reset();
	IPCMessage::Header _header = { static_cast<uint32_t>(_SoftCmd::Configure), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> configInfoBuf;
	std::tie(configInfoBuf, std::ignore) =
		IPADataSerializer<IPAConfigInfo>::serialize(configInfo
, &controlSerializer_);
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), configInfoBuf.begin(), configInfoBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call configure: " << _ret;
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);






	return _retValue;

}


void IPAProxySoft::queueRequest(
	const uint32_t frame,
	const ControlList &sensorControls)
{
	if (isolate_)
		return queueRequestIPC(frame, sensorControls);
	else
		return queueRequestThread(frame, sensorControls);
}

void IPAProxySoft::queueRequestThread(
	const uint32_t frame,
	const ControlList &sensorControls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::queueRequest, ConnectionTypeQueued, frame, sensorControls);
}

void IPAProxySoft::queueRequestIPC(
	const uint32_t frame,
	const ControlList &sensorControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_SoftCmd::QueueRequest), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(sensorControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call queueRequest: " << _ret;
		return;
	}
}


void IPAProxySoft::computeParams(
	const uint32_t frame)
{
	if (isolate_)
		return computeParamsIPC(frame);
	else
		return computeParamsThread(frame);
}

void IPAProxySoft::computeParamsThread(
	const uint32_t frame)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::computeParams, ConnectionTypeQueued, frame);
}

void IPAProxySoft::computeParamsIPC(
	const uint32_t frame)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_SoftCmd::ComputeParams), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call computeParams: " << _ret;
		return;
	}
}


void IPAProxySoft::processStats(
	const uint32_t frame,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	if (isolate_)
		return processStatsIPC(frame, bufferId, sensorControls);
	else
		return processStatsThread(frame, bufferId, sensorControls);
}

void IPAProxySoft::processStatsThread(
	const uint32_t frame,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::processStats, ConnectionTypeQueued, frame, bufferId, sensorControls);
}

void IPAProxySoft::processStatsIPC(
	const uint32_t frame,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_SoftCmd::ProcessStats), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> bufferIdBuf;
	std::tie(bufferIdBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(bufferId
);
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(sensorControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), bufferIdBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), bufferIdBuf.begin(), bufferIdBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call processStats: " << _ret;
		return;
	}
}




void IPAProxySoft::setSensorControlsThread(
	const ControlList &sensorControls)
{
	ASSERT(state_ != ProxyStopped);
	setSensorControls.emit(sensorControls);
}

void IPAProxySoft::setSensorControlsIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{


	const size_t sensorControlsStart = 0;


	ControlList sensorControls =
        IPADataSerializer<ControlList>::deserialize(
        	data + sensorControlsStart,
        	data + dataSize,
        	&controlSerializer_);

	setSensorControls.emit(sensorControls);
}

void IPAProxySoft::setIspParamsThread()
{
	ASSERT(state_ != ProxyStopped);
	setIspParams.emit();
}

void IPAProxySoft::setIspParamsIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{




	setIspParams.emit();
}

void IPAProxySoft::metadataReadyThread(
	const uint32_t frame,
	const ControlList &metadata)
{
	ASSERT(state_ != ProxyStopped);
	metadataReady.emit(frame, metadata);
}

void IPAProxySoft::metadataReadyIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{

	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t metadataBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t frameStart = 8;
	const size_t metadataStart = frameStart + frameBufSize;


	uint32_t frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + frameStart + frameBufSize);

	ControlList metadata =
        IPADataSerializer<ControlList>::deserialize(
        	data + metadataStart,
        	data + metadataStart + metadataBufSize,
        	&controlSerializer_);

	metadataReady.emit(frame, metadata);
}


} /* namespace soft */

} /* namespace ipa */

} /* namespace libcamera */