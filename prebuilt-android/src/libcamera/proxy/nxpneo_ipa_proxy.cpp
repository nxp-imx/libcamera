/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy for nxpneo
 *
 * This file is auto-generated. Do not edit.
 */

#include <libcamera/ipa/nxpneo_ipa_proxy.h>

#include <memory>
#include <string>
#include <vector>

#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/nxpneo_ipa_interface.h>
#include <libcamera/ipa/nxpneo_ipa_serializer.h>

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

namespace nxpneo {


IPAProxyNxpNeo::IPAProxyNxpNeo(IPAModule *ipam, bool isolate)
	: IPAProxy(ipam), isolate_(isolate),
	  controlSerializer_(ControlSerializer::Role::Proxy), seq_(0)
{
	LOG(IPAProxy, Debug)
		<< "initializing nxpneo proxy: loading IPA from "
		<< ipam->path();

	if (isolate_) {
		const std::string proxyWorkerPath = resolvePath("nxpneo_ipa_proxy");
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

		ipc_->recv.connect(this, &IPAProxyNxpNeo::recvMessage);

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

	ipa_ = std::unique_ptr<IPANxpNeoInterface>(static_cast<IPANxpNeoInterface *>(ipai));
	proxy_.setIPA(ipa_.get());


	ipa_->paramsComputed.connect(this, &IPAProxyNxpNeo::paramsComputedThread);
	ipa_->setSensorControls.connect(this, &IPAProxyNxpNeo::setSensorControlsThread);
	ipa_->metadataReady.connect(this, &IPAProxyNxpNeo::metadataReadyThread);

	valid_ = true;
}

IPAProxyNxpNeo::~IPAProxyNxpNeo()
{
	if (isolate_) {
		IPCMessage::Header header =
			{ static_cast<uint32_t>(_NxpNeoCmd::Exit), seq_++ };
		IPCMessage msg(header);
		ipc_->sendAsync(msg);
	}
}


void IPAProxyNxpNeo::recvMessage(const IPCMessage &data)
{
	size_t dataSize = data.data().size();
	_NxpNeoEventCmd _cmd = static_cast<_NxpNeoEventCmd>(data.header().cmd);

	switch (_cmd) {
	case _NxpNeoEventCmd::ParamsComputed: {
		paramsComputedIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	case _NxpNeoEventCmd::SetSensorControls: {
		setSensorControlsIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	case _NxpNeoEventCmd::MetadataReady: {
		metadataReadyIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	default:
		LOG(IPAProxy, Error) << "Unknown command " << static_cast<uint32_t>(_cmd);
	}
}


int32_t IPAProxyNxpNeo::init(
	const IPASettings &settings,
	const InitParams &params,
	ControlInfoMap *ipaControls,
	SensorConfig *sensorConfig)
{
	if (isolate_)
		return initIPC(settings, params, ipaControls, sensorConfig);
	else
		return initThread(settings, params, ipaControls, sensorConfig);
}

int32_t IPAProxyNxpNeo::initThread(
	const IPASettings &settings,
	const InitParams &params,
	ControlInfoMap *ipaControls,
	SensorConfig *sensorConfig)
{
	int32_t _ret = ipa_->init(settings, params, ipaControls, sensorConfig);

	proxy_.moveToThread(&thread_);

	return _ret;
}

int32_t IPAProxyNxpNeo::initIPC(
	const IPASettings &settings,
	const InitParams &params,
	ControlInfoMap *ipaControls,
	SensorConfig *sensorConfig)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::Init), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> settingsBuf;
	std::tie(settingsBuf, std::ignore) =
		IPADataSerializer<IPASettings>::serialize(settings
);
	std::vector<uint8_t> paramsBuf;
	std::tie(paramsBuf, std::ignore) =
		IPADataSerializer<InitParams>::serialize(params
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), settingsBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), paramsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), settingsBuf.begin(), settingsBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), paramsBuf.begin(), paramsBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call init: " << _ret;
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);


	[[maybe_unused]] const size_t ipaControlsBufSize = readPOD<uint32_t>(_ipcOutputBuf.data(), 4);
	[[maybe_unused]] const size_t sensorConfigBufSize = readPOD<uint32_t>(_ipcOutputBuf.data(), 8);

	const size_t ipaControlsStart = 12;
	const size_t sensorConfigStart = ipaControlsStart + ipaControlsBufSize;


	if (ipaControls) {
                *ipaControls =
                IPADataSerializer<ControlInfoMap>::deserialize(
                	_ipcOutputBuf.data().cbegin() + ipaControlsStart,
                	_ipcOutputBuf.data().cbegin() + ipaControlsStart + ipaControlsBufSize,
                	&controlSerializer_);
	}

	if (sensorConfig) {
                *sensorConfig =
                IPADataSerializer<SensorConfig>::deserialize(
                	_ipcOutputBuf.data().cbegin() + sensorConfigStart,
                	_ipcOutputBuf.data().cend());
	}


	return _retValue;

}


int32_t IPAProxyNxpNeo::start()
{
	if (isolate_)
		return startIPC();
	else
		return startThread();
}

int32_t IPAProxyNxpNeo::startThread()
{
	state_ = ProxyRunning;
	thread_.start();

	return proxy_.invokeMethod(&ThreadProxy::start, ConnectionTypeBlocking);
}

int32_t IPAProxyNxpNeo::startIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::Start), seq_++ };
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


void IPAProxyNxpNeo::stop()
{
	if (isolate_)
		return stopIPC();
	else
		return stopThread();
}

void IPAProxyNxpNeo::stopThread()
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

void IPAProxyNxpNeo::stopIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::Stop), seq_++ };
	IPCMessage _ipcInputBuf(_header);




	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call stop: " << _ret;
		return;
	}
}


int32_t IPAProxyNxpNeo::configure(
	const IPAConfigInfo &configInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	const IPAModeType mode,
	const IPAColorSpace &colorSpace,
	ControlInfoMap *ipaControls)
{
	if (isolate_)
		return configureIPC(configInfo, streamConfig, mode, colorSpace, ipaControls);
	else
		return configureThread(configInfo, streamConfig, mode, colorSpace, ipaControls);
}

int32_t IPAProxyNxpNeo::configureThread(
	const IPAConfigInfo &configInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	const IPAModeType mode,
	const IPAColorSpace &colorSpace,
	ControlInfoMap *ipaControls)
{
	return ipa_->configure(configInfo, streamConfig, mode, colorSpace, ipaControls);

}

int32_t IPAProxyNxpNeo::configureIPC(
	const IPAConfigInfo &configInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	const IPAModeType mode,
	const IPAColorSpace &colorSpace,
	ControlInfoMap *ipaControls)
{
	controlSerializer_.reset();
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::Configure), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> configInfoBuf;
	std::tie(configInfoBuf, std::ignore) =
		IPADataSerializer<IPAConfigInfo>::serialize(configInfo
, &controlSerializer_);
	std::vector<uint8_t> streamConfigBuf;
	std::tie(streamConfigBuf, std::ignore) =
		IPADataSerializer<std::map<uint32_t, libcamera::IPAStream>>::serialize(streamConfig
);
	static_assert(sizeof(ipa::nxpneo::IPAModeType) <= 4);
	std::vector<uint8_t> modeBuf;
	std::tie(modeBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(static_cast<uint32_t>(mode));
	std::vector<uint8_t> colorSpaceBuf;
	std::tie(colorSpaceBuf, std::ignore) =
		IPADataSerializer<IPAColorSpace>::serialize(colorSpace
);
	appendPOD<uint32_t>(_ipcInputBuf.data(), configInfoBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), streamConfigBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), modeBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), colorSpaceBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), configInfoBuf.begin(), configInfoBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), streamConfigBuf.begin(), streamConfigBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), modeBuf.begin(), modeBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), colorSpaceBuf.begin(), colorSpaceBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call configure: " << _ret;
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);



	const size_t ipaControlsStart = 4;


	if (ipaControls) {
                *ipaControls =
                IPADataSerializer<ControlInfoMap>::deserialize(
                	_ipcOutputBuf.data().cbegin() + ipaControlsStart,
                	_ipcOutputBuf.data().cend(),
                	&controlSerializer_);
	}


	return _retValue;

}


void IPAProxyNxpNeo::mapBuffers(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	if (isolate_)
		return mapBuffersIPC(buffers);
	else
		return mapBuffersThread(buffers);
}

void IPAProxyNxpNeo::mapBuffersThread(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	return ipa_->mapBuffers(buffers);

}

void IPAProxyNxpNeo::mapBuffersIPC(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::MapBuffers), seq_++ };
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
		LOG(IPAProxy, Error) << "Failed to call mapBuffers: " << _ret;
		return;
	}
}


void IPAProxyNxpNeo::unmapBuffers(
	const std::vector<uint32_t> &ids)
{
	if (isolate_)
		return unmapBuffersIPC(ids);
	else
		return unmapBuffersThread(ids);
}

void IPAProxyNxpNeo::unmapBuffersThread(
	const std::vector<uint32_t> &ids)
{
	return ipa_->unmapBuffers(ids);

}

void IPAProxyNxpNeo::unmapBuffersIPC(
	const std::vector<uint32_t> &ids)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::UnmapBuffers), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> idsBuf;
	std::tie(idsBuf, std::ignore) =
		IPADataSerializer<std::vector<uint32_t>>::serialize(ids
);
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), idsBuf.begin(), idsBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call unmapBuffers: " << _ret;
		return;
	}
}


void IPAProxyNxpNeo::queueRequest(
	const uint32_t frame,
	const ControlList &reqControls)
{
	if (isolate_)
		return queueRequestIPC(frame, reqControls);
	else
		return queueRequestThread(frame, reqControls);
}

void IPAProxyNxpNeo::queueRequestThread(
	const uint32_t frame,
	const ControlList &reqControls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::queueRequest, ConnectionTypeQueued, frame, reqControls);
}

void IPAProxyNxpNeo::queueRequestIPC(
	const uint32_t frame,
	const ControlList &reqControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::QueueRequest), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> reqControlsBuf;
	std::tie(reqControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(reqControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), reqControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), reqControlsBuf.begin(), reqControlsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call queueRequest: " << _ret;
		return;
	}
}


void IPAProxyNxpNeo::computeParams(
	const uint32_t frame,
	const IPAContextType context,
	const std::map<uint32_t, uint32_t> &bufferIds)
{
	if (isolate_)
		return computeParamsIPC(frame, context, bufferIds);
	else
		return computeParamsThread(frame, context, bufferIds);
}

void IPAProxyNxpNeo::computeParamsThread(
	const uint32_t frame,
	const IPAContextType context,
	const std::map<uint32_t, uint32_t> &bufferIds)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::computeParams, ConnectionTypeQueued, frame, context, bufferIds);
}

void IPAProxyNxpNeo::computeParamsIPC(
	const uint32_t frame,
	const IPAContextType context,
	const std::map<uint32_t, uint32_t> &bufferIds)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::ComputeParams), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	static_assert(sizeof(ipa::nxpneo::IPAContextType) <= 4);
	std::vector<uint8_t> contextBuf;
	std::tie(contextBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(static_cast<uint32_t>(context));
	std::vector<uint8_t> bufferIdsBuf;
	std::tie(bufferIdsBuf, std::ignore) =
		IPADataSerializer<std::map<uint32_t, uint32_t>>::serialize(bufferIds
);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), contextBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), bufferIdsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), contextBuf.begin(), contextBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), bufferIdsBuf.begin(), bufferIdsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call computeParams: " << _ret;
		return;
	}
}


void IPAProxyNxpNeo::processStats(
	const uint32_t frame,
	const IPAContextType context,
	const std::map<uint32_t, uint32_t> &bufferIds,
	const ControlList &sensorControls)
{
	if (isolate_)
		return processStatsIPC(frame, context, bufferIds, sensorControls);
	else
		return processStatsThread(frame, context, bufferIds, sensorControls);
}

void IPAProxyNxpNeo::processStatsThread(
	const uint32_t frame,
	const IPAContextType context,
	const std::map<uint32_t, uint32_t> &bufferIds,
	const ControlList &sensorControls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::processStats, ConnectionTypeQueued, frame, context, bufferIds, sensorControls);
}

void IPAProxyNxpNeo::processStatsIPC(
	const uint32_t frame,
	const IPAContextType context,
	const std::map<uint32_t, uint32_t> &bufferIds,
	const ControlList &sensorControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::ProcessStats), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	static_assert(sizeof(ipa::nxpneo::IPAContextType) <= 4);
	std::vector<uint8_t> contextBuf;
	std::tie(contextBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(static_cast<uint32_t>(context));
	std::vector<uint8_t> bufferIdsBuf;
	std::tie(bufferIdsBuf, std::ignore) =
		IPADataSerializer<std::map<uint32_t, uint32_t>>::serialize(bufferIds
);
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(sensorControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), contextBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), bufferIdsBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), contextBuf.begin(), contextBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), bufferIdsBuf.begin(), bufferIdsBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call processStats: " << _ret;
		return;
	}
}




void IPAProxyNxpNeo::paramsComputedThread(
	const uint32_t frame,
	const IPAContextType context)
{
	ASSERT(state_ != ProxyStopped);
	paramsComputed.emit(frame, context);
}

void IPAProxyNxpNeo::paramsComputedIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{

	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t contextBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t frameStart = 8;
	const size_t contextStart = frameStart + frameBufSize;


	uint32_t frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + frameStart + frameBufSize);

	IPAContextType context =
        static_cast<ipa::nxpneo::IPAContextType>(IPADataSerializer<uint32_t>::deserialize(
        	data + contextStart,
        	data + contextStart + contextBufSize));

	paramsComputed.emit(frame, context);
}

void IPAProxyNxpNeo::setSensorControlsThread(
	const uint32_t frame,
	const IPAContextType context,
	const ControlList &sensorControls)
{
	ASSERT(state_ != ProxyStopped);
	setSensorControls.emit(frame, context, sensorControls);
}

void IPAProxyNxpNeo::setSensorControlsIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{

	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t contextBufSize = readPOD<uint32_t>(data, 4, data + dataSize);
	[[maybe_unused]] const size_t sensorControlsBufSize = readPOD<uint32_t>(data, 8, data + dataSize);

	const size_t frameStart = 12;
	const size_t contextStart = frameStart + frameBufSize;
	const size_t sensorControlsStart = contextStart + contextBufSize;


	uint32_t frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + frameStart + frameBufSize);

	IPAContextType context =
        static_cast<ipa::nxpneo::IPAContextType>(IPADataSerializer<uint32_t>::deserialize(
        	data + contextStart,
        	data + contextStart + contextBufSize));

	ControlList sensorControls =
        IPADataSerializer<ControlList>::deserialize(
        	data + sensorControlsStart,
        	data + sensorControlsStart + sensorControlsBufSize,
        	&controlSerializer_);

	setSensorControls.emit(frame, context, sensorControls);
}

void IPAProxyNxpNeo::metadataReadyThread(
	const uint32_t frame,
	const IPAContextType context,
	const ControlList &metadata)
{
	ASSERT(state_ != ProxyStopped);
	metadataReady.emit(frame, context, metadata);
}

void IPAProxyNxpNeo::metadataReadyIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{

	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t contextBufSize = readPOD<uint32_t>(data, 4, data + dataSize);
	[[maybe_unused]] const size_t metadataBufSize = readPOD<uint32_t>(data, 8, data + dataSize);

	const size_t frameStart = 12;
	const size_t contextStart = frameStart + frameBufSize;
	const size_t metadataStart = contextStart + contextBufSize;


	uint32_t frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + frameStart + frameBufSize);

	IPAContextType context =
        static_cast<ipa::nxpneo::IPAContextType>(IPADataSerializer<uint32_t>::deserialize(
        	data + contextStart,
        	data + contextStart + contextBufSize));

	ControlList metadata =
        IPADataSerializer<ControlList>::deserialize(
        	data + metadataStart,
        	data + metadataStart + metadataBufSize,
        	&controlSerializer_);

	metadataReady.emit(frame, context, metadata);
}


} /* namespace nxpneo */

} /* namespace ipa */

} /* namespace libcamera */