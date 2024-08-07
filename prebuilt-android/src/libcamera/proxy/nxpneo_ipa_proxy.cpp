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


	ipa_->paramsBufferReady.connect(this, &IPAProxyNxpNeo::paramsBufferReadyThread);
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
	case _NxpNeoEventCmd::ParamsBufferReady: {
		paramsBufferReadyIPC(data.data().cbegin(), dataSize, data.fds());
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
	const uint32_t hwRevision,
	const IPACameraSensorInfo &sensorInfo,
	const ControlInfoMap &sensorControls,
	ControlInfoMap *ipaControls,
	SensorConfig *sensorConfig)
{
	if (isolate_)
		return initIPC(settings, hwRevision, sensorInfo, sensorControls, ipaControls, sensorConfig);
	else
		return initThread(settings, hwRevision, sensorInfo, sensorControls, ipaControls, sensorConfig);
}

int32_t IPAProxyNxpNeo::initThread(
	const IPASettings &settings,
	const uint32_t hwRevision,
	const IPACameraSensorInfo &sensorInfo,
	const ControlInfoMap &sensorControls,
	ControlInfoMap *ipaControls,
	SensorConfig *sensorConfig)
{
	int32_t _ret = ipa_->init(settings, hwRevision, sensorInfo, sensorControls, ipaControls, sensorConfig);

	proxy_.moveToThread(&thread_);

	return _ret;
}

int32_t IPAProxyNxpNeo::initIPC(
	const IPASettings &settings,
	const uint32_t hwRevision,
	const IPACameraSensorInfo &sensorInfo,
	const ControlInfoMap &sensorControls,
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
	std::vector<uint8_t> hwRevisionBuf;
	std::tie(hwRevisionBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(hwRevision
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
	appendPOD<uint32_t>(_ipcInputBuf.data(), hwRevisionBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorInfoBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), settingsBuf.begin(), settingsBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), hwRevisionBuf.begin(), hwRevisionBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorInfoBuf.begin(), sensorInfoBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call init";
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
		LOG(IPAProxy, Error) << "Failed to call start";
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);






	return _retValue;

}


void IPAProxyNxpNeo::stop()
{
	if (isolate_)
		stopIPC();
	else
		stopThread();
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

	Thread::current()->dispatchMessages(Message::Type::InvokeMessage);

	state_ = ProxyStopped;
}

void IPAProxyNxpNeo::stopIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::Stop), seq_++ };
	IPCMessage _ipcInputBuf(_header);




	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call stop";
		return;
	}
}


int32_t IPAProxyNxpNeo::configure(
	const IPAConfigInfo &configInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	ControlInfoMap *ipaControls)
{
	if (isolate_)
		return configureIPC(configInfo, streamConfig, ipaControls);
	else
		return configureThread(configInfo, streamConfig, ipaControls);
}

int32_t IPAProxyNxpNeo::configureThread(
	const IPAConfigInfo &configInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	ControlInfoMap *ipaControls)
{
	return ipa_->configure(configInfo, streamConfig, ipaControls);

}

int32_t IPAProxyNxpNeo::configureIPC(
	const IPAConfigInfo &configInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
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
	appendPOD<uint32_t>(_ipcInputBuf.data(), configInfoBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), streamConfigBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), configInfoBuf.begin(), configInfoBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), streamConfigBuf.begin(), streamConfigBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call configure";
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
		mapBuffersIPC(buffers);
	else
		mapBuffersThread(buffers);
}

void IPAProxyNxpNeo::mapBuffersThread(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	ipa_->mapBuffers(buffers);

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
		LOG(IPAProxy, Error) << "Failed to call mapBuffers";
		return;
	}
}


void IPAProxyNxpNeo::unmapBuffers(
	const std::vector<uint32_t> &ids)
{
	if (isolate_)
		unmapBuffersIPC(ids);
	else
		unmapBuffersThread(ids);
}

void IPAProxyNxpNeo::unmapBuffersThread(
	const std::vector<uint32_t> &ids)
{
	ipa_->unmapBuffers(ids);

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
		LOG(IPAProxy, Error) << "Failed to call unmapBuffers";
		return;
	}
}


void IPAProxyNxpNeo::queueRequest(
	const uint32_t frame,
	const ControlList &reqControls)
{
	if (isolate_)
		queueRequestIPC(frame, reqControls);
	else
		queueRequestThread(frame, reqControls);
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
		LOG(IPAProxy, Error) << "Failed to call queueRequest";
		return;
	}
}


void IPAProxyNxpNeo::fillParamsBuffer(
	const uint32_t frame,
	const uint32_t paramsbufferId,
	const uint32_t rawBufferId)
{
	if (isolate_)
		fillParamsBufferIPC(frame, paramsbufferId, rawBufferId);
	else
		fillParamsBufferThread(frame, paramsbufferId, rawBufferId);
}

void IPAProxyNxpNeo::fillParamsBufferThread(
	const uint32_t frame,
	const uint32_t paramsbufferId,
	const uint32_t rawBufferId)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::fillParamsBuffer, ConnectionTypeQueued, frame, paramsbufferId, rawBufferId);
}

void IPAProxyNxpNeo::fillParamsBufferIPC(
	const uint32_t frame,
	const uint32_t paramsbufferId,
	const uint32_t rawBufferId)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::FillParamsBuffer), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> paramsbufferIdBuf;
	std::tie(paramsbufferIdBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(paramsbufferId
);
	std::vector<uint8_t> rawBufferIdBuf;
	std::tie(rawBufferIdBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(rawBufferId
);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), paramsbufferIdBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), rawBufferIdBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), paramsbufferIdBuf.begin(), paramsbufferIdBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), rawBufferIdBuf.begin(), rawBufferIdBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call fillParamsBuffer";
		return;
	}
}


void IPAProxyNxpNeo::processStatsBuffer(
	const uint32_t frame,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	if (isolate_)
		processStatsBufferIPC(frame, bufferId, sensorControls);
	else
		processStatsBufferThread(frame, bufferId, sensorControls);
}

void IPAProxyNxpNeo::processStatsBufferThread(
	const uint32_t frame,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::processStatsBuffer, ConnectionTypeQueued, frame, bufferId, sensorControls);
}

void IPAProxyNxpNeo::processStatsBufferIPC(
	const uint32_t frame,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_NxpNeoCmd::ProcessStatsBuffer), seq_++ };
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
		LOG(IPAProxy, Error) << "Failed to call processStatsBuffer";
		return;
	}
}




void IPAProxyNxpNeo::paramsBufferReadyThread(
	const uint32_t frame)
{
	ASSERT(state_ != ProxyStopped);
	paramsBufferReady.emit(frame);
}

void IPAProxyNxpNeo::paramsBufferReadyIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{
	uint32_t frame;


	const size_t frameStart = 0;


	frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + dataSize);

	paramsBufferReady.emit(frame);
}

void IPAProxyNxpNeo::setSensorControlsThread(
	const uint32_t frame,
	const ControlList &sensorControls)
{
	ASSERT(state_ != ProxyStopped);
	setSensorControls.emit(frame, sensorControls);
}

void IPAProxyNxpNeo::setSensorControlsIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{
	uint32_t frame;
	ControlList sensorControls;

	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t sensorControlsBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t frameStart = 8;
	const size_t sensorControlsStart = frameStart + frameBufSize;


	frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + frameStart + frameBufSize);

	sensorControls =
        IPADataSerializer<ControlList>::deserialize(
        	data + sensorControlsStart,
        	data + sensorControlsStart + sensorControlsBufSize,
        	&controlSerializer_);

	setSensorControls.emit(frame, sensorControls);
}

void IPAProxyNxpNeo::metadataReadyThread(
	const uint32_t frame,
	const ControlList &metadata)
{
	ASSERT(state_ != ProxyStopped);
	metadataReady.emit(frame, metadata);
}

void IPAProxyNxpNeo::metadataReadyIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{
	uint32_t frame;
	ControlList metadata;

	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t metadataBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t frameStart = 8;
	const size_t metadataStart = frameStart + frameBufSize;


	frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + frameStart + frameBufSize);

	metadata =
        IPADataSerializer<ControlList>::deserialize(
        	data + metadataStart,
        	data + metadataStart + metadataBufSize,
        	&controlSerializer_);

	metadataReady.emit(frame, metadata);
}


} /* namespace nxpneo */

} /* namespace ipa */

} /* namespace libcamera */