/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy for nxpneo
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/nxpneo_ipa_interface.h>

#include <libcamera/base/object.h>
#include <libcamera/base/thread.h>

#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_pipe_unixsocket.h"
#include "libcamera/internal/ipc_unixsocket.h"

namespace libcamera {

namespace ipa {

namespace nxpneo {


class IPAProxyNxpNeoThreaded;
class IPAProxyNxpNeoIsolated;

class IPAProxyNxpNeo : public IPAProxy, public IPANxpNeoInterface, public Object
{
public:
	using Threaded = IPAProxyNxpNeoThreaded;
	using Isolated = IPAProxyNxpNeoIsolated;

protected:
	using IPAProxy::IPAProxy;
};

class IPAProxyNxpNeoThreaded : public IPAProxyNxpNeo
{
public:
	IPAProxyNxpNeoThreaded(IPAModule *ipam, const GlobalConfiguration &configuration);
	~IPAProxyNxpNeoThreaded();


        int32_t init(
        	const IPASettings &settings,
        	const InitParams &params,
        	ControlInfoMap *ipaControls,
        	SensorConfig *sensorConfig) override;

        int32_t start() override;

        void stop() override;

        int32_t configure(
        	const IPAConfigInfo &configInfo,
        	const std::map<ipa::nxpneo::IPAStreamType, libcamera::IPAStream> &streamConfig,
        	ControlInfoMap *ipaControls) override;

        void mapBuffers(
        	const std::vector<libcamera::IPABuffer> &buffers) override;

        void unmapBuffers(
        	const std::vector<uint32_t> &ids) override;

        void queueRequest(
        	const uint32_t frame,
        	const ControlList &reqControls) override;

        void computeParams(
        	const uint32_t frame,
        	const IPACameraContext context,
        	const std::map<ipa::nxpneo::IPABufferType, uint32_t> &bufferIds) override;

        void processStats(
        	const uint32_t frame,
        	const IPACameraContext context,
        	const std::map<ipa::nxpneo::IPABufferType, uint32_t> &bufferIds,
        	const ControlList &sensorControls) override;


private:

        void paramsComputedHandler(
        	const uint32_t frame,
        	const IPACameraContext context,
        	const uint32_t bytesused);

        void setSensorControlsHandler(
        	const uint32_t frame,
        	const ControlList &sensorControls);

        void setLensControlsHandler(
        	const ControlList &controls);

        void metadataReadyHandler(
        	const uint32_t frame,
        	const IPACameraContext context,
        	const ControlList &metadata);


	/* Helper class to invoke async functions in another thread. */
	class ThreadProxy : public Object
	{
	public:
		ThreadProxy()
			: ipa_(nullptr)
		{
		}

		void setIPA(IPANxpNeoInterface *ipa)
		{
			ipa_ = ipa;
		}

		void stop()
		{
			ipa_->stop();
		}

		int32_t start()
		{
			return ipa_->start();
		}
		void queueRequest(
                	const uint32_t frame,
                	const ControlList &reqControls)
		{
			ipa_->queueRequest(frame, reqControls);
		}
		void computeParams(
                	const uint32_t frame,
                	const IPACameraContext context,
                	const std::map<ipa::nxpneo::IPABufferType, uint32_t> &bufferIds)
		{
			ipa_->computeParams(frame, context, bufferIds);
		}
		void processStats(
                	const uint32_t frame,
                	const IPACameraContext context,
                	const std::map<ipa::nxpneo::IPABufferType, uint32_t> &bufferIds,
                	const ControlList &sensorControls)
		{
			ipa_->processStats(frame, context, bufferIds, sensorControls);
		}

	private:
		IPANxpNeoInterface *ipa_;
	};

	Thread thread_;
	ThreadProxy proxy_;
	std::unique_ptr<IPANxpNeoInterface> ipa_;
};

class IPAProxyNxpNeoIsolated : public IPAProxyNxpNeo
{
public:
	IPAProxyNxpNeoIsolated(IPAModule *ipam, const GlobalConfiguration &configuration);
	~IPAProxyNxpNeoIsolated();


        int32_t init(
        	const IPASettings &settings,
        	const InitParams &params,
        	ControlInfoMap *ipaControls,
        	SensorConfig *sensorConfig) override;

        int32_t start() override;

        void stop() override;

        int32_t configure(
        	const IPAConfigInfo &configInfo,
        	const std::map<ipa::nxpneo::IPAStreamType, libcamera::IPAStream> &streamConfig,
        	ControlInfoMap *ipaControls) override;

        void mapBuffers(
        	const std::vector<libcamera::IPABuffer> &buffers) override;

        void unmapBuffers(
        	const std::vector<uint32_t> &ids) override;

        void queueRequest(
        	const uint32_t frame,
        	const ControlList &reqControls) override;

        void computeParams(
        	const uint32_t frame,
        	const IPACameraContext context,
        	const std::map<ipa::nxpneo::IPABufferType, uint32_t> &bufferIds) override;

        void processStats(
        	const uint32_t frame,
        	const IPACameraContext context,
        	const std::map<ipa::nxpneo::IPABufferType, uint32_t> &bufferIds,
        	const ControlList &sensorControls) override;


private:
	void recvMessage(const IPCMessage &data);


	void paramsComputedHandler(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);

	void setSensorControlsHandler(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);

	void setLensControlsHandler(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);

	void metadataReadyHandler(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);


	std::unique_ptr<IPCPipeUnixSocket> ipc_;

	ControlSerializer controlSerializer_;


	uint32_t seq_;
};

} /* namespace nxpneo */

} /* namespace ipa */

} /* namespace libcamera */