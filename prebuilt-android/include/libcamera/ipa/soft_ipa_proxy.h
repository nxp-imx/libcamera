/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy for soft
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/soft_ipa_interface.h>

#include <libcamera/base/object.h>
#include <libcamera/base/thread.h>

#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_pipe_unixsocket.h"
#include "libcamera/internal/ipc_unixsocket.h"

namespace libcamera {

namespace ipa {

namespace soft {


class IPAProxySoftThreaded;
class IPAProxySoftIsolated;

class IPAProxySoft : public IPAProxy, public IPASoftInterface, public Object
{
public:
	using Threaded = IPAProxySoftThreaded;
	using Isolated = IPAProxySoftIsolated;

protected:
	using IPAProxy::IPAProxy;
};

class IPAProxySoftThreaded : public IPAProxySoft
{
public:
	IPAProxySoftThreaded(IPAModule *ipam, const CameraManager &cm);
	~IPAProxySoftThreaded();


        int32_t init(
        	const IPASettings &settings,
        	const SharedFD &fdStats,
        	const SharedFD &fdParams,
        	const IPACameraSensorInfo &sensorInfo,
        	const ControlInfoMap &sensorControls,
        	ControlInfoMap *ipaControls,
        	bool *ccmEnabled) override;

        int32_t start() override;

        void stop() override;

        int32_t configure(
        	const IPAConfigInfo &configInfo) override;

        void queueRequest(
        	const uint32_t frame,
        	const ControlList &sensorControls) override;

        void computeParams(
        	const uint32_t frame) override;

        void processStats(
        	const uint32_t frame,
        	const uint32_t bufferId,
        	const ControlList &sensorControls) override;


private:

        void setSensorControlsHandler(
        	const ControlList &sensorControls);

        void setIspParamsHandler();

        void metadataReadyHandler(
        	const uint32_t frame,
        	const ControlList &metadata);


	/* Helper class to invoke async functions in another thread. */
	class ThreadProxy : public Object
	{
	public:
		ThreadProxy()
			: ipa_(nullptr)
		{
		}

		void setIPA(IPASoftInterface *ipa)
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
                	const ControlList &sensorControls)
		{
			ipa_->queueRequest(frame, sensorControls);
		}
		void computeParams(
                	const uint32_t frame)
		{
			ipa_->computeParams(frame);
		}
		void processStats(
                	const uint32_t frame,
                	const uint32_t bufferId,
                	const ControlList &sensorControls)
		{
			ipa_->processStats(frame, bufferId, sensorControls);
		}

	private:
		IPASoftInterface *ipa_;
	};

	Thread thread_;
	ThreadProxy proxy_;
	std::unique_ptr<IPASoftInterface> ipa_;
};

class IPAProxySoftIsolated : public IPAProxySoft
{
public:
	IPAProxySoftIsolated(IPAModule *ipam, const CameraManager &cm);
	~IPAProxySoftIsolated();


        int32_t init(
        	const IPASettings &settings,
        	const SharedFD &fdStats,
        	const SharedFD &fdParams,
        	const IPACameraSensorInfo &sensorInfo,
        	const ControlInfoMap &sensorControls,
        	ControlInfoMap *ipaControls,
        	bool *ccmEnabled) override;

        int32_t start() override;

        void stop() override;

        int32_t configure(
        	const IPAConfigInfo &configInfo) override;

        void queueRequest(
        	const uint32_t frame,
        	const ControlList &sensorControls) override;

        void computeParams(
        	const uint32_t frame) override;

        void processStats(
        	const uint32_t frame,
        	const uint32_t bufferId,
        	const ControlList &sensorControls) override;


private:
	void recvMessage(const IPCMessage &data);


	void setSensorControlsHandler(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);

	void setIspParamsHandler(
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

} /* namespace soft */

} /* namespace ipa */

} /* namespace libcamera */