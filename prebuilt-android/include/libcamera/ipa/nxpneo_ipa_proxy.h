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


class IPAProxyNxpNeo : public IPAProxy, public IPANxpNeoInterface, public Object
{
public:
	IPAProxyNxpNeo(IPAModule *ipam, bool isolate);
	~IPAProxyNxpNeo();


        int32_t init(
        	const IPASettings &settings,
        	const uint32_t hwRevision,
        	const IPACameraSensorInfo &sensorInfo,
        	const ControlInfoMap &sensorControls,
        	ControlInfoMap *ipaControls,
        	SensorConfig *sensorConfig) override;

        int32_t start() override;

        void stop() override;

        int32_t configure(
        	const IPAConfigInfo &configInfo,
        	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
        	ControlInfoMap *ipaControls) override;

        void mapBuffers(
        	const std::vector<libcamera::IPABuffer> &buffers) override;

        void unmapBuffers(
        	const std::vector<uint32_t> &ids) override;

        void queueRequest(
        	const uint32_t frame,
        	const ControlList &reqControls) override;

        void fillParamsBuffer(
        	const uint32_t frame,
        	const uint32_t paramsbufferId,
        	const uint32_t rawBufferId) override;

        void processStatsBuffer(
        	const uint32_t frame,
        	const uint32_t bufferId,
        	const ControlList &sensorControls) override;

	Signal<uint32_t> paramsBufferReady;

	Signal<uint32_t, const ControlList &> setSensorControls;

	Signal<uint32_t, const ControlList &> metadataReady;


private:
	void recvMessage(const IPCMessage &data);


        int32_t initThread(
        	const IPASettings &settings,
        	const uint32_t hwRevision,
        	const IPACameraSensorInfo &sensorInfo,
        	const ControlInfoMap &sensorControls,
        	ControlInfoMap *ipaControls,
        	SensorConfig *sensorConfig);
        int32_t initIPC(
        	const IPASettings &settings,
        	const uint32_t hwRevision,
        	const IPACameraSensorInfo &sensorInfo,
        	const ControlInfoMap &sensorControls,
        	ControlInfoMap *ipaControls,
        	SensorConfig *sensorConfig);

        int32_t startThread();
        int32_t startIPC();

        void stopThread();
        void stopIPC();

        int32_t configureThread(
        	const IPAConfigInfo &configInfo,
        	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
        	ControlInfoMap *ipaControls);
        int32_t configureIPC(
        	const IPAConfigInfo &configInfo,
        	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
        	ControlInfoMap *ipaControls);

        void mapBuffersThread(
        	const std::vector<libcamera::IPABuffer> &buffers);
        void mapBuffersIPC(
        	const std::vector<libcamera::IPABuffer> &buffers);

        void unmapBuffersThread(
        	const std::vector<uint32_t> &ids);
        void unmapBuffersIPC(
        	const std::vector<uint32_t> &ids);

        void queueRequestThread(
        	const uint32_t frame,
        	const ControlList &reqControls);
        void queueRequestIPC(
        	const uint32_t frame,
        	const ControlList &reqControls);

        void fillParamsBufferThread(
        	const uint32_t frame,
        	const uint32_t paramsbufferId,
        	const uint32_t rawBufferId);
        void fillParamsBufferIPC(
        	const uint32_t frame,
        	const uint32_t paramsbufferId,
        	const uint32_t rawBufferId);

        void processStatsBufferThread(
        	const uint32_t frame,
        	const uint32_t bufferId,
        	const ControlList &sensorControls);
        void processStatsBufferIPC(
        	const uint32_t frame,
        	const uint32_t bufferId,
        	const ControlList &sensorControls);


        void paramsBufferReadyThread(
        	const uint32_t frame);
	void paramsBufferReadyIPC(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);

        void setSensorControlsThread(
        	const uint32_t frame,
        	const ControlList &sensorControls);
	void setSensorControlsIPC(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);

        void metadataReadyThread(
        	const uint32_t frame,
        	const ControlList &metadata);
	void metadataReadyIPC(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);


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
		void fillParamsBuffer(
                	const uint32_t frame,
                	const uint32_t paramsbufferId,
                	const uint32_t rawBufferId)
		{
			ipa_->fillParamsBuffer(frame, paramsbufferId, rawBufferId);
		}
		void processStatsBuffer(
                	const uint32_t frame,
                	const uint32_t bufferId,
                	const ControlList &sensorControls)
		{
			ipa_->processStatsBuffer(frame, bufferId, sensorControls);
		}

	private:
		IPANxpNeoInterface *ipa_;
	};

	Thread thread_;
	ThreadProxy proxy_;
	std::unique_ptr<IPANxpNeoInterface> ipa_;

	const bool isolate_;

	std::unique_ptr<IPCPipeUnixSocket> ipc_;

	ControlSerializer controlSerializer_;


	uint32_t seq_;
};

} /* namespace nxpneo */

} /* namespace ipa */

} /* namespace libcamera */