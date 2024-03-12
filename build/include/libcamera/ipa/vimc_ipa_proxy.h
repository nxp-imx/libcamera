/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * vimc_ipa_proxy.h - Image Processing Algorithm proxy for vimc
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/vimc_ipa_interface.h>

#include <libcamera/base/thread.h>

#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_pipe_unixsocket.h"
#include "libcamera/internal/ipc_unixsocket.h"

namespace libcamera {

namespace ipa {

namespace vimc {


class IPAProxyVimc : public IPAProxy, public IPAVimcInterface, public Object
{
public:
	IPAProxyVimc(IPAModule *ipam, bool isolate);
	~IPAProxyVimc();


        int32_t init(
        	const IPASettings &settings,
        	const IPAOperationCode code,
        	const Flags<ipa::vimc::TestFlag> inFlags,
        	Flags<ipa::vimc::TestFlag> *outFlags) override;

        int32_t configure(
        	const IPACameraSensorInfo &sensorInfo,
        	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
        	const std::map<uint32_t, libcamera::ControlInfoMap> &entityControls) override;

        int32_t start() override;

        void stop() override;

        void mapBuffers(
        	const std::vector<libcamera::IPABuffer> &buffers) override;

        void unmapBuffers(
        	const std::vector<uint32_t> &ids) override;

        void queueRequest(
        	const uint32_t frame,
        	const ControlList &controls) override;

        void fillParamsBuffer(
        	const uint32_t frame,
        	const uint32_t bufferId) override;

	Signal<uint32_t, const Flags<ipa::vimc::TestFlag>> paramsBufferReady;


private:
	void recvMessage(const IPCMessage &data);


        int32_t initThread(
        	const IPASettings &settings,
        	const IPAOperationCode code,
        	const Flags<ipa::vimc::TestFlag> inFlags,
        	Flags<ipa::vimc::TestFlag> *outFlags);
        int32_t initIPC(
        	const IPASettings &settings,
        	const IPAOperationCode code,
        	const Flags<ipa::vimc::TestFlag> inFlags,
        	Flags<ipa::vimc::TestFlag> *outFlags);

        int32_t configureThread(
        	const IPACameraSensorInfo &sensorInfo,
        	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
        	const std::map<uint32_t, libcamera::ControlInfoMap> &entityControls);
        int32_t configureIPC(
        	const IPACameraSensorInfo &sensorInfo,
        	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
        	const std::map<uint32_t, libcamera::ControlInfoMap> &entityControls);

        int32_t startThread();
        int32_t startIPC();

        void stopThread();
        void stopIPC();

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
        	const ControlList &controls);
        void queueRequestIPC(
        	const uint32_t frame,
        	const ControlList &controls);

        void fillParamsBufferThread(
        	const uint32_t frame,
        	const uint32_t bufferId);
        void fillParamsBufferIPC(
        	const uint32_t frame,
        	const uint32_t bufferId);


        void paramsBufferReadyThread(
        	const uint32_t bufferId,
        	const Flags<ipa::vimc::TestFlag> flags);
	void paramsBufferReadyIPC(
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

		void setIPA(IPAVimcInterface *ipa)
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
                	const ControlList &controls)
		{
			ipa_->queueRequest(frame, controls);
		}
		void fillParamsBuffer(
                	const uint32_t frame,
                	const uint32_t bufferId)
		{
			ipa_->fillParamsBuffer(frame, bufferId);
		}

	private:
		IPAVimcInterface *ipa_;
	};

	Thread thread_;
	ThreadProxy proxy_;
	std::unique_ptr<IPAVimcInterface> ipa_;

	const bool isolate_;

	std::unique_ptr<IPCPipeUnixSocket> ipc_;

	ControlSerializer controlSerializer_;


	uint32_t seq_;
};

} /* namespace vimc */

} /* namespace ipa */

} /* namespace libcamera */