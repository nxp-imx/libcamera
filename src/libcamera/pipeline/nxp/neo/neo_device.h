/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Neo ISP device
 *
 * Based on Intel IPU3 ImgU
 *     src/libcamera/pipeline/ipu3/imgu.h
 * Copyright (C) 2019, Google Inc.
 */

#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <linux/nxp_neoisp.h>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

/* All imx95x blocks are supported by default */
constexpr unsigned long long kDefaultParamsSupported =
	_BITULL(NEOISP_PARAM_BLK_PIPE_CONF) |
	_BITULL(NEOISP_PARAM_BLK_HEAD_COLOR) |
	_BITULL(NEOISP_PARAM_BLK_HDR_DECOMPRESS0) |
	_BITULL(NEOISP_PARAM_BLK_HDR_DECOMPRESS1) |
	_BITULL(NEOISP_PARAM_BLK_OBWB0) |
	_BITULL(NEOISP_PARAM_BLK_OBWB1) |
	_BITULL(NEOISP_PARAM_BLK_OBWB2) |
	_BITULL(NEOISP_PARAM_BLK_HDR_MERGE) |
	_BITULL(NEOISP_PARAM_BLK_RGBIR) |
	_BITULL(NEOISP_PARAM_BLK_STAT) |
	_BITULL(NEOISP_PARAM_BLK_CTEMP) |
	_BITULL(NEOISP_PARAM_BLK_IR_COMPRESS) |
	_BITULL(NEOISP_PARAM_BLK_BNR) |
	_BITULL(NEOISP_PARAM_BLK_VIGNETTING_CTRL) |
	_BITULL(NEOISP_PARAM_BLK_DEMOSAIC) |
	_BITULL(NEOISP_PARAM_BLK_RGB2YUV) |
	_BITULL(NEOISP_PARAM_BLK_DR_COMP) |
	_BITULL(NEOISP_PARAM_BLK_NR) |
	_BITULL(NEOISP_PARAM_BLK_AF) |
	_BITULL(NEOISP_PARAM_BLK_EE) |
	_BITULL(NEOISP_PARAM_BLK_DF) |
	_BITULL(NEOISP_PARAM_BLK_CONVMED) |
	_BITULL(NEOISP_PARAM_BLK_CAS) |
	_BITULL(NEOISP_PARAM_BLK_GCM) |
	_BITULL(NEOISP_PARAM_BLK_VIGNETTING_TABLE) |
	_BITULL(NEOISP_PARAM_BLK_DRC_GLOBAL_TONEMAP) |
	_BITULL(NEOISP_PARAM_BLK_DRC_LOCAL_TONEMAP);

class DeviceEnumerator;
class FrameBuffer;
class MediaDevice;
class MediaEntity;
class PipelineHandler;

namespace nxpneo {

class NeoDevice
{
public:
	static constexpr unsigned int kRawWidthMax = 4096;
	static constexpr unsigned int kWidthAlignment = 16;

	struct PipeConfig {
		unsigned int topLines;
	};

	enum class VideoDevice {
		Input0,
		Input1,
		Params,
		Frame,
		Ir,
		Stats,
	};

	NeoDevice(std::shared_ptr<MediaDevice> media, MediaEntity *subdevEntity);
	~NeoDevice() = default;

	int allocateBuffers(unsigned int bufferCount);
	void freeBuffers();

	int configure(const PipeConfig &pipeConfig,
		      const std::map<VideoDevice, V4L2DeviceFormat *> &formats);

	int start();
	int stop();

	V4L2VideoDevice *videoDevice(VideoDevice device) const;
	const std::string &subdeviceName() const;
	const std::string &videoDeviceName(VideoDevice device) const;

	const std::vector<PixelFormat> &capturePixelFormats(VideoDevice device) const;
	static const std::vector<V4L2PixelFormat> &outputFormats(VideoDevice device);

	std::shared_ptr<MediaDevice> media() const { return media_; }
	uint32_t hwCapabilities() const { return hwCapabilities_; }
	uint64_t supportedParamsBlocks() const { return supportedParamsBlocks_; }

	bool isValid() const { return valid_; }

	std::unique_ptr<V4L2Subdevice> isp_;
	std::unique_ptr<V4L2VideoDevice> input0_;
	std::unique_ptr<V4L2VideoDevice> input1_;
	std::unique_ptr<V4L2VideoDevice> params_;
	std::unique_ptr<V4L2VideoDevice> frame_;
	std::unique_ptr<V4L2VideoDevice> ir_;
	std::unique_ptr<V4L2VideoDevice> stats_;

	std::vector<std::unique_ptr<FrameBuffer>> paramsBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> statsBuffers_;

private:
	int configureVideoDeviceLink(VideoDevice device, bool enable);
	int configureVideoDevice(VideoDevice device, V4L2DeviceFormat *format);
	int configureVideoDeviceMeta(VideoDevice device);

	std::string logPrefix() const
	{
		return "Neo[" + isp_->deviceNode() + "] ";
	}

	std::map<VideoDevice, MediaEntity *> vdevEntities_;
	MediaEntity *sdevEntity_;
	std::vector<VideoDevice> configured_;

	uint32_t hwCapabilities_;
	uint64_t supportedParamsBlocks_;

	std::shared_ptr<MediaDevice> media_;
	bool valid_;
};

class NeoMediaDevice
{
public:
	NeoMediaDevice(std::shared_ptr<MediaDevice> media);
	~NeoMediaDevice() = default;

	std::unique_ptr<NeoDevice> createDevice();
	std::shared_ptr<MediaDevice> media() const { return media_; }
	bool isValid() const { return valid_; }

private:
	std::shared_ptr<MediaDevice> media_;
	std::deque<MediaEntity *> subdevs_;
	bool valid_ = false;
};

class NeoDeviceAllocator
{
public:
	NeoDeviceAllocator(PipelineHandler *pipeline, DeviceEnumerator *enumerator);
	std::unique_ptr<NeoDevice> createDevice();
	const std::shared_ptr<MediaDevice> feMedia() { return feMedia_; }
	bool isValid() const { return valid_; }

private:
	PipelineHandler *pipeline_;
	DeviceEnumerator *enumerator_;
	std::shared_ptr<MediaDevice> feMedia_;
	std::unique_ptr<NeoMediaDevice> feNeoMediaDevice_;
	std::vector<std::shared_ptr<MediaDevice>> legacyMedias_;
	bool valid_ = false;
};

} /* namespace nxpneo */

} /* namespace libcamera */
