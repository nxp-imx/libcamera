/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Intel IPU3 ImgU
 *     src/libcamera/pipeline/ipu3/imgu.h
 * Copyright (C) 2019, Google Inc.
 *
 * Copyright 2024 NXP
 * neo_device.h - NXP NEO
 */

#pragma once

#include <memory>
#include <string>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class FrameBuffer;
class MediaDevice;
class Size;
struct StreamConfiguration;

class NeoDevice
{
public:
	NeoDevice(unsigned int index)
		: index_(index) {}

	int init(MediaDevice *media);

	int allocateBuffers(unsigned int bufferCount);
	void freeBuffers();

	int configure(V4L2DeviceFormat &formatInput0,
		      V4L2DeviceFormat &formatInput1,
		      V4L2DeviceFormat &formatFrame,
		      V4L2DeviceFormat &formatIr);

	int start();
	int stop();

	int enableLinks(bool enable);

	std::unique_ptr<V4L2Subdevice> isp_;
	std::unique_ptr<V4L2VideoDevice> input0_;
	std::unique_ptr<V4L2VideoDevice> input1_;
	std::unique_ptr<V4L2VideoDevice> params_;
	std::unique_ptr<V4L2VideoDevice> frame_;
	std::unique_ptr<V4L2VideoDevice> ir_;
	std::unique_ptr<V4L2VideoDevice> stats_;

	std::vector<std::unique_ptr<FrameBuffer>> paramsBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> statsBuffers_;

	static std::string kDriverName()
	{
		return "neoisp";
	}

	static std::string kSDevNeoEntityName()
	{
		return "neoisp";
	}

	static std::string kVDevInput0EntityName()
	{
		return "neoisp-input0";
	}

	static std::string kVDevInput1EntityName()
	{
		return "neoisp-input1";
	}

	static std::string kVDevEntityParamsName()
	{
		return "neoisp-params";
	}

	static std::string kVDevEntityFrameName()
	{
		return "neoisp-frame";
	}

	static std::string kVDevEntityIrName()
	{
		return "neoisp-ir";
	}

	static std::string kVDevEntityStatsName()
	{
		return "neoisp-stats";
	}

	std::string logPrefix() const
	{
		return "Neo[" + std::to_string(index_) + "] ";
	}

	static const std::vector<V4L2PixelFormat> &frameFormats();
	static const std::vector<V4L2PixelFormat> &irFormats();
	static const std::vector<V4L2PixelFormat> &input0Formats();
	static const std::vector<V4L2PixelFormat> &input1Formats();

	const MediaDevice *media() const
	{
		return media_;
	}

private:
	enum {
		PAD_INPUT0 = 0,
		PAD_INPUT1 = 1,
		PAD_PARAMS = 2,
		PAD_FRAME = 3,
		PAD_IR = 4,
		PAD_STATS = 5,
	};

	int linkSetup(const std::string &source, unsigned int sourcePad,
		      const std::string &sink, unsigned int sinkPad,
		      bool enable);
	int configureVideoDevice(V4L2VideoDevice *dev, unsigned int pad,
				 V4L2DeviceFormat &format);
	int configureVideoDeviceMeta(V4L2VideoDevice *dev,
				     unsigned int pad, uint32_t fourcc);

	bool padActiveInput1() const
	{
		return configInput1_;
	}

	bool padActiveFrame() const
	{
		return configFrame_;
	}

	bool padActiveIr() const
	{
		return configIr_;
	}

	unsigned int index_;
	MediaDevice *media_ = nullptr;

	bool configInput1_ = false;
	bool configFrame_ = false;
	bool configIr_ = false;
};

} /* namespace libcamera */
