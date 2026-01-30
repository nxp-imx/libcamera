/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * ISI device
 *
 * Based on Intel IPU3 CIO2
 *     src/libcamera/pipeline/ipu3/cio2.h
 * Copyright (C) 2019, Google Inc.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class FrameBuffer;
class MediaDevice;
class PixelFormat;

namespace nxpneo {

class ISIPipe
{
public:
	static constexpr unsigned int kUnchainedWidthMax = 2048;
	static constexpr unsigned int kChainedWidthMax = 4096;

	ISIPipe(unsigned int index);

	int init(const MediaDevice *media);

	int configure(V4L2SubdeviceFormat &sinkFormat,
		      V4L2DeviceFormat &videoFormat);

	const std::string &subdeviceName() const { return subdeviceName_; }
	const std::string &videoDeviceName() const { return videoDeviceName_; }

	unsigned int index() const { return index_; };

	static const std::vector<uint32_t> &bayerMbusCodes();
	static const std::vector<uint32_t> &metaMbusCodes();
	static const std::vector<uint32_t> &sinkMbusCodesProcessed();
	static const std::vector<PixelFormat> &pixelFormatsProcessed();
	static const V4L2PixelFormat mbusCodeToPixelFormatBypass(unsigned int code);

	std::unique_ptr<V4L2VideoDevice> capture_;

private:
	std::string logPrefix() const
	{
		return "Pipe[" + capture_->deviceNode() + "] ";
	}

	unsigned int index_;
	std::unique_ptr<V4L2Subdevice> pipe_;
	std::string subdeviceName_;
	std::string videoDeviceName_;
};

class ISIDevice
{
public:
	ISIDevice(std::shared_ptr<MediaDevice> media);

	ISIPipe *reservePipe(unsigned int width);
	void releasePipe(ISIPipe *pipe);

	static const std::string &driverName();
	static const std::string &crossbarSubdevName();

	unsigned int crossbarFirstSourcePad() const { return xbarSinkPads_; }
	unsigned int crossbarSourcePads() const { return pipeEntries_.size(); }

	std::shared_ptr<MediaDevice> media() const { return media_; }
	bool isValid() const { return valid_; }

private:
	struct PipeWrapper {
		PipeWrapper(unsigned int index)
			: pipe(index), free(true), chained(false) {}
		ISIPipe pipe;
		bool free;
		bool chained;
	};

	std::vector<PipeWrapper> pipeEntries_;
	std::unique_ptr<V4L2Subdevice> crossbar_;
	unsigned int xbarSinkPads_ = 0;

	std::shared_ptr<MediaDevice> media_;
	bool valid_ = false;
};

} /* namespace nxpneo */

} /* namespace libcamera */
