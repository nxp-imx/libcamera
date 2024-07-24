/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Intel IPU3 CIO2
 *     src/libcamera/pipeline/ipu3/cio2.h
 * Copyright (C) 2019, Google Inc.
 *
 * Copyright 2024 NXP
 * isi_device.h - NXP ISI
 */

#pragma once

#include <memory>
#include <queue>
#include <vector>

#include <libcamera/base/signal.h>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class CameraSensor;
class FrameBuffer;
class MediaDevice;
class PixelFormat;
class Request;
class Size;
class SizeRange;
struct StreamConfiguration;

class ISIPipe
{
public:
	enum {
		kStateIdle,
		kStateConfigured,
		kStateActive,
	};

	ISIPipe(unsigned int index)
		: index_(index), state_(kStateIdle) {}

	int exportBuffers(unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	int configure(const V4L2SubdeviceFormat &sinkFormat,
		      V4L2DeviceFormat *sourceFormat);

	int start();
	int stop();

	Signal<FrameBuffer *> &bufferReady() { return output_->bufferReady; }

	std::string logPrefix() const
	{
		return "Pipe[" + std::to_string(index_) + "] ";
	}

	unsigned int index() const { return index_; }
	int allocateBuffers(unsigned int bufferCount);
	std::vector<std::unique_ptr<FrameBuffer>> &buffers() { return buffers_; }
	void freeBuffers();

	std::unique_ptr<V4L2VideoDevice> output_;

private:
	friend class ISIDevice;

	int init(const MediaDevice *media);

	static const std::map<uint32_t, V4L2PixelFormat> &mediaBusToPixelFormats();

	void setState(unsigned int state) { state_ = state; }
	unsigned getState() const { return state_; }
	bool stateIdle() const { return state_ == kStateIdle; }
	bool stateConfigured() const { return state_ == kStateConfigured; }
	bool stateActive() const { return state_ == kStateActive; }

	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
	std::unique_ptr<V4L2Subdevice> pipe_;

	unsigned int index_;
	unsigned int state_;
};

class ISIDevice
{
public:
	ISIDevice() {}

	static constexpr unsigned int kPipesMax = 16;
	static constexpr unsigned int kUnchainedWidthMax = 2048;
	static constexpr unsigned int kChainedWidthMax = 4096;

	int init(const MediaDevice *media);

	int reservePipeBySize(Size &sizeMax, unsigned int *index);
	int reservePipeByIndex(Size &sizeMax, unsigned int index);
	void releasePipe(unsigned int index);
	ISIPipe *getPipeByIndex(unsigned int index);

	static std::string kDriverName() { return "mxc-isi"; }
	static std::string kSDevCrossBarEntityName() { return "crossbar"; }

	static std::string kSDevPipeEntityName(unsigned int i)
	{
		return "mxc_isi." + std::to_string(i);
	}

	static std::string kVDevPipeEntityName(unsigned int i)
	{
		return "mxc_isi." + std::to_string(i) + ".capture";
	}

	static const std::map<uint32_t, V4L2PixelFormat> &mediaBusToPixelFormats()
	{
		return ISIPipe::mediaBusToPixelFormats();
	}

	V4L2Subdevice *crossbar() const { return crossbar_.get(); }
	unsigned int crossbarFirstSourcePad() const { return xbarSinkPads_; }
	unsigned int crossbarSourcePads() const { return pipeEntries_.size(); }
	const MediaDevice *media() const { return media_; }

private:
	struct PipeWrapper {
		PipeWrapper(unsigned int index)
			: pipe_(index), free(true), chained(false){};
		ISIPipe pipe_;
		bool free;
		bool chained;
	};

	std::vector<PipeWrapper> pipeEntries_;
	std::unique_ptr<V4L2Subdevice> crossbar_;
	unsigned int xbarSinkPads_ = 0;
	const MediaDevice *media_ = nullptr;
};

} /* namespace libcamera */
