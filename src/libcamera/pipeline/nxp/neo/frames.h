/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Intel IPU3 Frames helper
 *     src/libcamera/pipeline/ipu3/frame.h
 * Copyright (C) 2020, Google Inc.
 *
 * frames.h - NXP NEO ISP Frames helper
 * Copyright 2024 NXP
 */

#pragma once

#include <map>
#include <memory>
#include <queue>
#include <vector>

#include <libcamera/base/signal.h>

#include <libcamera/controls.h>

namespace libcamera {

class FrameBuffer;
class IPAProxy;
class PipelineHandler;
class Request;
class V4L2VideoDevice;
struct IPABuffer;

class NxpNeoFrames
{
public:
	struct Info {
		unsigned int id;
		Request *request;

		FrameBuffer *input0Buffer;
		FrameBuffer *input1Buffer;
		FrameBuffer *embeddedBuffer;
		FrameBuffer *paramsBuffer;
		FrameBuffer *statsBuffer;

		ControlList effectiveSensorControls;

		bool paramDequeued;
		bool metadataProcessed;
		bool isRawOnly;
		bool hasRawStreamBuffer;
	};

	NxpNeoFrames();

	void init(const std::vector<std::unique_ptr<FrameBuffer>> &input0Buffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &input1Buffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &embeddedBuffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &paramsBuffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &statsBuffers);
	void clear();

	Info *create(Request *request, bool rawOnly, FrameBuffer *rawStreamBuffer);
	void remove(Info *info);
	bool tryComplete(Info *info);

	Info *find(unsigned int id);
	Info *find(FrameBuffer *buffer);

	Signal<> bufferAvailable;

private:
	FrameBuffer *allocBuffer(std::queue<FrameBuffer *> *queue);

	std::queue<FrameBuffer *> availableInput0Buffers_;
	std::queue<FrameBuffer *> availableInput1Buffers_;
	std::queue<FrameBuffer *> availableEmbeddedBuffers_;
	std::queue<FrameBuffer *> availableParamsBuffers_;
	std::queue<FrameBuffer *> availableStatsBuffers_;

	std::map<unsigned int, std::unique_ptr<Info>> frameInfo_;

	bool hasInput1_ = false;
	bool hasEmbedded_ = false;
};

} /* namespace libcamera */
