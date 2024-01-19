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

		FrameBuffer *rawBuffer;
		FrameBuffer *paramsBuffer;
		FrameBuffer *statsBuffer;
		FrameBuffer *input1Buffer;
		FrameBuffer *edBuffer;

		ControlList effectiveSensorControls;

		bool paramDequeued;
		bool metadataProcessed;
		bool isRawOnly;
	};

	NxpNeoFrames();

	void init(const std::vector<std::unique_ptr<FrameBuffer>> &paramsBuffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &statsBuffers);
	void clear();

	Info *create(Request *request, bool rawOnly);
	void remove(Info *info);
	bool tryComplete(Info *info);

	Info *find(unsigned int id);
	Info *find(FrameBuffer *buffer);

	Signal<> bufferAvailable;

private:
	std::queue<FrameBuffer *> availableParamsBuffers_;
	std::queue<FrameBuffer *> availableStatsBuffers_;

	std::map<unsigned int, std::unique_ptr<Info>> frameInfo_;
};

} /* namespace libcamera */
