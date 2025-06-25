/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Intel IPU3 Frames helper
 *     src/libcamera/pipeline/ipu3/frame.h
 * Copyright (C) 2020, Google Inc.
 *
 * frames.h - NXP NEO ISP Frames helper
 * Copyright 2024-2025 NXP
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

		FrameBuffer *image0Buffer;
		FrameBuffer *image1Buffer;
		FrameBuffer *eDataBuffer;
		FrameBuffer *paramsBuffer;
		FrameBuffer *statsBuffer;

		FrameBuffer *rawStreamBuffer;

		bool image0Pending;
		bool image1Pending;
		bool eDataPending;

		bool paramDequeued;
		bool metadataProcessed;
	};

	NxpNeoFrames();

	void init(const std::vector<std::unique_ptr<FrameBuffer>> &image0Buffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &image1Buffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &eDataBuffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &paramsBuffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &statsBuffers,
		  bool rawStreamOnly,
		  bool alternatedRawStreams);

	int destroy(unsigned int id);
	void clear();

	Info *create(Request *request, FrameBuffer *rawStreamBuffer);

	Info *find(unsigned int id);
	Info *find(FrameBuffer *buffer);
	Info *find(Request *request);

	Signal<> bufferAvailable;

private:
	FrameBuffer *allocBuffer(std::queue<FrameBuffer *> *queue);

	std::queue<FrameBuffer *> availableImage0Buffers_;
	std::queue<FrameBuffer *> availableImage1Buffers_;
	std::queue<FrameBuffer *> availableEmbeddedDataBuffers_;
	std::queue<FrameBuffer *> availableParamsBuffers_;
	std::queue<FrameBuffer *> availableStatsBuffers_;

	std::map<unsigned int, std::unique_ptr<Info>> frameInfo_;

	bool hasImage1_ = false;
	bool hasEmbeddedData_ = false;
	bool rawStreamOnly_ = false;
	bool alternatedRawStreams_ = false;
};

} /* namespace libcamera */
