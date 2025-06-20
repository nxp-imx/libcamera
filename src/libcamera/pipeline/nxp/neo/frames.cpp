/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Intel IPU3 Frames helper
 *     src/libcamera/pipeline/ipu3/frame.cpp
 * Copyright (C) 2020, Google Inc.
 *
 * frames.cpp - NXP NEO ISP Frames helper
 * Copyright 2024-2025 NXP
 */

#include "frames.h"

#include <libcamera/framebuffer.h>
#include <libcamera/request.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpNeoPipe)

NxpNeoFrames::NxpNeoFrames()
{
}

void NxpNeoFrames::init(const std::vector<std::unique_ptr<FrameBuffer>> &image0Buffers,
			const std::vector<std::unique_ptr<FrameBuffer>> &image1Buffers,
			const std::vector<std::unique_ptr<FrameBuffer>> &eDataBuffers,
			const std::vector<std::unique_ptr<FrameBuffer>> &paramsBuffers,
			const std::vector<std::unique_ptr<FrameBuffer>> &statsBuffers,
			bool alternatedRawStreams)
{
	for (const std::unique_ptr<FrameBuffer> &buffer : image0Buffers)
		availableImage0Buffers_.push(buffer.get());

	hasImage1_ = !!image1Buffers.size();
	if (hasImage1_) {
		for (const std::unique_ptr<FrameBuffer> &buffer : image1Buffers)
			availableImage1Buffers_.push(buffer.get());
	}

	hasEmbeddedData_ = !!eDataBuffers.size();
	if (hasEmbeddedData_) {
		for (const std::unique_ptr<FrameBuffer> &buffer : eDataBuffers)
			availableEmbeddedDataBuffers_.push(buffer.get());
	}

	for (const std::unique_ptr<FrameBuffer> &buffer : paramsBuffers)
		availableParamsBuffers_.push(buffer.get());

	for (const std::unique_ptr<FrameBuffer> &buffer : statsBuffers)
		availableStatsBuffers_.push(buffer.get());

	frameInfo_.clear();

	alternatedRawStreams_ = hasImage1_ && alternatedRawStreams;
}

int NxpNeoFrames::destroy(unsigned int id)
{
	Info *info = find(id);
	if (!info)
		return -ENOENT;

	/* Return internal buffers for reuse. */
	if (info->image0Buffer != info->rawStreamBuffer)
		availableImage0Buffers_.push(info->image0Buffer);
	if (info->image1Buffer && info->image1Buffer != info->rawStreamBuffer)
		availableImage1Buffers_.push(info->image1Buffer);
	if (info->eDataBuffer)
		availableEmbeddedDataBuffers_.push(info->eDataBuffer);
	availableParamsBuffers_.push(info->paramsBuffer);
	availableStatsBuffers_.push(info->statsBuffer);

	/* Delete the extended frame information. */
	frameInfo_.erase(info->id);

	bufferAvailable.emit();

	return 0;
}

void NxpNeoFrames::clear()
{
	availableImage0Buffers_ = {};
	availableImage1Buffers_ = {};
	availableEmbeddedDataBuffers_ = {};
	availableParamsBuffers_ = {};
	availableStatsBuffers_ = {};
}

NxpNeoFrames::Info *NxpNeoFrames::create(Request *request, bool rawOnly,
					 FrameBuffer *rawStreamBuffer)
{
	unsigned int id = request->sequence();

	FrameBuffer *image0Buffer = nullptr;
	FrameBuffer *image1Buffer = nullptr;
	FrameBuffer *eDataBuffer = nullptr;
	FrameBuffer *paramsBuffer = nullptr;
	FrameBuffer *statsBuffer = nullptr;

	if (availableImage0Buffers_.empty()) {
		LOG(NxpNeoPipe, Warning) << "Image0 buffer underrun";
		return nullptr;
	}

	if (hasImage1_ && availableImage1Buffers_.empty()) {
		LOG(NxpNeoPipe, Warning) << "Image1 buffer underrun";
		return nullptr;
	}

	if (hasEmbeddedData_ && availableEmbeddedDataBuffers_.empty()) {
		LOG(NxpNeoPipe, Warning) << "Embedded buffer underrun";
		return nullptr;
	}

	if (availableParamsBuffers_.empty()) {
		LOG(NxpNeoPipe, Warning) << "Parameters buffer underrun";
		return nullptr;
	}

	if (availableStatsBuffers_.empty()) {
		LOG(NxpNeoPipe, Warning) << "Statistics buffer underrun";
		return nullptr;
	}

	if (rawStreamBuffer) {
		/*
		 * To map the raw stream alternately to both input streams, rely
		 * on the request sequence number.
		 */
		bool evenId = (id % 2 == 0);
		if (!alternatedRawStreams_ || evenId)
			image0Buffer = rawStreamBuffer;
		else
			image1Buffer = rawStreamBuffer;
	}
	if (!image0Buffer)
		image0Buffer = allocBuffer(&availableImage0Buffers_);
	if (hasImage1_ && !image1Buffer)
		image1Buffer = allocBuffer(&availableImage1Buffers_);
	if (hasEmbeddedData_)
		eDataBuffer = allocBuffer(&availableEmbeddedDataBuffers_);

	/* ISP internal buffers allocation */
	paramsBuffer = allocBuffer(&availableParamsBuffers_);
	statsBuffer = allocBuffer(&availableStatsBuffers_);

	/* \todo Remove the dynamic allocation of Info */
	std::unique_ptr<Info> info = std::make_unique<Info>();

	info->id = id;
	info->request = request;
	info->image0Buffer = image0Buffer;
	info->image1Buffer = image1Buffer;
	info->eDataBuffer = eDataBuffer;
	info->paramsBuffer = paramsBuffer;
	info->statsBuffer = statsBuffer;

	info->image0Pending = true;
	info->image1Pending = !!info->image1Buffer;
	info->eDataPending = !!info->eDataBuffer;

	info->isRawOnly = rawOnly;
	info->rawStreamBuffer = rawStreamBuffer;

	/* ISP and IPA are bypassed in raw-only */
	bool doneStatus = rawOnly ? true : false;
	info->paramDequeued = doneStatus;
	info->metadataProcessed = doneStatus;

	frameInfo_[id] = std::move(info);

	return frameInfo_[id].get();
}

NxpNeoFrames::Info *NxpNeoFrames::find(unsigned int id)
{
	const auto &itInfo = frameInfo_.find(id);

	if (itInfo != frameInfo_.end())
		return itInfo->second.get();

	LOG(NxpNeoPipe, Debug) << "Can't find tracking information for frame " << id;

	return nullptr;
}

NxpNeoFrames::Info *NxpNeoFrames::find(FrameBuffer *buffer)
{
	for (auto const &itInfo : frameInfo_) {
		Info *info = itInfo.second.get();

		for (auto const itBuffers : info->request->buffers())
			if (itBuffers.second == buffer)
				return info;

		if (info->image0Buffer == buffer || info->image1Buffer == buffer ||
		    info->eDataBuffer == buffer ||
		    info->paramsBuffer == buffer || info->statsBuffer == buffer)
			return info;
	}

	LOG(NxpNeoPipe, Debug) << "Can't find tracking information from buffer";

	return nullptr;
}

NxpNeoFrames::Info *NxpNeoFrames::find(Request *request)
{
	for (auto const &itInfo : frameInfo_) {
		Info *info = itInfo.second.get();
		if (info->request == request)
			return info;
	}

	LOG(NxpNeoPipe, Debug) << "Can't find tracking information from request";

	return nullptr;
}

FrameBuffer *NxpNeoFrames::allocBuffer(std::queue<FrameBuffer *> *queue)
{
	ASSERT(!queue->empty());
	FrameBuffer *buffer = queue->front();
	queue->pop();
	return buffer;
}

} /* namespace libcamera */
