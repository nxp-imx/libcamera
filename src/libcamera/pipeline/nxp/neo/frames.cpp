/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Intel IPU3 Frames helper
 *     src/libcamera/pipeline/ipu3/frame.cpp
 * Copyright (C) 2020, Google Inc.
 *
 * frames.cpp - NXP NEO ISP Frames helper
 * Copyright 2024 NXP
 */

#include "frames.h"

#include <libcamera/framebuffer.h>
#include <libcamera/request.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpNeo)

NxpNeoFrames::NxpNeoFrames()
{
}

void NxpNeoFrames::init(const std::vector<std::unique_ptr<FrameBuffer>> &input0Buffers,
			const std::vector<std::unique_ptr<FrameBuffer>> &input1Buffers,
			const std::vector<std::unique_ptr<FrameBuffer>> &embeddedBuffers,
			const std::vector<std::unique_ptr<FrameBuffer>> &paramsBuffers,
			const std::vector<std::unique_ptr<FrameBuffer>> &statsBuffers)
{
	for (const std::unique_ptr<FrameBuffer> &buffer : input0Buffers)
		availableInput0Buffers_.push(buffer.get());

	hasInput1_ = !!input1Buffers.size();
	if (hasInput1_) {
		for (const std::unique_ptr<FrameBuffer> &buffer : input1Buffers)
			availableInput1Buffers_.push(buffer.get());
	}

	hasEmbedded_ = !!embeddedBuffers.size();
	if (hasEmbedded_) {
		for (const std::unique_ptr<FrameBuffer> &buffer : embeddedBuffers)
			availableEmbeddedBuffers_.push(buffer.get());
	}

	for (const std::unique_ptr<FrameBuffer> &buffer : paramsBuffers)
		availableParamsBuffers_.push(buffer.get());

	for (const std::unique_ptr<FrameBuffer> &buffer : statsBuffers)
		availableStatsBuffers_.push(buffer.get());

	frameInfo_.clear();
}

void NxpNeoFrames::clear()
{
	availableInput0Buffers_ = {};
	availableInput1Buffers_ = {};
	availableEmbeddedBuffers_ = {};
	availableParamsBuffers_ = {};
	availableStatsBuffers_ = {};
}

NxpNeoFrames::Info *NxpNeoFrames::create(Request *request, bool rawOnly,
					 FrameBuffer *rawStreamBuffer)
{
	unsigned int id = request->sequence();

	FrameBuffer *input0Buffer = nullptr;
	FrameBuffer *input1Buffer = nullptr;
	FrameBuffer *embeddedBuffer = nullptr;
	FrameBuffer *paramsBuffer = nullptr;
	FrameBuffer *statsBuffer = nullptr;

	if (availableInput0Buffers_.empty()) {
		LOG(NxpNeo, Warning) << "Input0 buffer underrun";
		return nullptr;
	}

	if (hasInput1_ && availableInput1Buffers_.empty()) {
		LOG(NxpNeo, Warning) << "Input1 buffer underrun";
		return nullptr;
	}

	if (hasEmbedded_ && availableEmbeddedBuffers_.empty()) {
		LOG(NxpNeo, Warning) << "Input1 buffer underrun";
		return nullptr;
	}

	if (availableParamsBuffers_.empty()) {
		LOG(NxpNeo, Warning) << "Parameters buffer underrun";
		return nullptr;
	}

	if (availableStatsBuffers_.empty()) {
		LOG(NxpNeo, Warning) << "Statistics buffer underrun";
		return nullptr;
	}

	/*
	 * ISI internal buffers allocation
	 * Input0 buffer may come from application when raw stream is enabled
	 */
	if (!rawStreamBuffer)
		input0Buffer = allocBuffer(&availableInput0Buffers_);
	else
		input0Buffer = rawStreamBuffer;

	if (hasInput1_)
		input1Buffer = allocBuffer(&availableInput1Buffers_);

	if (hasEmbedded_)
		embeddedBuffer = allocBuffer(&availableEmbeddedBuffers_);

	/* ISP internal buffers allocation */
	paramsBuffer = allocBuffer(&availableParamsBuffers_);
	statsBuffer = allocBuffer(&availableStatsBuffers_);

	/* \todo Remove the dynamic allocation of Info */
	std::unique_ptr<Info> info = std::make_unique<Info>();

	info->id = id;
	info->request = request;
	info->input0Buffer = input0Buffer;
	info->input1Buffer = input1Buffer;
	info->embeddedBuffer = embeddedBuffer;
	info->paramsBuffer = paramsBuffer;
	info->statsBuffer = statsBuffer;

	info->isRawOnly = rawOnly;
	info->hasRawStreamBuffer = !!rawStreamBuffer;

	/* ISP and IPA are bypassed in raw-only */
	bool doneStatus = rawOnly ? true : false;
	info->paramDequeued = doneStatus;
	info->metadataProcessed = doneStatus;

	frameInfo_[id] = std::move(info);

	return frameInfo_[id].get();
}

void NxpNeoFrames::remove(NxpNeoFrames::Info *info)
{
	/* Return internal buffers for reuse. */
	if (!info->hasRawStreamBuffer)
		availableInput0Buffers_.push(info->input0Buffer);
	if (hasInput1_)
		availableInput1Buffers_.push(info->input1Buffer);
	if (hasEmbedded_)
		availableEmbeddedBuffers_.push(info->embeddedBuffer);
	availableParamsBuffers_.push(info->paramsBuffer);
	availableStatsBuffers_.push(info->statsBuffer);

	/* Delete the extended frame information. */
	frameInfo_.erase(info->id);
}

bool NxpNeoFrames::tryComplete(NxpNeoFrames::Info *info)
{
	Request *request = info->request;

	if (request->hasPendingBuffers())
		return false;

	if (!info->metadataProcessed)
		return false;

	if (!info->paramDequeued)
		return false;

	remove(info);

	bufferAvailable.emit();

	return true;
}

NxpNeoFrames::Info *NxpNeoFrames::find(unsigned int id)
{
	const auto &itInfo = frameInfo_.find(id);

	if (itInfo != frameInfo_.end())
		return itInfo->second.get();

	LOG(NxpNeo, Info) << "Can't find tracking information for frame " << id;

	return nullptr;
}

NxpNeoFrames::Info *NxpNeoFrames::find(FrameBuffer *buffer)
{
	for (auto const &itInfo : frameInfo_) {
		Info *info = itInfo.second.get();

		for (auto const itBuffers : info->request->buffers())
			if (itBuffers.second == buffer)
				return info;

		if (info->input0Buffer == buffer || info->input1Buffer == buffer ||
		    info->embeddedBuffer == buffer ||
		    info->paramsBuffer == buffer || info->statsBuffer == buffer)
			return info;
	}

	LOG(NxpNeo, Info) << "Can't find tracking information from buffer";

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
