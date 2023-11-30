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

void NxpNeoFrames::init(const std::vector<std::unique_ptr<FrameBuffer>> &paramsBuffers,
			const std::vector<std::unique_ptr<FrameBuffer>> &statsBuffers)
{
	for (const std::unique_ptr<FrameBuffer> &buffer : paramsBuffers)
		availableParamsBuffers_.push(buffer.get());

	for (const std::unique_ptr<FrameBuffer> &buffer : statsBuffers)
		availableStatsBuffers_.push(buffer.get());

	frameInfo_.clear();
}

void NxpNeoFrames::clear()
{
	availableParamsBuffers_ = {};
	availableStatsBuffers_ = {};
}

NxpNeoFrames::Info *NxpNeoFrames::create(Request *request)
{
	unsigned int id = request->sequence();

	if (availableParamsBuffers_.empty()) {
		LOG(NxpNeo, Debug) << "Parameters buffer underrun";
		return nullptr;
	}

	if (availableStatsBuffers_.empty()) {
		LOG(NxpNeo, Debug) << "Statistics buffer underrun";
		return nullptr;
	}

	FrameBuffer *paramsBuffer = availableParamsBuffers_.front();
	FrameBuffer *statsBuffer = availableStatsBuffers_.front();

	paramsBuffer->_d()->setRequest(request);
	statsBuffer->_d()->setRequest(request);

	availableParamsBuffers_.pop();
	availableStatsBuffers_.pop();

	/* \todo Remove the dynamic allocation of Info */
	std::unique_ptr<Info> info = std::make_unique<Info>();

	info->id = id;
	info->request = request;
	info->rawBuffer = nullptr;
	info->vsBuffer = nullptr;
	info->edBuffer = nullptr;
	info->paramsBuffer = paramsBuffer;
	info->statsBuffer = statsBuffer;
	info->paramDequeued = false;
	info->metadataProcessed = false;

	frameInfo_[id] = std::move(info);

	return frameInfo_[id].get();
}

void NxpNeoFrames::remove(NxpNeoFrames::Info *info)
{
	/* Return params and stat buffer for reuse. */
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

	LOG(NxpNeo, Fatal) << "Can't find tracking information for frame " << id;

	return nullptr;
}

NxpNeoFrames::Info *NxpNeoFrames::find(FrameBuffer *buffer)
{
	for (auto const &itInfo : frameInfo_) {
		Info *info = itInfo.second.get();

		for (auto const itBuffers : info->request->buffers())
			if (itBuffers.second == buffer)
				return info;

		if (info->rawBuffer == buffer || info->paramsBuffer == buffer ||
		    info->statsBuffer == buffer ||
		    info->vsBuffer == buffer || info->edBuffer == buffer)
			return info;
	}

	LOG(NxpNeo, Fatal) << "Can't find tracking information from buffer";

	return nullptr;
}

} /* namespace libcamera */
