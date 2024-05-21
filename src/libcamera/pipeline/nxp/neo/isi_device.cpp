/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Intel IPU3 CIO2
 *     src/libcamera/pipeline/ipu3/cio2.cpp
 * Copyright (C) 2019, Google Inc.
 *
 * Copyright 2024 NXP
 * isi_device.cpp - NXP ISI
 */

#include <limits>
#include <math.h>

#include <linux/media-bus-format.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>
#include <libcamera/transform.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

#include "isi_device.h"


namespace libcamera {

LOG_DEFINE_CATEGORY(IsiDev)

/*
 * -------------------------------- ISIPipe --------------------------------
 */

/**
 * \brief Initialize components of the ISI pipe
 * \param[in] media The ISI media device
 *
 * Create and open the video device and subdevice of the ISI pipe channel.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::init(const MediaDevice *media)
{
	int ret;

	std::string subDevEntityName = ISIDevice::kSDevPipeEntityName(index_);
	pipe_ = V4L2Subdevice::fromEntityName(media, subDevEntityName);
	if (!pipe_)
		return -ENODEV;

	ret = pipe_->open();
	if (ret)
		LOG(IsiDev, Debug) << logPrefix() << "failed to open subdev";

	std::string videoDevEntityName = ISIDevice::kVDevPipeEntityName(index_);
	output_ = V4L2VideoDevice::fromEntityName(media, videoDevEntityName);
	if (!output_)
		return -ENODEV;

	ret = output_->open();
	if (ret)
		LOG(IsiDev, Debug) << logPrefix() << "failed to open videodev";

	return ret;
}

/**
 * \brief Create and export \a count buffers from ISI channel capture video device
 * \param[in] count The number of buffers to export
 * \param[out] buffers Vector of allocated buffers
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::exportBuffers(unsigned int count,
			   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	if (!stateConfigured()) {
		LOG(IsiDev, Error)
			<< logPrefix()
			<< "Export buffer while not configured "
			<< "(" << getState() << ")";
	}

	return output_->exportBuffers(count, buffers);
}

/**
 * \brief Start the ISI channel capture video device
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::start()
{
	int ret;

	if (!stateConfigured()) {
		LOG(IsiDev, Error)
			<< logPrefix()
			<< "Starting while not in configured state"
			<< "(" << getState() << ")";
		return -EAGAIN;
	}

	ret = output_->streamOn();
	if (ret)
		return ret;

	setState(kStateActive);

	return 0;
}

/**
 * \brief Stop the ISI channel capture video device
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::stop()
{
	if (!stateActive()) {
		LOG(IsiDev, Error)
			<< logPrefix()
			<< "Stopping while not active "
			<< "(" << getState() << ")";
		return -EAGAIN;
	}

	int ret = output_->streamOff();
	setState(kStateConfigured);

	return ret;
}

/**
 * \brief Configure the ISI channel with \a sinkFormat subdevice format
 * \param[in] sinkFormat Subdevice format fed to the ISI channel
 * \param[out] sourceFormat Corresponding device format configured in capture
 * device node.
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::configure(const V4L2SubdeviceFormat &sinkFormat,
		       V4L2DeviceFormat *sourceFormat)
{
	int ret;

	if (!(stateIdle() || stateConfigured())) {
		LOG(IsiDev, Error)
			<< logPrefix()
			<< "Can't be configured in state " << getState();
		return -ENODEV;
	}

	*sourceFormat = {};
	uint32_t code = sinkFormat.code;

	const std::map<uint32_t, V4L2PixelFormat> &formats = mediaBusToPixelFormats();
	if (!formats.count(code)) {
		LOG(IsiDev, Error)
			<< logPrefix()
			<< "mbus code " << code << " not supported by ISI ";
		return -ENOTSUP;
	}

	sourceFormat->fourcc = formats.at(code);
	sourceFormat->size = sinkFormat.size;

	/* \todo Set stride and format. */
	ret = output_->setFormat(sourceFormat);
	if (ret) {
		LOG(IsiDev, Error)
			<< logPrefix()
			<< "Failed to configure video device";
		return ret;
	}

	setState(kStateConfigured);

	LOG(IsiDev, Debug)
		<< logPrefix() << " Video device configured "
		<< " dev fmt " << sourceFormat->toString();

	return 0;
}

/**
 * \brief Allocate buffers for ISI channel
 * \param[in] bufferCount The number of buffers to allocate
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::allocateBuffers(unsigned int bufferCount)
{
	if (!stateConfigured()) {
		LOG(IsiDev, Error)
			<< logPrefix()
			<< "Can't allocate buffers in state " << getState();
		return -ENODEV;
	}

	int ret = output_->exportBuffers(bufferCount, &buffers_);
	if (ret < 0) {
		LOG(IsiDev, Error) << logPrefix() << "failed to export buffers";
		return ret;
	}

	ret = output_->importBuffers(bufferCount);
	if (ret < 0) {
		LOG(IsiDev, Error) << logPrefix() << "failed to import buffers";
		freeBuffers();
		return ret;
	}

	return 0;
}

/**
 * \brief Release the pool of preallocated buffers created by allocateBuffers()
 */
void ISIPipe::freeBuffers()
{
	if (!stateConfigured()) {
		LOG(IsiDev, Error)
			<< logPrefix()
			<< "Can't deallocate buffers in state " << getState();
		return;
	}

	buffers_.clear();

	if (output_->releaseBuffers())
		LOG(IsiDev, Error) << logPrefix() << "failed to free buffers";
}

/**
 * \brief Return the table of raw Bayer formats supported by ISI channels
 * \return A map of media bus code and corresponding capture device pixel
 *  formats
 */
const std::map<uint32_t, V4L2PixelFormat> &ISIPipe::mediaBusToPixelFormats()
{
	/*
	 * ISI channels are operated in bypass mode.
	 * Table defines pixel formats of the channel output device node
	 * depending on the media bus format of the incoming stream.
	 */
	static const std::map<uint32_t, V4L2PixelFormat> formats = {

		/* Bayer formats (frame output) */
		{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8) },
		{ MEDIA_BUS_FMT_SGBRG8_1X8, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG8) },
		{ MEDIA_BUS_FMT_SGRBG8_1X8, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG8) },
		{ MEDIA_BUS_FMT_SRGGB8_1X8, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB8) },
		{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10) },
		{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10) },
		{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10) },
		{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10) },
		{ MEDIA_BUS_FMT_SBGGR12_1X12, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12) },
		{ MEDIA_BUS_FMT_SGBRG12_1X12, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12) },
		{ MEDIA_BUS_FMT_SGRBG12_1X12, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12) },
		{ MEDIA_BUS_FMT_SRGGB12_1X12, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12) },
		{ MEDIA_BUS_FMT_SBGGR14_1X14, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR14) },
		{ MEDIA_BUS_FMT_SGBRG14_1X14, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG14) },
		{ MEDIA_BUS_FMT_SGRBG14_1X14, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG14) },
		{ MEDIA_BUS_FMT_SRGGB14_1X14, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB14) },
		{ MEDIA_BUS_FMT_SBGGR16_1X16, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR16) },
		{ MEDIA_BUS_FMT_SGBRG16_1X16, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG16) },
		{ MEDIA_BUS_FMT_SGRBG16_1X16, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG16) },
		{ MEDIA_BUS_FMT_SRGGB16_1X16, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB16) },
	};

	return formats;
}

/*
 * -------------------------------- ISIDevice --------------------------------
 */

/**
 * \brief ISI device capabilities discovery from a \a media device
 * \param[in] media The media device embedding the ISI entity
 *
 * Examines the ISI device entities to discover and initialize the associated
 * channels.
 *
 * \return 0 in case of success, or a negative error value
 */
int ISIDevice::init(const MediaDevice *media)
{
	int ret;

	media_ = media;

	crossbar_ = V4L2Subdevice::fromEntityName(
		media, kSDevCrossBarEntityName());
	if (!crossbar_)
		return -ENODEV;
	ret = crossbar_->open();
	if (ret)
		return ret;

	/*
	 * Discover the number of sink pads
	 */
	xbarSinkPads_ = 0;
	for (MediaPad *pad : crossbar_->entity()->pads()) {
		if (!(pad->flags() & MEDIA_PAD_FL_SINK))
			continue;
		xbarSinkPads_++;
	}
	if (!xbarSinkPads_) {
		LOG(IsiDev, Error) << "No sink pads detected";
		return -ENODEV;
	} else {
		LOG(IsiDev, Debug) << xbarSinkPads_ << " sink pads detected";
	}

	/*
	 * Discover the number of ISI pipes
	 */
	for (unsigned int i = 0;; ++i) {
		std::unique_ptr<ISIPipe> pipe = std::make_unique<ISIPipe>(i);
		ret = pipe->init(media);
		if (ret)
			break;

		pipeEntries_.push_back(make_tuple(std::move(pipe), false));
		if (pipeEntries_.size() >= kPipesMax)
			break;
	}

	if (pipeEntries_.empty()) {
		LOG(IsiDev, Error) << "Unable to enumerate pipes";
		return -ENODEV;
	} else {
		LOG(IsiDev, Debug) << pipeEntries_.size() << " pipes enumerated";
	}

	return 0;
}

/**
 * \brief Dynamically allocate any ISI channel context
 * \return The ISI channel context, nullptr otherwise
 */
ISIPipe *ISIDevice::allocPipe()
{
	/* \todo handle line buffers sharing between channels */
	auto it = std::find_if(pipeEntries_.begin(),
			       pipeEntries_.end(),
			       [](const auto &entry) -> bool {
				       return std::get<1>(entry) == false;
			       });

	if (it == pipeEntries_.end()) {
		LOG(IsiDev, Error) << "No more ISI channel available";
		return nullptr;
	}

	ISIPipe *pipe = std::get<0>(*it).get();
	/* allocation state */
	std::get<1>(*it) = true;
	return pipe;
}

/**
 * \brief Dynamically allocate a specific ISI channel context
 * \param[in] index The ISI pipe index to be allocated
 * \return The ISI channel context if available, nullptr otherwise
 */
ISIPipe *ISIDevice::allocPipe(unsigned int index)
{
	if (index >= pipeEntries_.size()) {
		LOG(IsiDev, Error) << "Invalid ISI channel " << index;
		return nullptr;
	}

	auto &pipeEntry = pipeEntries_[index];
	if (std::get<1>(pipeEntry) == true) {
		LOG(IsiDev, Error) << "ISI channel already allocated" << index;
		return nullptr;
	}

	ISIPipe *pipe = std::get<0>(pipeEntry).get();
	/* allocation state */
	std::get<1>(pipeEntry) = true;
	return pipe;
}

/**
 * \brief Free an ISI channel context allocated by ISIDevice::allocPipe()
 */
void ISIDevice::freePipe(ISIPipe *pipe)
{
	if (!pipe)
		return;

	auto it = std::find_if(pipeEntries_.begin(),
			       pipeEntries_.end(),
			       [=](auto &entry) -> bool {
				       return std::get<0>(entry).get() == pipe;
			       });

	if (it == pipeEntries_.end()) {
		LOG(IsiDev, Error) << "Unknown pipe to free";
		return;
	}

	if (!std::get<1>(*it)) {
		LOG(IsiDev, Error)
			<< "Pipe " << pipe->index() << "Already freed";
		return;
	}

	if (pipe->stateActive()) {
		LOG(IsiDev, Error)
			<< "Can't free pipe " << pipe->index() << " while active";
		return;
	}

	/* free again */
	std::get<1>(*it) = false;
}

} /* namespace libcamera */
