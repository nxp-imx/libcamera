/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Intel IPU3 ImgU
 *     src/libcamera/pipeline/ipu3/imgu.cpp
 * Copyright (C) 2019, Google Inc.
 *
 * Copyright 2024 NXP
 * neo_device.cpp - NXP NEO
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include <linux/media-bus-format.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/media_device.h"

#include "neo_device.h"


namespace libcamera {

LOG_DEFINE_CATEGORY(NeoDev)

/**
 * \brief Initialize components of the NEO instance
 * \param[in] media The NEO instance media device
 *
 * Create and open the V4L2 devices and subdevices of the NEO instance.
 *
 * In case of errors the created V4L2VideoDevice and V4L2Subdevice instances
 * are destroyed at pipeline handler delete time.
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::init(MediaDevice *media)
{
	int ret;

	media_ = media;

	/*
	 * Presence of the media device has been verified by the match()
	 * function. There is no more need to check for devices availability.
	 */
	isp_ = V4L2Subdevice::fromEntityName(media, kSDevNeoEntityName());
	ret = isp_->open();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix() << "Failed to open NEO";
		return ret;
	}

	input0_ = V4L2VideoDevice::fromEntityName(media, kVDevInput0EntityName());
	ret = input0_->open();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix() << "Failed to open NEO input0";
		return ret;
	}

	input1_ = V4L2VideoDevice::fromEntityName(media, kVDevInput1EntityName());
	ret = input1_->open();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix() << "Failed to open NEO input1";
		return ret;
	}

	params_ = V4L2VideoDevice::fromEntityName(media, kVDevEntityParamsName());
	ret = params_->open();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix() << "Failed to open NEO params";
		return ret;
	}

	frame_ = V4L2VideoDevice::fromEntityName(media, kVDevEntityFrameName());
	ret = frame_->open();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix() << "Failed to open NEO frame";
		return ret;
	}

	ir_ = V4L2VideoDevice::fromEntityName(media, kVDevEntityIrName());
	ret = ir_->open();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix() << "Failed to open NEO ir";
		return ret;
	}

	stats_ = V4L2VideoDevice::fromEntityName(media, kVDevEntityStatsName());
	ret = stats_->open();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix() << "Failed to open NEO stats";
		return ret;
	}

	return 0;
}

/**
 * \brief Allocate buffers for all the NEO video devices
 * \param[in] bufferCount The number of buffers to allocate
 *
 * Function handles buffer allocation for every pad of NEO.
 * Buffers queued onto INPUT0 and INPUT1 pads come from the
 * pipeline and are imported.
 * Params and stats buffer are allocated on respective device
 * nodes.
 * Buffers for output (Capture) frame and IR pads are provided
 * by the application and are imported.
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::allocateBuffers(unsigned int bufferCount)
{
	int ret;

	/* Share buffers between ISI outputs and NEO input0/input1 inputs. */
	ret = input0_->importBuffers(bufferCount);
	if (ret) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to import NEO input0 input buffers";
		return ret;
	}

	if (padActiveInput1()) {
		ret = input1_->importBuffers(bufferCount);
		if (ret) {
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to import NEO input1 input buffers";
			return ret;
		}
	}

	/* Params/stats buffers allocated internally */
	ret = params_->allocateBuffers(bufferCount, &paramsBuffers_);
	if (ret < 0) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to allocate NEO params buffers";
		goto error;
	}

	ret = stats_->allocateBuffers(bufferCount, &statsBuffers_);
	if (ret < 0) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to allocate NEO stats buffers";
		goto error;
	}

	/* NEO Frame/ir output buffers allocated by application */
	if (padActiveFrame()) {
		ret = frame_->importBuffers(bufferCount);
		if (ret < 0) {
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to import NEO frame buffers";
			goto error;
		}
	}

	if (padActiveIr()) {
		ret = ir_->importBuffers(bufferCount);
		if (ret < 0) {
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to import NEO ir buffers";
			goto error;
		}
	}

	return 0;

error:
	freeBuffers();

	return ret;
}

/**
 * \brief Release buffers for all the NEO video devices
 */
void NeoDevice::freeBuffers()
{
	int ret;

	paramsBuffers_.clear();
	statsBuffers_.clear();

	ret = input0_->releaseBuffers();
	if (ret)
		LOG(NeoDev, Error)  << logPrefix()
			<< "Failed to release NEO input0 buffers";

	if (padActiveInput1()) {
		ret = input1_->releaseBuffers();
		if (ret)
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to release NEO input1 buffers";
	}

	ret = params_->releaseBuffers();
	if (ret)
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to release NEO params buffers";

	ret = stats_->releaseBuffers();
	if (ret)
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to release NEO stats buffers";

	if (padActiveFrame()) {
		ret = frame_->releaseBuffers();
		if (ret)
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to release NEO frame buffers";
	}

	if (padActiveIr()) {
		ret = ir_->releaseBuffers();
		if (ret)
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to release NEO ir buffers";
	}
}

/**
 * \brief Start NEO video devices operation
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::start()
{
	int ret;

	/* Start the NEO output video devices. */
	if (padActiveFrame()) {
		ret = frame_->streamOn();
		if (ret) {
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to start NEO frame";
			return ret;
		}
	}

	if (padActiveIr()) {
		ret = ir_->streamOn();
		if (ret) {
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to start NEO ir";
			return ret;
		}
	}

	/* Start the NEO params and stats devices. */
	ret = params_->streamOn();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to start NEO params";
		return ret;
	}

	ret = stats_->streamOn();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to start NEO stats";
		return ret;
	}

	/* Start the NEO input video devices. */
	ret = input0_->streamOn();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to start NEO input0";
		return ret;
	}

	if (padActiveInput1()) {
		ret = input1_->streamOn();
		if (ret) {
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to start NEO input1";
			return ret;
		}
	}

	ret = isp_->setFrameStartEnabled(true);
	if (ret) {
		LOG(NeoDev, Error) << "FrameStart failure ret";
		return ret;
	}

	return 0;
}

/**
 * \brief Stop video devices operation
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::stop()
{
	int ret;

	ret = isp_->setFrameStartEnabled(false);

	if (padActiveFrame())
		ret |= frame_->streamOff();
	if (padActiveIr())
		ret |= ir_->streamOff();
	ret |= params_->streamOff();
	ret |= stats_->streamOff();
	ret |= input0_->streamOff();
	if (padActiveInput1())
		ret |= input1_->streamOff();

	if (ret)
		LOG(NeoDev, Error) << logPrefix() << "Failed to stop";

	return ret;
}

/**
 * \brief Enable or disable a single link on the NEO instance
 *
 * This function assumes that the media device associated with the NEO instance
 * is opened.
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::linkSetup(const std::string &source, unsigned int sourcePad,
			 const std::string &sink, unsigned int sinkPad,
			 bool enable)
{
	int ret;

	MediaLink *link = media_->link(source, sourcePad, sink, sinkPad);
	if (!link) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to get link: '" << source << "':"
			<< sourcePad << " -> '" << sink << "':" << sinkPad;
		return -ENODEV;
	}

	ret = link->setEnabled(enable);
	if (ret) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to enable/disable link (" << enable
			<< ") : '" << source << "':"
			<< sourcePad << " -> '" << sink << "':" << sinkPad;
		return -ENODEV;
	}

	return 0;
}

/**
 * \brief Enable or disable all media links in the NEO instance to prepare
 * for capture operations
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::enableLinks(bool enable)
{
	int ret;

	ret = linkSetup(kVDevInput0EntityName(), 0,
			kSDevNeoEntityName(), PAD_INPUT0, enable);
	if (ret)
		return ret;

	if (padActiveInput1() || !enable) {
		ret = linkSetup(kVDevInput1EntityName(), 0,
				kSDevNeoEntityName(), PAD_INPUT1, enable);
		if (ret)
			return ret;
	}

	ret = linkSetup(kVDevEntityParamsName(), 0,
			kSDevNeoEntityName(), PAD_PARAMS, enable);
	if (ret)
		return ret;

	if (padActiveFrame() || !enable) {
		ret = linkSetup(kSDevNeoEntityName(), PAD_FRAME,
				kVDevEntityFrameName(), 0, enable);
		if (ret)
			return ret;
	}

	if (padActiveIr() || !enable) {
		ret = linkSetup(kSDevNeoEntityName(), PAD_IR,
				kVDevEntityIrName(), 0, enable);
		if (ret)
			return ret;
	}

	ret = linkSetup(kSDevNeoEntityName(), PAD_STATS,
			kVDevEntityStatsName(), 0, enable);
	if (ret)
		return ret;

	return 0;
}

/**
 * \brief Configure NEO video device \a dev with the given \a format.
 * \param[in] dev video device
 * \param[in] pad NEO subdevice pad linked to the video node
 * \param[in] format video device format to be configured
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::configureVideoDevice(V4L2VideoDevice *dev, unsigned int pad,
				    V4L2DeviceFormat &format)
{
	int ret;

	LOG(NeoDev, Debug) << logPrefix()
		<< "Configure video device (" << pad << ") format "
		<< format.toString();

	ret = dev->setFormat(&format);
	if (ret) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to set device pad format";
		return ret;
	}

	return 0;
}

/**
 * \brief Configure NEO meta video device \a dev with the given \a fourcc.
 * \param[in] dev video device
 * \param[in] pad NEO subdevice pad linked to the video node
 * \param[in] fourcc fourcc to be configured
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::configureVideoDeviceMeta(V4L2VideoDevice *dev,
					unsigned int pad, uint32_t fourcc)
{
	int ret;
	V4L2DeviceFormat devFormat = {};

	devFormat.fourcc = V4L2PixelFormat(fourcc);
	ret = dev->setFormat(&devFormat);
	if (ret) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to set format for meta video device pad ("
			<< pad << ")";
		return ret;
	}

	LOG(NeoDev, Debug) << logPrefix()
		<< "Configured meta video device format ("<< pad << ") "
		<< devFormat;

	return 0;
}


/**
 * \brief Configure NEO video devices according to their formats
 * \param[in] formatInput0 INPUT0 video device format
 * \param[in] formatInput1 INPUT1 video device format
 * \param[in] formatFrame FRAME device node format
 * \param[in] formatIr IR video device format
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::configure(V4L2DeviceFormat &formatInput0,
			 V4L2DeviceFormat &formatInput1,
			 V4L2DeviceFormat &formatFrame,
			 V4L2DeviceFormat &formatIr)
{
	int ret;

	/* \todo remove meta format local definitions */
#ifndef V4L2_META_FMT_NEO_ISP_PARAMS
	#define V4L2_META_FMT_NEO_ISP_PARAMS	v4l2_fourcc('N', 'N', 'I', 'P')
	#define V4L2_META_FMT_NEO_ISP_STATS	v4l2_fourcc('N', 'N', 'I', 'S')
#endif

	configInput1_ = formatInput1.fourcc.isValid();
	configFrame_ = formatFrame.fourcc.isValid();
	configIr_ = formatIr.fourcc.isValid();

	/*
	 * \todo NEO driver currently defines immutable links.
	 * When that is changed, dynamically enable relevant links to match
	 * configuration and optional usage of INPUT1, INPUT0 and IR.
	 */

	/*
	enableLinks(false);
	enableLinks(true);
	*/

	ret = configureVideoDevice(input0_.get(), PAD_INPUT0, formatInput0);
	if (ret)
		return ret;

	if (padActiveInput1()) {
		ret = configureVideoDevice(input1_.get(), PAD_INPUT1, formatInput1);
		if (ret)
			return ret;
	}

	if (padActiveFrame()) {
		ret = configureVideoDevice(frame_.get(), PAD_FRAME, formatFrame);
		if (ret)
			return ret;
	}

	if (padActiveIr()) {
		ret = configureVideoDevice(ir_.get(), PAD_IR, formatIr);
		if (ret)
			return ret;
	}

	ret = configureVideoDeviceMeta(params_.get(), PAD_PARAMS,
				       V4L2_META_FMT_NEO_ISP_PARAMS);
	if (ret)
		return ret;

	ret = configureVideoDeviceMeta(stats_.get(), PAD_STATS,
				       V4L2_META_FMT_NEO_ISP_STATS);
	if (ret)
		return ret;

	return 0;
}

const std::vector<V4L2PixelFormat> &NeoDevice::frameFormats()
{
	/* \todo update per actual driver capability */
	static const std::vector<V4L2PixelFormat> formats = {
		V4L2PixelFormat(V4L2_PIX_FMT_RGB24),
		V4L2PixelFormat(V4L2_PIX_FMT_YUYV),
		V4L2PixelFormat(V4L2_PIX_FMT_NV12),
		V4L2PixelFormat(V4L2_PIX_FMT_NV16),
	};

	return formats;
}

const std::vector<V4L2PixelFormat> &NeoDevice::irFormats()
{
	/* \todo update per actual driver capability */
	static const std::vector<V4L2PixelFormat> formats = {
		V4L2PixelFormat(V4L2_PIX_FMT_GREY),
		V4L2PixelFormat(V4L2_PIX_FMT_Y10),
		V4L2PixelFormat(V4L2_PIX_FMT_Y12),
	};

	return formats;
}

const std::vector<V4L2PixelFormat> &NeoDevice::input0Formats()
{
	/* \todo update per actual driver capability */
	static const std::vector<V4L2PixelFormat> formats = {
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG8),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG8),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB8),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR14),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG14),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG14),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB14),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR16),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG16),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG16),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB16),
	};

	return formats;
}

const std::vector<V4L2PixelFormat> &NeoDevice::input1Formats()
{
	/* \todo update per actual driver capability */
	static const std::vector<V4L2PixelFormat> formats = {
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG8),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG8),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB8),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR14),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG14),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG14),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB14),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR16),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG16),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG16),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB16),
	};

	return formats;
}

} /* namespace libcamera */
