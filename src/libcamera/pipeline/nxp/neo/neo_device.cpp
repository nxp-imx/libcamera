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

	dcg_ = V4L2VideoDevice::fromEntityName(media, kVDevDcgEntityName());
	ret = dcg_->open();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix() << "Failed to open NEO dcg";
		return ret;
	}

	vs_ = V4L2VideoDevice::fromEntityName(media, kVDevVsEntityName());
	ret = vs_->open();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix() << "Failed to open NEO vs";
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
 * Buffers queued onto DCG and VS pads come from the
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

	/* Share buffers between ISI outputs and NEO dcg/vs inputs. */
	ret = dcg_->importBuffers(bufferCount);
	if (ret) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to import NEO dcg input buffers";
		return ret;
	}

	if (padActiveVs()) {
		ret = vs_->importBuffers(bufferCount);
		if (ret) {
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to import NEO vs input buffers";
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

	ret = dcg_->releaseBuffers();
	if (ret)
		LOG(NeoDev, Error)  << logPrefix()
			<< "Failed to release NEO dcg buffers";

	if (padActiveVs()) {
		ret = vs_->releaseBuffers();
		if (ret)
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to release NEO vs buffers";
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
	ret = dcg_->streamOn();
	if (ret) {
		LOG(NeoDev, Error) << logPrefix()
			<< "Failed to start NEO dcg";
		return ret;
	}

	if (padActiveVs()) {
		ret = vs_->streamOn();
		if (ret) {
			LOG(NeoDev, Error) << logPrefix()
				<< "Failed to start NEO vs";
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
	ret |= dcg_->streamOff();
	if (padActiveVs())
		ret |= vs_->streamOff();

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

	ret = linkSetup(kVDevDcgEntityName(), 0,
			kSDevNeoEntityName(), PAD_DCG, enable);
	if (ret)
		return ret;

	if (padActiveVs() || !enable) {
		ret = linkSetup(kVDevVsEntityName(), 0,
				kSDevNeoEntityName(), PAD_VS, enable);
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
 * \param[in] formatDcg DCG video device format
 * \param[in] formatVs VS video device format
 * \param[in] formatDcg FRAME device node format
 * \param[in] formatDcg IR video device format
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::configure(V4L2DeviceFormat &formatDcg,
			 V4L2DeviceFormat &formatVs,
			 V4L2DeviceFormat &formatFrame,
			 V4L2DeviceFormat &formatIr)
{
	int ret;

	/* \todo remove meta format local definitions */
#ifndef V4L2_META_FMT_NEO_ISP_PARAMS
	#define V4L2_META_FMT_NEO_ISP_PARAMS	v4l2_fourcc('N', 'N', 'I', 'P')
	#define V4L2_META_FMT_NEO_ISP_STATS	v4l2_fourcc('N', 'N', 'I', 'S')
#endif

	configVs_ = formatVs.fourcc.isValid();
	configFrame_ = formatFrame.fourcc.isValid();
	configIr_ = formatIr.fourcc.isValid();

	/*
	 * \todo NEO driver currently defines immutable links.
	 * When that is changed, dynamically enable relevant links to match
	 * configuration and optional usage of VS, DCG and IR.
	 */

	/*
	enableLinks(false);
	enableLinks(true);
	*/

	ret = configureVideoDevice(dcg_.get(), PAD_DCG, formatDcg);
	if (ret)
		return ret;

	if (padActiveVs()) {
		ret = configureVideoDevice(vs_.get(), PAD_VS, formatVs);
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

const std::vector<V4L2PixelFormat> &NeoDevice::dcgFormats()
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

const std::vector<V4L2PixelFormat> &NeoDevice::vsFormats()
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
