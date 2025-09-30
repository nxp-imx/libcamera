/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Intel IPU3 CIO2
 *     src/libcamera/pipeline/ipu3/cio2.cpp
 * Copyright (C) 2019, Google Inc.
 *
 * Copyright 2024-2025 NXP
 * isi_device.cpp - NXP ISI
 */

#include <limits>

#include <linux/media-bus-format.h>

#include <libcamera/base/utils.h>

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

LOG_DEFINE_CATEGORY(NxpNeoIsiDev)

/*
 * -------------------------------- ISIPipe --------------------------------
 */

namespace {

/*
 * Those are the mbus codes usable on a pipe sink pad for the processed channels.
 * These codes are the ones available for the upstream graph configuration.
 */
const std::vector<unsigned int> processedSinkCodes = {
	MEDIA_BUS_FMT_UYVY8_2X8,
	MEDIA_BUS_FMT_YUYV8_2X8,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_YUV8_1X24,
	MEDIA_BUS_FMT_RGB565_1X16,
	MEDIA_BUS_FMT_RGB888_1X24,
};

/*
 * This table maps the bayer video device formats to the relevant pipe pads mbus
 * code, for a channel operated in bypass mode.
 */
const std::map<V4L2PixelFormat, uint32_t> bayerFormatsMap = {
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8), MEDIA_BUS_FMT_SBGGR8_1X8 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG8), MEDIA_BUS_FMT_SGBRG8_1X8 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG8), MEDIA_BUS_FMT_SGRBG8_1X8 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB8), MEDIA_BUS_FMT_SRGGB8_1X8 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10), MEDIA_BUS_FMT_SBGGR10_1X10, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10), MEDIA_BUS_FMT_SGBRG10_1X10, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10), MEDIA_BUS_FMT_SGRBG10_1X10, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10), MEDIA_BUS_FMT_SRGGB10_1X10, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12), MEDIA_BUS_FMT_SBGGR12_1X12, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12), MEDIA_BUS_FMT_SGBRG12_1X12, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12), MEDIA_BUS_FMT_SGRBG12_1X12, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12), MEDIA_BUS_FMT_SRGGB12_1X12, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR14), MEDIA_BUS_FMT_SBGGR14_1X14, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG14), MEDIA_BUS_FMT_SGBRG14_1X14, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG14), MEDIA_BUS_FMT_SGRBG14_1X14, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB14), MEDIA_BUS_FMT_SRGGB14_1X14, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR16), MEDIA_BUS_FMT_SBGGR16_1X16, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG16), MEDIA_BUS_FMT_SGBRG16_1X16, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG16), MEDIA_BUS_FMT_SGRBG16_1X16, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB16), MEDIA_BUS_FMT_SRGGB16_1X16, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_GREY), MEDIA_BUS_FMT_Y8_1X8, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_Y10), MEDIA_BUS_FMT_Y10_1X10, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_Y12), MEDIA_BUS_FMT_Y12_1X12, },
	{ V4L2PixelFormat(V4L2_PIX_FMT_Y16), MEDIA_BUS_FMT_Y16_1X16, },
};

/*
 * This table maps the meta video device formats to the relevant pipe pads mbus
 * code, for a channel operated in bypass mode.
 */
const std::map<V4L2PixelFormat, uint32_t> metaFormatsMap = {
	{ V4L2PixelFormat(V4L2_META_FMT_GENERIC_8), MEDIA_BUS_FMT_META_8 },
};

/*
 * This table maps the RGB/YUV video device formats to the relevant pipe source
 * pad mbus code, for a channel operated in processed mode.
 */
const std::map<V4L2PixelFormat, uint32_t> processedFormatsMap = {
	{ V4L2PixelFormat(V4L2_PIX_FMT_YUYV), MEDIA_BUS_FMT_YUV8_1X24 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_YUVA32), MEDIA_BUS_FMT_YUV8_1X24 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_NV12), MEDIA_BUS_FMT_YUV8_1X24 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_NV16), MEDIA_BUS_FMT_YUV8_1X24 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_YUV444M), MEDIA_BUS_FMT_YUV8_1X24 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_RGB565), MEDIA_BUS_FMT_RGB888_1X24 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_RGB24), MEDIA_BUS_FMT_RGB888_1X24 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_BGR24), MEDIA_BUS_FMT_RGB888_1X24 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_XBGR32), MEDIA_BUS_FMT_RGB888_1X24 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_RGBA32), MEDIA_BUS_FMT_RGB888_1X24 },
};

} // namespace

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
		LOG(NxpNeoIsiDev, Debug) << logPrefix() << "failed to open subdev";

	std::string videoDevEntityName = ISIDevice::kVDevPipeEntityName(index_);
	output_ = V4L2VideoDevice::fromEntityName(media, videoDevEntityName);
	if (!output_)
		return -ENODEV;

	ret = output_->open();
	if (ret)
		LOG(NxpNeoIsiDev, Debug) << logPrefix() << "failed to open videodev";

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
		LOG(NxpNeoIsiDev, Error)
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
		LOG(NxpNeoIsiDev, Error)
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
		LOG(NxpNeoIsiDev, Error)
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
 * \brief Configure the ISI channel subdevice and video node formats
 * \param[inout] sinkFormat The format applied to the subdevice sink pad
 * \param[inout] videoFormat The format applied to the video device
 *
 * This function configures ISI pipe formats: the subdevice sink and source
 * pads, and its capture video device.
 * In channel bypass mode (bayer or meta), the subdevice sink and source formats
 * are the same and the function infers both the subdevice source and the video
 * device formats from the subdevice sink format.
 * In processed mode, the subdevice source format differs from the sink, and is
 * inferred from the video device format.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::configure(V4L2SubdeviceFormat &sinkFormat,
		       V4L2DeviceFormat &deviceFormat)
{
	int ret;

	if (!(stateIdle() || stateConfigured())) {
		LOG(NxpNeoIsiDev, Error)
			<< logPrefix()
			<< "Can't be configured in state " << getState();
		return -ENODEV;
	}

	V4L2SubdeviceFormat sourceFormat;
	const std::vector<unsigned int> processedSinkCodes = sinkMbusCodesProcessed();
	auto itSinkCode = std::find(processedSinkCodes.begin(), processedSinkCodes.end(),
				    sinkFormat.code);
	if (itSinkCode != processedSinkCodes.end()) {
		/*
		 * This is a processed channel, infer subdevice source format
		 * from the video device format.
		 */
		const std::vector<V4L2PixelFormat> pixelFormats =
			utils::map_keys(processedFormatsMap);
		auto itPixelFormat = std::find(pixelFormats.begin(),
					       pixelFormats.end(),
					       deviceFormat.fourcc);
		if (itPixelFormat == pixelFormats.end()) {
			LOG(NxpNeoIsiDev, Error)
				<< logPrefix()
				<< "Invalid device pixel format "
				<< deviceFormat.fourcc.toString();
			return -EINVAL;
		}
		sourceFormat = {};
		sourceFormat.code = processedFormatsMap.at(deviceFormat.fourcc);
		sourceFormat.size = sinkFormat.size;
		sourceFormat.colorSpace = sinkFormat.colorSpace;
	} else {
		/*
		 * This is a bypass channel, so check that bayer/meta format is
		 * supported and infer the subdevice source and video device
		 * formats.
		 */
		const std::vector<unsigned int> bayerCodes = bayerMbusCodes();
		auto itBayerCode = std::find(bayerCodes.begin(),
					     bayerCodes.end(), sinkFormat.code);
		bool isBayer = itBayerCode != bayerCodes.end();
		const std::vector<unsigned int> metaCodes = metaMbusCodes();
		auto itMetaCode = std::find(metaCodes.begin(),
					    metaCodes.end(), sinkFormat.code);
		bool isMeta = itMetaCode != metaCodes.end();
		if (!isBayer && !isMeta) {
			LOG(NxpNeoIsiDev, Error)
				<< logPrefix()
				<< "Invalid sink code " << sinkFormat.code;
			return -EINVAL;
		}

		sourceFormat = sinkFormat;

		deviceFormat = {};
		/* Look up for the appropriate device format. */
		const std::map<V4L2PixelFormat, uint32_t> &formatsMap =
			isBayer ? bayerFormatsMap : metaFormatsMap;
		auto itDeviceFormat =
			std::find_if(formatsMap.begin(), formatsMap.end(),
				     [&](const std::pair<const V4L2PixelFormat, uint32_t> &pair) {
					     return pair.second == sourceFormat.code;
				     });
		if (itDeviceFormat == formatsMap.end()) {
			LOG(NxpNeoIsiDev, Error)
				<< logPrefix()
				<< "Invalid source code " << sourceFormat.code;
			return -EINVAL;
		}
		deviceFormat.fourcc = itDeviceFormat->first;
		deviceFormat.size = sourceFormat.size;
		deviceFormat.colorSpace = sourceFormat.colorSpace;
	}

	ret = pipe_->setFormat(0, &sinkFormat);
	if (ret) {
		LOG(NxpNeoIsiDev, Error)
			<< logPrefix()
			<< "Failed to configure subdevice sink";
		return ret;
	}

	ret = pipe_->setFormat(1, &sourceFormat);
	if (ret) {
		LOG(NxpNeoIsiDev, Error)
			<< logPrefix()
			<< "Failed to configure subdevice source";
		return ret;
	}

	ret = output_->setFormat(&deviceFormat);
	if (ret) {
		LOG(NxpNeoIsiDev, Error)
			<< logPrefix()
			<< "Failed to configure video device";
		return ret;
	}

	setState(kStateConfigured);

	LOG(NxpNeoIsiDev, Debug)
		<< logPrefix() << " Video device configured "
		<< " dev fmt " << deviceFormat.toString();

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
		LOG(NxpNeoIsiDev, Error)
			<< logPrefix()
			<< "Can't allocate buffers in state " << getState();
		return -ENODEV;
	}

	int ret = output_->exportBuffers(bufferCount, &buffers_);
	if (ret < 0) {
		LOG(NxpNeoIsiDev, Error) << logPrefix() << "failed to export buffers";
		return ret;
	}

	return importBuffers(bufferCount);
}

/**
 * \brief Import buffers for ISI channel
 * \param[in] bufferCount The number of buffers to import
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::importBuffers(unsigned int bufferCount)
{
	if (!stateConfigured()) {
		LOG(NxpNeoIsiDev, Error)
			<< logPrefix()
			<< "Can't import buffers in state " << getState();
		return -ENODEV;
	}

	int ret = output_->importBuffers(bufferCount);
	if (ret < 0) {
		LOG(NxpNeoIsiDev, Error) << logPrefix() << "failed to import buffers";
		freeBuffers();
	}

	return ret;
}

/**
 * \brief Release the pool of preallocated buffers created by allocateBuffers()
 */
void ISIPipe::freeBuffers()
{
	if (!stateConfigured()) {
		LOG(NxpNeoIsiDev, Error)
			<< logPrefix()
			<< "Can't deallocate buffers in state " << getState();
		return;
	}

	buffers_.clear();

	if (output_->releaseBuffers())
		LOG(NxpNeoIsiDev, Error) << logPrefix() << "failed to free buffers";
}

/**
 * \brief Return the supported bayer codes on the pipe pads of a bypass channel
 * \return The bayer mbus codes
 */
const std::vector<uint32_t> &ISIPipe::bayerMbusCodes()
{
	/* Initialize once */
	static const std::vector<uint32_t> bayerCodes = []() {
		std::vector<uint32_t> codes;
		std::transform(bayerFormatsMap.begin(), bayerFormatsMap.end(),
			       std::back_inserter(codes),
			       [](const std::pair<const V4L2PixelFormat, unsigned int> &pair) {
				       return pair.second;
			       });
		return codes;
	}();
	return bayerCodes;
}

/**
 * \brief Return the supported meta codes on the pipe pads of a bypass channel
 * \return The meta mbus codes
 */
const std::vector<uint32_t> &ISIPipe::metaMbusCodes()
{
	/* Initialize once */
	static const std::vector<uint32_t> metaCodes = []() {
		std::vector<uint32_t> codes;
		std::transform(metaFormatsMap.begin(), metaFormatsMap.end(),
			       std::back_inserter(codes),
			       [](const std::pair<const V4L2PixelFormat, unsigned int> &pair) {
				       return pair.second;
			       });
		return codes;
	}();
	return metaCodes;
}

/**
 * \brief Return the supported codes on the pipe sink pad of a processed channel
 * \return The mbus codes
 */
const std::vector<uint32_t> &ISIPipe::sinkMbusCodesProcessed()
{
	return processedSinkCodes;
}

/**
 * \brief Return the supported video device pixel formats on a processed channel
 * \return The pixel formats
 */
const std::vector<PixelFormat> &ISIPipe::pixelFormatsProcessed()
{
	/* Initialize once */
	static std::vector<PixelFormat> pixelFormats = []() {
		std::vector<PixelFormat> formats;
		std::vector<V4L2PixelFormat> deviceFormats =
			utils::map_keys(processedFormatsMap);
		std::transform(deviceFormats.begin(), deviceFormats.end(),
			       std::back_inserter(formats),
			       [](const V4L2PixelFormat &format) {
				       return format.toPixelFormat();
			       });
		return formats;
	}();

	return pixelFormats;
}

/**
 * \brief Return the pixel format associated to a bypass channel mbus code
 * \return The pixel formats
 */
const V4L2PixelFormat ISIPipe::mbusCodeToPixelFormatBypass(unsigned int code)
{
	auto itRaw = std::find_if(bayerFormatsMap.begin(), bayerFormatsMap.end(),
				  [=](const std::pair<V4L2PixelFormat, unsigned int> &pair) {
					  return pair.second == code;
				  });
	if (itRaw != bayerFormatsMap.end())
		return itRaw->first;

	auto itMeta = std::find_if(metaFormatsMap.begin(), metaFormatsMap.end(),
				   [=](const std::pair<V4L2PixelFormat, unsigned int> &pair) {
					   return pair.second == code;
				   });

	if (itMeta != bayerFormatsMap.end())
		return itMeta->first;

	LOG(NxpNeoIsiDev, Error) << "Unknown bypass format";
	return {};
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
int ISIDevice::init(MediaDevice *media)
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
		LOG(NxpNeoIsiDev, Error) << "No sink pads detected";
		return -ENODEV;
	} else {
		LOG(NxpNeoIsiDev, Debug) << xbarSinkPads_ << " sink pads detected";
	}

	/*
	 * Discover the number of ISI pipes
	 */
	for (unsigned int i = 0;; ++i) {
		PipeWrapper wrapper(i);
		if (wrapper.pipe_.init(media))
			break;
		pipeEntries_.push_back(std::move(wrapper));
	}

	if (pipeEntries_.empty()) {
		LOG(NxpNeoIsiDev, Error) << "Unable to enumerate pipes";
		return -ENODEV;
	} else {
		LOG(NxpNeoIsiDev, Debug) << pipeEntries_.size() << " pipes enumerated";
	}

	return 0;
}

/**
 * \brief Reserve an ISI pipe based on its image size usage
 * \param[in] sizeMax The size of the image
 * \param[out] index The pipe channel index reserved
 *
 * Reserve the necessary number of ISI channels given the image size.
 * Above a certain width, a second adjacent channel has to be reserved
 * in order to chain the 2 channel buffers.
 * Only the first channel index is reported, as the chaining remains internal to
 * the ISI device. However, the chained channel buffer is marked as reserved as
 * it can no longer be used.
 *
 * \return 0 on success, or a negative error code otherwise
 */
int ISIDevice::reservePipeBySize(Size &sizeMax, unsigned int *index)
{
	*index = std::numeric_limits<unsigned int>::max();
	bool chained = false;

	if (sizeMax.width > kChainedWidthMax) {
		LOG(NxpNeoIsiDev, Error)
			<< "Maximum width " << kChainedWidthMax
			<< " exceeded by size " << sizeMax.toString();
		return -EINVAL;
	}

	if (sizeMax.width > kUnchainedWidthMax)
		chained = true;

	unsigned int pipes = pipeEntries_.size();
	for (const auto [i, entry] : utils::enumerate(pipeEntries_)) {
		/* Last pipe can not be chained */
		if (chained && i + 1 >= pipes)
			break;
		if (entry.free) {
			if (!chained || pipeEntries_[i + 1].free) {
				*index = i;
				break;
			}
		}
	}

	/* Available pipe found, mark it as allocated */
	bool found = *index < pipes;
	if (found) {
		pipeEntries_[*index].free = false;
		if (chained) {
			pipeEntries_[*index].chained = true;
			pipeEntries_[*index + 1].free = false;
		} else {
			pipeEntries_[*index].chained = false;
		}
	}

	return found ? 0 : -EBUSY;
}

/**
 * \brief Reserve an ISI pipe based on its index
 * \param[in] sizeMax The size of the image
 * \param[out] index The pipe channel index to be reserved
 *
 * Reserve the necessary number of ISI channels given the provided channel
 * index. This variant of ISI pipe reservation is to be used when the caller
 * needs to explicitly select the channels to be reserved.
 *
 * \return 0 on success, or a negative error code otherwise
 */
int ISIDevice::reservePipeByIndex(Size &sizeMax, unsigned int index)
{
	bool chained = false;

	unsigned int pipes = pipeEntries_.size();
	if (index >= pipes) {
		LOG(NxpNeoIsiDev, Error)
			<< "Invalid pipe index " << index
			<< " max " << pipes;
		return -EINVAL;
	}

	if (!pipeEntries_[index].free) {
		LOG(NxpNeoIsiDev, Error)
			<< "Pipe index " << index << " already allocated";
		return -EBUSY;
	}

	if (sizeMax.width > kUnchainedWidthMax)
		chained = true;

	if (chained && (index + 1 >= pipes ||
			!pipeEntries_[index + 1].free)) {
		LOG(NxpNeoIsiDev, Error)
			<< "Pipe index " << index << " can't be chained";
		return -EINVAL;
	}

	pipeEntries_[index].free = false;
	if (chained) {
		pipeEntries_[index].chained = true;
		pipeEntries_[index + 1].free = false;
	} else {
		pipeEntries_[index].chained = false;
	}

	return 0;
}

/**
 * \brief Release previously reserved ISI pipe
 * \param[out] index The first channel index reserved
 * \return 0 on success, or a negative error code otherwise
 */
void ISIDevice::releasePipe(unsigned int index)
{
	unsigned int pipes = pipeEntries_.size();
	if (index >= pipes) {
		LOG(NxpNeoIsiDev, Error)
			<< "Invalid pipe index " << index
			<< " max " << pipes;
		return;
	}

	if (pipeEntries_[index].free) {
		LOG(NxpNeoIsiDev, Error)
			<< "Pipe index " << index << " already freed";
		return;
	}

	ASSERT(index + 1 < pipes || !pipeEntries_[index].chained);
	if (pipeEntries_[index].chained) {
		pipeEntries_[index + 1].free = true;
		pipeEntries_[index].chained = false;
	}
	pipeEntries_[index].free = true;
}

/**
 * \brief Get the ISIPipe instance associated to a previously reserved pipe
 * \param[in] index The pipe channel index
 *
 * Return the ISIPipe instance for a given pipe index. The pipe must have been
 * reserved beforehand.
 *
 * \return The ISIPipe on success, nullptr otherwise
 */
ISIPipe *ISIDevice::getPipeByIndex(unsigned int index)
{
	unsigned int pipes = pipeEntries_.size();

	if (index >= pipes) {
		LOG(NxpNeoIsiDev, Error)
			<< "Invalid pipe index " << index << " max " << pipes;
		return nullptr;
	}

	if (pipeEntries_[index].free) {
		LOG(NxpNeoIsiDev, Error)
			<< "Pipe not reserved " << index;
		return nullptr;
	}

	return &pipeEntries_[index].pipe_;
}

} /* namespace libcamera */
