/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * ISI device
 *
 * Based on Intel IPU3 CIO2
 *     src/libcamera/pipeline/ipu3/cio2.cpp
 * Copyright (C) 2019, Google Inc.
 */

#include <limits>
#include <string_view>

#include <linux/media-bus-format.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

#include "isi_device.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoIsiDev)

namespace nxpneo {

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
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10), MEDIA_BUS_FMT_SBGGR10_1X10 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10), MEDIA_BUS_FMT_SGBRG10_1X10 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10), MEDIA_BUS_FMT_SGRBG10_1X10 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10), MEDIA_BUS_FMT_SRGGB10_1X10 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12), MEDIA_BUS_FMT_SBGGR12_1X12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12), MEDIA_BUS_FMT_SGBRG12_1X12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12), MEDIA_BUS_FMT_SGRBG12_1X12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12), MEDIA_BUS_FMT_SRGGB12_1X12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR14), MEDIA_BUS_FMT_SBGGR14_1X14 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG14), MEDIA_BUS_FMT_SGBRG14_1X14 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG14), MEDIA_BUS_FMT_SGRBG14_1X14 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB14), MEDIA_BUS_FMT_SRGGB14_1X14 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR16), MEDIA_BUS_FMT_SBGGR16_1X16 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG16), MEDIA_BUS_FMT_SGBRG16_1X16 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG16), MEDIA_BUS_FMT_SGRBG16_1X16 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB16), MEDIA_BUS_FMT_SRGGB16_1X16 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_GREY), MEDIA_BUS_FMT_Y8_1X8 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_Y10), MEDIA_BUS_FMT_Y10_1X10 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_Y12), MEDIA_BUS_FMT_Y12_1X12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_Y16), MEDIA_BUS_FMT_Y16_1X16 },
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
 * \brief Construct an ISIPipe instance
 * \param[in] index The index of the ISI pipe channel
 *
 * Constructs an ISIPipe object for the specified channel index.
 * The actual device initialization is performed separately via the init()
 * method.
 */
ISIPipe::ISIPipe(unsigned int index)
	: index_(index)
{
	static constexpr std::string_view kDeviceEntityPrefix = "mxc_isi.";
	static constexpr std::string_view kVDeviceEntitySuffix = ".capture";

	subdeviceName_ =
		std::string(kDeviceEntityPrefix) + std::to_string(index);
	videoDeviceName_ =
		std::string(kDeviceEntityPrefix) + std::to_string(index) +
		std::string(kVDeviceEntitySuffix);
}

/**
 * \brief Initialize components of the ISI pipe
 * \param[in] media The ISI media device
 *
 * Create and open the video device and subdevice of the ISI pipe channel.
 * This function locates the subdevice and video device entities by name,
 * creates the corresponding V4L2Subdevice and V4L2VideoDevice instances,
 * and opens them for use.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::init(const MediaDevice *media)
{
	int ret;

	LOG(NxpNeoIsiDev, Debug) << "Init pipe index " << index_;

	pipe_ = V4L2Subdevice::fromEntityName(media, subdeviceName());
	if (!pipe_)
		return -ENODEV;

	ret = pipe_->open();
	if (ret) {
		LOG(NxpNeoIsiDev, Error) << logPrefix() << "Failed to open subdev";
		return ret;
	}

	capture_ = V4L2VideoDevice::fromEntityName(media, videoDeviceName());
	if (!capture_)
		return -ENODEV;

	ret = capture_->open();
	if (ret)
		LOG(NxpNeoIsiDev, Error) << logPrefix() << "Failed to open videodev";

	return ret;
}

/**
 * \brief Configure the ISI channel subdevice and video node formats
 * \param[inout] sinkFormat The format applied to the subdevice sink pad
 * \param[inout] deviceFormat The format applied to the video device
 *
 * This function configures ISI pipe formats: the subdevice sink and source
 * pads, and its capture video device.
 *
 * In channel bypass mode (bayer or meta), the subdevice sink and source formats
 * are the same and the function infers both the subdevice source and the video
 * device formats from the subdevice sink format.
 *
 * In processed mode, the subdevice source format differs from the sink, and is
 * inferred from the video device format.
 *
 * The function validates the input formats against the supported format maps
 * (bayerFormatsMap, metaFormatsMap, or processedFormatsMap) and configures:
 * - The subdevice sink pad (pad 0)
 * - The subdevice source pad (pad 1)
 * - The capture video device
 *
 * \return 0 on success or a negative error code otherwise
 */
int ISIPipe::configure(V4L2SubdeviceFormat &sinkFormat,
		       V4L2DeviceFormat &deviceFormat)
{
	int ret;

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

	ret = capture_->setFormat(&deviceFormat);
	if (ret) {
		LOG(NxpNeoIsiDev, Error)
			<< logPrefix()
			<< "Failed to configure video device";
		return ret;
	}

	LOG(NxpNeoIsiDev, Debug)
		<< logPrefix() << "Video device configured dev fmt "
		<< deviceFormat.toString();

	return 0;
}

/**
 * \fn ISIPipe::subdeviceName()
 * \brief Get the subdevice entity name
 * \return The subdevice entity name
 */

/**
 * \fn ISIPipe::videoDeviceName()
 * \brief Get the video device entity name
 * \return The video device entity name
 */

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

	if (itMeta != metaFormatsMap.end())
		return itMeta->first;

	LOG(NxpNeoIsiDev, Error) << "Unknown bypass format";
	return {};
}

/*
 * -------------------------------- ISIDevice --------------------------------
 */

/**
 * \brief Construct and initialize the ISI device
 * \param[in] media The media device containing the ISI entities
 *
 * This constructor initializes the ISI device by discovering and configuring
 * its components from the provided media device.
 *
 * It opens the crossbar subdevice, discovers the number of sink pads which
 * represent the input interfaces to the ISI device, and enumerates the ISI
 * processing pipes. Each pipe is initialized and stored for later use.
 */
ISIDevice::ISIDevice(std::shared_ptr<MediaDevice> media)
	: media_(media), valid_(false)
{
	if (!media_) {
		LOG(NxpNeoIsiDev, Error) << "Invalid media device";
		return;
	}

	crossbar_ = V4L2Subdevice::fromEntityName(media.get(), crossbarSubdevName());
	if (!crossbar_)
		return;
	int ret = crossbar_->open();
	if (ret)
		return;

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
		return;
	} else {
		LOG(NxpNeoIsiDev, Debug) << xbarSinkPads_ << " sink pads detected";
	}

	/*
	 * Discover the number of ISI pipes
	 */
	unsigned int pipeCount =
		crossbar_->entity()->pads().size() - xbarSinkPads_;
	pipeEntries_.reserve(pipeCount);
	for (unsigned int i = 0; i < pipeCount; ++i) {
		PipeWrapper wrapper(i);
		if (wrapper.pipe.init(media.get()))
			return;
		pipeEntries_.push_back(std::move(wrapper));
	}

	if (pipeEntries_.empty()) {
		LOG(NxpNeoIsiDev, Error) << "Unable to enumerate pipes";
		return;
	}

	LOG(NxpNeoIsiDev, Debug) << pipeEntries_.size() << " pipes enumerated";
	valid_ = true;
}

/**
 * \brief Reserve an ISI pipe based on image width requirements
 * \param[in] width The width of the image in pixels
 *
 * Reserve the necessary number of ISI channels given the image width.
 * Above a certain width threshold, a second adjacent channel must be reserved
 * to chain the two channel buffers together.
 *
 * Only a pointer to the first channel's ISIPipe is returned, as the chaining
 * remains internal to the ISI device. However, the chained channel is marked
 * as reserved and cannot be used independently.
 *
 * \return A pointer to the reserved ISIPipe on success, or nullptr on failure
 */
ISIPipe *ISIDevice::reservePipe(unsigned int width)
{
	bool chained = false;
	ISIPipe *pipe = nullptr;

	if (!valid_)
		return nullptr;

	if (width > ISIPipe::kChainedWidthMax) {
		LOG(NxpNeoIsiDev, Error)
			<< "Maximum width " << ISIPipe::kChainedWidthMax
			<< " exceeded by width " << width;
		return nullptr;
	}

	if (width > ISIPipe::kUnchainedWidthMax)
		chained = true;

	unsigned int pipeCount = pipeEntries_.size();
	unsigned int index = 0;
	for (const auto &[i, entry] : utils::enumerate(pipeEntries_)) {
		if (chained && i + 1 >= pipeCount)
			break;
		if (!entry.free)
			continue;
		/* Last pipe can not be chained */
		if (!chained || (i + 1 < pipeCount && pipeEntries_[i + 1].free)) {
			entry.free = false;
			entry.chained = chained;
			if (chained) {
				PipeWrapper &next = pipeEntries_[i + 1];
				next.free = false;
				next.chained = false;
			}
			pipe = &entry.pipe;
			index = i;
			break;
		}
	}

	if (pipe) {
		LOG(NxpNeoIsiDev, Debug)
			<< "Reserved pipe index " << index
			<< " width " << width << " chained " << chained;
	} else {
		LOG(NxpNeoIsiDev, Error)
			<< "Unable to reserve pipe for size " << width;
	}

	return pipe;
}

/**
 * \brief Release a previously reserved ISI pipe
 * \param[in] pipe Pointer to the ISIPipe to release
 *
 * Releases the specified ISI pipe and marks it as available for future use.
 * If the pipe was part of a chained configuration, both the primary and
 * secondary (chained) pipes are released.
 *
 * Error messages are logged if the pipe is invalid or already freed.
 */
void ISIDevice::releasePipe(ISIPipe *pipe)
{
	if (!valid_)
		return;

	auto it = std::find_if(pipeEntries_.begin(), pipeEntries_.end(),
			       [pipe](const PipeWrapper &entry) {
				       return &entry.pipe == pipe;
			       });

	if (it == pipeEntries_.end()) {
		LOG(NxpNeoIsiDev, Error) << "Invalid pipe";
		return;
	}

	PipeWrapper &entry = *it;
	unsigned int index = std::distance(pipeEntries_.begin(), it);

	LOG(NxpNeoIsiDev, Debug)
		<< "Release pipe index " << index << " chained " << entry.chained;

	if (entry.free) {
		LOG(NxpNeoIsiDev, Error)
			<< "Pipe index " << index << " already freed";
		return;
	}
	entry.free = true;
	if (entry.chained) {
		ASSERT(index + 1 < pipeEntries_.size());
		entry.chained = false;
		PipeWrapper &next = pipeEntries_[index + 1];
		ASSERT(!next.free);
		next.free = true;
		next.chained = false;
	}
}

/**
 * \brief Get the ISI driver name
 *
 * Returns the name of the ISI driver used to identify the device in the media
 * controller framework.
 *
 * \return The ISI driver name string ("mxc-isi")
 */
const std::string &ISIDevice::driverName()
{
	static const std::string driverName = "mxc-isi";
	return driverName;
}

/**
 * \brief Get the ISI crossbar subdevice name
 *
 * Returns the name of the ISI crossbar subdevice used to identify the device
 * in the media controller framework.
 *
 * \return The ISI crossbar subdevice name string ("crossbar")
 */
const std::string &ISIDevice::crossbarSubdevName()
{
	static const std::string crossbarName = "crossbar";
	return crossbarName;
}

} /* namespace nxpneo */

} /* namespace libcamera */
