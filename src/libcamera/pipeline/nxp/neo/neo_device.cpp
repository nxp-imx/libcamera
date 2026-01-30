/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Neo ISP device
 *
 * Based on Intel IPU3 ImgU
 *     src/libcamera/pipeline/ipu3/imgu.cpp
 * Copyright (C) 2019, Google Inc.
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <regex>

#include <linux/media-bus-format.h>
#include <linux/nxp_neoisp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"

#include "neo_device.h"

namespace libcamera {

/* \todo Remove meta format local definitions. */
#ifndef V4L2_META_FMT_NEO_ISP_PARAMS
#define V4L2_META_FMT_NEO_ISP_EXT_PARAMS v4l2_fourcc('N', 'N', 'E', 'P')
#define V4L2_META_FMT_NEO_ISP_EXT_STATS v4l2_fourcc('N', 'N', 'E', 'S')
#endif

/* \todo Remove when control is available in v4l2-controls header */
#ifndef V4L2_CID_USER_NEOISP_BASE
#define V4L2_CID_USER_NEOISP_BASE (V4L2_CID_USER_BASE + 0x1240)
#endif

LOG_DEFINE_CATEGORY(NxpNeoDev)

namespace nxpneo {

/**
 * \struct PipeConfig
 * \brief Configuration parameters for the Neo ISP device
 *
 * This structure contains the configuration settings that control the behavior
 * of the Neo ISP processing pipeline.
 */

/**
 * \var PipeConfig::topLines
 * \brief The number of top embedded data lines to crop from the input raw image
 */

/**
 * \enum NeoDevice::VideoDevice
 * \brief The video devices exposed by the ISP context
 *
 * \var NeoDevice::VideoDevice::Input0
 * \brief The input0 video device output node (long/main exposure)
 *
 * \var NeoDevice::VideoDevice::Input1
 * \brief The input1 video device output node (short exposure for HDR)
 *
 * \var NeoDevice::VideoDevice::Params
 * \brief The params video device output node (ISP parameters)
 *
 * \var NeoDevice::VideoDevice::Frame
 * \brief The frame video device capture node (Rgb/Yuv output pixels)
 *
 * \var NeoDevice::VideoDevice::Ir
 * \brief The Infrared video device capture node (Infrared output pixels)
 *
 * \var NeoDevice::VideoDevice::Stats
 * \brief The stats video device capture node (ISP statistics)
 */

namespace {

/* Keep those definitions in sync with NeoDevice::VideoDevice enum class. */
constexpr unsigned int kVideoDeviceCount = 6;
constexpr std::array<NeoDevice::VideoDevice, kVideoDeviceCount>
	kAllVideoDevices{
		NeoDevice::VideoDevice::Input0,
		NeoDevice::VideoDevice::Input1,
		NeoDevice::VideoDevice::Params,
		NeoDevice::VideoDevice::Frame,
		NeoDevice::VideoDevice::Ir,
		NeoDevice::VideoDevice::Stats,
	};

constexpr unsigned int kVideoDeviceMetaCount = 2;
constexpr std::array<NeoDevice::VideoDevice, kVideoDeviceMetaCount>
	kMetaVideoDevices{
		NeoDevice::VideoDevice::Params,
		NeoDevice::VideoDevice::Stats,
	};

constexpr unsigned int kVideoDeviceMutableCount = 5;
constexpr std::array<NeoDevice::VideoDevice, kVideoDeviceMutableCount>
	kMutableVideoDevices = {
		NeoDevice::VideoDevice::Input1,
		NeoDevice::VideoDevice::Params,
		NeoDevice::VideoDevice::Frame,
		NeoDevice::VideoDevice::Ir,
		NeoDevice::VideoDevice::Stats,
	};

bool isDeviceValid(NeoDevice::VideoDevice device)
{
	return std::find(kAllVideoDevices.begin(),
			 kAllVideoDevices.end(),
			 device) != kAllVideoDevices.end();
}

bool isDeviceMeta(NeoDevice::VideoDevice device)
{
	return std::find(kMetaVideoDevices.begin(),
			 kMetaVideoDevices.end(),
			 device) != kMetaVideoDevices.end();
}

bool isDeviceMutable(NeoDevice::VideoDevice device)
{
	return std::find(kMutableVideoDevices.begin(),
			 kMutableVideoDevices.end(),
			 device) != kMutableVideoDevices.end();
}

/*
 * Regex definitions for subdevices and video devices media entity names.
 * Support loose definition of entity name suffix when multiple instances are
 * present in the media device.
 */
const std::string kEntitySuffixRe = ".?[0-9]*$";
const std::string kSDevEntityRe = "^neoisp" + kEntitySuffixRe;
const std::string kVDevEntityInput0Re = "^neoisp-input0" + kEntitySuffixRe;
const std::string kVDevEntityInput1Re = "^neoisp-input1" + kEntitySuffixRe;
const std::string kVDevEntityParamsRe = "^neoisp-params" + kEntitySuffixRe;
const std::string kVDevEntityFrameRe = "^neoisp-frame" + kEntitySuffixRe;
const std::string kVDevEntityIrRe = "^neoisp-ir" + kEntitySuffixRe;
const std::string kVDevEntityStatsRe = "^neoisp-stats" + kEntitySuffixRe;

/* NEO ISP driver name. */
const std::string kNeoDriverName = "neoisp";

/* ISI front-end driver name. */
const std::string kIsiDriverName = "mxc-isi";

} /* namespace */

/**
 * \class NeoDevice
 * \brief Represents a context for a NEO hardware device
 *
 * A NeoDevice is an object instantiated for each ISP context operated over a
 * NEO hardware instance. Such ISP context shares the access to the hardware
 * resources to process a video stream.
 */

/**
 * \brief Construct a NEO ISP device context
 * \param[in] media The media device containing the NEO ISP entities
 * \param[in] subdevEntity The media entity representing the ISP subdevice
 *
 * This constructor initializes a NEO ISP device context by performing the
 * following operations:
 * - Validates the media device and subdevice entity parameters
 * - Opens the ISP subdevice associated with the provided entity
 * - Discovers and opens all video devices by following media links from the
 *   subdevice entity
 * - Queries the ISP hardware and driver capabilities
 *
 * The constructor validates that all expected video devices are found and
 * successfully opened. If any device is missing or fails to open, the object
 * is marked as invalid.
 */
NeoDevice::NeoDevice(std::shared_ptr<MediaDevice> media, MediaEntity *subdevEntity)
	: hwCapabilities_(0), supportedParamsBlocks_(kDefaultParamsSupported),
	  media_(media), valid_(false)
{
	if (!media || !subdevEntity)
		return;

	if (subdevEntity->function() != MEDIA_ENT_F_PROC_VIDEO_ISP)
		return;

	sdevEntity_ = subdevEntity;
	isp_ = std::move(V4L2Subdevice::fromEntityName(media.get(), subdevEntity->name()));
	int ret = isp_.get() ? isp_->open() : -ENODEV;
	if (ret) {
		LOG(NxpNeoDev, Error) << "Failed to open subdevice";
		return;
	}
	/* Discover video devices, identified by their entity name. */
	struct vdevEntry {
		std::string regex;
		std::unique_ptr<V4L2VideoDevice> *vdev;
	};
	const std::map<NeoDevice::VideoDevice, vdevEntry> vdevEntries{
		{ NeoDevice::VideoDevice::Input0, { kVDevEntityInput0Re, &input0_ } },
		{ NeoDevice::VideoDevice::Input1, { kVDevEntityInput1Re, &input1_ } },
		{ NeoDevice::VideoDevice::Params, { kVDevEntityParamsRe, &params_ } },
		{ NeoDevice::VideoDevice::Frame, { kVDevEntityFrameRe, &frame_ } },
		{ NeoDevice::VideoDevice::Ir, { kVDevEntityIrRe, &ir_ } },
		{ NeoDevice::VideoDevice::Stats, { kVDevEntityStatsRe, &stats_ } },
	};
	const std::vector<MediaPad *> pads = subdevEntity->pads();
	for (MediaPad *pad : pads) {
		if (pad->links().empty())
			continue;
		bool isPadSource = pad->flags() & MEDIA_PAD_FL_SOURCE;
		MediaLink *link = pad->links()[0];
		MediaEntity *linkedEntity = isPadSource
						    ? link->sink()->entity()
						    : link->source()->entity();
		if (linkedEntity->function() != MEDIA_ENT_F_IO_V4L)
			continue;
		for (const auto &[vdevId, entry] : vdevEntries) {
			std::regex re(entry.regex);
			if (std::regex_match(linkedEntity->name(), re)) {
				*entry.vdev =
					std::move(V4L2VideoDevice::fromEntityName(media.get(),
										  linkedEntity->name()));
				V4L2VideoDevice *vdev = entry.vdev->get();
				ret = vdev ? vdev->open() : -ENODEV;
				if (ret) {
					LOG(NxpNeoDev, Error)
						<< "Failed to open video device";
					return;
				}
				vdevEntities_.insert({ vdevId, linkedEntity });
				break;
			}
		}
	}
	if (vdevEntities_.size() != vdevEntries.size()) {
		LOG(NxpNeoDev, Error) << "Missing video devices";
		return;
	}
	/*
	 * Context capabilities and negotations.
	 * \todo Once API version negotiation is removed, consider
	 * moving these from context to device level.
	 */
	V4L2Subdevice *isp = isp_.get();
	/*
	 * Get Neo ISP hardware and driver capabilities.
	 * Fallback to MSB alignment for backward compatibility with older
	 * kernel drivers that don't expose V4L2_CID_NEOISP_QUERYCAP. This was
	 * the default behavior in earlier driver versions.
	 */
	const struct v4l2_query_ext_ctrl *ctlInfo = isp->controlInfo(V4L2_CID_NEOISP_QUERYCAP);
	if (ctlInfo != nullptr) {
		const std::array<uint32_t, 1> cids = { V4L2_CID_NEOISP_QUERYCAP };
		ControlList ctrls = isp->getControls(cids);
		if (!ctrls.empty())
			hwCapabilities_ =
				ctrls.get(V4L2_CID_NEOISP_QUERYCAP).get<int32_t>();
	} else {
		hwCapabilities_ = NEO_CAP_ALIGNMENT_MSB;
	}

	/*
	 * Get the Neo ISP blocks list
	 */
	ctlInfo = isp_->controlInfo(V4L2_CID_NEOISP_SUPPORTED_PARAMS_BLOCKS);
	if (ctlInfo != nullptr)
		supportedParamsBlocks_ = static_cast<uint64_t>(ctlInfo->default_value);

	valid_ = true;
}

/**
 * \brief Allocate buffers for all the NEO video devices
 * \param[in] bufferCount The number of buffers to allocate
 *
 * This function allocates and imports buffers for all configured NEO video
 * devices. It handles two types of buffer allocation:
 *
 * 1. **Pixel buffers**: These buffers are imported from external sources
 *
 * 2. **Metadata buffers**: These are allocated and exported to produce a buffer
 *      pool available to the class user. Then, buffer queues are configured for
 *      buffers import.
 *
 * The function only processes devices that were configured in the most recent
 * configure() call. If any allocation fails, all buffers are freed via
 * freeBuffers() to ensure a clean state.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENOMEM if buffer allocation or import fails
 */
int NeoDevice::allocateBuffers(unsigned int bufferCount)
{
	int ret = 0;
	LOG(NxpNeoDev, Debug) << logPrefix() << "Allocate buffers " << bufferCount;

	if (!valid_)
		return -EINVAL;

	auto createBuffers =
		[this](VideoDevice device,
		       std::vector<std::unique_ptr<FrameBuffer>> &buffers,
		       unsigned int count) {
			V4L2VideoDevice *video = videoDevice(device);
			ASSERT(video);
			int res = video->exportBuffers(count, &buffers);
			if (res < 0 || static_cast<unsigned int>(res) != count) {
				LOG(NxpNeoDev, Error)
					<< logPrefix() << "Failed to export buffers device "
					<< static_cast<int>(device);
				return -ENOMEM;
			}

			res = video->importBuffers(count);
			if (res) {
				LOG(NxpNeoDev, Error)
					<< logPrefix() << "Failed to import buffers device "
					<< static_cast<int>(device);
				return -ENOMEM;
			}
			return res;
		};

	const std::map<VideoDevice, std::vector<std::unique_ptr<FrameBuffer>> *>
		metaBuffers = {
			{ VideoDevice::Params, &paramsBuffers_ },
			{ VideoDevice::Stats, &statsBuffers_ },
		};

	for (auto device : configured_) {
		if (isDeviceMeta(device))
			continue;
		V4L2VideoDevice *video = videoDevice(device);
		ASSERT(video);
		ret = video->importBuffers(bufferCount);
		if (ret) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Failed to import buffers device "
				<< static_cast<int>(device);
			break;
		}
	}
	if (ret)
		goto done;

	for (auto device : kMetaVideoDevices) {
		auto buffers = metaBuffers.at(device);
		ret = createBuffers(device, *buffers, bufferCount);
		if (ret)
			break;
	}

done:
	if (ret)
		freeBuffers();

	return ret;
}

/**
 * \brief Release buffers for all the NEO video devices
 *
 * This function releases all buffers that were previously allocated by
 * allocateBuffers(). It performs the following operations:
 *
 * 1. **Metadata buffers**: Clears the internal metadata buffer vectors
 *
 * 2. **Video device buffers**: Calls releaseBuffers() on all configured
 *    video devices to release their V4L2 buffer queues.
 *
 * The function processes only devices that were configured in the most recent
 * configure() call. Any errors during buffer release are logged but do not
 * prevent the function from attempting to release buffers from remaining
 * devices.
 */
void NeoDevice::freeBuffers()
{
	LOG(NxpNeoDev, Debug) << logPrefix() << "Free buffers";

	if (!valid_)
		return;

	for (auto device : configured_) {
		V4L2VideoDevice *video = videoDevice(device);
		ASSERT(video);
		int ret = video->releaseBuffers();
		if (ret) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Failed to release buffers device "
				<< static_cast<int>(device);
		}
	}

	paramsBuffers_.clear();
	statsBuffers_.clear();
}

/**
 * \brief Configure NEO video devices according to their formats
 * \param[in] pipeConfig The ISP pipeline configuration
 * \param[in] formats Map of video devices to their corresponding V4L2 format
 *  configurations
 *
 * This function configures all NEO video devices based on the provided
 * format map. It performs the following operations:
 * - Validates that all required immutable devices have formats specified
 * - Configures pixel video devices (input0, input1, frame, ir) with their
 *   formats
 * - Enables links for mutable devices that are being configured
 * - Configures metadata video devices (params, stats) with appropriate API
 *   version
 * - Disables links for unconfigured mutable devices
 * - Applies crop settings to input devices to remove embedded data lines
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::configure(const PipeConfig &pipeConfig,
			 const std::map<VideoDevice, V4L2DeviceFormat *> &formats)
{
	int ret;

	if (!valid_)
		return -EINVAL;

	/* Validate that all required immutable devices have formats. */
	for (VideoDevice device : kAllVideoDevices) {
		if (isDeviceMutable(device))
			continue;
		if (formats.find(device) == formats.end()) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Missing format for immutable device "
				<< static_cast<int>(device);
			return -EINVAL;
		}
	}

	/* Configure pixel video devices. */
	std::vector<VideoDevice> configured;
	for (const auto [device, format] : formats) {
		if (isDeviceMeta(device)) {
			LOG(NxpNeoDev, Warning)
				<< logPrefix() << "Ignoring metadata format device "
				<< static_cast<int>(device);
			continue;
		}

		if (isDeviceMutable(device)) {
			ret = configureVideoDeviceLink(device, true);
			if (ret)
				return ret;
		}
		ret = configureVideoDevice(device, format);
		if (ret)
			return ret;

		configured.push_back(device);
	}

	/* Configure metadata video devices - mutable and always enabled. */
	for (VideoDevice device : kMetaVideoDevices) {
		ret = configureVideoDeviceLink(device, true);
		if (ret)
			return ret;
		ret = configureVideoDeviceMeta(device);
		if (ret)
			return ret;

		configured.push_back(device);
	}

	/* Disable the links for unconfigured mutable devices. */
	for (VideoDevice device : kMutableVideoDevices) {
		if (std::find(configured.begin(),
			      configured.end(), device) == configured.end()) {
			ret = configureVideoDeviceLink(device, false);
			if (ret)
				return ret;
		}
	}

	/* Set crop on enabled input. */
	auto setCropSelection =
		[this](VideoDevice device, const V4L2DeviceFormat *format, int topLines) {
			V4L2VideoDevice *video = videoDevice(device);
			ASSERT(video);
			Size sizeInput = format->size;
			Rectangle rect{ 0, topLines, sizeInput.width,
					sizeInput.height - topLines };
			int res = video->setSelection(V4L2_SEL_TGT_CROP, &rect);
			if (res)
				LOG(NxpNeoDev, Error)
					<< logPrefix() << "Failed to set crop selection device "
					<< static_cast<int>(device);
			return res;
		};

	for (auto device : { VideoDevice::Input0, VideoDevice::Input1 }) {
		auto it = formats.find(device);
		if (it != formats.end()) {
			const auto &[dev, format] = *it;
			ret = setCropSelection(device, format, pipeConfig.topLines);
			if (ret)
				return ret;
		}
	}

	configured_ = std::move(configured);

	return 0;
}

/**
 * \brief Start streaming on all configured NEO video devices
 *
 * This function initiates video streaming on the NEO ISP device by:
 * - Enabling frame start events on the ISP subdevice
 * - Starting streaming (streamOn) on all configured video devices
 *
 * The function iterates through all video devices that were configured during
 * the most recent configure() call and starts streaming on each one.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EBUSY if a device is already streaming
 * \retval -EINVAL if the device is not properly configured
 */
int NeoDevice::start()
{
	LOG(NxpNeoDev, Debug) << logPrefix() << "Start";

	if (!valid_)
		return -EINVAL;

	int ret = isp_->setFrameStartEnabled(true);
	if (ret) {
		LOG(NxpNeoDev, Error)
			<< logPrefix() << "FrameStart failure " << ret;
		return ret;
	}

	for (auto device : configured_) {
		V4L2VideoDevice *video = videoDevice(device);
		ASSERT(video);
		ret = video->streamOn();
		if (ret) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Failed to start NEO device "
				<< static_cast<int>(device);
			break;
		}
	}

	if (ret)
		stop();

	return ret;
}

/**
 * \brief Stop streaming on all configured NEO video devices
 *
 * This function stops video streaming on the NEO ISP device by:
 * - Disabling frame start events on the ISP subdevice
 * - Stopping streaming (streamOff) on all configured video devices
 *
 * The function iterates through all video devices that were configured during
 * the most recent configure() call and stops streaming on each one. Errors
 * from individual devices are logged but do not prevent the function from
 * attempting to stop remaining devices.
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::stop()
{
	LOG(NxpNeoDev, Debug) << logPrefix() << "Stop";

	if (!valid_)
		return -EINVAL;

	int ret = isp_->setFrameStartEnabled(false);
	if (ret) {
		LOG(NxpNeoDev, Error)
			<< logPrefix() << "FrameStart failure " << ret;
	}

	for (auto device : configured_) {
		V4L2VideoDevice *video = videoDevice(device);
		ASSERT(video);
		int res = video->streamOff();
		if (res) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Failed to stop NEO device "
				<< static_cast<int>(device);
			ret |= res;
		}
	}

	return ret;
}

/**
 * \brief Retrieve a specific Neo video device
 * \param[in] device The video device identifier
 *
 * This function returns a pointer to the V4L2VideoDevice corresponding to
 * the specified device identifier. That is an alternative to accessing
 * a video device directly through its pointer exposed in the public interface
 * of the class.
 *
 * \return Pointer to the requested V4L2VideoDevice
 */
V4L2VideoDevice *NeoDevice::videoDevice(VideoDevice device) const
{
	if (!valid_)
		return nullptr;

	const std::map<NeoDevice::VideoDevice, const std::unique_ptr<V4L2VideoDevice> *> vdevs{
		{ NeoDevice::VideoDevice::Input0, &input0_ },
		{ NeoDevice::VideoDevice::Input1, &input1_ },
		{ NeoDevice::VideoDevice::Params, &params_ },
		{ NeoDevice::VideoDevice::Frame, &frame_ },
		{ NeoDevice::VideoDevice::Ir, &ir_ },
		{ NeoDevice::VideoDevice::Stats, &stats_ },
	};
	auto it = vdevs.find(device);
	if (it == vdevs.end()) {
		LOG(NxpNeoDev, Error) << "Invalid device";
		return nullptr;
	}
	return it->second->get();
}

/**
 * \brief Get the subdevice entity name for the Neo ISP
 *
 * This function returns the media entity name for the Neo ISP subdevice.
 *
 * \return The Neo ISP subdevice entity name string
 */
const std::string &NeoDevice::subdeviceName() const
{
	static const std::string empty = "";

	if (!valid_)
		return empty;

	if (sdevEntity_)
		return sdevEntity_->name();

	LOG(NxpNeoDev, Error) << "Invalid subdevice";
	return empty;
}

/**
 * \brief Get the entity name for a Neo video device
 * \param[in] device The video device identifier
 *
 * This function returns the media entity name associated with the specified
 * Neo video device.
 *
 * \return The entity name string for valid devices, or an empty string for
 * invalid device identifiers
 */
const std::string &NeoDevice::videoDeviceName(VideoDevice device) const
{
	static const std::string empty = "";

	if (!valid_)
		return empty;

	auto it = vdevEntities_.find(device);
	if (it != vdevEntities_.end())
		return it->second->name();

	LOG(NxpNeoDev, Error) << "Invalid device " << static_cast<int>(device);
	return empty;
}

namespace {

std::vector<PixelFormat> queryPixelFormats(V4L2VideoDevice *device)
{
	V4L2VideoDevice::Formats deviceFormats = device->formats();
	std::vector<PixelFormat> formats;
	for (const auto &[format, ranges] : deviceFormats) {
		PixelFormat pixelFormat = format.toPixelFormat(false);
		if (pixelFormat.isValid())
			formats.push_back(pixelFormat);
	}
	return formats;
}

} /* namespace */

/**
 * \brief Get the supported pixel formats for a NEO capture video device
 * \param[in] device The video device identifier
 *
 * This function returns the list of supported pixel formats for the specified
 * NEO capture video device.
 *
 * The pixel formats are queried once during the first call and cached in
 * static variables for subsequent calls. The formats are obtained by querying
 * the underlying V4L2 video device and converting V4L2 formats to libcamera
 * PixelFormat objects.
 *
 * \return A reference to the vector of supported PixelFormat objects for the
 * specified device, or an empty vector if the device is invalid
 */
const std::vector<PixelFormat> &NeoDevice::capturePixelFormats(VideoDevice device) const
{
	static const std::vector<PixelFormat> frameFormats =
		queryPixelFormats(videoDevice(VideoDevice::Frame));
	static const std::vector<PixelFormat> irFormats =
		queryPixelFormats(videoDevice(VideoDevice::Ir));
	static const std::vector<PixelFormat> empty;

	if (!valid_)
		return empty;

	if (device == VideoDevice::Frame)
		return frameFormats;
	else if (device == VideoDevice::Ir)
		return irFormats;

	LOG(NxpNeoDev, Error)
		<< "Invalid capture pixel device " << static_cast<int>(device);
	return empty;
}

/**
 * \brief Report the supported V4L2 pixel formats on the output nodes
 * \param[in] device The video device identifier
 *
 * This function returns the supported V4L2 pixel formats for the output nodes.
 * These formats represent raw Bayer patterns and grayscale formats at various
 * bit depths.
 *
 * This function is static with a predefined list of V4L2 pixel formats as it
 * may be called during the early stages of the pipeline handler creation before
 * the NeoMediaDevice object is created.
 *
 * \return A reference to the vector of supported V4L2PixelFormat objects for
 * the relevant devices, or an empty vector if the device is invalid
 */
const std::vector<V4L2PixelFormat> &NeoDevice::outputFormats(VideoDevice device)
{
	static const std::vector<V4L2PixelFormat> outputFormats = {
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
		V4L2PixelFormat(V4L2_PIX_FMT_GREY),
		V4L2PixelFormat(V4L2_PIX_FMT_Y10),
		V4L2PixelFormat(V4L2_PIX_FMT_Y12),
		V4L2PixelFormat(V4L2_PIX_FMT_Y16),
	};
	static const std::vector<V4L2PixelFormat> empty;

	if (device == VideoDevice::Input0 || device == VideoDevice::Input1)
		return outputFormats;

	LOG(NxpNeoDev, Error)
		<< "Invalid output pixel device " << static_cast<int>(device);
	return empty;
}

/**
 * \brief Configure the media link for a NEO video device
 * \param[in] device The video device identifier
 * \param[in] enable True to enable the link, false to disable it
 *
 * This function enables or disables the media link associated with the
 * specified NEO video device. It retrieves the media entity corresponding
 * to the device, validates that it has exactly one pad with one link, and
 * then sets the link's enabled state.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV if the video device entity or its link is not found
 */
int NeoDevice::configureVideoDeviceLink(VideoDevice device, bool enable)
{
	unsigned int id = static_cast<int>(device);
	LOG(NxpNeoDev, Debug)
		<< logPrefix() << "Configure video device " << id
		<< " link status " << (enable ? "enabled" : "disabled");

	auto it = vdevEntities_.find(device);
	if (it == vdevEntities_.end()) {
		LOG(NxpNeoDev, Error) << "Device " << id << " not found";
		return -ENODEV;
	}
	MediaEntity *entity = it->second;
	if (!entity || (entity->pads().size()) != 1 ||
	    (entity->pads()[0]->links().size() != 1)) {
		LOG(NxpNeoDev, Error) << "Invalid device " << id;
		return -EINVAL;
	}

	MediaLink *link = entity->pads()[0]->links()[0];
	int ret = link->setEnabled(enable);
	if (ret)
		LOG(NxpNeoDev, Error) << "Error setting link device " << id;

	return ret;
}

/**
 * \brief Configure a NEO video device with the specified format
 * \param[in] device The video device identifier
 * \param[in,out] format The V4L2 device format to configure
 *
 * This function configures a pixel video device (input0, input1, frame, or ir)
 * with the provided format. Metadata devices (params, stats) are not supported
 * by this function and should be configured using configureVideoDeviceMeta()
 * instead.
 *
 * The function validates that the device is a valid pixel video device before
 * attempting to apply the format configuration.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL if the device is invalid or is a metadata device
 */
int NeoDevice::configureVideoDevice(VideoDevice device, V4L2DeviceFormat *format)
{
	unsigned int id = static_cast<int>(device);
	LOG(NxpNeoDev, Debug)
		<< logPrefix() << "Configure video device " << id
		<< " format " << format->toString();

	if (!isDeviceValid(device) || isDeviceMeta(device)) {
		LOG(NxpNeoDev, Error)
			<< "Invalid pixel video device " << id;
		return -EINVAL;
	}

	V4L2VideoDevice *video = videoDevice(device);
	ASSERT(video);
	int ret = video->setFormat(format);
	if (ret) {
		LOG(NxpNeoDev, Error)
			<< logPrefix() << "Failed to set video device format " << id;
	}

	return ret;
}

/**
 * \brief Configure a NEO metadata video device with API version-specific format
 * \param[in] device The video device identifier (must be a metadata device)
 *
 * This function configures a metadata video device (params or stats) with the
 * generic extensible format from v4l2-isp framework.
 *
 * The function validates that the device is a valid metadata device before
 * applying the format configuration. Pixel video devices (input0, input1,
 * frame, ir) are not supported by this function and should be configured
 * using configureVideoDevice() instead.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL if the device is invalid or is not a metadata device
 */
int NeoDevice::configureVideoDeviceMeta(VideoDevice device)
{
	unsigned int id = static_cast<int>(device);
	LOG(NxpNeoDev, Debug)
		<< logPrefix() << "Configure video device meta " << id;

	if (!isDeviceValid(device) || !isDeviceMeta(device)) {
		LOG(NxpNeoDev, Error)
			<< "Invalid metadata video device " << id;
		return -EINVAL;
	}

	V4L2DeviceFormat format = {};
	if (device == VideoDevice::Params) {
		format.fourcc = V4L2PixelFormat(V4L2_META_FMT_NEO_ISP_EXT_PARAMS);
		format.planes[0].size =
			offsetof(struct v4l2_isp_buffer, data) + NEOISP_EXT_PARAMS_MAX_SIZE;
	} else {
		format.fourcc = V4L2PixelFormat(V4L2_META_FMT_NEO_ISP_EXT_STATS);
		format.planes[0].size =
			offsetof(struct v4l2_isp_buffer, data) + NEOISP_EXT_STATS_MAX_SIZE;
	}

	V4L2VideoDevice *video = videoDevice(device);
	ASSERT(video);
	int ret = video->setFormat(&format);
	if (ret) {
		LOG(NxpNeoDev, Error)
			<< logPrefix() << "Failed to set video device format device " << id;
	}

	return ret;
}

/**
 * \class NeoMediaDevice
 * \brief Represents a media device that holds NEO ISP devices
 *
 * NeoMediaDevice represents a media device that contains the necessary entities
 * to operate a context of a NEO ISP hardware instance.
 * At media graph level, the device driver instantiates (i.e. duplicates) the
 * ISP entities subgraph for each context. Such instantiation will be replaced
 * at some point by the Media Controller context API.
 */

/**
 * \brief Construct a NeoMediaDevice from a media device
 * \param[in] media Shared pointer to the MediaDevice containing NEO entities
 *
 * This constructor initializes a NeoMediaDevice by discovering all NEO ISP
 * context subgraphs within the provided media device. Each context subgraph
 * is identified by its subdevice entity name.
 *
 * The actual NeoDevice instances are created on-demand via createDevice()
 * rather than during construction.
 */
NeoMediaDevice::NeoMediaDevice(std::shared_ptr<MediaDevice> media)
	: media_(media), valid_(false)
{
	/*
	 * Detect all context subgraphs - Each subgraph is identified by its
	 * subdevice entity name. The other devices will be discovered at
	 * creation time by following the sudevice media links.
	 */
	if (!media)
		return;
	const std::vector<MediaEntity *> &entities = media->entities();
	std::vector<MediaEntity *> sdevs;

	const std::regex re(kSDevEntityRe);
	std::copy_if(entities.begin(), entities.end(),
		     std::back_inserter(subdevs_),
		     [&re](MediaEntity *e) {
			     if (!e)
				     return false;
			     if (e->function() != MEDIA_ENT_F_PROC_VIDEO_ISP)
				     return false;
			     if (!std::regex_match(e->name(), re))
				     return false;
			     return true;
		     });

	unsigned int count = subdevs_.size();
	LOG(NxpNeoDev, Debug)
		<< "Media [" << media->deviceNode() << "] contexts " << count;

	valid_ = !!count;
}

/**
 * \brief Create and initialize a NEO device context
 *
 * This function creates a new NEO device context retrieving the next available
 * subdevice entity from the pool. If device creation or initialization fails,
 * nullptr is returned.
 *
 * \return A unique pointer to an initialized NeoDevice on success, or nullptr
 * if no subdevices are available or initialization fails
 */
std::unique_ptr<NeoDevice> NeoMediaDevice::createDevice()
{
	if (!valid_ || subdevs_.empty())
		return {};

	MediaEntity *subdev = subdevs_.front();
	subdevs_.pop_front();
	if (!subdev)
		return {};

	std::unique_ptr<NeoDevice> dev =
		std::make_unique<NeoDevice>(media_, subdev);

	if (!dev || !dev->isValid())
		return {};

	LOG(NxpNeoDev, Debug)
		<< "Allocated device from media " << media_->deviceNode()
		<< " subdev " << dev->isp_->deviceNode();
	return dev;
}

/**
 * \fn NeoMediaDevice::media()
 * \brief Get the shared pointer to the underlying media device
 *
 * This function returns the shared pointer to the MediaDevice that was
 * initialized during the construction. The MediaDevice represents the media
 * controller device that contains all the NEO ISP context subgraphs.
 *
 * \return A shared pointer to the MediaDevice, or nullptr if not initialized
 */

/**
 * \class NeoDeviceAllocator
 * \brief Allocate Neo device contexts from media devices
 *
 * The NeoDeviceAllocator is responsible for discovering and managing NEO ISP
 * device contexts across different kernel driver configurations. It handles
 * two distinct setups:
 *
 * **Modern configuration**: NEO ISP entities are embedded within the front-end
 * media device registered by the ISI driver. In this case, the allocator
 * creates a NeoMediaDevice from the front-end media device, which can then
 * allocate multiple NeoDevice contexts from the ISP entities it contains.
 *
 * **Legacy configuration**: Each NEO ISP instance has its own separate media
 * device registered by the NEO ISP driver. When no ISP entities are found in
 * the front-end media device, the allocator falls back to this legacy mode,
 * acquiring individual media devices (one per ISP context) as needed.
 *
 * The allocator is initialized during pipeline handler construction and
 * provides NeoDevice contexts on demand through createDevice().
 *
 * \todo Remove legacy implementation support when kernel transition is complete
 */

/**
 * \brief Construct a NeoDeviceAllocator
 * \param[in] pipeline The pipeline handler that owns this allocator
 * \param[in] enumerator The device enumerator for discovering media devices
 *
 * This constructor initializes the allocator by:
 * - Acquiring the front-end media device (ISI driver)
 * - Attempting to create a NeoMediaDevice from the front-end media device
 * - If successful, operating in modern mode with embedded ISP entities
 * - If no ISP entities are found, preparing for legacy mode fallback
 *
 * The allocator is marked valid if the front-end media device is successfully
 * acquired, regardless of whether it contains ISP entities (modern mode) or
 * requires legacy mode operation.
 */
NeoDeviceAllocator::NeoDeviceAllocator(PipelineHandler *pipeline,
				       DeviceEnumerator *enumerator)
	: pipeline_(pipeline), enumerator_(enumerator), valid_(false)
{
	if (!pipeline || !enumerator)
		return;
	DeviceMatch feMatch(kIsiDriverName);
	feMedia_ = pipeline->acquireMediaDevice(enumerator, feMatch);
	if (!feMedia_)
		LOG(NxpNeoDev, Debug) << "Front end media device not found";

	std::unique_ptr<NeoMediaDevice> neoMediaDev =
		std::make_unique<NeoMediaDevice>(feMedia_);
	if (neoMediaDev->isValid())
		feNeoMediaDevice_ = std::move(neoMediaDev);

	valid_ = true;
	LOG(NxpNeoDev, Debug)
		<< "Neo device allocator legacy mode " << !feNeoMediaDevice_;
}

/**
 * \brief Create and allocate a NEO device context
 *
 * This function creates a new NeoDevice context using the appropriate method
 * based on the detected configuration:
 *
 * **Modern mode**: If a NeoMediaDevice was successfully created from the
 * front-end media device, delegates to its createDevice() method to allocate
 * a context from the available ISP entities.
 *
 * **Legacy mode**: Acquires a new media device from the NEO ISP driver (one
 * media device per ISP instance), creates a temporary NeoMediaDevice from it,
 * and allocates a single NeoDevice context. The media device reference is
 * retained in legacyMedias_ to maintain ownership.
 *
 * \return A unique pointer to an initialized NeoDevice on success, or nullptr
 * if no devices are available or initialization fails
 */
std::unique_ptr<NeoDevice> NeoDeviceAllocator::createDevice()
{
	if (!valid_)
		return {};

	if (feNeoMediaDevice_)
		return feNeoMediaDevice_->createDevice();

	/*
	 * Fallback to the legacy Neo media devices with one ISP instance per
	 * media device. A reference to the underlying media device is kept.
	 */
	DeviceMatch match(kNeoDriverName);
	std::shared_ptr<MediaDevice> media =
		pipeline_->acquireMediaDevice(enumerator_, match);
	if (!media) {
		LOG(NxpNeoDev, Debug) << "No legacy media device";
		return {};
	}

	NeoMediaDevice neoMediadev{ media };
	if (!neoMediadev.isValid()) {
		LOG(NxpNeoDev, Error) << "Invalid legacy media device";
		return {};
	}

	std::unique_ptr<NeoDevice> dev = neoMediadev.createDevice();
	if (!dev || !dev->isValid()) {
		LOG(NxpNeoDev, Error) << "Invalid neo device";
		return {};
	}

	legacyMedias_.push_back(media);
	return dev;
}

/**
 * \fn NeoDeviceAllocator::feMedia()
 * \brief Get the front-end media device
 *
 * This function returns the shared pointer to the front-end MediaDevice
 * (ISI driver) that was acquired during construction.
 *
 * \return A shared pointer to the front-end MediaDevice
 */

/**
 * \fn NeoDeviceAllocator::isValid()
 * \brief Check if the allocator is valid and ready to create devices
 *
 * \return True if the allocator successfully acquired the front-end media
 * device, false otherwise
 */

} /* namespace nxpneo */

} /* namespace libcamera */
