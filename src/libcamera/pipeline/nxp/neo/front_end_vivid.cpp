/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2026 NXP
 *
 * VIVID-based Front End support for neo ISP pipeline
 */

#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <linux/videodev2.h>

#include <libcamera/base/utils.h>

#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "front_end.h"
#include "isi_device.h"
#include "media_graph.h"
#include "neo_device.h"
#include "neo_utils.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoFeVivid)

namespace nxpneo {

namespace {

/* VIVID media device definitions. */
const std::string kMediaDriverName = "vivid";
const std::string kVideoCaptureNameRe = "vivid-(\\d+)-vid-cap";
const std::string kVideoOutputNameRe = "vivid-(\\d+)-vid-out";

} /* namespace */

/**
 * \class CameraSensorVivid
 * \brief Virtual camera sensor implementation for VIVID-based front end
 *
 * The CameraSensorVivid class provides a CameraSensor implementation for the
 * VIVID (Virtual Video Test Driver) based front end. This class simulates a
 * camera sensor without requiring actual hardware, making it useful for testing
 * and development purposes.
 *
 * Unlike physical camera sensors, CameraSensorVivid does not interact with
 * V4L2 subdevices or media entities. Instead, it provides a software-based
 * implementation that supports configurable pixel formats, resolutions, and
 * multiple streams (Image0, Image1).
 *
 * The class supports:
 * - Multiple Bayer and monochrome pixel formats (8-16 bits per pixel)
 * - Configurable image resolutions
 * - Multiple output streams (primary image, auxiliary image)
 * - Simulated sensor controls (exposure, gain, blanking, pixel rate)
 * - Fixed sensor properties and delays for consistent behavior
 *
 * This implementation is primarily used in conjunction with the VIVID kernel
 * driver to provide a complete virtual camera for the Neo ISP pipeline.
 */
class CameraSensorVivid : public CameraSensor
{
public:
	CameraSensorVivid(
		const std::string &name,
		const std::map<FEStream, std::pair<PixelFormat, Size>> &formats);

	const std::string &model() const override;
	const std::string &id() const override;

	const MediaEntity *entity() const override;
	V4L2Subdevice *device() override;

	CameraLens *focusLens() override;

	const std::vector<unsigned int> &mbusCodes() const override;
	std::vector<Size> sizes(unsigned int mbusCode) const override;
	Size resolution() const override;

	V4L2SubdeviceFormat
	getFormat(Span<const unsigned int> mbusCodes,
		  const Size &size, const Size maxSize = Size()) const override;
	int setFormat(V4L2SubdeviceFormat *format,
		      Transform transform = Transform::Identity) override;
	int tryFormat(V4L2SubdeviceFormat *format) const;

	int applyConfiguration(const SensorConfiguration &config,
			       Transform transform = Transform::Identity,
			       V4L2SubdeviceFormat *sensorFormat = nullptr) override;

	V4L2Subdevice::Stream imageStream() const override;
	std::optional<V4L2Subdevice::Stream> embeddedDataStream() const override;
	V4L2SubdeviceFormat embeddedDataFormat() const override;
	int setEmbeddedDataEnabled(bool enable) override;
	std::optional<V4L2Subdevice::Stream> auxiliaryStream() const override;
	V4L2SubdeviceFormat auxiliaryFormat() const override;
	int setAuxiliaryEnabled(bool enable) override;

	const ControlList &properties() const override;
	int sensorInfo(IPACameraSensorInfo *info) const override;
	Transform computeTransform(Orientation *orientation) const override;
	BayerFormat::Order bayerOrder(Transform t) const override;
	Orientation mountingOrientation() const override;

	const ControlInfoMap &controls() const override;
	ControlList getControls(Span<const uint32_t> ids) override;
	int setControls(ControlList *ctrls) override;

	const std::vector<controls::draft::TestPatternModeEnum> &
	testPatternModes() const override;
	int setTestPatternMode(controls::draft::TestPatternModeEnum mode) override;
	const CameraSensorProperties::SensorDelays &sensorDelays() override;

	bool isValid() const { return valid_; }

private:
	bool valid_;
	std::string name_;
	std::map<FEStream, V4L2SubdeviceFormat> streamFormats_;
	std::vector<unsigned int> mbusCodes_;
	ControlInfoMap controlInfoMap_;
	BayerFormat bayerFormat_;
	ControlList properties_;
};

/**
 * \class V4L2VideoDeviceVivid
 * \brief Extended V4L2VideoDevice for VIVID driver with direct ioctl access
 *
 * The V4L2VideoDeviceVivid class extends V4L2VideoDevice to provide
 * specialized functionality for the VIVID (Virtual Video Test Driver) kernel
 * driver.
 *
 * The sole extension provided by this class is exposing the protected
 * ioctl() method from the base V4L2Device class, allowing direct V4L2 ioctl
 * operations on VIVID video devices. This is necessary for configuring
 * VIVID-specific controls and settings that are not exposed through the
 * standard V4L2VideoDevice interface, such as:
 * - HDMI input/output connector selection (VIDIOC_S_INPUT, VIDIOC_S_OUTPUT)
 * - DV timings configuration (VIDIOC_S_DV_TIMINGS, VIDIOC_ENUM_DV_TIMINGS)
 * - Test pattern generator controls
 * - HDMI loopback configuration
 */
class V4L2VideoDeviceVivid : public V4L2VideoDevice
{
public:
	V4L2VideoDeviceVivid(const std::string &deviceNode)
		: V4L2VideoDevice(deviceNode){};
	V4L2VideoDeviceVivid(const MediaEntity *entity)
		: V4L2VideoDevice(entity){};
	~V4L2VideoDeviceVivid() = default;

	static std::unique_ptr<V4L2VideoDeviceVivid>
	fromEntityName(const MediaDevice *media, const std::string &entity);

	using V4L2Device::ioctl;
};

class FrontEndCameraVivid : public FrontEndCamera
{
public:
	FrontEndCameraVivid() {}
	~FrontEndCameraVivid() = default;

	const std::string &name() const override;
	CameraSensor *sensor() const override;

	const Attributes &attributes() const override;

	Orientation validateOrientation(Orientation orientation) const override;

	const std::vector<FEStream> &streams() const override;
	bool hasStream(FEStream stream) const override;
	V4L2VideoDevice *videoDevice(FEStream stream) const override;

	int configure(
		V4L2SubdeviceFormat &subdevFormat, Transform transform,
		std::map<FEStream, V4L2DeviceFormat> *videoFormats,
		const std::map<FEStream, V4L2DeviceFormat> *processedVideoFormats) override;

	const Formats &formats() const override;
	const std::vector<NeoDevice *> &neoDevices() const override;

	/* Subclass definitions. */
	int init(const FrontEndHandler::MatchParams &matchParams, unsigned int index);

	std::unique_ptr<CameraSensorVivid> sensor_;
	Formats formats_;

private:
	struct StreamEntry;
	int initVividConfig(const FrontEndHandler::MatchParams &matchParams,
			    const CameraProperties::VividConfig &instance,
			    FEStream stream);
	int configureInstance(FEStream stream);
	int configureInstanceTpg(const StreamEntry &entry);
	int configureInstanceLoopback(const StreamEntry &entry);

	std::string name_;
	Attributes attributes_;
	std::vector<FEStream> streams_;
	std::vector<NeoDevice *> neoDevices_;
	std::vector<std::unique_ptr<NeoDevice>> neoDevicesOwned_;

	struct StreamEntry {
		std::shared_ptr<MediaDevice> media;
		MediaEntity *captureEntity;
		MediaEntity *outputEntity;
		V4L2DeviceFormat deviceFormat;
		std::unique_ptr<V4L2VideoDeviceVivid> captureDevice;
		std::unique_ptr<V4L2VideoDeviceVivid> outputDevice;
		unsigned int mediaInstance;
		CameraProperties::VividConfig vividConfig;
	};
	std::map<FEStream, StreamEntry> streamEntries_;
};

class FrontEndHandlerVivid : public FrontEndHandler
{
public:
	FrontEndHandlerVivid(const std::string &name)
		: FrontEndHandler(name){};
	~FrontEndHandlerVivid() = default;

	bool match(const MatchParams &params) override;
	const std::vector<FrontEndCamera *> &cameras() const override;
	int acquireDevice(const std::string &name) override;
	void releaseDevice(const std::string &name) override;

private:
	std::vector<std::unique_ptr<FrontEndCameraVivid>> vividCameras_;
	std::vector<FrontEndCamera *> cameras_;
	static unsigned int cameraCount_;
};

/*
 * CameraSensorVivid implementation
 */

namespace {

/* Mandatory sensor control definitions. */
const Control<int32_t> v4l2AnalogueGain(
	V4L2_CID_ANALOGUE_GAIN, "AnalogueGain", "libcamera", ControlId::Direction::Out);
const Control<int32_t> v4l2Exposure(
	V4L2_CID_EXPOSURE, "Exposure", "libcamera", ControlId::Direction::Out);
const Control<int32_t> v4l2HBlank(
	V4L2_CID_HBLANK, "HBlank", "libcamera", ControlId::Direction::Out);
const Control<int32_t> v4l2VBlank(
	V4L2_CID_VBLANK, "VBlank", "libcamera", ControlId::Direction::Out);
const Control<int32_t> v4l2PixelRate(
	V4L2_CID_PIXEL_RATE, "PixelRate", "libcamera", ControlId::Direction::Out);

const ControlIdMap vividControlIdMap{
	{ V4L2_CID_ANALOGUE_GAIN, &v4l2AnalogueGain },
	{ V4L2_CID_EXPOSURE, &v4l2Exposure },
	{ V4L2_CID_HBLANK, &v4l2HBlank },
	{ V4L2_CID_VBLANK, &v4l2VBlank },
	{ V4L2_CID_PIXEL_RATE, &v4l2PixelRate },
};

/* Controls info min, max and default set to dummy values. */
const ControlInfoMap::Map vividControlInfoMap{
	{ &v4l2AnalogueGain, ControlInfo(1, 1, 1) },
	{ &v4l2Exposure, ControlInfo(1, 1, 1) },
	{ &v4l2HBlank, ControlInfo(1, 1, 1) },
	{ &v4l2VBlank, ControlInfo(1, 1, 1) },
	{ &v4l2PixelRate, ControlInfo(1, 1, 1) },
};

const CameraSensorProperties::SensorDelays vividSensorDelays{
	.exposureDelay = 2,
	.gainDelay = 1,
	.vblankDelay = 2,
	.hblankDelay = 0,
};

const std::string kVividCameraNamePrefix = "vivid-camera:";
const std::string kVividCameraModel = "vivid";

} /* namespace */

CameraSensorVivid::CameraSensorVivid(
	const std::string &name,
	const std::map<FEStream, std::pair<PixelFormat, Size>> &formats)
	: valid_(false), name_(name)
{
	/*
	 * Pixel format to media bus code conversion
	 * \todo Add meta formats
	 */
	static const std::map<PixelFormat, unsigned int> pixelFormatCode{
		{ formats::SRGGB8, MEDIA_BUS_FMT_SRGGB8_1X8 },
		{ formats::SGRBG8, MEDIA_BUS_FMT_SGRBG8_1X8 },
		{ formats::SGBRG8, MEDIA_BUS_FMT_SGBRG8_1X8 },
		{ formats::SBGGR8, MEDIA_BUS_FMT_SBGGR8_1X8 },
		{ formats::SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10 },
		{ formats::SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10 },
		{ formats::SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10 },
		{ formats::SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10 },
		{ formats::SRGGB12, MEDIA_BUS_FMT_SRGGB12_1X12 },
		{ formats::SGRBG12, MEDIA_BUS_FMT_SGRBG12_1X12 },
		{ formats::SGBRG12, MEDIA_BUS_FMT_SGBRG12_1X12 },
		{ formats::SBGGR12, MEDIA_BUS_FMT_SBGGR12_1X12 },
		{ formats::SRGGB14, MEDIA_BUS_FMT_SRGGB14_1X14 },
		{ formats::SGRBG14, MEDIA_BUS_FMT_SGRBG14_1X14 },
		{ formats::SGBRG14, MEDIA_BUS_FMT_SGBRG14_1X14 },
		{ formats::SBGGR14, MEDIA_BUS_FMT_SBGGR14_1X14 },
		{ formats::SRGGB16, MEDIA_BUS_FMT_SRGGB16_1X16 },
		{ formats::SGRBG16, MEDIA_BUS_FMT_SGRBG16_1X16 },
		{ formats::SGBRG16, MEDIA_BUS_FMT_SGBRG16_1X16 },
		{ formats::SBGGR16, MEDIA_BUS_FMT_SBGGR16_1X16 },
		{ formats::R8, MEDIA_BUS_FMT_Y8_1X8 },
		{ formats::R10, MEDIA_BUS_FMT_Y10_1X10 },
		{ formats::R12, MEDIA_BUS_FMT_Y12_1X12 },
		{ formats::R16, MEDIA_BUS_FMT_Y16_1X16 },
	};

	if (!formats.count(FEStream::Image0)) {
		LOG(NxpNeoFeVivid, Error)
			<< "No Image0 stream format provided";
		return;
	}

	for (const auto &[stream, formatPair] : formats) {
		if (!(stream == FEStream::Image0 || stream == FEStream::Image1)) {
			LOG(NxpNeoFeVivid, Error)
				<< "Unsupported stream: " << static_cast<int>(stream);
			continue;
		}

		const auto &[pixelFormat, size] = formatPair;
		auto it = pixelFormatCode.find(pixelFormat);
		if (it == pixelFormatCode.end()) {
			LOG(NxpNeoFeVivid, Error)
				<< "Unsupported pixel format: " << pixelFormat;
			return;
		}

		if (size.isNull()) {
			LOG(NxpNeoFeVivid, Error)
				<< "Invalid size for stream";
			return;
		}

		V4L2SubdeviceFormat &sdFormat = streamFormats_[stream];
		unsigned int code = it->second;
		sdFormat.code = code;
		sdFormat.size = size;

		if (stream == FEStream::Image0) {
			mbusCodes_.push_back(code);
			bayerFormat_ = BayerFormat::fromMbusCode(code);
		}
	}

	/* Add mandatory sensor controls. */
	ControlInfoMap::Map sensorControlInfoMap = vividControlInfoMap;
	controlInfoMap_ = { std::move(sensorControlInfoMap), vividControlIdMap };

	/* Create sensor properties. */
	properties_ = ControlList{ properties::properties };
	properties_.set(properties::Model, kVividCameraModel);
	properties_.set(properties::Location, properties::CameraLocationExternal);
	properties_.set(properties::Rotation, 0);

	const Size &size0 = streamFormats_[FEStream::Image0].size;
	properties_.set(properties::PixelArraySize, size0);
	properties_.set(properties::PixelArrayActiveAreas, { Rectangle{ size0 } });

	int32_t cfa;
	switch (bayerFormat_.order) {
	case BayerFormat::BGGR:
		cfa = properties::draft::BGGR;
		break;
	case BayerFormat::GBRG:
		cfa = properties::draft::GBRG;
		break;
	case BayerFormat::GRBG:
		cfa = properties::draft::GRBG;
		break;
	case BayerFormat::RGGB:
		cfa = properties::draft::RGGB;
		break;
	case BayerFormat::MONO:
		cfa = properties::draft::MONO;
		break;
	default:
		LOG(NxpNeoFeVivid, Error) << "Unknown Bayer format order";
		cfa = properties::draft::BGGR;
		break;
	}
	properties_.set(properties::draft::ColorFilterArrangement, cfa);

	valid_ = true;
}

const std::string &CameraSensorVivid::model() const
{
	return kVividCameraModel;
}

const std::string &CameraSensorVivid::id() const
{
	return name_;
}

const MediaEntity *CameraSensorVivid::entity() const
{
	return nullptr;
}

V4L2Subdevice *CameraSensorVivid::device()
{
	return nullptr;
}

CameraLens *CameraSensorVivid::focusLens()
{
	return nullptr;
}

const std::vector<unsigned int> &CameraSensorVivid::mbusCodes() const
{
	return mbusCodes_;
}

std::vector<Size> CameraSensorVivid::sizes(unsigned int mbusCode) const
{
	V4L2SubdeviceFormat format = streamFormats_.at(FEStream::Image0);
	if (format.code != mbusCode)
		return {};

	return { format.size };
}

Size CameraSensorVivid::resolution() const
{
	V4L2SubdeviceFormat format = streamFormats_.at(FEStream::Image0);
	return format.size;
}

V4L2SubdeviceFormat
CameraSensorVivid::getFormat([[maybe_unused]] Span<const unsigned int> mbusCodes,
			     [[maybe_unused]] const Size &size,
			     [[maybe_unused]] const Size maxSize) const
{
	return streamFormats_.at(FEStream::Image0);
}

int CameraSensorVivid::setFormat(V4L2SubdeviceFormat *format,
				 [[maybe_unused]] Transform transform)
{
	return tryFormat(format);
}

int CameraSensorVivid::tryFormat(V4L2SubdeviceFormat *format) const
{
	if (format)
		*format = streamFormats_.at(FEStream::Image0);
	return 0;
}

int CameraSensorVivid::applyConfiguration(
	[[maybe_unused]] const SensorConfiguration &config,
	[[maybe_unused]] Transform transform,
	V4L2SubdeviceFormat *sensorFormat)
{
	if (!sensorFormat)
		return 0;

	return tryFormat(sensorFormat);
}

V4L2Subdevice::Stream CameraSensorVivid::imageStream() const
{
	return V4L2Subdevice::Stream{ 0, 0 };
}

std::optional<V4L2Subdevice::Stream> CameraSensorVivid::embeddedDataStream() const
{
	auto it = streamFormats_.find(FEStream::EData);
	if (it == streamFormats_.end())
		return std::nullopt;

	return V4L2Subdevice::Stream{ 0, 1 };
}

V4L2SubdeviceFormat CameraSensorVivid::embeddedDataFormat() const
{
	auto it = streamFormats_.find(FEStream::EData);
	if (it == streamFormats_.end())
		return {};

	return it->second;
}

int CameraSensorVivid::setEmbeddedDataEnabled([[maybe_unused]] bool enable)
{
	auto it = streamFormats_.find(FEStream::EData);
	if (it == streamFormats_.end())
		return -EINVAL;

	return 0;
}

std::optional<V4L2Subdevice::Stream> CameraSensorVivid::auxiliaryStream() const
{
	auto it = streamFormats_.find(FEStream::Image1);
	if (it == streamFormats_.end())
		return std::nullopt;

	return V4L2Subdevice::Stream{ 0, 2 };
}

V4L2SubdeviceFormat CameraSensorVivid::auxiliaryFormat() const
{
	auto it = streamFormats_.find(FEStream::Image1);
	if (it == streamFormats_.end())
		return {};

	return it->second;
}

int CameraSensorVivid::setAuxiliaryEnabled([[maybe_unused]] bool enable)
{
	auto it = streamFormats_.find(FEStream::Image1);
	if (it == streamFormats_.end())
		return -EINVAL;

	return 0;
}

const ControlList &CameraSensorVivid::properties() const
{
	return properties_;
}

int CameraSensorVivid::sensorInfo(IPACameraSensorInfo *info) const
{
	if (!info)
		return -EINVAL;

	info->model = model();

	const V4L2SubdeviceFormat &format = streamFormats_.at(FEStream::Image0);
	info->activeAreaSize = format.size;
	info->analogCrop = Rectangle(format.size);
	info->outputSize = format.size;

	info->bitsPerPixel = MediaBusFormatInfo::info(format.code).bitsPerPixel;
	info->outputSize = format.size;

	static const std::map<BayerFormat::Order,
			      properties::draft::ColorFilterArrangementEnum>
		cfaPatternMap = {
			{ BayerFormat::Order::BGGR, properties::draft::BGGR },
			{ BayerFormat::Order::GBRG, properties::draft::GBRG },
			{ BayerFormat::Order::GRBG, properties::draft::GRBG },
			{ BayerFormat::Order::RGGB, properties::draft::RGGB },
			{ BayerFormat::Order::MONO, properties::draft::MONO },
		};

	auto it = cfaPatternMap.find(bayerFormat_.order);
	if (bayerFormat_.isValid() && it != cfaPatternMap.end())
		info->cfaPattern = it->second;
	else
		info->cfaPattern = properties::draft::RGB;

	/* Dummy values to avoid division by zero. */
	info->pixelRate = 1;
	info->minLineLength = 1;
	info->maxLineLength = 1;
	info->minFrameLength = 1;
	info->maxFrameLength = 1;

	return 0;
}

Transform CameraSensorVivid::computeTransform(Orientation *orientation) const
{
	if (orientation)
		*orientation = Orientation::Rotate0;
	return Transform::Identity;
}

BayerFormat::Order CameraSensorVivid::bayerOrder(
	[[maybe_unused]] Transform t) const
{
	return bayerFormat_.order;
}

Orientation CameraSensorVivid::mountingOrientation() const
{
	return Orientation::Rotate0;
}

const ControlInfoMap &CameraSensorVivid::controls() const
{
	return controlInfoMap_;
}

ControlList CameraSensorVivid::getControls(Span<const uint32_t> ids)
{
	ControlList controlList(controlInfoMap_);
	/* Reports control values as default. */
	for (uint32_t id : ids) {
		auto it = controlInfoMap_.find(id);
		if (it != controlInfoMap_.end()) {
			const ControlInfo &info = it->second;
			controlList.set(id, info.def());
		} else {
			LOG(NxpNeoFeVivid, Warning)
				<< "Control " << id << " not found in map";
		}
	}
	return controlList;
}

int CameraSensorVivid::setControls([[maybe_unused]] ControlList *ctrls)
{
	return 0;
}

const std::vector<controls::draft::TestPatternModeEnum> &
CameraSensorVivid::testPatternModes() const
{
	static const std::vector<controls::draft::TestPatternModeEnum> patterns;
	return patterns;
}

int CameraSensorVivid::setTestPatternMode(
	[[maybe_unused]] controls::draft::TestPatternModeEnum mode)
{
	return 0;
}

const CameraSensorProperties::SensorDelays &CameraSensorVivid::sensorDelays()
{
	return vividSensorDelays;
}

/*
 * V4L2VideoDeviceVivid implementation
 */

std::unique_ptr<V4L2VideoDeviceVivid>
V4L2VideoDeviceVivid::fromEntityName(const MediaDevice *media, const std::string &entity)
{
	MediaEntity *videoEntity = media->getEntityByName(entity);
	if (!videoEntity)
		return nullptr;

	return std::make_unique<V4L2VideoDeviceVivid>(videoEntity->deviceNode());
}

/*
 * FrontEndCameraVivid implementation
 */

const std::string &FrontEndCameraVivid::name() const
{
	return name_;
}

CameraSensor *FrontEndCameraVivid::sensor() const
{
	return sensor_.get();
}

const FrontEndCamera::Attributes &FrontEndCameraVivid::attributes() const
{
	return attributes_;
}

Orientation FrontEndCameraVivid::validateOrientation(Orientation orientation) const
{
	return orientation;
}

const std::vector<FEStream> &FrontEndCameraVivid::streams() const
{
	return streams_;
}

bool FrontEndCameraVivid::hasStream(FEStream stream) const
{
	return streamEntries_.count(stream);
}

V4L2VideoDevice *FrontEndCameraVivid::videoDevice(FEStream stream) const
{
	auto it = streamEntries_.find(stream);
	if (it == streamEntries_.end()) {
		LOG(NxpNeoFeVivid, Error) << "Invalid stream";
		return nullptr;
	}

	const StreamEntry &entry = it->second;
	return entry.captureDevice.get();
}

int FrontEndCameraVivid::configure(
	V4L2SubdeviceFormat &subdevFormat, Transform transform,
	std::map<FEStream, V4L2DeviceFormat> *videoFormats,
	[[maybe_unused]] const std::map<FEStream, V4L2DeviceFormat> *processedVideoFormats)
{
	/* Configure the Vivid instances. */
	for (FEStream stream : streams_) {
		int ret = configureInstance(stream);
		if (ret)
			return ret;
	}

	/* Configure test sensor and video device. */
	int ret = sensor_->setFormat(&subdevFormat, transform);
	if (ret)
		return ret;

	if (videoFormats)
		videoFormats->clear();

	for (const auto &[stream, entry] : streamEntries_) {
		if (!entry.captureDevice)
			continue;
		V4L2DeviceFormat formatLocal;
		V4L2DeviceFormat &formatRef =
			videoFormats ? (*videoFormats)[stream] : formatLocal;
		formatRef = entry.deviceFormat;
		ret = entry.captureDevice->setFormat(&formatRef);
		if (ret) {
			LOG(NxpNeoFeVivid, Error)
				<< "Failed to set format on capture device";
			return ret;
		}
	}

	return 0;
}

const FrontEndCamera::Formats &FrontEndCameraVivid::formats() const
{
	return formats_;
}

const std::vector<NeoDevice *> &FrontEndCameraVivid::neoDevices() const
{
	return neoDevices_;
}

int FrontEndCameraVivid::init(
	const FrontEndHandler::MatchParams &matchParams, unsigned int index)
{
	int ret;

	if (!matchParams.pipeline ||
	    !matchParams.enumerator ||
	    !matchParams.neoAllocator)
		return -EINVAL;

	name_ = kVividCameraNamePrefix + std::to_string(index);
	const CameraProperties &cameraProperties =
		matchParams.pipelineConfig->cameraProperties(name_, kVividCameraModel);
	const auto &instances = cameraProperties.vividInstances;

	/*
	 * At least one vivid instance for the image0 stream is required.
	 * Stream for image1 is created only if it is declared in the camera
	 * properties and it has an associated vivid instance configuration.
	 * \todo add support for embedded data stream.
	 */
	if (!instances.has_value() || instances->empty()) {
		LOG(NxpNeoFeVivid, Debug)
			<< "No vivid instances configured for camera " << name_;
		return -ENODEV;
	}

	if (instances->size() > 2) {
		LOG(NxpNeoFeVivid, Error) << "Too many vivid instances configured";
		return -EINVAL;
	}

	constexpr std::array<FEStream, 2>
		kImageStreams{ FEStream::Image0, FEStream::Image1 };
	std::map<FEStream, std::pair<PixelFormat, Size>> sensorStreamFormats;
	for (FEStream stream : kImageStreams) {
		if (stream == FEStream::Image1 && !cameraProperties.image1Stream)
			continue;

		unsigned int streamIndex = static_cast<unsigned int>(stream);
		if (streamIndex >= instances->size()) {
			LOG(NxpNeoFeVivid, Error)
				<< "No instance configuration for stream " << streamIndex;
			return -EINVAL;
		}

		const CameraProperties::VividConfig &instance =
			instances->at(streamIndex);
		ret = initVividConfig(matchParams, instance, stream);
		if (ret)
			return ret;

		streams_.push_back(stream);
		sensorStreamFormats[stream] = { instance.pixelFormat, instance.size };
	}

	/* Create a Vivid camera with the selected streams. */
	sensor_ = std::make_unique<CameraSensorVivid>(name_, sensorStreamFormats);
	if (!sensor_->isValid())
		return -EINVAL;

	/* Populate front-end formats. */
	const auto &[pixelFormat0, size0] = sensorStreamFormats.at(FEStream::Image0);
	unsigned int code = sensor_->mbusCodes()[0];
	formats_.mbusCodeSizesMap[code].push_back(size0);
	formats_.sizeMbusCodesMap[size0].push_back(code);
	formats_.mbusCodePixelFormatsMap[code].push_back(pixelFormat0);

	/* Populate attributes. */
	attributes_.ispBypass = false;
	attributes_.rgbIrCfa = cameraProperties.rgbirCfa;

	/* Allocate ISP instance(s) - 2 instances for RGBIr dual context. */
	bool isRgbIr = attributes_.rgbIrCfa && streams_.size() == 2;
	unsigned int count = isRgbIr ? 2 : 1;
	for (unsigned int i = 0; i < count; ++i) {
		std::unique_ptr<NeoDevice> neo =
			matchParams.neoAllocator->createDevice();
		if (!neo) {
			LOG(NxpNeoFeVivid, Error)
				<< "Failed to allocate NeoDevice instance";
			return -ENODEV;
		}
		neoDevices_.push_back(neo.get());
		neoDevicesOwned_.push_back(std::move(neo));
	}

	std::stringstream ss;
	ss << "Camera: " << name_ << " ";
	ss << pixelFormat0.toString() << "/" << size0.toString() << " ";
	for (const auto &[stream, entry] : streamEntries_) {
		ss << "[stream:" << static_cast<int>(stream);
		if (entry.captureEntity)
			ss << " capture:" << entry.captureEntity->deviceNode();
		if (entry.outputEntity)
			ss << " output:" << entry.outputEntity->deviceNode();
		ss << "] ";
	}
	LOG(NxpNeoFeVivid, Info) << ss.str();

	return 0;
}

int FrontEndCameraVivid::initVividConfig(
	const FrontEndHandler::MatchParams &matchParams,
	const CameraProperties::VividConfig &instance,
	FEStream stream)
{
	/* Check the validity of vivid instances parameters for that stream. */
	const PixelFormat &pixelFormat = instance.pixelFormat;
	const BayerFormat &bayer =
		BayerFormat::fromPixelFormat(pixelFormat);
	if (!bayer.isValid()) {
		LOG(NxpNeoFeVivid, Error)
			<< "Invalid pixel format: " << pixelFormat.toString();
		return -EINVAL;
	}

	if ((bayer.bitDepth < 8) || (bayer.bitDepth > 16)) {
		LOG(NxpNeoFeVivid, Error)
			<< "Unsupported bits per pixel value: " << bayer.bitDepth;
		return -EINVAL;
	}

	const Size &size = instance.size;
	if (size.isNull() || size.width > NeoDevice::kRawWidthMax ||
	    size.width % NeoDevice::kWidthAlignment) {
		LOG(NxpNeoFeVivid, Error) << "Invalid size: " << size.toString();
		return -EINVAL;
	}

	PipelineHandler *pipeline = matchParams.pipeline;
	DeviceEnumerator *enumerator = matchParams.enumerator;

	StreamEntry &entry = streamEntries_[stream];

	/* Save properties for later use at configure() time. */
	entry.vividConfig = instance;

	/* Output device is required only in case of loopback. */
	std::regex videoDeviceCaptureRe{ kVideoCaptureNameRe };
	std::regex videoDeviceOutputRe{ kVideoOutputNameRe };
	DeviceMatch dm(kMediaDriverName);
	dm.add(videoDeviceCaptureRe);
	if (instance.loopback)
		dm.add(videoDeviceOutputRe);

	std::shared_ptr<MediaDevice> &media = entry.media;
	media = pipeline->acquireMediaDevice(enumerator, dm);
	if (!media) {
		LOG(NxpNeoFeVivid, Debug)
			<< "No vivid media device found for stream "
			<< static_cast<int>(stream);
		return -ENODEV;
	}

	entry.captureEntity = media->getEntityByName(videoDeviceCaptureRe);
	if (!entry.captureEntity) {
		LOG(NxpNeoFeVivid, Error) << "Capture entity not found";
		return -ENODEV;
	}

	/* Extract vivid instance from the capture video device name. */
	std::smatch match;
	const std::string &captureName = entry.captureEntity->name();
	std::regex_search(captureName, match, videoDeviceCaptureRe);
	if (match.size() < 2) {
		LOG(NxpNeoFeVivid, Error)
			<< "Invalid capture device entity name " << captureName;
		return -ENODEV;
	}
	entry.mediaInstance = static_cast<unsigned int>(std::stoi(match[1]));

	/* Capture (input) device configuration. */
	entry.captureDevice =
		V4L2VideoDeviceVivid::fromEntityName(media.get(), captureName);
	if (!entry.captureDevice) {
		LOG(NxpNeoFeVivid, Error) << "Failed to create capture device";
		return -ENODEV;
	}

	V4L2VideoDevice *vdev = entry.captureDevice.get();
	int ret = vdev->open();
	if (ret) {
		LOG(NxpNeoFeVivid, Error) << "Failed to open capture video device";
		return ret;
	}

	V4L2DeviceFormat &devFormat = entry.deviceFormat;
	devFormat.fourcc = vdev->toV4L2PixelFormat(pixelFormat);
	devFormat.size = size;

	/* Optional output device configuration for loopback. */
	if (!instance.loopback)
		return 0;

	entry.outputEntity = media->getEntityByName(videoDeviceOutputRe);
	if (!entry.outputEntity) {
		LOG(NxpNeoFeVivid, Error) << "Output entity not found";
		return -ENODEV;
	}

	const std::string &outputName = entry.outputEntity->name();
	entry.outputDevice =
		V4L2VideoDeviceVivid::fromEntityName(media.get(), outputName);
	if (!entry.outputDevice) {
		LOG(NxpNeoFeVivid, Error) << "Failed to create output device";
		return -ENODEV;
	}

	vdev = entry.outputDevice.get();
	ret = vdev->open();
	if (ret) {
		LOG(NxpNeoFeVivid, Error) << "Failed to open output video device";
		return ret;
	}

	return 0;
};

namespace {

/**
 * \brief Set the input connector for a VIVID video capture device
 * \param[in] vdev The VIVID video device to configure
 * \param[in] connector The name of the input connector to select
 * \param[out] index The index of the selected input connector
 *
 * This function enumerates all available input connectors on the specified
 * VIVID video capture device and selects the one matching the provided name.
 * The input connector is typically an HDMI input instance used for capturing
 * video data.
 *
 * The function iterates through all available inputs using VIDIOC_ENUMINPUT
 * until it finds a match with the requested connector name. Once found, it
 * sets the input using VIDIOC_S_INPUT and returns the connector index.
 *
 * \return 0 on success, negative error code on failure
 * \retval -ENODEV if vdev is null
 */
int setInputConnector(V4L2VideoDeviceVivid *vdev,
		      const std::string &connector, unsigned int &index)
{
	if (!vdev)
		return -ENODEV;

	v4l2_input input{};
	std::string name;
	for (input.index = 0;; input.index++) {
		int ret = vdev->ioctl(VIDIOC_ENUMINPUT, &input);
		if (ret) {
			LOG(NxpNeoFeVivid, Error)
				<< "'" << connector << "' input not found";
			return ret;
		}
		name = std::string(reinterpret_cast<char *>(input.name));
		if (connector == name)
			break;
	}

	index = input.index;
	int ret = vdev->ioctl(VIDIOC_S_INPUT, &index);
	if (ret) {
		LOG(NxpNeoFeVivid, Error)
			<< "Failed to set '" << connector << "' input";
		return ret;
	}
	LOG(NxpNeoFeVivid, Debug)
		<< "Set input to '" << connector << "' index " << index;

	return 0;
}

/**
 * \brief Set the output connector for a VIVID video output device
 * \param[in] vdev The VIVID video device to configure
 * \param[in] connector The name of the output connector to select
 * \param[out] index The index of the selected output connector
 *
 * This function enumerates all available output connectors on the specified
 * VIVID video output device and selects the one matching the provided name.
 * The output connector is typically an HDMI output instance used for
 * transmitting video data in loopback configurations.
 *
 * The function iterates through all available outputs using VIDIOC_ENUMOUTPUT
 * until it finds a match with the requested connector name. Once found, it
 * sets the output using VIDIOC_S_OUTPUT and returns the connector index.
 *
 * \return 0 on success, negative error code on failure
 * \retval -ENODEV if vdev is null
 * \retval <0 if the connector is not found or setting the output fails
 */
int setOutputConnector(V4L2VideoDeviceVivid *vdev,
		       const std::string &connector, unsigned int &index)
{
	if (!vdev)
		return -ENODEV;

	v4l2_output output{};
	std::string name;
	for (output.index = 0;; output.index++) {
		int ret = vdev->ioctl(VIDIOC_ENUMOUTPUT, &output);
		if (ret) {
			LOG(NxpNeoFeVivid, Error)
				<< "'" << connector << "' output not found";
			return ret;
		}
		name = std::string(reinterpret_cast<char *>(output.name));
		if (connector == name)
			break;
	}

	index = output.index;
	int ret = vdev->ioctl(VIDIOC_S_OUTPUT, &index);
	if (ret) {
		LOG(NxpNeoFeVivid, Error)
			<< "Failed to set '" << connector << "' output";
		return ret;
	}
	LOG(NxpNeoFeVivid, Debug)
		<< "Set output to '" << name << "' index " << index;

	return 0;
}

/**
 * \brief Find a V4L2 control ID by its name
 * \param[in] vdev The VIVID video device to query
 * \param[in] controlName The name of the control to find
 *
 * This function searches through the control information map of the specified
 * VIVID video device to find a control matching the given name. The control
 * name is compared against the name field of each control in the device's
 * control map.
 *
 * \return The control ID if found, std::nullopt otherwise
 */
std::optional<unsigned int> controlIdByName(
	V4L2VideoDeviceVivid *vdev, const std::string &controlName)
{
	const auto &infoMap = vdev->controls();

	auto it = std::find_if(
		infoMap.begin(), infoMap.end(),
		[&controlName](const auto &pair) {
			return pair.first->name() == controlName;
		});

	if (it == infoMap.end()) {
		LOG(NxpNeoFeVivid, Error)
			<< "Control '" << controlName << "' not found";
		return std::nullopt;
	}

	return { it->first->id() };
}

/**
 * \brief Set a V4L2 extended control on a VIVID video device
 * \param[in] vdev The VIVID video device to configure
 * \param[in] id The V4L2 control ID to set
 * \param[in] value The integer value to set for the control
 *
 * This function sets a single V4L2 extended control on the specified VIVID
 * video device using the VIDIOC_S_EXT_CTRLS ioctl. The control class is
 * automatically determined from the control ID using V4L2_CTRL_ID2CLASS.
 *
 * \return 0 on success, negative error code on failure
 */
int setControl(V4L2VideoDeviceVivid *vdev, unsigned int id, int value)
{
	v4l2_ext_control ec{};
	ec.id = id;
	ec.value = value;

	v4l2_ext_controls ecs{};
	ecs.ctrl_class = V4L2_CTRL_ID2CLASS(id);
	ecs.count = 1;
	ecs.controls = &ec;

	int ret = vdev->ioctl(VIDIOC_S_EXT_CTRLS, &ecs);
	if (ret) {
		LOG(NxpNeoFeVivid, Error)
			<< "Failed to set control " << utils::hex(id)
			<< " value " << value;
	}

	return ret;
}

/**
 * \brief Set a V4L2 menu control on a VIVID video device by string value
 * \param[in] vdev The VIVID video device to configure
 * \param[in] controlName The name of the menu control to set
 * \param[in] value The string value of the menu item to select
 *
 * This function sets a V4L2 menu-type control on the specified VIVID video
 * device by matching the provided string value against the available menu
 * items. Unlike integer-based control setting, this function allows selecting
 * menu items by their human-readable names.
 *
 * The function first looks up the control ID by name, then verifies that the
 * control is of type V4L2_CTRL_TYPE_MENU. It enumerates all available menu
 * items using VIDIOC_QUERYMENU to find the index corresponding to the
 * requested value string. Once found, it sets the control to that index.
 *
 * \return 0 on success, negative error code on failure
 * \retval -EINVAL if the control is not found or is not a menu type
 * \retval -ENOENT if the menu item value is not found
 */
int setMenuControlByString(
	V4L2VideoDeviceVivid *vdev,
	const std::string &controlName, const std::string &value)
{
	std::optional<unsigned int> id = controlIdByName(vdev, controlName);
	if (!id.has_value())
		return -EINVAL;

	/*
	 * Find the menu item index matching the value string. Those are not
	 * directly available from libcamera classes, so we need to query them
	 * via V4L2 commands.
	 */
	const v4l2_query_ext_ctrl *ext = vdev->controlInfo(id.value());
	if (!ext || ext->type != V4L2_CTRL_TYPE_MENU) {
		LOG(NxpNeoFeVivid, Error)
			<< "Control '" << controlName << "' is not a menu";
		return -EINVAL;
	}

	std::optional<int> menuIndex;
	for (int index = ext->minimum; index <= ext->maximum; ++index) {
		v4l2_querymenu menu{};
		menu.id = id.value();
		menu.index = index;
		int ret = vdev->ioctl(VIDIOC_QUERYMENU, &menu);
		if (ret)
			continue;
		std::string name(reinterpret_cast<const char *>(menu.name));
		if (name == value) {
			menuIndex = index;
			break;
		}
	}

	if (!menuIndex.has_value()) {
		LOG(NxpNeoFeVivid, Error)
			<< "Menu item '" << value << "' not found for control '"
			<< controlName << "'";
		return -ENOENT;
	}

	LOG(NxpNeoFeVivid, Debug)
		<< "Set menu control '" << controlName
		<< "' value '" << value << "'";

	return setControl(vdev, id.value(), menuIndex.value());
}

/**
 * \brief Set a V4L2 menu control on a VIVID video device by integer value
 * \param[in] vdev The VIVID video device to configure
 * \param[in] controlName The name of the menu control to set
 * \param[in] value The integer index of the menu item to select
 *
 * This function sets a V4L2 menu-type control on the specified VIVID video
 * device using a direct integer index value. Unlike setMenuControlByString(),
 * which matches menu items by their human-readable names, this function
 * directly sets the control to the specified index value.
 *
 * The function first looks up the control ID by name, then verifies that the
 * control is of type V4L2_CTRL_TYPE_MENU. It validates that the provided
 * value falls within the control's minimum and maximum range before setting
 * the control to that index.
 *
 * This is useful when the menu item index is known in advance, such as when
 * using predefined constants or configuration values that directly correspond
 * to menu indices (e.g., test pattern modes, movement directions).
 *
 * \return 0 on success, negative error code on failure
 * \retval -EINVAL if the control is not found, is not a menu type, or the
 *                 value is out of range
 */
int setMenuControlByValue(
	V4L2VideoDeviceVivid *vdev, const std::string &controlName, int value)
{
	std::optional<unsigned int> id = controlIdByName(vdev, controlName);
	if (!id.has_value())
		return -EINVAL;

	const v4l2_query_ext_ctrl *ext = vdev->controlInfo(id.value());
	if (!ext || ext->type != V4L2_CTRL_TYPE_MENU) {
		LOG(NxpNeoFeVivid, Error)
			<< "Control '" << controlName << "' is not a menu";
		return -EINVAL;
	}

	if (value > ext->maximum || value < ext->minimum) {
		LOG(NxpNeoFeVivid, Error)
			<< "Menu value " << value << " out of range for control '"
			<< controlName << "'";
		return -EINVAL;
	}

	LOG(NxpNeoFeVivid, Debug)
		<< "Set menu control '" << controlName
		<< "' value " << value;

	return setControl(vdev, id.value(), value);
}

/**
 * \brief Select appropriate DV timings for a given image size
 * \param[in] vdev The VIVID video device to query for available DV timings
 * \param[in] size The requested image size (width and height)
 *
 * This function searches through all available DV (Digital Video) timings
 * supported by the specified VIVID video device to find the most appropriate
 * timing configuration for the requested image size.
 *
 * The selection algorithm prioritizes:
 * 1. Progressive scan timings only (interlaced formats are excluded)
 * 2. BT.656/1120 format (the only DV timing type supported by V4L2)
 * 3. Timings with dimensions at least as large as the requested size
 * 4. Among valid candidates, the smallest resolution is preferred
 * 5. For equal resolutions, the lowest frame rate is selected
 *
 * The function enumerates all available DV timings using VIDIOC_ENUM_DV_TIMINGS
 * and calculates the frame rate for each timing based on the pixel clock and
 * total horizontal/vertical dimensions (including blanking periods).
 *
 * \return The selected v4l2_dv_timings structure if a suitable timing is found,
 *         std::nullopt otherwise
 */
std::optional<v4l2_dv_timings>
selectDvTimings(V4L2VideoDeviceVivid *vdev, const Size &size)
{
	if (!vdev)
		return std::nullopt;

	/*
	 * Look for the progressive timings with the lowest fps at least as
	 * large as the requested size. Format BT656/1120 is the only one
	 * supported in v4l2 as of now.
	 */
	double minFps = std::numeric_limits<double>::max();
	unsigned minIndex = std::numeric_limits<unsigned int>::max();
	std::optional<v4l2_dv_timings> dvTimings;
	v4l2_enum_dv_timings dvTimingsEnum{};
	dvTimingsEnum.index = 0;
	for (dvTimingsEnum.index = 0;; dvTimingsEnum.index++) {
		if (vdev->ioctl(VIDIOC_ENUM_DV_TIMINGS, &dvTimingsEnum))
			break;

		if (dvTimingsEnum.timings.type != V4L2_DV_BT_656_1120)
			continue;

		v4l2_bt_timings &bt = dvTimingsEnum.timings.bt;
		if (bt.interlaced != V4L2_DV_PROGRESSIVE)
			continue;

		if (bt.width < size.width || bt.height < size.height)
			continue;

		unsigned int htotal =
			bt.width + bt.hfrontporch + bt.hsync + bt.hbackporch;
		unsigned int vtotal =
			bt.height + bt.vfrontporch + bt.vsync + bt.vbackporch;
		double fps = static_cast<double>(bt.pixelclock) /
			     (static_cast<double>(htotal) * vtotal);

		/*
		 * Select current timings profile if:
		 * - No profile is selected yet
		 * - Profile has a smaller resolution than current selection
		 * - Profile has the same resolution but a lower fps
		 */
		unsigned int currentPixels = bt.width * bt.height;
		unsigned int selectedPixels =
			dvTimings.has_value()
				? dvTimings->bt.width * dvTimings->bt.height
				: std::numeric_limits<unsigned int>::max();
		if (currentPixels < selectedPixels ||
		    (currentPixels == selectedPixels && fps < minFps)) {
			minFps = fps;
			minIndex = dvTimingsEnum.index;
			dvTimings = dvTimingsEnum.timings;
		}
	}

	if (!dvTimings.has_value()) {
		LOG(NxpNeoFeVivid, Error) << "No suitable DV timings found";
		return std::nullopt;
	}

	LOG(NxpNeoFeVivid, Debug)
		<< "DV timings selected for size " << size.toString()
		<< " index " << minIndex << " " << dvTimings->bt.width
		<< "x" << dvTimings->bt.height << "@" << minFps;

	return dvTimings;
}

/**
 * \brief Set DV timings on a VIVID video device
 * \param[in] vdev The VIVID video device to configure
 * \param[in] dvTimings The DV timings structure to apply
 *
 * This function configures the Digital Video (DV) timings on the specified
 * VIVID video device using the VIDIOC_S_DV_TIMINGS ioctl. DV timings define
 * the video format parameters including resolution, frame rate, and blanking
 * intervals for digital video interfaces such as HDMI.
 *
 * This is commonly used in conjunction with selectDvTimings() to configure
 * VIVID devices for specific resolutions and frame rates in both test pattern
 * generator and HDMI loopback modes.
 *
 * \return 0 on success, negative error code on failure
 * \retval -ENODEV if vdev is null
 * \retval -EINVAL if setting the DV timings fails
 */
int setDvTimings(V4L2VideoDeviceVivid *vdev, const v4l2_dv_timings &dvTimings)
{
	if (!vdev)
		return -ENODEV;

	v4l2_dv_timings timings = dvTimings;
	if (vdev->ioctl(VIDIOC_S_DV_TIMINGS, &timings)) {
		LOG(NxpNeoFeVivid, Error) << "Failed to set DV timings";
		return -EINVAL;
	}

	LOG(NxpNeoFeVivid, Debug)
		<< "DV timings set " << dvTimings.bt.width << "x" << dvTimings.bt.height;
	return 0;
}

/**
 * \brief Generate a standardized HDMI instance name for VIVID devices
 * \param[in] vividInstance The VIVID media device instance number
 * \param[in] hdmiInstance The HDMI connector instance number within the VIVID
 * device
 *
 * This function creates a standardized name string for HDMI input/output
 * connectors associated with VIVID (Virtual Video Test Driver) devices.
 * The naming format follows the pattern "HDMI XXX-Y" where XXX is the
 * zero-padded VIVID instance number (3 digits) and Y is the HDMI instance
 * number.
 *
 * \return A string containing the formatted HDMI instance name
 */
std::string hdmiInstanceName(unsigned int vividInstance,
			     unsigned int hdmiInstance)
{
	char buffer[64];
	std::snprintf(buffer, sizeof(buffer), "HDMI %03d-%d",
		      vividInstance, hdmiInstance);
	return std::string(buffer);
}

/**
 * \brief Enable or disable HDMI loopback on a VIVID video device
 * \param[in] vdev The VIVID video device to configure
 * \param[in] vividInstance The VIVID media device instance number
 * \param[in] enable True to enable loopback, false to connect to test pattern
 * generator
 *
 * This function configures the HDMI loopback mode on a VIVID (Virtual Video
 * Test Driver) device by controlling the connection between HDMI input and
 * output instances.
 *
 * When loopback is enabled, the HDMI input instance 0 is connected to the
 * HDMI output instance 0 within the same VIVID instance, creating a loopback
 * path where video data output from the device is fed back to its input.
 * This is useful for testing video pipelines without external hardware.
 *
 * When loopback is disabled, the HDMI input is instead connected to the
 * internal test pattern generator, allowing the device to produce synthetic
 * video patterns for testing purposes.
 *
 * \return 0 on success, negative error code on failure
 * \retval <0 if the control cannot be found or set
 */
int setHdmiLoopback(
	V4L2VideoDeviceVivid *vdev, unsigned int vividInstance, bool enable)
{
	/*
	 * Loopback on a given vivid instance is created by connecting the
	 * HDMI output instance 0 to the HDMI input instance 0, both HDMI
	 * instances belonging to that same vivid instance.
	 * When loopback is disabled, the HDMI input is connected to the test
	 * pattern generator.
	 */
	const std::string hdmiInstance = hdmiInstanceName(vividInstance, 0);
	const std::string controlName = hdmiInstance + " Is Connected To";
	const std::string value =
		enable ? "Output " + hdmiInstance : "Test Pattern Generator";

	return setMenuControlByString(vdev, controlName, value);
}

} /* namespace */

int FrontEndCameraVivid::configureInstance(FEStream stream)
{
	const auto it = streamEntries_.find(stream);
	if (it == streamEntries_.end())
		return -ENODEV;
	const StreamEntry &entry = it->second;

	const auto &config = entry.vividConfig;
	bool loopback = config.loopback;
	int ret;
	if (loopback)
		ret = configureInstanceLoopback(entry);
	else
		ret = configureInstanceTpg(entry);

	if (ret)
		return ret;

	V4L2VideoDeviceVivid *vdevCapture = entry.captureDevice.get();
	if (!vdevCapture)
		return -ENODEV;

	LOG(NxpNeoFeVivid, Info)
		<< name_ << " [" << static_cast<int>(stream) << "] "
		<< "capture " << vdevCapture->deviceNode()
		<< " [" << config.pixelFormat.toString()
		<< " " << config.size.toString() << "]"
		<< (loopback ? " (loopback)" : " (tpg)");

	if (loopback) {
		V4L2VideoDeviceVivid *vdevOutput = entry.outputDevice.get();
		if (!vdevOutput)
			return -ENODEV;

		LOG(NxpNeoFeVivid, Info)
			<< name_ << " [" << static_cast<int>(stream) << "] "
			<< "output " << vdevOutput->deviceNode();
	}

	return 0;
}

int FrontEndCameraVivid::configureInstanceTpg(const StreamEntry &entry)
{
	const CameraProperties::VividConfig &config = entry.vividConfig;

	/*
	 * Capture device configurations:
	 * - V4L2 device HDMI connector and DV timings
	 * - Test Pattern Generator control configurations
	 *     Default values come from Linux driver vivid-ctrls.c
	 * - HDMI input instance connected to the pattern generator
	 */
	V4L2VideoDeviceVivid *vdevCapture = entry.captureDevice.get();
	if (!vdevCapture)
		return -ENODEV;

	unsigned int inputIndex;
	const std::string hdmiInstance =
		hdmiInstanceName(entry.mediaInstance, 0);
	int ret = setInputConnector(vdevCapture, hdmiInstance, inputIndex);
	if (ret)
		return ret;

	std::optional<v4l2_dv_timings> dv_timings =
		selectDvTimings(vdevCapture, config.size);
	if (!dv_timings.has_value())
		return -EINVAL;

	ret = setDvTimings(vdevCapture, dv_timings.value());
	if (ret)
		return ret;

	/* OSD mode values - 0:"All" 1:"Counters only" 2:"None". */
	ret = setMenuControlByString(vdevCapture, "OSD Text Mode", "None");
	if (ret)
		return ret;

	/* TPG pattern: default to "100% Colorbar" (1)  */
	ret = setMenuControlByValue(vdevCapture, "Test Pattern",
				    config.tpgPattern);
	if (ret)
		return ret;

	/* Horizontal pattern movement: default to "None" (3) */
	ret = setMenuControlByValue(vdevCapture, "Horizontal Movement",
				    config.tpgHorizontalMovement);
	if (ret)
		return ret;

	/* Vertical pattern movement: default to "None" (3) */
	ret = setMenuControlByValue(vdevCapture, "Vertical Movement",
				    config.tpgVerticalMovement);
	if (ret)
		return ret;

	/* Loopback disabled on the capture device (TPG). */
	ret = setHdmiLoopback(vdevCapture, entry.mediaInstance, false);
	if (ret)
		return ret;

	return 0;
}

int FrontEndCameraVivid::configureInstanceLoopback(const StreamEntry &entry)
{
	const CameraProperties::VividConfig &config = entry.vividConfig;

	/*
	 * Capture and output devices configurations:
	 * - V4L2 devices HDMI connector and DV timings
	 * - HDMI input (capture) instance connected to the HDMI output instance
	 *   for the loopback mode
	 */
	V4L2VideoDeviceVivid *vdevCapture = entry.captureDevice.get();
	V4L2VideoDeviceVivid *vdevOutput = entry.outputDevice.get();
	if (!vdevCapture || !vdevOutput)
		return -ENODEV;

	/* OSD mode values - 0:"All" 1:"Counters only" 2:"None". */
	int ret = setMenuControlByString(vdevCapture, "OSD Text Mode", "None");
	if (ret)
		return ret;

	const std::string hdmiInstance =
		hdmiInstanceName(entry.mediaInstance, 0);

	unsigned int inputIndex;
	ret = setInputConnector(vdevCapture, hdmiInstance, inputIndex);
	if (ret)
		return ret;
	unsigned int outputIndex;
	ret = setOutputConnector(vdevOutput, hdmiInstance, outputIndex);
	if (ret)
		return ret;

	std::optional<v4l2_dv_timings> dv_timings =
		selectDvTimings(vdevCapture, config.size);
	if (!dv_timings.has_value())
		return -EINVAL;

	ret = setDvTimings(vdevCapture, dv_timings.value());
	if (ret)
		return ret;

	ret = setDvTimings(vdevOutput, dv_timings.value());
	if (ret)
		return ret;

	/* Loopback enabled on the capture device. */
	ret = setHdmiLoopback(vdevCapture, entry.mediaInstance, true);
	if (ret)
		return ret;

	return 0;
}

/*
 * FrontEndHandlerVivid implementation
 */

/*
 * \var FrontEndHandlerVivid::cameraCount_
 * \brief Counter of the number of vivid-based cameras registered.
 *
 * As the match() function may be called multiple times, it is necessary to
 * track the number of vivid cameras already created in order to keep a
 * distinct name based on index, for the new cameras being detected.
 */
unsigned int FrontEndHandlerVivid::cameraCount_ = 0;

bool FrontEndHandlerVivid::match([[maybe_unused]] const MatchParams &params)
{
	unsigned int i = 0;
	while (true) {
		std::unique_ptr<FrontEndCameraVivid> feCamera =
			std::make_unique<FrontEndCameraVivid>();
		int ret = feCamera->init(params, cameraCount_ + i);
		if (ret)
			break;

		cameras_.push_back(feCamera.get());
		vividCameras_.push_back(std::move(feCamera));
		i++;
	}

	cameraCount_ += i;
	return !!i;
}

const std::vector<FrontEndCamera *> &FrontEndHandlerVivid::cameras() const
{
	return cameras_;
}

int FrontEndHandlerVivid::acquireDevice([[maybe_unused]] const std::string &name)
{
	return 0;
}

void FrontEndHandlerVivid::releaseDevice([[maybe_unused]] const std::string &name)
{
	return;
}

REGISTER_FRONT_END_HANDLER("frontend-vivid", FrontEndHandlerVivid);

} /* namespace nxpneo */

} /* namespace libcamera */
