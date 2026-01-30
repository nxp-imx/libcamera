/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2026 NXP
 *
 * ISI-based Front End support for neo ISP pipeline
 */

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <libcamera/base/utils.h>
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "front_end.h"
#include "isi_device.h"
#include "media_graph.h"
#include "neo_device.h"
#include "neo_utils.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoFeIsi)

namespace nxpneo {

class FrontEndCameraIsi : public FrontEndCamera
{
public:
	FrontEndCameraIsi()
		: rawCamera_(true), sharedGraph_(false) {}
	~FrontEndCameraIsi() = default;

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
	struct InitParams {
		MediaDevice *mediaDevice;
		MediaEntity *sensorEntity;
		ISIDevice *isiDevice;
		PadStreamsMap *deviceStreamMap;
	};
	int init(const FrontEndHandler::MatchParams &matchParams,
		 const InitParams &initParams);
	int configureFrontEnd(V4L2SubdeviceFormat &sdFormat, Transform transform);
	bool sharedGraph() { return sharedGraph_; }
	int setSharedGraph();

	struct StreamEntry {
		ISIPipe *isiPipe;
		StreamGraph streamGraph;
		V4L2SubdeviceFormat pipeSubdevFormat;
	};
	std::map<FEStream, StreamEntry> streamEntries_;

	std::unique_ptr<CameraSensor> sensor_;
	std::map<MediaEntity *, V4L2Subdevice::Routing> routings_;
	Formats formats_;
	std::optional<Orientation> defaultOrientation_;

private:
	int enumerateFormatsRaw(const CameraProperties &properties);
	int enumerateFormatsYuv(const CameraProperties &properties);
	int populateVideoFormats();

	std::string name_;
	Attributes attributes_;
	std::vector<FEStream> streams_;
	std::vector<NeoDevice *> neoDevices_;
	std::vector<std::unique_ptr<NeoDevice>> neoDevicesOwned_;
	bool rawCamera_;
	bool sharedGraph_;
};

class FrontEndHandlerIsi : public FrontEndHandler
{
public:
	FrontEndHandlerIsi(const std::string &name)
		: FrontEndHandler(name){};
	~FrontEndHandlerIsi() = default;

	bool match(const MatchParams &params) override;
	const std::vector<FrontEndCamera *> &cameras() const override;
	int acquireDevice(const std::string &name) override;
	void releaseDevice(const std::string &name) override;

private:
	int setupRoutings() const;
	int setupSharedGraphs();

	std::shared_ptr<MediaDevice> media_;
	std::unique_ptr<ISIDevice> isiDevice_;
	std::vector<std::unique_ptr<FrontEndCameraIsi>> isiCameras_;
	std::vector<FrontEndCamera *> cameras_;
	std::map<MediaEntity *, V4L2Subdevice::Routing> routings_;

	unsigned int useCount_ = 0;
};

namespace {

/**
 * \brief Get the sizes supported by a camera for a mbus code
 * \param[in] sensor The camera sensor
 * \param[in] code The mbus code
 * \param[out] sizes The sizes supported by \a sensor for \a code
 *
 * This is essentially a wrapper around the CameraSensor::sizes() function.
 * It introduces a workaround for external ISP that exposes a single
 * (min, max) size range corresponding to its full rescaling range capability.
 * The CameraSensor::sizes() function reports only the max value of the range
 * that may exceed the width limit of the ISI or ISP devices.
 * For that case, make sure that at least the sensor native resolution is
 * reported in the sizes list, to be considered as a valid size option.
 */
void cameraSizesFixup(CameraSensor *sensor, int code, std::vector<Size> &sizes)
{
	sizes.clear();
	sizes = sensor->sizes(code);
	Size resolution = sensor->resolution();
	if (sizes.size() == 1 &&
	    std::find(sizes.begin(), sizes.end(), resolution) == sizes.end()) {
		sizes.push_back(std::move(resolution));
	}
	std::sort(sizes.begin(), sizes.end());
}

/**
 * \brief Merge routing maps of media entities
 * \param[in,out] base The base routing map to merge into
 * \param[in] delta The delta routing map containing routings to merge
 *
 * Merge the routings for entities in the delta map into the base map of
 * routings. For each entity in the delta map, if the entity already exists
 * in the base map, append the routing entries to the existing routing.
 * Otherwise, add the entity and its routing as a new entry in the base map.
 */
void mergeRoutingMap(std::map<MediaEntity *, V4L2Subdevice::Routing> &base,
		     const std::map<MediaEntity *, V4L2Subdevice::Routing> &delta)
{
	for (const auto &[entity, routing] : delta) {
		auto &dest = base[entity];
		dest.insert(dest.end(), routing.begin(), routing.end());
	}
}

} /* namespace */

const std::string &FrontEndCameraIsi::name() const
{
	static const std::string defaultName = "undefined";
	if (sensor_)
		return sensor_->entity()->name();
	else
		return defaultName;
}

CameraSensor *FrontEndCameraIsi::sensor() const
{
	return sensor_.get();
}

const FrontEndCamera::Attributes &FrontEndCameraIsi::attributes() const
{
	return attributes_;
}

Orientation FrontEndCameraIsi::validateOrientation(Orientation orientation) const
{
	if (defaultOrientation_.has_value())
		return defaultOrientation_.value();

	if (sharedGraph_) {
		LOG(NxpNeoFeIsi, Warning)
			<< "Default orientation expected for a shared graph";
	}

	return orientation;
}

const std::vector<FEStream> &FrontEndCameraIsi::streams() const
{
	return streams_;
}

bool FrontEndCameraIsi::hasStream(FEStream stream) const
{
	return streamEntries_.count(stream);
}

V4L2VideoDevice *FrontEndCameraIsi::videoDevice(FEStream stream) const
{
	auto it = streamEntries_.find(stream);
	if (it == streamEntries_.end()) {
		LOG(NxpNeoFeIsi, Error) << "Invalid stream";
		return nullptr;
	}
	const StreamEntry &entry = it->second;
	ISIPipe *pipe = entry.isiPipe;
	if (!pipe) {
		LOG(NxpNeoFeIsi, Error)
			<< "ISIPipe not initialized for stream";
		return nullptr;
	}
	V4L2VideoDevice *videoDevice = pipe->capture_.get();
	if (!videoDevice)
		LOG(NxpNeoFeIsi, Error)
			<< "Video device not available for stream";
	return videoDevice;
}

int FrontEndCameraIsi::configure(
	V4L2SubdeviceFormat &subdevFormat, Transform transform,
	std::map<FEStream, V4L2DeviceFormat> *videoFormats,
	const std::map<FEStream, V4L2DeviceFormat> *processedVideoFormats)
{
	/*
	 * Cameras having shared graph have their front end configured at
	 * acquire() time that should not be reconfigured here. Conversely, the
	 * ISI pipe video devices can always be reconfigured, which is useful
	 * for the processed streams (smart cameras) where the stream output
	 * format can be updated at configure() time.
	 */
	if (!sharedGraph_) {
		int ret = configureFrontEnd(subdevFormat, transform);
		if (ret)
			return ret;
	}

	/* Use a local video formats map if none provided by the caller. */
	std::map<FEStream, V4L2DeviceFormat> videoFormatsLocal;
	std::map<FEStream, V4L2DeviceFormat> &videoFormatsRef =
		videoFormats ? *videoFormats : videoFormatsLocal;

	videoFormatsRef.clear();
	for (const auto &[stream, entity] : streamEntries_) {
		V4L2SubdeviceFormat pipeSubdevFormat = entity.pipeSubdevFormat;
		ISIPipe *pipe = entity.isiPipe;
		V4L2DeviceFormat &vdevFormat = videoFormatsRef[stream];

		/*
		 * In processed mode, video format map is an input/output
		 * argument whose input value defines the desired capture device
		 * format, and output value returns the actual format
		 * configured by the ISI pipe.
		 * In the non-processed mode, the video format returns the
		 * non-processed format that matches the subdevice format.
		 */
		if (processedVideoFormats) {
			const auto it = processedVideoFormats->find(stream);
			if (it != processedVideoFormats->end())
				vdevFormat = it->second;
		}

		int ret = pipe->configure(pipeSubdevFormat, vdevFormat);
		if (ret)
			return ret;
	};

	return 0;
}

const FrontEndCamera::Formats &FrontEndCameraIsi::formats() const
{
	return formats_;
}

const std::vector<NeoDevice *> &FrontEndCameraIsi::neoDevices() const
{
	return neoDevices_;
}

int FrontEndCameraIsi::init(const FrontEndHandler::MatchParams &matchParams,
			    const InitParams &initParams)
{
	sensor_ = CameraSensorFactoryBase::create(initParams.sensorEntity);
	if (!sensor_) {
		LOG(NxpNeoFeIsi, Error)
			<< "Could not construct camera sensor "
			<< initParams.sensorEntity->name();
		return -ENODEV;
	}

	const std::string &sensorName = sensor_->entity()->name();
	const std::string &sensorModel = sensor_->model();
	const CameraProperties &cameraProperties =
		matchParams.pipelineConfig->cameraProperties(sensorName, sensorModel);

	/*
	 * Allocate the ISP instances for that camera.
	 * RGBIr dual context requires 2 instances - For now we rely on camera
	 * properties hints to detect the case.
	 * \todo Handle ISP contexts release in case of failure.
	 */
	unsigned int ispCount =
		cameraProperties.rgbirCfa && cameraProperties.image1Stream ? 2 : 1;
	for (unsigned int i = 0; i < ispCount; ++i) {
		std::unique_ptr<NeoDevice> neo =
			matchParams.neoAllocator->createDevice();
		if (!neo) {
			LOG(NxpNeoFeIsi, Info)
				<< "Failed to allocate NeoDevice instance";
			return -ENODEV;
		}
		neoDevices_.push_back(neo.get());
		neoDevicesOwned_.push_back(std::move(neo));
	}

	/* Check for compatible sensor formats. */
	int ret = enumerateFormatsRaw(cameraProperties);
	if (ret) {
		ret = enumerateFormatsYuv(cameraProperties);
		if (ret) {
			LOG(NxpNeoFeIsi, Debug)
				<< "No supported format for " << name();
			return -EINVAL;
		}
		rawCamera_ = false;
	} else {
		rawCamera_ = true;
	}
	ret = populateVideoFormats();
	if (ret)
		return ret;

	/* Select the max sensor resolution for ISI pipes allocation. */
	const std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		formats_.sizeMbusCodesMap;
	const Size &sizeMax = sizeToCodes.rbegin()->first;

	/*
	 * Local copy of the global streams map - original stream map is
	 * restore in case of error during the camera front end initialization.
	 */
	PadStreamsMap &deviceStreamMap = *initParams.deviceStreamMap;
	PadStreamsMap deviceStreamMapOrigin(deviceStreamMap);

	/*
	 * Discover graphs associated to each stream enabled for that sensor.
	 */
	constexpr std::array<FEStream, 3> allStreams{
		FEStream::Image0,
		FEStream::Image1,
		FEStream::EData,
	};
	ISIDevice *isiDevice = initParams.isiDevice;
	for (FEStream stream : allStreams) {
		V4L2Subdevice::Stream sensorStream;
		ISIPipe *isiPipe;
		if (stream == FEStream::Image0) {
			sensorStream = sensor_->imageStream();
		} else if (stream == FEStream::Image1) {
			bool enable = cameraProperties.image1Stream;
			if (!enable)
				continue;
			if (!sensor_->auxiliaryStream().has_value()) {
				LOG(NxpNeoFeIsi, Warning)
					<< "Sensor has no auxiliary stream, image1 disabled";
				continue;
			}
			sensorStream = sensor_->auxiliaryStream().value();
		} else if (stream == FEStream::EData) {
			bool enable = cameraProperties.eDataStream;
			if (!enable)
				continue;
			if (!sensor_->embeddedDataStream().has_value()) {
				LOG(NxpNeoFeIsi, Warning)
					<< "Sensor has no embedded data stream, edata disabled";
				continue;
			}
			sensorStream = sensor_->embeddedDataStream().value();
		} else {
			LOG(NxpNeoFeIsi, Warning) << "Invalid sensor stream";
			goto cleanup;
		}
		isiPipe = isiDevice->reservePipe(sizeMax.width);
		if (!isiPipe) {
			LOG(NxpNeoFeIsi, Error) << "Input pipe allocation failed";
			goto cleanup;
		}

		const std::string &pipeName = isiPipe->videoDeviceName();
		MediaDevice *media = initParams.mediaDevice;
		MediaEntity *videoDevEntity = media->getEntityByName(pipeName);
		if (!videoDevEntity) {
			LOG(NxpNeoFeIsi, Error)
				<< "Could not find video device entity for pipe";
			goto cleanup;
		}
		auto [it, inserted] =
			streamEntries_.emplace(stream, StreamEntry{ isiPipe, {}, {} });
		StreamGraph &streamGraph = it->second.streamGraph;
		ret = streamGraph.init(initParams.sensorEntity, sensorStream,
				       videoDevEntity, &deviceStreamMap);
		if (ret) {
			LOG(NxpNeoFeIsi, Info)
				<< "Failed to initialize stream graph";
			goto cleanup;
		}
	}

	/* Aggregate the routings for that camera. */
	for (const auto &[stream, entry] : streamEntries_) {
		const StreamGraph &streamGraph = entry.streamGraph;
		mergeRoutingMap(routings_, streamGraph.routings());
	}

	/* Store properties from config file and set camera attributes. */
	defaultOrientation_ = cameraProperties.orientation;
	attributes_ = {
		.ispBypass = !rawCamera_,
		.rgbIrCfa = cameraProperties.rgbirCfa,
		.controlsDelay = cameraProperties.controlsDelay,
	};

	streams_ = std::move(utils::map_keys(streamEntries_));
	return 0;

cleanup:
	/*
	 * Camera front end initialization failed, release ISI pipes and restore
	 * the device stream map.
	 */
	deviceStreamMap = std::move(deviceStreamMapOrigin);
	for (const auto &[stream, entry] : streamEntries_)
		isiDevice->releasePipe(entry.isiPipe);
	return -EINVAL;
}

int FrontEndCameraIsi::configureFrontEnd(V4L2SubdeviceFormat &sensorFormat,
					 Transform transform)
{
	int ret;

	/* Setup the media links for each stream graph. */
	for (const auto &[stream, entity] : streamEntries_) {
		const StreamGraph &streamGraph = entity.streamGraph;
		ret = streamGraph.initLinks();
		if (ret)
			return ret;
	}

	/*
	 * Configure sensor internal streams.
	 * Disabling may fail for immutable routes.
	 */
	if (sensor_->auxiliaryStream().has_value()) {
		bool enable = hasStream(FEStream::Image1);
		ret = sensor_->setAuxiliaryEnabled(enable);
		if (ret && enable) {
			LOG(NxpNeoFeIsi, Warning)
				<< "Auxiliary stream configuration failed"
				<< " [" << enable << "]";
			return ret;
		}
	}

	if (sensor_->embeddedDataStream().has_value()) {
		bool enable = hasStream(FEStream::EData);
		ret = sensor_->setEmbeddedDataEnabled(enable);
		if (ret && enable) {
			LOG(NxpNeoFeIsi, Warning)
				<< "Embedded data stream configuration failed"
				<< " [" << enable << "]";
			return ret;
		}
	}

	/*
	 * Configure sensor format for each stream.
	 * The configuration of the main stream is expected to configure in
	 * turn the other sensor streams to their relevant formats.
	 */
	ret = sensor_->setFormat(&sensorFormat, transform);
	if (ret)
		return ret;

	for (auto &[stream, entity] : streamEntries_) {
		V4L2SubdeviceFormat &subdevFormat = entity.pipeSubdevFormat;

		if (stream == FEStream::Image0) {
			subdevFormat = sensorFormat;
		} else if (stream == FEStream::Image1) {
			subdevFormat = sensor_->auxiliaryFormat();
		} else if (stream == FEStream::EData) {
			subdevFormat = sensor_->embeddedDataFormat();
		} else {
			LOG(NxpNeoFeIsi, Error) << "Invalid stream";
			continue;
		};

		StreamGraph &streamGraph = entity.streamGraph;
		ret = streamGraph.configure(subdevFormat);
		if (ret)
			return ret;
	}

	return ret;
}

/**
 * \brief Mark the camera as having a shared graph with other cameras
 *
 * Camera with a shared graph have limitations in the way they can be configured
 * at runtime. This function marks the camera as having a shared graph in order
 * to adapt accordingly to the formats it can support.
 * This is late initialization as the shared graph condition can be detected
 * only once all the cameras have been initialized.
 *
 * \return 0 on success or a negative error code otherwise
 */
int FrontEndCameraIsi::setSharedGraph()
{
	/*
	 * With a shared graph, no runtime format reconfiguration is possible so
	 * one format has to be selected as the default and unique format
	 * available for that camera.
	 * Default format is arbitrarily selected as the one with the highest
	 * size and bitdepth. However, the set of supported formats may have
	 * been filtered by the user through pipeline configuration file to
	 * select a specific resolution.
	 */
	std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		formats_.sizeMbusCodesMap;
	std::map<unsigned int, std::vector<Size>> &codeToSizes =
		formats_.mbusCodeSizesMap;
	if (sizeToCodes.empty() || sizeToCodes.begin()->second.empty() ||
	    codeToSizes.empty() || codeToSizes.begin()->second.empty()) {
		LOG(NxpNeoFeIsi, Error) << "No formats available";
		return -EINVAL;
	}
	const Size &sizeMax = sizeToCodes.rbegin()->first;
	const std::vector<unsigned int> &codes = sizeToCodes.rbegin()->second;
	unsigned int codeBitDepthMax = codes.back();
	sizeToCodes.clear();
	sizeToCodes[sizeMax] = { codeBitDepthMax };
	codeToSizes.clear();
	codeToSizes[codeBitDepthMax] = { sizeMax };

	/*
	 * Trim the mbus code to pixel formats map to keep as a single entry
	 * the one corresponding to the selected code for the camera.
	 */
	auto it = formats_.mbusCodePixelFormatsMap.begin();
	while (it != formats_.mbusCodePixelFormatsMap.end()) {
		if (it->first != codeBitDepthMax)
			it = formats_.mbusCodePixelFormatsMap.erase(it);
		else
			++it;
	}

	/*
	 * Orientation can not be changed at runtime with a shared graph, as it
	 * usually changes the sensor code which would require graph
	 * reconfiguration. Select a default fixed orientation if not already
	 * defined in the config file.
	 */
	if (!defaultOrientation_.has_value())
		defaultOrientation_ = sensor_->mountingOrientation();

	sharedGraph_ = true;
	return 0;
}

/**
 * \brief Enumerate the compatible sizes and mbus-codes for a raw sensor
 *
 * Enumerate the sizes and associated mbus-codes provided by the sensor modes.
 * Two format maps are stored in the class for later usage:
 * - All sizes associated to a given mbus code
 * - All mbus codes associated to a given size
 * Mbus codes selected have to be Bayer formats supported by the front end and
 * the ISP. Also, size widths selected must be within ISI and ISP supported
 * ranges.
 * \param[in] properties The camera properties from the config file
 *
 * \return 0 in case of success or a negative error code
 */
int FrontEndCameraIsi::enumerateFormatsRaw(const CameraProperties &properties)
{
	std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		formats_.sizeMbusCodesMap;
	std::map<unsigned int, std::vector<Size>> &codeToSizes =
		formats_.mbusCodeSizesMap;

	const std::vector<unsigned int> &mbusCodes = sensor_->mbusCodes();
	const std::vector<unsigned int> &bayerCodes = ISIPipe::bayerMbusCodes();
	const std::vector<V4L2PixelFormat> &neoPixelFormats =
		NeoDevice::outputFormats(NeoDevice::VideoDevice::Input0);

	/*  Camera formats filtering may be defined in the config file. */
	const std::optional<unsigned int> &bppFilter = properties.formatBpp;
	const std::optional<Size> &sizeFilter = properties.formatSize;

	for (unsigned int code : mbusCodes) {
		auto itBayerCode = std::find(bayerCodes.begin(),
					     bayerCodes.end(), code);
		if (itBayerCode == bayerCodes.end())
			continue;

		const V4L2PixelFormat deviceFormat =
			ISIPipe::mbusCodeToPixelFormatBypass(code);

		if (std::find(neoPixelFormats.begin(), neoPixelFormats.end(),
			      deviceFormat) == neoPixelFormats.end())
			continue;

		const BayerFormat &bayerFormat = BayerFormat::fromMbusCode(code);
		if (bppFilter && bayerFormat.bitDepth != bppFilter.value())
			continue;

		std::vector<Size> sizes = sensor_->sizes(code);
		for (const Size &size : sizes) {
			if (size.width > NeoDevice::kRawWidthMax)
				continue;

			if (size.width & (NeoDevice::kWidthAlignment - 1))
				continue;

			if (sizeFilter && size != sizeFilter.value())
				continue;

			sizeToCodes[size].push_back(code);
			codeToSizes[code].push_back(size);
		}
	}

	/* Make sure there is at least one compatible size and code */
	if (!sizeToCodes.size() || !sizeToCodes.begin()->second.size()) {
		LOG(NxpNeoFeIsi, Debug)
			<< "No compatible raw sensor format found for the pipeline";
		return -EINVAL;
	}

	/*
	 * At least one compatible format has been found.
	 * Sort the map values by code bitdepth and size ascending order.
	 */
	for (auto &[size, codes] : sizeToCodes) {
		std::sort(codes.begin(), codes.end(),
			  [](const unsigned int &lhs, const unsigned int &rhs) {
				  const BayerFormat &bayerFormatLhs =
					  BayerFormat::fromMbusCode(lhs);
				  const BayerFormat &bayerFormatRhs =
					  BayerFormat::fromMbusCode(rhs);
				  return bayerFormatLhs.bitDepth < bayerFormatRhs.bitDepth;
			  });
	}
	for (auto &[code, sizes] : codeToSizes)
		std::sort(sizes.begin(), sizes.end());

	std::ostringstream oss;
	oss << "Raw formats size [ mbuscodes ] : ";
	for (const auto &[size, codes] : sizeToCodes) {
		oss << size.toString() << " [ ";
		for (const unsigned int code : codes)
			oss << utils::hex(code) << " ";
		oss << "] ";
	}
	LOG(NxpNeoFeIsi, Debug) << oss.str();

	return 0;
}

/**
 * \brief Enumerate the compatible sizes and mbus-codes for a smart sensor
 *
 * Enumerate the sizes and associated mbus-codes provided by the sensor modes.
 * Two maps are stored in the class for later usage:
 * - All sizes associated to a given mbus code
 * - All mbus codes associated to a given size
 * Mbus codes selected have to be formats supported by the front end. Also, size
 * widths selected must be within ISI supported range.
 * \param[in] properties The camera properties from the config file
 *
 * \return 0 in case of success or a negative error code
 */
int FrontEndCameraIsi::enumerateFormatsYuv(const CameraProperties &properties)
{
	std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		formats_.sizeMbusCodesMap;
	std::map<unsigned int, std::vector<Size>> &codeToSizes =
		formats_.mbusCodeSizesMap;

	const std::vector<unsigned int> &mbusCodes = sensor_->mbusCodes();
	const std::vector<uint32_t> &sinkCodes = ISIPipe::sinkMbusCodesProcessed();

	/*  Camera formats filtering may be defined in the config file */
	std::optional<unsigned int> bppFilter =
		properties.formatBpp;
	std::optional<Size> sizeFilter =
		properties.formatSize;

	for (unsigned int code : mbusCodes) {
		if (std::find(sinkCodes.begin(), sinkCodes.end(), code) == sinkCodes.end())
			continue;

		const MediaBusFormatInfo &info = MediaBusFormatInfo::info(code);
		if (bppFilter && info.bitsPerPixel != bppFilter.value())
			continue;

		std::vector<Size> sizes;
		cameraSizesFixup(sensor_.get(), code, sizes);

		for (const Size &size : sizes) {
			if (size.width > ISIPipe::kChainedWidthMax)
				continue;

			if (sizeFilter && size != sizeFilter.value())
				continue;

			sizeToCodes[size].push_back(code);
			codeToSizes[code].push_back(size);
		}
	}

	/* Make sure there is at least one compatible size and code */
	if (!sizeToCodes.size() || !sizeToCodes.begin()->second.size()) {
		LOG(NxpNeoFeIsi, Debug)
			<< "No compatible rgb/yuv sensor format found for the pipeline";
		return -EINVAL;
	}

	/*
	 * At least one compatible format has been found.
	 * Sort the map values by code bitdepth and size ascending order.
	 */
	for (auto &[size, codes] : sizeToCodes) {
		std::sort(codes.begin(), codes.end(),
			  [](const unsigned int &lhs, const unsigned int &rhs) {
				  const MediaBusFormatInfo &MbusInfoLhs =
					  MediaBusFormatInfo::info(lhs);
				  const MediaBusFormatInfo &MbusInfoRhs =
					  MediaBusFormatInfo::info(rhs);
				  return MbusInfoLhs.bitsPerPixel < MbusInfoRhs.bitsPerPixel;
			  });
	}
	for (auto &[code, sizes] : codeToSizes)
		std::sort(sizes.begin(), sizes.end());

	std::ostringstream oss;
	oss << "Yuv formats size [ mbuscodes ] : ";
	for (const auto &[size, codes] : sizeToCodes) {
		oss << size.toString() << " [ ";
		for (const unsigned int code : codes)
			oss << utils::hex(code) << " ";
		oss << "] ";
	}
	LOG(NxpNeoFeIsi, Debug) << oss.str();

	return 0;
}

/**
 * \brief Populate video format mappings from mbus codes to pixel formats
 *
 * This function builds the mapping between media bus codes and their
 * corresponding pixel formats that can be produced by the ISI pipeline.
 * For processed mbus codes (YUV/RGB formats), all pixel formats that an
 * ISI pipe can produce are reported. For bypass mbus codes (Bayer or
 * grayscale formats), only the unique pixel format that the pipe produces
 * in bypass mode is reported.
 *
 * The populated mapping is stored for later use during format negotiation and
 * stream configuration.
 *
 * \return 0 on success, or a negative error code if an invalid pixel
 * format is encountered during bypass format conversion
 */
int FrontEndCameraIsi::populateVideoFormats()
{
	/*
	 * For processed mbus codes, report all the pixel formats that an ISI
	 * pipe can produce. For bypass mbus codes (bayer or gray formats),
	 * report the unique pixel format that the pipe produces.
	 */
	const std::vector<unsigned int> &sinkMbusCodesProcessed =
		ISIPipe::sinkMbusCodesProcessed();
	const std::vector<PixelFormat> &pixelFormatsProcessed =
		ISIPipe::pixelFormatsProcessed();

	for (const auto &[code, sizes] : formats_.mbusCodeSizesMap) {
		auto it = std::find(sinkMbusCodesProcessed.begin(),
				    sinkMbusCodesProcessed.end(), code);
		if (it != sinkMbusCodesProcessed.end()) {
			formats_.mbusCodePixelFormatsMap.emplace(
				code, pixelFormatsProcessed);
		} else {
			V4L2PixelFormat v4L2pixFormat =
				ISIPipe::mbusCodeToPixelFormatBypass(code);
			if (!v4L2pixFormat.isValid())
				return -EINVAL;
			PixelFormat pixelFormat = v4L2pixFormat.toPixelFormat();
			formats_.mbusCodePixelFormatsMap.emplace(
				code, std::vector<PixelFormat>{ pixelFormat });
		}
	}

	return 0;
}

bool FrontEndHandlerIsi::match([[maybe_unused]] const MatchParams &params)
{
	/*
	 * ISI front end media device is a special case: it does not have to be
	 * acquired from the device enumerator as it has already been acquired
	 * by the Neo device allocator: both front end and ISP subgraphs coexist
	 * in the same media device.
	 */
	media_ = params.neoAllocator->feMedia();
	if (!media_) {
		LOG(NxpNeoFeIsi, Debug)
			<< "ISI front end media device not available";
		return false;
	}

	isiDevice_ = std::make_unique<ISIDevice>(media_);
	if (!isiDevice_->isValid()) {
		LOG(NxpNeoFeIsi, Warning) << "Failed to initialize ISI device";
		return false;
	}

	/*
	 * Build a set of the sensors entities from the media device to have an
	 * ordered list by name and guarantee a consistent topology detection.
	 * The usual sensor naming convention is 'model xx-yyyy'
	 * where xx is the i2c bus number and yyyy is the bus address.
	 * Sensors set ordering is defined with the criteria below:
	 * - simple alphabetical order when above convention is not used or
	 *   for different sensor model
	 * - increasing i2c bus for identical sensor model on different i2c bus
	 * - increasing i2c address for identical sensor model on the same bus
	 * \todo Remove when mx95mbcam module driver limitation with regard to
	 * stream numbering of the multiple cameras modules is fixed.
	 */
	auto compareName =
		[](MediaEntity *a, MediaEntity *b) {
			std::regex r("(\\S+) (\\d+)-(\\d+)");
			const std::string &nameA = a->name();
			const std::string &nameB = b->name();
			std::smatch matchesA, matchesB;
			if (std::regex_search(nameA, matchesA, r) &&
			    std::regex_search(nameB, matchesB, r)) {
				const std::string &modelA = matchesA[1];
				const std::string &modelB = matchesB[1];
				unsigned int i2cBusA =
					static_cast<unsigned int>(std::stoul(matchesA[2]));
				unsigned int i2cBusB =
					static_cast<unsigned int>(std::stoul(matchesB[2]));
				unsigned int i2cAddrA =
					static_cast<unsigned int>(std::stoul(matchesA[3]));
				unsigned int i2cAddrB =
					static_cast<unsigned int>(std::stoul(matchesB[3]));
				if (modelA != modelB)
					return nameA < nameB;
				else if (i2cBusA != i2cBusB)
					return i2cBusA < i2cBusB;
				else
					return i2cAddrA < i2cAddrB;
			} else {
				return nameA < nameB;
			}
		};

	std::vector<MediaEntity *> entities = locateSensors(media_.get());
	std::sort(entities.begin(), entities.end(), compareName);

	/*
	 * Start with an empty streams map for the media device. It will be
	 * incrementally populated with FrontEndCamera instantiations.
	 */
	PadStreamsMap deviceStreamMap;

	/*
	 * Go through every detected sensor entity and create its associated
	 * FrontEndCamera object.
	 */
	for (const auto entity : entities) {
		std::unique_ptr<FrontEndCameraIsi> feCamera =
			std::make_unique<FrontEndCameraIsi>();
		FrontEndCameraIsi::InitParams initParams = {
			.mediaDevice = media_.get(),
			.sensorEntity = entity,
			.isiDevice = isiDevice_.get(),
			.deviceStreamMap = &deviceStreamMap,
		};
		int ret = feCamera->init(params, initParams);
		if (ret)
			continue;
		isiCameras_.push_back(std::move(feCamera));
	}

	for (auto &camera : isiCameras_) {
		/*
		 * Detect cameras sharing some parts of their graph. This is
		 * currently a very basic detection where we check overlap on
		 * the image0 streams. Each camera is tested against all others
		 * to see if they share that stream, so procedure may be
		 * optimized later if needed.
		 */
		auto itLeft = camera->streamEntries_.find(FEStream::Image0);
		ASSERT(itLeft != camera->streamEntries_.end());
		bool shared = false;
		for (auto &other : isiCameras_) {
			if (other.get() == camera.get())
				continue;
			auto itRight =
				other->streamEntries_.find(FEStream::Image0);
			ASSERT(itRight != other->streamEntries_.end());
			const StreamGraph &sharedGraphLeft =
				itLeft->second.streamGraph;
			const StreamGraph &sharedGraphRight =
				itRight->second.streamGraph;
			shared = sharedGraphLeft.isShared(&sharedGraphRight);
			if (shared) {
				camera->setSharedGraph();
				break;
			}
		}

		LOG(NxpNeoFeIsi, Debug)
			<< "Camera " << camera->name() << " shared " << shared;
		cameras_.push_back(camera.get());

		/* Aggregate routings from all cameras*/
		mergeRoutingMap(routings_, camera->routings_);
	}

	unsigned int count = cameras_.size();
	return !!count;
}

const std::vector<FrontEndCamera *> &FrontEndHandlerIsi::cameras() const
{
	return cameras_;
}

int FrontEndHandlerIsi::acquireDevice([[maybe_unused]] const std::string &name)
{
	if (useCount_) {
		useCount_++;
		return 0;
	}

	/*
	 * Frontend media controller device has been locked by the process.
	 * Global routing for all cameras is to be configured now as it will no
	 * longer be possible to update it after any streaming has started.
	 * Also, camera with shared graphs have to be statically preconfigured
	 * as they are dependent on each other.
	 */
	int ret = setupRoutings();
	if (ret)
		return ret;

	ret = setupSharedGraphs();
	if (ret)
		return ret;

	useCount_++;
	return 0;
}

void FrontEndHandlerIsi::releaseDevice([[maybe_unused]] const std::string &name)
{
	useCount_--;
	return;
}

/**
 * \brief Configure the V4L2 subdevices routings
 *
 * Configure the subdevices routings in the system. As routing configuration can
 * not be updated while a device is streaming, and because subdevices may be
 * shared by the streams from multiple cameras, routings have to be setup
 * once at startup and no longer updated afterwards.
 * Such setup could be limited to the subset of entities which are shared
 * between multiple cameras such as the ISI crossbar, but routings remain the
 * same regardless of the camera configuration so they are applied globally.
 *
 * \return 0 on success, or a negative error code otherwise
 */
int FrontEndHandlerIsi::setupRoutings() const
{
	LOG(NxpNeoFeIsi, Debug) << "Set global routings";

	for (const auto &[entity, routing] : routings_) {
		const std::string &name = entity->name();
		LOG(NxpNeoFeIsi, Debug)
			<< "Configure routing for entity " << name
			<< " routing " << routing;

		std::unique_ptr<V4L2Subdevice> sdev =
			V4L2Subdevice::fromEntityName(media_.get(), name);
		if (!sdev.get()) {
			LOG(NxpNeoFeIsi, Error) << "Subdevice does not exist " << name;
			return -EINVAL;
		}

		int ret = sdev->open();
		if (ret) {
			LOG(NxpNeoFeIsi, Error)
				<< "Error opening entity " << name;
			return -EINVAL;
		}

		V4L2Subdevice::Routing _routing = routing;
		ret = sdev->setRouting(&_routing, V4L2Subdevice::ActiveFormat);
		if (ret) {
			LOG(NxpNeoFeIsi, Error)
				<< "Error setting routing for entity " << name;
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * \brief Initialize the shared camera graphs from the media controller device
 *
 * Cameras managed by the pipeline operate on different streams of the front end
 * media controller device. Those streams share subdevice pads that may be
 * common to multiple cameras.
 * When multiple cameras are multiplexed over the same MIPI-CSI2 port, typically
 * through the usage of a GMSL SerDes, some limitations coming from the
 * front end media device apply to that set of cameras:
 * - A given camera graph to be started requires a valid format to be configured
 *   for every other camera graphs of the set
 * - A camera graph can not be reconfigured when another camera from the set is
 *   active
 * With such multi-camera case, these limitations prevent from configuring the
 * camera graph at configure() time, because another camera may already be
 * streaming. Thus, a default graph configuration is necessary for each camera
 * of the set before streaming operation is started on another camera. This is
 * done when the frontend media device is locked.
 * Configuration of the ISP device will still be done at configure() time as
 * there is one ISP media instance per camera. These ISP instances can be
 * reconfigured independently from each other.
 *
 * \return 0 on success or a negative error code otherwise
 */
int FrontEndHandlerIsi::setupSharedGraphs()
{
	int ret = 0;

	for (auto const &camera : isiCameras_) {
		if (!camera->sharedGraph())
			continue;

		LOG(NxpNeoFeIsi, Debug)
			<< "Setup shared graph camera " << camera->name();

		/* Configure the default format on that camera frontend graph. */
		V4L2SubdeviceFormat sensorFormat = {};
		const std::map<Size, std::vector<unsigned int>> &sizeToCodes =
			camera->formats_.sizeMbusCodesMap;
		if (sizeToCodes.size() != 1) {
			LOG(NxpNeoFeIsi, Error)
				<< "Single size expected with a shared graph";
			return -EINVAL;
		}
		sensorFormat.size = sizeToCodes.begin()->first;
		const std::vector<unsigned int> &codes = sizeToCodes.begin()->second;
		if (codes.size() != 1) {
			LOG(NxpNeoFeIsi, Error)
				<< "Single code expected with a shared graph";
			return -EINVAL;
		}
		sensorFormat.code = codes.back();

		if (!camera->defaultOrientation_.has_value()) {
			LOG(NxpNeoFeIsi, Error)
				<< "Default orientation expected with a shared graph";
			return -EINVAL;
		}
		Orientation orientation = camera->defaultOrientation_.value();
		Transform transform = camera->sensor_->computeTransform(&orientation);
		ret = camera->configureFrontEnd(sensorFormat, transform);
		if (ret)
			return ret;
	}

	return ret;
}

REGISTER_FRONT_END_HANDLER("frontend-isi", FrontEndHandlerIsi);

} /* namespace nxpneo */

} /* namespace libcamera */
