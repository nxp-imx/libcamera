/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Pipeline handler for Intel IPU3
 *     src/libcamera/pipeline/ipu3/ipu3.cpp
 * Copyright (C) 2019, Google Inc.
 *
 * and on Pipeline handler for ISI interface
 *     src/libcamera/pipeline/imx8-isi/ims8-isi.cpp
 * Copyright (C) 2022 - Jacopo Mondi <jacopo@jmondi.org>
 *
 * neo_pipeline.cpp - Pipeline handler for NXP NEO ISP
 * Copyright 2024-2025 NXP
 */

#include <algorithm>
#include <iomanip>
#include <memory>
#include <queue>
#include <sstream>
#include <vector>

#include <linux/nxp_neoisp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <libcamera/ipa/nxpneo_ipa_interface.h>
#include <libcamera/ipa/nxpneo_ipa_proxy.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_lens.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"

#include "frames.h"
#include "isi_device.h"
#include "neo_device.h"
#include "neo_utils.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoPipe)

using namespace libcamera::nxpneo;

class PipelineHandlerNxpNeo;

class NxpNeoCameraData : public Camera::Private
{
public:
	NxpNeoCameraData(PipelineHandler *pipe,
			 std::unique_ptr<CameraSensor> sensor,
			 std::unique_ptr<NeoDevice> neo,
			 const CameraInfo *cameraInfo)
		: Camera::Private(pipe),
		  sensor_(std::move(sensor)),
		  neo_(std::move(neo)),
		  cameraInfo_(cameraInfo){};

	int configure(CameraConfiguration *c);
	int exportFrameBuffers(Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	int start(const ControlList *controls);
	void stopDevice();

	void queuePendingRequests();

	int init();
	PipelineHandlerNxpNeo *pipe();

	bool sensorIsRgbIr() const { return sensorIsRgbIr_; }
	void adjustTopLinesSize(Size *size) const;
	int enumerateRawFormats();

	int configureFrontEndFormat(V4L2SubdeviceFormat &sensorFormat,
				    Transform transform);

	CameraSensor *sensor() const { return sensor_.get(); }
	NeoDevice *neoDevice() const { return neo_.get(); }
	std::string cameraName() const { return sensor_->entity()->name(); }
	bool multiCamera() const { return cameraInfo_->cameraProperties().multiCamera; }
	const std::map<Size, std::vector<unsigned int>> &
	rawFormatsSizeToCodes() const { return rawFormatsSizeToCodes_; }
	const std::map<unsigned int, std::vector<Size>> &
	rawFormatsCodeToSizes() const { return rawFormatsCodeToSizes_; }
	const Orientation *mountingOrientation() const { return &mountingOrientation_; }

	bool rawStreamOnly_ = false;

	Stream streamFrame_;
	Stream streamIr_;
	Stream streamRaw_;

	/* Requests for which no buffer has been queued to the frontend  device yet */
	std::queue<Request *> pendingRequests_;
	/* Requests queued to the frontend device but not yet processed by the ISP */
	std::queue<Request *> processingRequests_;

private:
	int updateControls();
	int loadIPA();

	int allocateBuffers();
	int freeBuffers();

	int configureFrontEndStream(const std::vector<CameraMediaStream::StreamLink> &streamLinks,
				    V4L2SubdeviceFormat &sdFormat);
	int configureFrontEndLinks() const;

	void clearRequest(NxpNeoFrames::Info *info);
	void cancelCompleteRequest(NxpNeoFrames::Info *info);
	void tryCompleteRequest(NxpNeoFrames::Info *info);

	void isiInputBufferReady(NxpNeoFrames::Info *info);
	void isiInput0BufferReady(FrameBuffer *buffer);
	void isiInput1BufferReady(FrameBuffer *buffer);
	void isiEmbeddedDataBufferReady(FrameBuffer *buffer);

	void neoInput0BufferReady(FrameBuffer *buffer);
	void neoInput1BufferReady(FrameBuffer *buffer);
	void neoOutputBufferReady(FrameBuffer *buffer);
	void neoParamsBufferReady(FrameBuffer *buffer);
	void neoStatsBufferReady(FrameBuffer *buffer);
	void frameStart(uint32_t sequence);

	void ipaParamsBufferReady(unsigned int id);
	void ipaMetadataReady(unsigned int id, const ControlList &metadata);
	void ipaSetSensorControls(unsigned int id, const ControlList &sensorControls);

	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<NeoDevice> neo_;
	const CameraInfo *cameraInfo_;
	Orientation mountingOrientation_;
	std::map<Size, std::vector<unsigned int>> rawFormatsSizeToCodes_;
	std::map<unsigned int, std::vector<Size>> rawFormatsCodeToSizes_;

	/* Front end pipes and video formats - maps per stream */
	std::map<unsigned int, ISIPipe *> pipes_;
	std::map<unsigned int, V4L2DeviceFormat> pipesDevFormats_;

	NxpNeoFrames frameInfos_;
	bool alternatedRawStream_ = false;

	std::unique_ptr<ipa::nxpneo::IPAProxyNxpNeo> ipa_;
	ControlInfoMap ipaControls_;
	std::vector<IPABuffer> ipaBuffers_;
	std::unique_ptr<DelayedControls> delayedCtrls_;

	unsigned int sequence_ = 0;
	bool sensorIsRgbIr_ = false;
	unsigned int embeddedTopLines_ = 0;
};

class NxpNeoCameraConfiguration : public CameraConfiguration
{
public:
	NxpNeoCameraConfiguration(Camera *camera, NxpNeoCameraData *data);

	Status validate() override;

	const V4L2SubdeviceFormat &sensorFormat() { return sensorFormat_; }
	const Transform &combinedTransform() { return combinedTransform_; }

private:
	/*
	 * The NxpNeoCameraData instance is guaranteed to be valid as long as the
	 * corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	std::shared_ptr<Camera> camera_;
	NxpNeoCameraData *data_;

	V4L2SubdeviceFormat sensorFormat_;
	Transform combinedTransform_;
};

class PipelineHandlerNxpNeo : public PipelineHandler
{
public:
	PipelineHandlerNxpNeo(CameraManager *manager)
		: PipelineHandler(manager) {}

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

	bool acquireDevice(Camera *camera) override;
	void releaseDevice(Camera *camera) override;

	unsigned int numCameras() const { return numCameras_; }
	ISIDevice *isiDevice() const { return isi_.get(); }
	const PipelineConfig *pipelineConfig() { return &pipelineConfig_; }

private:
	NxpNeoCameraData *cameraData(Camera *camera)
	{
		return static_cast<NxpNeoCameraData *>(camera->_d());
	}

	int createCamera(MediaEntity *sensorEntity, MediaDevice *neoMedia,
			 unsigned int neoInstance);

	int setupRouting() const;
	int setupCameraGraphs();
	int loadPipelineConfig();
	PipelineConfig pipelineConfig_;

	unsigned int numCameras_ = 0;
	unsigned int acquireCount_ = 0;
	std::shared_ptr<ISIDevice> isi_;
};

NxpNeoCameraConfiguration::NxpNeoCameraConfiguration(Camera *camera,
						     NxpNeoCameraData *data)
	: CameraConfiguration()
{
	camera_ = camera->shared_from_this();
	data_ = data;
}

CameraConfiguration::Status NxpNeoCameraConfiguration::validate()
{
	Status status = Valid;
	CameraSensor *sensor = data_->sensor();

	if (config_.empty())
		return Invalid;

	/*
	 * Validate the requested stream configuration verifying that there is
	 * a single raw stream, or a rgb/yuv stream with an optional IR stream
	 * when supported by the sensor.
	 */
	unsigned int rawCount = 0;
	unsigned int yuvRgbCount = 0;
	unsigned int irCount = 0;

	Stream *streamFrame = const_cast<Stream *>(&data_->streamFrame_);
	Stream *streamIr = const_cast<Stream *>(&data_->streamIr_);
	Stream *streamRaw = const_cast<Stream *>(&data_->streamRaw_);

	for (StreamConfiguration &cfg : config_) {
		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			rawCount++;
			cfg.setStream(streamRaw);
		} else if ((info.colourEncoding == PixelFormatInfo::ColourEncodingYUV) &&
			   (info.planes[0].bytesPerGroup <= 2) &&
			   (info.planes[1].bytesPerGroup == 0)) {
			/*  pixel formats Rn detection (grey/Yn) */
			if (data_->sensorIsRgbIr()) {
				/* iR stream handles only Y8 and Y16 formats */
				if ((irCount == 0) &&
				    ((info.bitsPerPixel % 8u) == 0)) {
					irCount++;
					cfg.setStream(streamIr);
				} else {
					yuvRgbCount++;
					cfg.setStream(streamFrame);
				}
			} else {
				yuvRgbCount++;
				cfg.setStream(streamFrame);
			}
		} else if ((info.colourEncoding == PixelFormatInfo::ColourEncodingYUV) ||
			   (info.colourEncoding == PixelFormatInfo::ColourEncodingRGB)) {
			yuvRgbCount++;
			cfg.setStream(streamFrame);
		} else {
			LOG(NxpNeoPipe, Debug) << "Unknown config pixel format";
			return Invalid;
		}
	}

	if (yuvRgbCount > 1) {
		LOG(NxpNeoPipe, Debug) << "Multiple rgb/yuv streams not supported";
		return Invalid;
	} else if (rawCount > 1) {
		LOG(NxpNeoPipe, Debug) << "Multiple raw streams not supported";
		return Invalid;
	} else if (irCount > 1) {
		LOG(NxpNeoPipe, Debug) << "Multiple Ir streams not supported";
		return Invalid;
	}

	Orientation requestedOrientation = orientation;
	if (!data_->multiCamera()) {
		combinedTransform_ = sensor->computeTransform(&orientation);
	} else {
		combinedTransform_ = Transform::Identity;
		orientation = *data_->mountingOrientation();
	}
	if (orientation != requestedOrientation)
		status = Adjusted;

	/*
	 * Work out the sensor format to be used. When a raw stream is specified
	 * its pixel output format defines explicitly the sensor bit depth and
	 * size. Thus, the raw stream configuration is checked first to find a
	 * possible match with the sensor format capabilities.
	 * If sensor format has not been resolved from the raw stream, check for
	 * every stream configured if the requested size can be provided by the
	 * sensor then derive a working code for that size.
	 * If none of the streams is configured with a size supported with that
	 * sensor, fall back onto selecting arbitrarily the highest size and the
	 * associated mbus code with the highest bit depth.
	 */
	Size sensorSize;
	unsigned int sensorMbusCode;
	bool sensorFormatFound = false;

	const std::map<unsigned int, std::vector<Size>> &codeToSizes =
		data_->rawFormatsCodeToSizes();
	const std::vector<unsigned int> sensorCodes = utils::map_keys(codeToSizes);
	if (rawCount) {
		auto rawConfigIt = std::find_if(
			config_.begin(), config_.end(),
			[=](StreamConfiguration &cfg) { return cfg.stream() == streamRaw; });
		ASSERT(rawConfigIt != config_.end());
		uint8_t bitDepthConfig =
			BayerFormat::fromPixelFormat(rawConfigIt->pixelFormat).bitDepth;
		auto rawCodeIt = std::find_if(
			sensorCodes.begin(), sensorCodes.end(),
			[=](unsigned int code) {
				uint8_t bitDepth = BayerFormat::fromMbusCode(code).bitDepth;
				return bitDepth == bitDepthConfig;
			});
		if (rawCodeIt != sensorCodes.end()) {
			unsigned int code = *rawCodeIt;
			const std::vector<Size> &sizes = codeToSizes.at(code);
			if (std::find(sizes.begin(), sizes.end(),
				      rawConfigIt->size) != sizes.end()) {
				sensorSize = rawConfigIt->size;
				sensorMbusCode = code;
				sensorFormatFound = true;
			}
		}
	}

	if (!sensorFormatFound) {
		const std::map<Size, std::vector<unsigned int>> &sizeToCodes =
			data_->rawFormatsSizeToCodes();
		const std::vector<Size> sensorSizes = utils::map_keys(sizeToCodes);
		auto anyConfig =
			std::find_if(
				config_.begin(), config_.end(),
				[&sensorSizes](StreamConfiguration &cfg) {
					return std::find(sensorSizes.begin(),
							 sensorSizes.end(),
							 cfg.size) != sensorSizes.end();
				});
		if (anyConfig != config_.end()) {
			sensorSize = anyConfig->size;
		} else {
			ASSERT(sensorSizes.size());
			sensorSize = sensorSizes.back();
		}
		const std::vector<unsigned int> &codes = sizeToCodes.at(sensorSize);
		ASSERT(codes.size());
		sensorMbusCode = codes.back();
	}

	/* Cache sensor format for later usage by configure() */
	sensorFormat_ = {};
	sensorFormat_.code = sensorMbusCode;
	sensorFormat_.size = sensorSize;
	LOG(NxpNeoPipe, Debug) << "Sensor format " << sensorFormat_.toString();

	Size pixelSize(sensorSize);
	data_->adjustTopLinesSize(&pixelSize);

	for (unsigned int i = 0; i < config_.size(); ++i) {
		const StreamConfiguration originalCfg = config_[i];
		StreamConfiguration *cfg = &config_[i];

		bool isFrame = (streamFrame == cfg->stream());
		bool isIr = (streamIr == cfg->stream());
		bool isRaw = (streamRaw == cfg->stream());

		LOG(NxpNeoPipe, Debug)
			<< "Stream " << i << " to validate cfg " << cfg->toString();

		if (isFrame || isIr) {
			const std::vector<V4L2PixelFormat> &formats =
				isFrame ? NeoDevice::frameFormats() : NeoDevice::irFormats();
			if (std::find_if(formats.begin(),
					 formats.end(),
					 [&](auto &format) {
						 return format.toPixelFormat() == cfg->pixelFormat;
					 }) == formats.end())
				cfg->pixelFormat = formats[0].toPixelFormat();
			cfg->size = pixelSize;

			V4L2DeviceFormat format = {};
			format.size = cfg->size;
			format.fourcc = (V4L2PixelFormat::fromPixelFormat(cfg->pixelFormat))[0];
			format.colorSpace = cfg->colorSpace;

			/* For frame stream, the user can choose the
			 * sRGB colorspace for the RGB output formats.
			 * The sRGB colorspace conversion from libcamera
			 * to v4l2 is not directly supported, so use
			 * libcamera sYCC instead that maps to v4l2
			 * JPEG colorspace that is a v4l2 sRGB alias.
			 */
			if (format.colorSpace == ColorSpace::Srgb)
				format.colorSpace = ColorSpace::Sycc;

			if (isFrame) {
				data_->neoDevice()->frame_->tryFormat(&format);
				cfg->colorSpace = format.colorSpace;
				cfg->stride = format.planes[0].bpl;
				cfg->frameSize = format.planes[0].size;
			} else if (isIr) {
				data_->neoDevice()->ir_->tryFormat(&format);
				/* IR node is fixed to RAW colorspace */
				cfg->colorSpace = ColorSpace::Raw;
				cfg->stride = format.planes[0].bpl;
				cfg->frameSize = format.planes[0].size;
			}

			LOG(NxpNeoPipe, Debug) << "Assigned " << cfg->toString()
					       << " to the "
					       << (isFrame ? "frame" : "ir")
					       << " stream";
		} else if (isRaw) {
			const BayerFormat &bayerFormat =
				BayerFormat::fromMbusCode(sensorFormat_.code);
			cfg->pixelFormat = bayerFormat.toPixelFormat();
			cfg->size = sensorSize;
			cfg->colorSpace = ColorSpace::Raw;
			const PixelFormatInfo &info =
				PixelFormatInfo::info(cfg->pixelFormat);
			cfg->stride = info.stride(cfg->size.width, 0);
			cfg->frameSize = info.frameSize(cfg->size, 1);

			LOG(NxpNeoPipe, Debug) << "Assigned " << cfg->toString()
					       << " to the raw stream";
		} else {
			LOG(NxpNeoPipe, Error) << "Unknown configuration stream";
			return Invalid;
		}

		const GlobalInfo &globalInfo =
			data_->pipe()->pipelineConfig()->globalInfo();
		cfg->bufferCount = globalInfo.bufferCount;

		if (cfg->pixelFormat != originalCfg.pixelFormat ||
		    cfg->size != originalCfg.size) {
			status = Adjusted;
		}

		if (originalCfg.colorSpace.has_value() &&
		    cfg->colorSpace != originalCfg.colorSpace) {
			status = Adjusted;
		}

		LOG(NxpNeoPipe, Debug)
			<< "Stream validated " << i << " cfg " << cfg->toString();
	}

	return status;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerNxpNeo::generateConfiguration(Camera *camera,
					     Span<const StreamRole> roles)
{
	NxpNeoCameraData *data = cameraData(camera);
	std::unique_ptr<NxpNeoCameraConfiguration> config =
		std::make_unique<NxpNeoCameraConfiguration>(camera, data);

	LOG(NxpNeoPipe, Debug) << "Generate configuration " << data->cameraName();

	if (roles.empty())
		return config;

	bool frameOutputAvailable = true;
	bool irOutputAvailable = data->sensorIsRgbIr();
	bool rawOutputAvailable = true;

	std::optional<ColorSpace> colorSpace;

	/*
	 * Top embedded data from sensor are cropped before being fed to ISP
	 * Cropped sensor sizes are used as proposed range for the ISP-decoded
	 * streams. Conversely, the raw stream uses uncropped sensor sizes.
	 */
	const std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		data->rawFormatsSizeToCodes();
	const std::vector<Size> sensorSizes = utils::map_keys(sizeToCodes);
	std::vector<Size> pixelSizes(sensorSizes);
	for (Size &size : pixelSizes)
		data->adjustTopLinesSize(&size);
	std::vector<SizeRange> pixelRanges;
	for (const Size &size : pixelSizes)
		pixelRanges.emplace_back(size);

	for (const StreamRole role : roles) {
		std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
		PixelFormat pixelFormat;
		Size cfgSize;

		switch (role) {
		case StreamRole::StillCapture:
		case StreamRole::Viewfinder:
		case StreamRole::VideoRecording: {
			/*
			 * Propose the resolutions supported by the sensor with
			 * all output formats supported by the ISP, including
			 * Infrared (gray) if supported by the sensor.
			 */
			const std::vector<V4L2PixelFormat> &frameFormats =
				NeoDevice::frameFormats();
			pixelFormat = frameFormats[0].toPixelFormat();
			for (const V4L2PixelFormat &format : frameFormats)
				streamFormats[format.toPixelFormat()] = pixelRanges;

			const std::vector<V4L2PixelFormat> &irFormats =
				NeoDevice::irFormats();
			if (data->sensorIsRgbIr()) {
				for (const V4L2PixelFormat &format : irFormats)
					streamFormats[format.toPixelFormat()] = pixelRanges;
			}

			/*
			 * Select only one format per ISP capture node so that
			 * the resulting stream configuration passes validate()
			 * check.
			 */
			if (frameOutputAvailable) {
				pixelFormat = frameFormats[0].toPixelFormat();
				colorSpace = ColorSpace::Sycc;
				frameOutputAvailable = false;
			} else if (irOutputAvailable) {
				pixelFormat = irFormats[0].toPixelFormat();
				colorSpace = ColorSpace::Raw;
				irOutputAvailable = false;
			} else {
				LOG(NxpNeoPipe, Error) << "Too many yuv/rgb streams";
				return nullptr;
			}
			ASSERT(pixelSizes.size());
			cfgSize = pixelSizes.back();

			break;
		}

		case StreamRole::Raw: {
			/*
			 * Expose the resolutions associated to each mbus code
			 * available from the different sensor modes.
			 */
			if (!rawOutputAvailable) {
				LOG(NxpNeoPipe, Error) << "Too many raw streams";
				return nullptr;
			}

			const std::map<unsigned int, std::vector<Size>> &codeToSizes =
				data->rawFormatsCodeToSizes();
			for (const auto &[code, sizes] : codeToSizes) {
				std::vector<SizeRange> sensorRanges;
				const BayerFormat &bayerFormat =
					BayerFormat::fromMbusCode(code);
				pixelFormat = bayerFormat.toPixelFormat();
				for (const Size &size : sizes)
					sensorRanges.emplace_back(size);
				streamFormats[pixelFormat] = sensorRanges;
				ASSERT(sizes.size());
				cfgSize = sizes.back();
			}

			colorSpace = ColorSpace::Raw;
			rawOutputAvailable = false;
			break;
		}

		default:
			LOG(NxpNeoPipe, Error)
				<< "Requested stream role not supported: " << role;
			return nullptr;
		}

		StreamFormats formats(streamFormats);
		StreamConfiguration cfg(formats);
		cfg.size = cfgSize;
		cfg.pixelFormat = pixelFormat;
		cfg.colorSpace = colorSpace;
		const GlobalInfo &globalInfo =
			data->pipe()->pipelineConfig()->globalInfo();
		cfg.bufferCount = globalInfo.bufferCount;

		config->addConfiguration(cfg);
		LOG(NxpNeoPipe, Debug)
			<< "Generated configuration " << cfg.toString()
			<< " for role " << role;
	}

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

int PipelineHandlerNxpNeo::configure(Camera *camera, CameraConfiguration *c)
{
	NxpNeoCameraData *data = cameraData(camera);
	return data->configure(c);
}

int PipelineHandlerNxpNeo::exportFrameBuffers(Camera *camera, Stream *stream,
					      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	NxpNeoCameraData *data = cameraData(camera);
	return data->exportFrameBuffers(stream, buffers);
}

int PipelineHandlerNxpNeo::start(Camera *camera, const ControlList *controls)
{
	NxpNeoCameraData *data = cameraData(camera);
	return data->start(controls);
}

void PipelineHandlerNxpNeo::stopDevice(Camera *camera)
{
	NxpNeoCameraData *data = cameraData(camera);
	data->stopDevice();
}

int PipelineHandlerNxpNeo::queueRequestDevice(Camera *camera, Request *request)
{
	NxpNeoCameraData *data = cameraData(camera);

	data->pendingRequests_.push(request);
	data->queuePendingRequests();

	return 0;
}

bool PipelineHandlerNxpNeo::match(DeviceEnumerator *enumerator)
{
	int ret;

	/*
	 * Prerequisite for pipeline operation is that frontend media controller
	 * device is present.
	 */
	DeviceMatch isi(ISIDevice::kDriverName());
	isi.add(ISIDevice::kSDevCrossBarEntityName());
	isi.add(ISIDevice::kSDevPipeEntityName(0));
	isi.add(ISIDevice::kVDevPipeEntityName(0));

	MediaDevice *isiMedia = acquireMediaDevice(enumerator, isi);
	if (!isiMedia)
		return false;

	isi_ = std::make_shared<ISIDevice>();
	ret = isi_->init(isiMedia);
	if (ret) {
		LOG(NxpNeoPipe, Debug) << "ISI media device init failed";
		return false;
	}

	ret = loadPipelineConfig();
	if (ret)
		return false;

	/*
	 * Discover camera entities from the frontend media controller device
	 * Bind each camera to an ISP entity
	 */
	numCameras_ = 0;

	DeviceMatch isp(NeoDevice::kDriverName());
	isp.add(NeoDevice::kSDevNeoEntityName());
	isp.add(NeoDevice::kVDevInput0EntityName());
	isp.add(NeoDevice::kVDevInput1EntityName());
	isp.add(NeoDevice::kVDevEntityParamsName());
	isp.add(NeoDevice::kVDevEntityFrameName());
	isp.add(NeoDevice::kVDevEntityIrName());
	isp.add(NeoDevice::kVDevEntityStatsName());

	for (MediaEntity *entity : isiMedia->entities()) {
		if (entity->function() != MEDIA_ENT_F_CAM_SENSOR)
			continue;

		MediaDevice *neoDevice = acquireMediaDevice(enumerator, isp);
		if (!neoDevice)
			break;

		ret = createCamera(entity, neoDevice, numCameras_);
		if (ret)
			LOG(NxpNeoPipe, Warning) << "Failed to probe camera "
						 << entity->name() << ": " << ret;
		else
			numCameras_++;
	}

	if (numCameras_ < 1)
		return false;

	return true;
}

bool PipelineHandlerNxpNeo::acquireDevice(Camera *camera)
{
	NxpNeoCameraData *data = cameraData(camera);

	acquireCount_++;
	LOG(NxpNeoPipe, Debug) << "acquireDevice " << data->cameraName()
			       << " count " << acquireCount_;
	if (acquireCount_ > 1)
		return true;

	/*
	 * Frontend media controller device has been locked by the process.
	 * Global routing for all cameras is to be configured now as it will no
	 * longer be possible to update it after any streaming has started.
	 * Also, camera graphs in multi-camera condition should be statically
	 * preconfigured as they are dependent on each other.
	 */
	int ret = setupRouting();
	if (ret)
		return false;

	ret = setupCameraGraphs();
	return (!ret);
}

void PipelineHandlerNxpNeo::releaseDevice(Camera *camera)
{
	NxpNeoCameraData *data = cameraData(camera);

	ASSERT(acquireCount_);
	acquireCount_--;
	LOG(NxpNeoPipe, Debug) << "releaseDevice " << data->cameraName()
			       << " count " << acquireCount_;
}

/**
 * \brief Probe, configure and register camera sensor
 * \return 0 on success or a negative error code otherwise
 */
int PipelineHandlerNxpNeo::createCamera(MediaEntity *sensorEntity,
					MediaDevice *neoMedia,
					unsigned int neoInstance)
{
	int ret;

	std::unique_ptr<CameraSensor> sensor =
		CameraSensorFactoryBase::create(sensorEntity);
	if (!sensor)
		return -ENODEV;

	std::string name = sensorEntity->name();
	const CameraInfo *cameraInfo = pipelineConfig_.cameraInfo(name);
	if (!cameraInfo) {
		LOG(NxpNeoPipe, Warning) << "No CameraInfo for " << name;
		return -EINVAL;
	}

	std::unique_ptr<NeoDevice> neo = std::make_unique<NeoDevice>(neoInstance);
	ret = neo->init(neoMedia);
	if (ret)
		return ret;

	/* CameraData instance creation */
	std::unique_ptr<NxpNeoCameraData> data =
		std::make_unique<NxpNeoCameraData>(this,
						   std::move(sensor),
						   std::move(neo),
						   cameraInfo);

	ret = data->init();
	if (ret)
		return ret;

	/* Create and register the Camera instance. */
	std::set<Stream *> streams = {
		&data->streamFrame_,
		&data->streamIr_,
		&data->streamRaw_,
	};
	const std::string &cameraId = data->sensor()->id();
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), cameraId, streams);

	registerCamera(std::move(camera));

	return 0;
}

/**
 * \brief Configure the V4L2 subdevices routing
 *
 * Configure the subdevices routing in the system. As routing configuration can
 * not be updated while a device is streaming, and because subdevices may be
 * shared by the streams from multiple cameras, routing has to be setup
 * once at startup and no longer updated afterwards.
 *
 * \return 0 on success, or a negative error code otherwise
 */
int PipelineHandlerNxpNeo::setupRouting() const
{
	int ret;

	const RoutingMap &routingMap = pipelineConfig_.routingMap();

	for (const auto &[entity, routing] : routingMap) {
		const std::string &name = entity->name();
		LOG(NxpNeoPipe, Debug)
			<< "Configure routing for entity " << name
			<< " routing " << routing;

		std::unique_ptr<V4L2Subdevice> sdev =
			V4L2Subdevice::fromEntityName(isiDevice()->media(), name);
		if (!sdev.get()) {
			LOG(NxpNeoPipe, Error) << "Subdevice does not exist " << name;
			return -EINVAL;
		}

		ret = sdev->open();
		if (ret) {
			LOG(NxpNeoPipe, Error)
				<< "Error opening entity " << name;
			return -EINVAL;
		}

		V4L2Subdevice::Routing _routing = routing;
		ret = sdev->setRouting(&_routing, V4L2Subdevice::ActiveFormat);
		if (ret) {
			LOG(NxpNeoPipe, Error)
				<< "Error setting routing for entity " << name;
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * \brief Initialize the multi-camera graphs from the media controller device
 *
 * Cameras managed by the pipeline operate on different streams of the frontend
 * media controller device. Those streams share subdevice pads that may be
 * common to multiple cameras.
 * When multiple cameras are multiplexed over the same MIPI-CSI2 port, typically
 * through the usage of a GMSL SerDes, some limitations coming from the frontend
 * media device apply to that set of cameras:
 * - A given camera graph to be started requires a valid format to be configured
 *   for every other camera graphs of the set
 * - A camera graph can not be reconfigured when an other camera from the set is
 *   active
 * With such multi-camera case, these limitations prevent from configuring the
 * camera graph at configure() time, because an other camera may already be
 * streaming. Thus, a defaut graph configuration is necessary for each camera of
 * the set before streaming operation is started on another camera. This is done
 * when the frontend media device is locked.
 * Configuration of the ISP device will still be done at configure() time as
 * there is one ISP media instance per camera. These ISP instances can be
 * reconfigured independently from each other.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineHandlerNxpNeo::setupCameraGraphs()
{
	int ret = 0;

	for (auto const &camera : manager_->cameras()) {
		/* Make sure this camera is controlled by our pipeline */
		if (camera->_d()->pipe() != this) {
			LOG(NxpNeoPipe, Debug)
				<< "Skip setup for " << camera->id();
			continue;
		}

		NxpNeoCameraData *data = cameraData(camera.get());
		LOG(NxpNeoPipe, Debug)
			<< "Setup graph for camera " << data->cameraName();

		if (!data->multiCamera())
			continue;

		/* Configure the default format on that camera frontend graph */
		V4L2SubdeviceFormat sensorFormat = {};
		const std::map<Size, std::vector<unsigned int>> &sizeToCodes =
			data->rawFormatsSizeToCodes();
		ASSERT(sizeToCodes.size() == 1);
		sensorFormat.size = sizeToCodes.begin()->first;
		const std::vector<unsigned int> &codes = sizeToCodes.begin()->second;
		ASSERT(codes.size() == 1);
		sensorFormat.code = codes.back();

		ret = data->configureFrontEndFormat(sensorFormat,
						    Transform::Identity);
		if (ret)
			return ret;
	}

	return ret;
}

/**
 * \brief Load the pipeline configuration file
 * \return 0 on success, or a negative error code otherwise
 */
int PipelineHandlerNxpNeo::loadPipelineConfig()
{
	int ret;
	std::string file;
	char const *configFromEnv =
		utils::secure_getenv("LIBCAMERA_NXP_NEO_CONFIG_FILE");
	if (configFromEnv && *configFromEnv != '\0')
		file = std::string(configFromEnv);
	else
		file = std::string(NXP_NEO_PIPELINE_DATA_DIR) +
		       std::string("/config.yaml");

	ret = pipelineConfig_.load(file, isi_);

	return ret;
}

int NxpNeoCameraData::configure(CameraConfiguration *c)
{
	NxpNeoCameraConfiguration *config =
		static_cast<NxpNeoCameraConfiguration *>(c);
	int ret;

	LOG(NxpNeoPipe, Debug) << "Configure " << cameraName();

	/*
	 * Camera frontend graph reconfiguration is only applicable to
	 * single camera case. For multi-camera case, it has been statically
	 * configured at frontend media device acquisition time.
	 */
	if (!multiCamera()) {
		V4L2SubdeviceFormat sensorFormat = config->sensorFormat();
		ret = configureFrontEndFormat(sensorFormat,
					      config->combinedTransform());
		if (ret)
			return ret;
	}

	/*
	 * ISP configuration
	 */

	/* Bypass ISP configuration in raw-only mode of operation */
	V4L2DeviceFormat devFormatFrame = {};
	V4L2DeviceFormat devFormatIr = {};

	V4L2DeviceFormat &devFormatInput0 =
		pipesDevFormats_[CameraInfo::STREAM_INPUT0];
	V4L2DeviceFormat &devFormatInput1 =
		pipesDevFormats_[CameraInfo::STREAM_INPUT1];

	rawStreamOnly_ = ((config->size() == 1) &&
			  ((*config)[0].stream() == &streamRaw_));
	if (!rawStreamOnly_) {
		for (unsigned int i = 0; i < config->size(); ++i) {
			StreamConfiguration &cfg = (*config)[i];
			Stream *stream = cfg.stream();

			if (stream == &streamRaw_)
				continue;

			const auto fmts =
				V4L2PixelFormat::fromPixelFormat(cfg.pixelFormat);
			V4L2PixelFormat fmt;
			if (fmts.size())
				fmt = fmts[0];

			V4L2DeviceFormat &devFormat =
				stream == &streamFrame_ ? devFormatFrame : devFormatIr;
			devFormat.size = cfg.size;
			devFormat.fourcc = fmt;

			/* Use libcamera sYCC colorspace definition that maps
			 * to a v4l2 sRGB colorspace equivalent.
			 */
			if (cfg.colorSpace == ColorSpace::Srgb)
				devFormat.colorSpace = ColorSpace::Sycc;
			else
				devFormat.colorSpace = cfg.colorSpace;
		}

		NeoDevice::PipeConfig pipeConfig = {};
		pipeConfig.topLines = embeddedTopLines_;
		ret = neo_->configure(pipeConfig,
				      &devFormatInput0, &devFormatInput1,
				      &devFormatFrame, &devFormatIr);
		if (ret)
			return ret;
	}

	/*
	 * For sensors using an auxiliary stream, when the raw stream is present
	 * distribute it alternately between the two input streams if they share
	 * the same video format, meaning that they can share the same buffers.
	 */
	if (devFormatInput0.fourcc == devFormatInput1.fourcc &&
	    devFormatInput0.size == devFormatInput1.size)
		alternatedRawStream_ = true;
	else
		alternatedRawStream_ = false;
	LOG(NxpNeoPipe, Debug) << "alternated raw streams " << alternatedRawStream_;

	/*
	 * IPA configuration
	 */
	IPACameraSensorInfo sensorInfo;
	ret = sensor_->sensorInfo(&sensorInfo);
	if (ret)
		return ret;
	adjustTopLinesSize(&sensorInfo.outputSize);

	std::map<unsigned int, IPAStream> streamConfig;

	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = (*config)[i];
		Stream *stream = cfg.stream();

		if (stream == &streamFrame_)
			streamConfig[0] = IPAStream(cfg.pixelFormat, cfg.size);
		else if (stream == &streamIr_)
			streamConfig[1] = IPAStream(cfg.pixelFormat, cfg.size);
	}

	ipa::nxpneo::IPAConfigInfo configInfo;
	configInfo.sensorControls = sensor_->controls();
	configInfo.sensorInfo = sensorInfo;

	ret = ipa_->configure(configInfo, streamConfig, &ipaControls_);
	if (ret) {
		LOG(NxpNeoPipe, Error) << "Failed to configure IPA: "
				       << strerror(-ret);
		return ret;
	}

	return updateControls();
}

int NxpNeoCameraData::exportFrameBuffers(Stream *stream,
					 std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &streamFrame_)
		return neo_->frame_->exportBuffers(count, buffers);
	else if (stream == &streamIr_)
		return neo_->ir_->exportBuffers(count, buffers);
	else if (stream == &streamRaw_)
		return pipes_[CameraInfo::STREAM_INPUT0]->exportBuffers(count, buffers);

	return -EINVAL;
}

int NxpNeoCameraData::start([[maybe_unused]] const ControlList *controls)
{
	int ret;

	LOG(NxpNeoPipe, Debug) << "Start " << cameraName();
	sequence_ = 0;

	/* Allocate buffers for internal pipeline usage. */
	ret = allocateBuffers();
	if (ret)
		return ret;

	ret = ipa_->start();
	if (ret)
		goto error;

	delayedCtrls_->reset();

	/*
	 * Start the Neo and ISI video devices.
	 * ISI secondary streams are started first, then the primary stream
	 */

	ret = neo_->start();
	if (ret)
		goto error;

	for (auto [stream, pipe] : pipes_) {
		if (stream == CameraInfo::STREAM_INPUT0)
			continue;
		ret = pipes_[stream]->start();
		if (ret)
			goto error;
	}

	ret = pipes_[CameraInfo::STREAM_INPUT0]->start();
	if (ret)
		goto error;

	return 0;

error:
	neo_->stop();

	pipes_[CameraInfo::STREAM_INPUT0]->stop();
	for (auto [stream, pipe] : pipes_) {
		if (stream == CameraInfo::STREAM_INPUT0)
			continue;
		pipes_[stream]->stop();
	}

	ipa_->stop();
	freeBuffers();
	LOG(NxpNeoPipe, Error) << "Failed to start camera " << cameraName();

	return ret;
}

void NxpNeoCameraData::stopDevice()
{
	int ret = 0;

	LOG(NxpNeoPipe, Debug) << "Stop device " << cameraName();

	/*
	 * Requests in the pending queue have not been pushed into the pipeline,
	 * thus cancelled buffers and requests can be completed immediately.
	 * Conversely, requests in the processing list are in flight in the
	 * pipeline. Associated buffers are cancelled and completed along with
	 * their requests. Bundled Frame::Info object is deleted so that those
	 * will be ignored during pipeline termination.
	 */
	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();
		pipe()->cancelRequest(request);
		pendingRequests_.pop();
	}

	while (!processingRequests_.empty()) {
		Request *request = processingRequests_.front();
		NxpNeoFrames::Info *info = frameInfos_.find(request);
		if (!info) {
			LOG(NxpNeoPipe, Warning) << "Frame info for request not found";
			break;
		}
		cancelCompleteRequest(info);
	}

	ipa_->stop();

	ret = pipes_[CameraInfo::STREAM_INPUT0]->stop();
	for (auto [stream, pipe] : pipes_) {
		if (stream == CameraInfo::STREAM_INPUT0)
			continue;
		ret |= pipes_[stream]->stop();
	}

	ret |= neo_->stop();

	if (ret)
		LOG(NxpNeoPipe, Warning) << "Failed to stop camera " << cameraName();

	freeBuffers();
}

void NxpNeoCameraData::queuePendingRequests()
{
	FrameBuffer *reqRawBuffer;
	NxpNeoFrames::Info *info;
	int ret = 0;

	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();

		reqRawBuffer = request->findBuffer(&streamRaw_);
		info = frameInfos_.create(request, rawStreamOnly_, reqRawBuffer);
		if (!info)
			break;

		V4L2VideoDevice *dev;
		FrameBuffer *buffer;

		for (auto [stream, pipe] : pipes_) {
			dev = pipe->output_.get();

			if (stream == CameraInfo::STREAM_INPUT0) {
				buffer = info->input0Buffer;
			} else if (stream == CameraInfo::STREAM_INPUT1) {
				buffer = info->input1Buffer;
			} else if (stream == CameraInfo::STREAM_EDATA) {
				buffer = info->eDataBuffer;
			} else {
				LOG(NxpNeoPipe, Error) << "Invalid stream " << stream;
				continue;
			};

			buffer->_d()->setRequest(request);
			ret |= dev->queueBuffer(buffer);
		}

		if (ret) {
			LOG(NxpNeoPipe, Error)
				<< "Failed to queue buffers, unbalanced queues";
			pipe()->cancelRequest(request);
			frameInfos_.destroy(info->id);
			pendingRequests_.pop();
			return;
		}

		info->paramsBuffer->_d()->setRequest(request);
		info->statsBuffer->_d()->setRequest(request);

		ipa_->queueRequest(info->id, request->controls());

		pendingRequests_.pop();
		processingRequests_.push(request);
	}

	return;
}

/**
 * \brief Initialize sensor, frontend, IPA and callbacks
 * \return 0 on success or a negative error code otherwise
 */
int NxpNeoCameraData::init()
{
	int ret;

	ret = enumerateRawFormats();
	if (ret) {
		LOG(NxpNeoPipe, Warning) << "No supported format for " << cameraName();
		return -EINVAL;
	}

	ret = loadIPA();
	if (ret)
		return ret;

	updateControls();

	/* Initialize the camera properties. */
	properties_ = sensor_->properties();

	const auto &rotation = properties_.get(properties::Rotation);
	mountingOrientation_ = orientationFromRotation(rotation.value_or(0));

	neo_->isp_->frameStart.connect(this, &NxpNeoCameraData::frameStart);

	/*
	 * Connect video devices' 'bufferReady' signals to their
	 * slot to implement the image processing pipeline.
	 *
	 * Frames produced by the ISI unit are passed to the
	 * associated NEO inputs where they get processed and
	 * returned through the NEO main and IR outputs.
	 */

	if (!cameraInfo_->stream(CameraInfo::STREAM_INPUT0)) {
		LOG(NxpNeoPipe, Error)
			<< "Mandatory stream input 0 is missing for " << cameraName();
		return -ENODEV;
	}

	const std::map<unsigned int, void (NxpNeoCameraData::*)(FrameBuffer *)> pipeReadyFuncs{
		{ CameraInfo::STREAM_INPUT0, &NxpNeoCameraData::isiInput0BufferReady },
		{ CameraInfo::STREAM_INPUT1, &NxpNeoCameraData::isiInput1BufferReady },
		{ CameraInfo::STREAM_EDATA, &NxpNeoCameraData::isiEmbeddedDataBufferReady },
	};

	ISIDevice *isi = pipe()->isiDevice();
	for (auto stream : CameraInfo::kCameraStreams) {
		const CameraMediaStream *cameraMediaStream = cameraInfo_->stream(stream);
		if (!cameraMediaStream)
			continue;
		unsigned int pipeIndex = cameraMediaStream->pipe();
		pipes_[stream] = isi->getPipeByIndex(pipeIndex);

		auto it = pipeReadyFuncs.find(stream);
		ASSERT(it != pipeReadyFuncs.end());
		pipes_[stream]->bufferReady().connect(this, it->second);
	}

	neo_->input0_->bufferReady.connect(
		this, &NxpNeoCameraData::neoInput0BufferReady);
	neo_->input1_->bufferReady.connect(
		this, &NxpNeoCameraData::neoInput1BufferReady);
	neo_->frame_->bufferReady.connect(
		this, &NxpNeoCameraData::neoOutputBufferReady);
	neo_->ir_->bufferReady.connect(
		this, &NxpNeoCameraData::neoOutputBufferReady);
	neo_->params_->bufferReady.connect(
		this, &NxpNeoCameraData::neoParamsBufferReady);
	neo_->stats_->bufferReady.connect(
		this, &NxpNeoCameraData::neoStatsBufferReady);

	return 0;
}

PipelineHandlerNxpNeo *NxpNeoCameraData::pipe()
{
	PipelineHandler *pipe = Camera::Private::pipe();
	return static_cast<PipelineHandlerNxpNeo *>(pipe);
}

/**
 * \brief Adjust a size to crop the top embedded data lines when present
 * \param[inout] Size The original size to be adjusted
 *
 * Some sensors have embedded data lines inserted at the top of the video frame.
 * Those lines are present in the video device buffers from frontend capture
 * (output) device. This function adjusts a size to the value it will have
 * after the top lines are cropped.
 */
void NxpNeoCameraData::adjustTopLinesSize(Size *size) const
{
	if (embeddedTopLines_) {
		ASSERT(size->height >= embeddedTopLines_);
		size->height -= embeddedTopLines_;
	}
}

/**
 * \brief Enumerate the compatible sizes and mbus-codes for the sensor
 *
 * Enumerate the sizes and associated mbus-codes provided by the sensor modes
 * compatible with the pipeline. Two maps are stored in the class for later
 * usage:
 * - All sizes associated to a given mbus code
 *   This map can be later accessed via getter rawFormatsCodeToSizes()
 * - All mbus codes associated to a given size
 *   This map can be later accessed via getter rawFormatsSizeToCodes()
 * Mbus codes selected have to be Bayer formats supported by the frontend and
 * the ISP. Also, size widths selected must be within ISP supported range.
 * In case of multi-camera condition, the set of available formats is limited
 * to a single default value that will be used for the graph preconfiguration.
 */
int NxpNeoCameraData::enumerateRawFormats()
{
	std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		rawFormatsSizeToCodes_;
	std::map<unsigned int, std::vector<Size>> &codeToSizes =
		rawFormatsCodeToSizes_;

	const std::vector<unsigned int> &mbusCodes = sensor_->mbusCodes();
	const std::map<uint32_t, V4L2PixelFormat> &isiFormats =
		ISIDevice::mediaBusToPixelFormats();
	const std::vector<V4L2PixelFormat> &neoPixelFormats =
		NeoDevice::input0Formats();

	/*  Camera formats filtering may be defined in the config file */
	std::optional<unsigned int> bppFilter =
		cameraInfo_->cameraProperties().formatBpp;
	std::optional<Size> sizeFilter =
		cameraInfo_->cameraProperties().formatSize;

	for (unsigned int code : mbusCodes) {
		const BayerFormat &bayerFormat = BayerFormat::fromMbusCode(code);
		if (!bayerFormat.isValid())
			continue;

		if (!isiFormats.count(code))
			continue;

		if (std::find(neoPixelFormats.begin(), neoPixelFormats.end(),
			      isiFormats.at(code)) == neoPixelFormats.end())
			continue;

		if (bppFilter && bayerFormat.bitDepth != bppFilter.value())
			continue;

		std::vector<Size> sizes = sensor_->sizes(code);
		for (const Size &size : sizes) {
			if (size.width > NeoDevice::kRawWidthMax)
				continue;

			if (sizeFilter && size != sizeFilter.value())
				continue;

			sizeToCodes[size].push_back(code);
			codeToSizes[code].push_back(size);
		}
	}

	/* Make sure there is at least one compatible size and code */
	if (!sizeToCodes.size() || !sizeToCodes.begin()->second.size()) {
		LOG(NxpNeoPipe, Debug)
			<< "No compatible sensor format found for the pipeline";
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

	/*
	 * For multi-camera, default configuration is set arbitrarily to the
	 * highest size/bitdepth.
	 */
	if (multiCamera()) {
		ASSERT(sizeToCodes.size());
		const Size &sizeMax = sizeToCodes.rbegin()->first;
		const std::vector<unsigned int> &codes = sizeToCodes.rbegin()->second;
		ASSERT(codes.size());
		unsigned int codeBitDepthMax = codes.back();
		sizeToCodes.clear();
		sizeToCodes[sizeMax] = { codeBitDepthMax };
		codeToSizes.clear();
		codeToSizes[codeBitDepthMax] = { sizeMax };
	}

	std::ostringstream oss;
	oss << "Raw formats size [ mbuscodes ] ";
	for (const auto &[size, codes] : sizeToCodes) {
		oss << size.toString() << " [ ";
		for (const unsigned int code : codes)
			oss << utils::hex(code) << " ";
		oss << size.toString() << "] ";
	}
	LOG(NxpNeoPipe, Debug) << oss.str();

	return 0;
}

/**
 * \brief Configure the front end media controller device
 * \param[in] sensorFormat The sensor subdevice format
 * \param[in] transform The sensor transform
 *
 * \return 0 in case of success or a negative error code
 */
int NxpNeoCameraData::configureFrontEndFormat(V4L2SubdeviceFormat &sensorFormat,
					      Transform transform)
{
	int ret;
	CameraSensor *sensor = this->sensor();

	/* Configure entities media links */
	ret = configureFrontEndLinks();
	if (ret)
		return ret;

	/* Configure sensor internal streams (disabling may fail for immutable routes) */
	if (sensor->auxiliaryStream().has_value()) {
		bool enable = pipes_.count(CameraInfo::STREAM_INPUT1);
		ret = sensor->setAuxiliaryEnabled(enable);
		if (ret && enable) {
			LOG(NxpNeoPipe, Warning)
				<< "Auxiliary stream configuration failed"
				<< " [" << enable << "]";
			return ret;
		}
	}

	if (sensor->embeddedDataStream().has_value()) {
		bool enable = pipes_.count(CameraInfo::STREAM_EDATA);
		ret = sensor->setEmbeddedDataEnabled(enable);
		if (ret && enable) {
			LOG(NxpNeoPipe, Warning)
				<< "Embedded data stream configuration failed"
				<< " [" << enable << "]";
			return ret;
		}
	}

	/* Configure sensor format */
	ret = sensor->setFormat(&sensorFormat, transform);
	if (ret)
		return ret;

	/* Configure the stream formats for each stream */
	pipesDevFormats_.clear();
	for (auto [stream, pipe] : pipes_) {
		const CameraMediaStream *cameraInfoStream = cameraInfo_->stream(stream);
		ASSERT(cameraInfoStream);
		const std::vector<CameraMediaStream::StreamLink> &streamLinks =
			cameraInfoStream->streamLinks();

		V4L2SubdeviceFormat format;
		if (stream == CameraInfo::STREAM_INPUT0) {
			format = sensorFormat;
		} else if (stream == CameraInfo::STREAM_INPUT1) {
			format = sensor->auxiliaryFormat();
		} else if (stream == CameraInfo::STREAM_EDATA) {
			format = sensor->embeddedDataFormat();
		} else {
			LOG(NxpNeoPipe, Error) << "Invalid stream " << stream;
			continue;
		};

		ret = configureFrontEndStream(streamLinks, format);
		if (ret)
			return ret;

		V4L2DeviceFormat devFormat;
		ret = pipe->configure(format, &devFormat);
		if (ret)
			return ret;
		pipesDevFormats_[stream] = std::move(devFormat);
	}

	return ret;
}

/**
 * \brief Update the camera controls
 *
 * Compute the camera controls by calculating controls which the pipeline
 * is reponsible for and merge them with the controls computed by the IPA.
 *
 * This function needs data->ipaControls_ to be refreshed when a new
 * configuration is applied to the camera by the IPA configure() function.
 *
 * Always call this function after IPA configure() to make sure to have a
 * properly refreshed IPA controls list.
 *
 * \return 0 on success or a negative error code otherwise
 */
int NxpNeoCameraData::updateControls()
{
	ControlInfoMap::Map controls = {};

	/* Add the IPA registered controls to list of camera controls. */
	for (const auto &ipaControl : ipaControls_)
		controls[ipaControl.first] = ipaControl.second;

	controlInfo_ = ControlInfoMap(std::move(controls),
				      controls::controls);

	return 0;
}

int NxpNeoCameraData::loadIPA()
{
	ipa_ = IPAManager::createIPA<ipa::nxpneo::IPAProxyNxpNeo>(pipe(), 1, 1);
	if (!ipa_)
		return -ENOENT;

	ipa_->setSensorControls.connect(this, &NxpNeoCameraData::ipaSetSensorControls);
	ipa_->paramsBufferReady.connect(this, &NxpNeoCameraData::ipaParamsBufferReady);
	ipa_->metadataReady.connect(this, &NxpNeoCameraData::ipaMetadataReady);

	IPACameraSensorInfo sensorInfo{};
	CameraSensor *sensor = this->sensor();
	int ret = sensor->sensorInfo(&sensorInfo);
	if (ret)
		return ret;

	/*
	 * The API tuning file is made from the sensor name. If the tuning file
	 * isn't found, fall back to the 'uncalibrated' file.
	 */
	std::string ipaTuningFile = ipa_->configurationFile(sensor->model() + ".yaml");
	if (ipaTuningFile.empty())
		ipaTuningFile = ipa_->configurationFile("uncalibrated.yaml");

	uint32_t hwRevision = 0;
	ipa::nxpneo::SensorConfig sensorConfig;
	const MediaEntity *entity = sensor->entity();
	ipa::nxpneo::InitParams initParams = { hwRevision, entity->name(),
					       sensorInfo, sensor->controls() };
	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor->model() },
			 initParams, &ipaControls_, &sensorConfig);

	if (ret) {
		LOG(NxpNeoPipe, Error) << "Failed to initialise the NxpNeo IPA";
		return ret;
	}

	sensorIsRgbIr_ = sensorConfig.rgbIr;
	embeddedTopLines_ = sensorConfig.embeddedTopLines;

	/*
	 * Delayed controls definition from the IPA init() has priority over the
	 * definition from the global sensor properties.
	 */
	std::map<int32_t, ipa::nxpneo::DelayedControlsParams> &ipaDelayParams =
		sensorConfig.delayedControlsParams;
	std::unordered_map<uint32_t, DelayedControls::ControlParams>
		delayedControlsParams;
	for (const auto &kv : ipaDelayParams) {
		auto k = kv.first;
		auto v = kv.second;
		DelayedControls::ControlParams params = { v.delay, v.priorityWrite };
		delayedControlsParams.emplace(k, params);
	}
	if (!delayedControlsParams.size()) {
		const CameraSensorProperties::SensorDelays &delays =
			sensor->sensorDelays();
		delayedControlsParams = {
			{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
			{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
		};
	}

	delayedCtrls_ =
		std::make_unique<DelayedControls>(sensor->device(),
						  delayedControlsParams);

	return 0;
}

/**
 * \brief Allocate buffers from ISI and ISP
 *
 * Internal buffers are allocated for ISI active channels and ISP params and
 * statistics buffers. Those buffers are aggregated into the list of shared
 * buffers between the pipeline and the IPA.
 * Lastly, the NxpNeoFrames object is initialized with respective internal
 * buffer lists in order to serve the incoming Request.
 *
 * \return 0 in case of success or a negative error code
 */
int NxpNeoCameraData::allocateBuffers()
{
	unsigned int bufferCount;
	int ret;

	bufferCount = std::max({
		streamFrame_.configuration().bufferCount,
		streamIr_.configuration().bufferCount,
		streamRaw_.configuration().bufferCount,
	});

	/* Allocate and map ISP buffers */
	ret = neo_->allocateBuffers(bufferCount);
	if (ret < 0)
		return ret;

	unsigned int ipaBufferId = 1;

	for (const std::unique_ptr<FrameBuffer> &buffer : neo_->paramsBuffers_) {
		buffer->setCookie(ipaBufferId++);
		ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
	}

	for (const std::unique_ptr<FrameBuffer> &buffer : neo_->statsBuffers_) {
		buffer->setCookie(ipaBufferId++);
		ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
	}

	for (auto [stream, pipe] : pipes_) {
		ret = pipe->allocateBuffers(bufferCount);
		if (ret)
			break;

		for (const std::unique_ptr<FrameBuffer> &buffer : pipe->buffers()) {
			buffer->setCookie(ipaBufferId++);
			ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
		}
	}

	if (ret) {
		freeBuffers();
		return ret;
	}

	ipa_->mapBuffers(ipaBuffers_);

	/* Reference can not be stored in a container, assign vectors one by one */
	const std::vector<std::unique_ptr<FrameBuffer>> empty;
	std::map<unsigned int, ISIPipe *>::iterator it;

	unsigned int stream;
	stream = CameraInfo::STREAM_INPUT0;
	const std::vector<std::unique_ptr<FrameBuffer>> &input0Buffers =
		pipes_.count(stream) ? pipes_[stream]->buffers() : empty;

	stream = CameraInfo::STREAM_INPUT1;
	const std::vector<std::unique_ptr<FrameBuffer>> &input1Buffers =
		pipes_.count(stream) ? pipes_[stream]->buffers() : empty;

	stream = CameraInfo::STREAM_EDATA;
	const std::vector<std::unique_ptr<FrameBuffer>> &eDataBuffers =
		pipes_.count(stream) ? pipes_[stream]->buffers() : empty;

	frameInfos_.init(input0Buffers, input1Buffers,
			 eDataBuffers,
			 neo_->paramsBuffers_, neo_->statsBuffers_,
			 alternatedRawStream_);

	frameInfos_.bufferAvailable.connect(
		this, &NxpNeoCameraData::queuePendingRequests);

	return 0;
}

/**
 * \brief Deallocate buffers from ISI and ISP
 * \return 0 in case of success or a negative error code
 */
int NxpNeoCameraData::freeBuffers()
{
	frameInfos_.clear();

	std::vector<unsigned int> ids;
	for (IPABuffer &ipabuf : ipaBuffers_)
		ids.push_back(ipabuf.id);

	ipa_->unmapBuffers(ids);
	ipaBuffers_.clear();

	neo_->freeBuffers();

	for (auto [stream, pipe] : pipes_)
		pipe->freeBuffers();

	return 0;
}

/**
 * \brief Configure the graph format for a stream of the camera
 * \param[in] streamLinks Vector of media links and streams
 * \param[in] sdFormat The subdevice format used for the stream
 *
 * The pad/stream involved in the camera stream graph are configured with the
 * specified format.
 *
 * \return 0 in case of success or a negative error code
 */
int NxpNeoCameraData::configureFrontEndStream(
	const std::vector<CameraMediaStream::StreamLink> &streamLinks,
	V4L2SubdeviceFormat &sdFormat)
{
	const MediaDevice *media = pipe()->isiDevice()->media();
	std::unique_ptr<V4L2Subdevice> subDev;
	int ret = 0;

	for (const auto &streamLink : streamLinks) {
		const MediaLink *mediaLink = streamLink.mediaLink_;
		const MediaPad *sourceMediaPad = mediaLink->source();
		const MediaPad *sinkMediaPad = mediaLink->sink();
		std::string sourceName = sourceMediaPad->entity()->name();
		std::string sinkName = sinkMediaPad->entity()->name();
		unsigned int sourcePad = sourceMediaPad->index();
		unsigned int sinkPad = sinkMediaPad->index();
		unsigned int sourceStream = streamLink.sourceStream_;
		unsigned int sinkStream = streamLink.sinkStream_;

		LOG(NxpNeoPipe, Debug)
			<< "Set format " << sdFormat.toString()
			<< " source " << sourceName << " "
			<< sourcePad << "/" << sourceStream
			<< " sink " << sinkName << " "
			<< sinkPad << "/" << sinkStream;

		subDev = V4L2Subdevice::fromEntityName(media, sourceName);
		ret = subDev->open();
		if (ret) {
			LOG(NxpNeoPipe, Warning)
				<< "Error opening subdev " << sourceName;
			return ret;
		}
		ret = subDev->setFormat({ sourcePad, sourceStream }, &sdFormat);
		if (ret) {
			LOG(NxpNeoPipe, Warning)
				<< "Error setting format " << sourceName;
			return ret;
		}

		/* Stop at capture video node */
		if (sinkMediaPad->entity()->function() == MEDIA_ENT_T_V4L2_VIDEO) {
			LOG(NxpNeoPipe, Debug)
				<< "Configuration completed at video device "
				<< sinkName;
			return 0;
		}

		subDev = V4L2Subdevice::fromEntityName(media, sinkName);
		ret = subDev->open();
		if (ret) {
			LOG(NxpNeoPipe, Warning)
				<< "Error opening subdev " << sinkName;
			return ret;
		}
		ret = subDev->setFormat({ sinkPad, sinkStream }, &sdFormat);
		if (ret) {
			LOG(NxpNeoPipe, Warning)
				<< "Error setting format " << sinkName;
			return ret;
		}
	}

	return 0;
}

/**
 * \brief Enable media links from the camera graph
 * \return 0 in case of success or a negative error code
 */
int NxpNeoCameraData::configureFrontEndLinks() const
{
	for (auto stream : CameraInfo::kCameraStreams) {
		const CameraMediaStream *cameraStream = cameraInfo_->stream(stream);
		if (!cameraStream)
			continue;

		std::vector<CameraMediaStream::StreamLink> links =
			cameraStream->streamLinks();
		for (auto &streamLink : links) {
			MediaLink *link = streamLink.mediaLink_;
			MediaPad *sourceMPad = link->source();
			MediaPad *sinkMPad = link->sink();
			std::string source = sourceMPad->entity()->name();
			std::string sink = sinkMPad->entity()->name();
			unsigned int sourcePad = sourceMPad->index();
			unsigned int sinkPad = sinkMPad->index();

			LOG(NxpNeoPipe, Debug)
				<< "Enable link stream " << stream
				<< " source "
				<< source << "/" << sourcePad
				<< " sink "
				<< sink << "/" << sinkPad;

			int ret = link->setEnabled(true);
			if (ret) {
				LOG(NxpNeoPipe, Error) << "Failed to enable Link";
				return ret;
			}
		}
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Buffer Handling
 */

/**
 * \brief Clear an active request by removing its reference from the pipeline
 * \param[in] info The frame Info bound to the request to be cleared
 *
 * Active requests in flight in the pipeline are tracked in the
 * processingRequest queue where they are processed in order.
 * When such request has been completed, the references to this request should
 * be removed from the pipeline handler.
 */
void NxpNeoCameraData::clearRequest(NxpNeoFrames::Info *info)
{
	Request *request = info->request;

	std::queue<Request *> &queue = processingRequests_;
	if (queue.empty() || queue.front() != request)
		LOG(NxpNeoPipe, Warning) << "Processing request not found";
	else
		queue.pop();

	int ret = frameInfos_.destroy(info->id);
	if (ret)
		LOG(NxpNeoPipe, Warning) << "Info frame could not be destroyed";
}

/**
 * \brief Complete an active request in the pipeline
 * \param[in] request The frame Info bound to the request to be cancelled
 *
 * Active requests in flight in the pipeline are tracked in the
 * processingRequest queue where they are processed in order.
 * When such request is cancelled, associated pending buffers should be marked
 * as cancelled before being individually completed. Then all the references
 * to this request should be removed from the pipeline handler.
 */
void NxpNeoCameraData::cancelCompleteRequest(NxpNeoFrames::Info *info)
{
	Request *request = info->request;
	pipe()->cancelRequest(request);

	clearRequest(info);
}

/**
 * \brief Complete an active request if no longer in use by the pipeline
 * \param[in] info The frame Info associated to the request
 *
 * Active requests in flight in the pipeline are tracked in the
 * processingRequest queue where they are processed in order.
 * When no more operation is needed by the pipeline handler on a request,
 * it can be completed. In that case, all the references to this request should
 * be removed from the pipeline handler.
 */
void NxpNeoCameraData::tryCompleteRequest(NxpNeoFrames::Info *info)
{
	Request *request = info->request;

	if (request->hasPendingBuffers())
		return;

	if (!info->metadataProcessed)
		return;

	if (!info->paramDequeued)
		return;

	pipe()->completeRequest(request);

	clearRequest(info);
}

/* -----------------------------------------------------------------------------
 * Buffer Ready slots
 */

/**
 * \brief Handle buffers availability at the ISI output
 * \param[in] info The frame info associated to ongoing request
 *
 * In case all front-end buffers associated to the request have been received,
 * the IPA can be invoked to retrieve ISP parameters. For a raw-only request
 * IPA is bypassed and request can be completed immediately.
 */
void NxpNeoCameraData::isiInputBufferReady(NxpNeoFrames::Info *info)
{
	if (info->input0Pending || info->input1Pending || info->eDataPending)
		return;

	if (!rawStreamOnly_) {
		std::map<uint32_t, uint32_t> bufferIds = {
			{ ipa::nxpneo::TypeParams, info->paramsBuffer->cookie() },
			{ ipa::nxpneo::TypeInput0, info->input0Buffer->cookie() },
		};

		if (info->input1Buffer) {
			bufferIds.insert(
				{ ipa::nxpneo::TypeInput1, info->input1Buffer->cookie() });
		}

		if (info->eDataBuffer) {
			bufferIds.insert(
				{ ipa::nxpneo::TypeEData, info->eDataBuffer->cookie() });
		}

		ipa_->fillParamsBuffer(info->id, bufferIds);
	} else {
		tryCompleteRequest(info);
	}
}

/**
 * \brief Handle INPUT0 buffers availability at the ISI output
 * \param[in] buffer The completed buffer
 */
void NxpNeoCameraData::isiInput0BufferReady(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	Request *request = info->request;

	unsigned int seq = buffer->metadata().sequence;
	if (seq != sequence_)
		LOG(NxpNeoPipe, Warning)
			<< "Input0 frame loss! expected " << sequence_
			<< " received " << seq;
	sequence_ = seq + 1;

	/*
	 * Record the sensor's timestamp in the request metadata.
	 *
	 * \todo The sensor timestamp should be better estimated by connecting
	 * to the V4L2Device::frameStart signal.
	 */
	request->metadata().set(controls::SensorTimestamp,
				buffer->metadata().timestamp);

	info->effectiveSensorControls =
		delayedCtrls_->get(buffer->metadata().sequence);

	if (request->findBuffer(&streamRaw_) == buffer)
		pipe()->completeBuffer(request, buffer);

	info->input0Pending = false;
	isiInputBufferReady(info);
}

/**
 * \brief Handle INPUT1 buffers availability at the ISI output
 * \param[in] buffer The completed buffer
 */
void NxpNeoCameraData::isiInput1BufferReady(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	Request *request = info->request;
	(void)request;

	if (request->findBuffer(&streamRaw_) == buffer)
		pipe()->completeBuffer(request, buffer);

	if (info->input0Pending)
		LOG(NxpNeoPipe, Warning) << "Out of order input frame receipt";

	info->input1Pending = false;
	isiInputBufferReady(info);
}

/**
 * \brief Handle Embedded Data buffers availability at the ISI output
 * \param[in] buffer The completed buffer
 *
 * Embedded data buffer is to be passed to IPA for 3A algorithms to use
 * along with sensor control info and ISP statistics.
 */
void NxpNeoCameraData::isiEmbeddedDataBufferReady(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	info->eDataPending = false;
	isiInputBufferReady(info);
}

/**
 * \brief Handle INPUT0 buffers consumed by ISP
 * \param[in] buffer The consumed buffer
 */
void NxpNeoCameraData::neoInput0BufferReady([[maybe_unused]] FrameBuffer *buffer)
{
	/* Nothing to do - buffer will be recycled when request completes */
}

/**
 * \brief Handle INPUT1 buffers consumed by ISP
 * \param[in] buffer The consumed buffer
 */
void NxpNeoCameraData::neoInput1BufferReady([[maybe_unused]] FrameBuffer *buffer)
{
	/* Nothing to do - buffer will be recycled when request completes */
}

/**
 * \brief Handle buffers completion at the NEO capture node
 * \param[in] buffer The completed buffer
 *
 * Buffers completed from the NEO output are directed to the application.
 * This callback is common to main (frame) and IR ISP outputs.
 */
void NxpNeoCameraData::neoOutputBufferReady(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	Request *request = info->request;
	pipe()->completeBuffer(request, buffer);

	tryCompleteRequest(info);
}

/**
 * \brief Handle params buffers consumed by ISP
 * \param[in] buffer The consumed buffer
 */
void NxpNeoCameraData::neoParamsBufferReady(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	info->paramDequeued = true;

	tryCompleteRequest(info);
}

/**
 * \brief Handle stats buffers produced by ISP
 * \param[in] buffer The produced buffer
 */
void NxpNeoCameraData::neoStatsBufferReady(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	std::map<uint32_t, uint32_t> bufferIds = {
		{ ipa::nxpneo::TypeStats, info->statsBuffer->cookie() },
	};

	ipa_->processStatsBuffer(info->id, bufferIds,
				 info->effectiveSensorControls);

	tryCompleteRequest(info);
}

/*
 * \brief Handle the start of frame exposure signal
 * \param[in] sequence The sequence number of frame
 *
 * Inspect the list of pending requests waiting for a RAW frame to be
 * produced and apply controls for the 'next' one.
 *
 * Some controls need to be applied immediately, such as the
 * TestPatternMode one. Other controls are handled through the delayed
 * controls class.
 */
void NxpNeoCameraData::frameStart(uint32_t sequence)
{
	delayedCtrls_->applyControls(sequence);

	if (processingRequests_.empty())
		return;

	/*
	 * Handle controls to be set immediately on the next frame.
	 * This currently only handle the TestPatternMode control.
	 *
	 * \todo Synchronize with the sequence number
	 */
	Request *request = processingRequests_.front();

	const auto &testPatternMode = request->controls().get(controls::draft::TestPatternMode);
	if (!testPatternMode)
		return;

	int ret = sensor_->setTestPatternMode(
		static_cast<controls::draft::TestPatternModeEnum>(*testPatternMode));
	if (ret) {
		LOG(NxpNeoPipe, Error)
			<< "Failed to set test pattern mode: " << ret;
		return;
	}

	request->metadata().set(controls::draft::TestPatternMode,
				*testPatternMode);
}

void NxpNeoCameraData::ipaParamsBufferReady(unsigned int id)
{
	NxpNeoFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

	/* Queue buffers from the request to ISP capture devices (outputs) */
	for (auto it : info->request->buffers()) {
		const Stream *stream = it.first;
		FrameBuffer *outbuffer = it.second;

		if (stream == &streamFrame_)
			neo_->frame_->queueBuffer(outbuffer);
		else if (stream == &streamIr_)
			neo_->ir_->queueBuffer(outbuffer);
	}

	info->paramsBuffer->_d()->metadata().planes()[0].bytesused =
		sizeof(struct neoisp_meta_params_s);
	neo_->params_->queueBuffer(info->paramsBuffer);
	neo_->stats_->queueBuffer(info->statsBuffer);

	/* Buffers coming from ISI pipes are already part of the request */
	neo_->input0_->queueBuffer(info->input0Buffer);
	if (pipes_.count(CameraInfo::STREAM_INPUT1))
		neo_->input1_->queueBuffer(info->input1Buffer);
}

void NxpNeoCameraData::ipaMetadataReady(unsigned int id, const ControlList &metadata)
{
	NxpNeoFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

	Request *request = info->request;
	request->metadata().merge(metadata);

	info->metadataProcessed = true;
	tryCompleteRequest(info);
}

void NxpNeoCameraData::ipaSetSensorControls([[maybe_unused]] unsigned int id,
					    const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerNxpNeo, "nxp/neo")

} /* namespace libcamera */
