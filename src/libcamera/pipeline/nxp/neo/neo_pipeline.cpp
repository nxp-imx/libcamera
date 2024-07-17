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
 * Copyright 2024 NXP
 */

#include <algorithm>
#include <iomanip>
#include <memory>
#include <queue>
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
	void cancelPendingRequests();

	int init();
	PipelineHandlerNxpNeo *pipe();

	bool sensorIsRgbIr() const { return sensorIsRgbIr_; }
	void adjustTopLinesSize(Size *size) const;
	unsigned int getRawMediaBusFormat(PixelFormat *pixelFormat = nullptr) const;
	int configureFrontEndFormat(const V4L2SubdeviceFormat &sensorFormat,
				    Transform transform);

	CameraSensor *sensor() const { return sensor_.get(); }
	std::string cameraName() const { return sensor_->entity()->name(); }

	bool rawStreamOnly_ = false;

	Stream streamFrame_;
	Stream streamIr_;
	Stream streamRaw_;

	/* Requests for which no buffer has been queued to the frontend  device yet */
	std::queue<Request *> pendingRequests_;
	/* Requests queued to the frontend device but not yet processed by the ISP */
	std::queue<Request *> processingRequests_;

private:
	int initControls();
	int updateControls();
	int loadIPA();

	int allocateBuffers();
	int freeBuffers();

	int setupCameraIsiPipes();
	int configureFrontEndStream(const std::vector<CameraMediaStream::StreamLink> &streamLinks,
				    V4L2SubdeviceFormat &sdFormat);
	int configureFrontEndLinks() const;

	bool screenCancelledBuffer(FrameBuffer *buffer, NxpNeoFrames::Info *info);

	int prepareISIPipeBuffers(ISIPipe *pipe, unsigned int bufferCount, unsigned int &id);

	void isiInput0BufferReady(FrameBuffer *buffer);
	void isiInput1BufferReady(FrameBuffer *buffer);
	void isiEdBufferReady(FrameBuffer *buffer);

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

	/* Front end pipes */
	std::shared_ptr<ISIPipe> pipeInput0_;
	std::shared_ptr<ISIPipe> pipeInput1_;
	std::shared_ptr<ISIPipe> pipeEmbedded_;

	/* Front end video device nodes format */
	V4L2DeviceFormat devFormatInput0_;
	V4L2DeviceFormat devFormatInput1_;
	V4L2DeviceFormat devFormatEmbedded_;

	NxpNeoFrames frameInfos_;

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
	/* \todo get number of buffers from configuration file */
	static constexpr unsigned int kBufferCount = 8;
	static constexpr unsigned int kMaxStreams = 3;

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
	void cancelRequest(Request *request);

	bool match(DeviceEnumerator *enumerator) override;

	unsigned int numCameras() const { return numCameras_; }
	bool multiCamera() const { return numCameras_ > 1; }
	ISIDevice *isiDevice() const { return isi_.get(); }
	MediaDevice *isiMedia() const { return isiMedia_; }

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
	std::unique_ptr<ISIDevice> isi_;
	MediaDevice *isiMedia_ = nullptr;
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
	 * Validate the requested transform against the sensor capabilities and
	 * rotation and store the final combined transform that configure() will
	 * need to apply to the sensor to save us working it out again.
	 * In multicamera mode, the graphs format is statically configured, so
	 * the only acceptable transform is the identity because applying other
	 * type of transform may change the sensor format.
	 */
	Orientation requestedOrientation = orientation;
	if (!data_->pipe()->multiCamera()) {
		combinedTransform_ = sensor->computeTransform(&orientation);
	} else {
		/*
		 * Find which orientation corresponds to Identity transform
		 * to update CameraConfiguration accordingly.
		 * \todo May be replaced by a CameraSensor helper to get the
		 * sensor mounting orientation directly.
		 */
		static const std::vector<Orientation> orientations = {
			Orientation::Rotate0,
			Orientation::Rotate0Mirror,
			Orientation::Rotate180,
			Orientation::Rotate180Mirror,
			Orientation::Rotate90Mirror,
			Orientation::Rotate270,
			Orientation::Rotate270Mirror,
			Orientation::Rotate90,
		};
		auto iter = std::find_if(orientations.begin(),
					 orientations.end(),
					 [&](const Orientation &o) {
						 orientation = o;
						 combinedTransform_ =
							 sensor->computeTransform(&orientation);
						 return combinedTransform_ == Transform::Identity;
					 });
		ASSERT(iter != orientations.end());
	}
	if (orientation != requestedOrientation)
		status = Adjusted;

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
			irCount++;
			cfg.setStream(streamIr);
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
	} else if ((irCount > 1) && !data_->sensorIsRgbIr()) {
		LOG(NxpNeoPipe, Debug) << "Sensor has no RGB-Ir support";
		return Invalid;
	}

	/*
	 * All streams shall use the same size and that size has to be
	 * supported by the sensor. First stream with valid resolution has its
	 * size used as reference for the other streams.
	 * If no valid size is found in any stream, or in multi camera mode,
	 * then fall back to the max supported size.
	 * Sensor and ISP sizes differ when sensor outputs top lines embedded
	 * data. Those lines are cropped at ISP input, thus streams that are
	 * decoded by ISP have their size adjusted accordingly.
	 */
	PixelFormat rawPixelFormat;
	unsigned int rawCode = data_->getRawMediaBusFormat(&rawPixelFormat);
	ASSERT(rawCode);

	std::vector<Size> sensorSizes = sensor->sizes(rawCode);
	std::vector<Size> pixelSizes(sensorSizes);
	for (Size &size : pixelSizes)
		data_->adjustTopLinesSize(&size);

	Size sensorSize = sensorSizes.back();
	Size pixelSize = pixelSizes.back();
	bool multiCamera = data_->pipe()->multiCamera();
	if (!multiCamera) {
		for (const StreamConfiguration &cfg : config_) {
			auto iter = std::find(pixelSizes.begin(),
					      pixelSizes.end(),
					      cfg.size);
			if (iter != pixelSizes.end()) {
				pixelSize = *iter;
				sensorSize = sensorSizes[iter - pixelSizes.begin()];
			}
		}
	}

	sensorFormat_ = {};
	sensorFormat_.code = rawCode;
	sensorFormat_.size = sensorSize;
	LOG(NxpNeoPipe, Debug) << "Sensor format " << sensorFormat_.toString();

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
			const PixelFormatInfo &info =
				PixelFormatInfo::info(cfg->pixelFormat);
			cfg->stride = info.stride(cfg->size.width, 0, 1);
			cfg->frameSize = info.frameSize(cfg->size, 1);

			LOG(NxpNeoPipe, Debug) << "Assigned " << cfg->toString()
					   << " to the "
					   << (isFrame ? "frame" : "ir")
					   << " stream";
		} else if (isRaw) {
			cfg->pixelFormat = rawPixelFormat;
			cfg->size = sensorSize;
			const PixelFormatInfo &info =
				PixelFormatInfo::info(cfg->pixelFormat);
			cfg->stride = info.stride(cfg->size.width, 0, 64);
			cfg->frameSize = info.frameSize(cfg->size, 64);

			LOG(NxpNeoPipe, Debug) << "Assigned " << cfg->toString()
					   << " to the raw stream";
		} else {
			LOG(NxpNeoPipe, Error) << "Unknown configuration stream";
			return Invalid;
		}

		cfg->bufferCount = NxpNeoCameraConfiguration::kBufferCount;

		if (cfg->pixelFormat != originalCfg.pixelFormat ||
		    cfg->size != originalCfg.size) {
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

	CameraSensor *sensor = data->sensor();
	PixelFormat rawPixelFormat;
	unsigned int rawCode = data->getRawMediaBusFormat(&rawPixelFormat);
	std::vector<Size> sensorSizes = sensor->sizes(rawCode);

	/* Top embedded data from sensor are cropped before being fed to ISP */
	std::vector<Size> pixelSizes(sensorSizes);
	for (Size &size : pixelSizes)
		data->adjustTopLinesSize(&size);

	/* Size configuration is possible only with a single camera */
	std::vector<SizeRange> sensorRanges;
	std::vector<SizeRange> pixelRanges;
	if (!multiCamera()) {
		for (Size &size : sensorSizes)
			sensorRanges.emplace_back(size);
		for (Size &size : pixelSizes)
			pixelRanges.emplace_back(size);
	} else {
		sensorRanges.emplace_back(sensorSizes.back());
		pixelRanges.emplace_back(pixelSizes.back());
	}

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
				frameOutputAvailable = false;
			} else if (irOutputAvailable) {
				pixelFormat = irFormats[0].toPixelFormat();
				irOutputAvailable = false;
			} else {
				LOG(NxpNeoPipe, Error) << "Too many yuv/rgb streams";
				return nullptr;
			}
			cfgSize = pixelSizes.back();

			break;
		}

		case StreamRole::Raw:
			/*
			 * Propose resolutions supported by sensor with raw
			 * format selected for the sensor.
			 */
			if (!rawOutputAvailable) {
				LOG(NxpNeoPipe, Error) << "Too many raw streams";
				return nullptr;
			}
			pixelFormat = rawPixelFormat;
			streamFormats[pixelFormat] = sensorRanges;
			rawOutputAvailable = false;
			cfgSize = sensorSizes.back();

			break;

		default:
			LOG(NxpNeoPipe, Error)
				<< "Requested stream role not supported: " << role;
			return nullptr;
		}

		StreamFormats formats(streamFormats);
		StreamConfiguration cfg(formats);
		cfg.size = cfgSize;
		cfg.pixelFormat = pixelFormat;
		cfg.bufferCount = NxpNeoCameraConfiguration::kBufferCount;

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

/**
 * \brief Cancel a \a Request.
 * \param[in] request The request to be cancelled
 *
 * Cancelling a request involves marking all its associated buffers as cancelled
 * before reporting them as complete with completeBuffer(). Finally, the request
 * itself is reported as complete with completeRequest().
 */
void PipelineHandlerNxpNeo::cancelRequest(Request *request)
{
	for (auto it : request->buffers()) {
		FrameBuffer *buffer = it.second;
		buffer->_d()->cancel();
		completeBuffer(request, buffer);
	}

	completeRequest(request);
}

bool PipelineHandlerNxpNeo::match(DeviceEnumerator *enumerator)
{
	int ret;
	constexpr unsigned int kMaxNeoDevices = 8;

	/*
	 * Prerequisite for pipeline operation is that frontend media controller
	 * device is present.
	 */
	DeviceMatch isi(ISIDevice::kDriverName());
	isi.add(ISIDevice::kSDevCrossBarEntityName());
	isi.add(ISIDevice::kSDevPipeEntityName(0));
	isi.add(ISIDevice::kVDevPipeEntityName(0));

	isiMedia_ = acquireMediaDevice(enumerator, isi);
	if (!isiMedia_)
		return false;

	isi_ = std::make_unique<ISIDevice>();
	ret = isi_->init(isiMedia_);
	if (ret)
		return false;

	ret = loadPipelineConfig();
	if (ret)
		return false;

	/*
	 * Discover Neo ISP media controller devices
	 */
	std::queue<MediaDevice *> neos;
	MediaDevice *neoDev;
	DeviceMatch isp(NeoDevice::kDriverName());
	isp.add(NeoDevice::kSDevNeoEntityName());
	isp.add(NeoDevice::kVDevInput0EntityName());
	isp.add(NeoDevice::kVDevInput1EntityName());
	isp.add(NeoDevice::kVDevEntityParamsName());
	isp.add(NeoDevice::kVDevEntityFrameName());
	isp.add(NeoDevice::kVDevEntityIrName());
	isp.add(NeoDevice::kVDevEntityStatsName());

	for (unsigned int i = 0; i < kMaxNeoDevices; i++) {
		neoDev = acquireMediaDevice(enumerator, isp);
		if (neoDev)
			neos.push(neoDev);
	}
	if (!neos.size())
		return false;

	/*
	 * Discover camera entities from the frontend media controller device
	 * Bind each camera to an ISP entity
	 */
	numCameras_ = 0;
	for (MediaEntity *entity : isiMedia_->entities()) {
		if (entity->function() != MEDIA_ENT_F_CAM_SENSOR)
			continue;

		if (!neos.size())
			break;

		ret = createCamera(entity, neos.front(), numCameras_);
		if (ret) {
			LOG(NxpNeoPipe, Warning) << "Failed to probe camera "
					     << entity->name() << ": " << ret;
		} else {
			numCameras_++;
			neos.pop();
		}
	}

	if (numCameras_ < 1)
		return false;

	/* Apply global routing and setup cameras frontend graphs */
	ret = setupRouting();
	if (ret)
		return ret;
	ret = setupCameraGraphs();
	if (ret)
		return ret;

	return true;
}

/**
 * \brief Probe, configure and register camera sensor
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineHandlerNxpNeo::createCamera(MediaEntity *sensorEntity,
					MediaDevice *neoMedia,
					unsigned int neoInstance)
{
	int ret;

	std::unique_ptr<CameraSensor> sensor;
	sensor = std::make_unique<CameraSensor>(sensorEntity);
	ret = sensor->init();
	if (ret)
		return ret;

	std::string name = sensorEntity->name();
	const CameraInfo *cameraInfo = pipelineConfig_.getCameraInfo(name);
	if (!cameraInfo) {
		LOG(NxpNeoPipe, Warning) << "No CameraInfo for " << name;
		return -EINVAL;
	}

	std::unique_ptr<NeoDevice> neo;
	neo = std::make_unique<NeoDevice>(neoInstance);
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
 * \return 0 on success, or a negative error code otherwise.
 */
int PipelineHandlerNxpNeo::setupRouting() const
{
	int ret;

	const RoutingMap &routingMap = pipelineConfig_.getRoutingMap();

	for (const auto &[entity, routing] : routingMap) {
		const std::string &name = entity->name();
		LOG(NxpNeoPipe, Debug)
			<< "Configure routing for entity " << name
			<< " routing " << routing;

		std::unique_ptr<V4L2Subdevice> sdev =
			V4L2Subdevice::fromEntityName(isiMedia_, name);
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
 * \brief Initialize the camera graphs from the media controller device
 *
 * Cameras managed by the pipeline operate on different streams of the media
 * controller device. Those streams share subdevice pads common to multiple
 * cameras.
 * A given stream to be started requires a valid format to be configured
 * for every other stream sharing a media entity pad.
 * Also, a stream can not be reconfigured for a subdevice that has an other
 * stream active.
 * In multicamera case, it prevents from configuring the graph at configure()
 * time, because an other camera may already be streaming at that time. Thus,
 * for multicamera, frontend graph is staticallly configured at pipeline
 * creation time.
 * Configuration of the ISP device will still be done at configure() time as
 * there is one device instance per camera, so there is no issue of sharing
 * streams on common entity pad.
 *
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineHandlerNxpNeo::setupCameraGraphs()
{
	int ret = 0;

	if (!multiCamera())
		return 0;

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

		/* Apply default format and transform to each media pad streams */
		unsigned int rawCode = data->getRawMediaBusFormat();
		ASSERT(rawCode);

		CameraSensor *sensor = data->sensor();
		std::vector<Size> sizes = sensor->sizes(rawCode);
		Size size = sizes.back();

		V4L2SubdeviceFormat sensorFormat = {};
		sensorFormat.code = rawCode;
		sensorFormat.size = size;

		ret = data->configureFrontEndFormat(sensorFormat,
						    Transform::Identity);
		if (ret)
			return ret;
	}

	return ret;
}

/**
 * \brief Load the pipeline configuration file
 * \return 0 on success, or a negative error code otherwise.
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

	ret = pipelineConfig_.load(file, isiMedia_, isi_.get());

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
	 * single camera case. For multicamera case, they have been statically
	 * configured at pipeline creation time.
	 */
	if (!pipe()->multiCamera()) {
		ret = configureFrontEndFormat(config->sensorFormat(),
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

	rawStreamOnly_ = ((config->size() == 1) &&
			  ((*config)[0].stream() == &streamRaw_));
	if (!rawStreamOnly_) {
		for (unsigned int i = 0; i < config->size(); ++i) {
			StreamConfiguration &cfg = (*config)[i];
			Stream *stream = cfg.stream();

			const auto fmts =
				V4L2PixelFormat::fromPixelFormat(cfg.pixelFormat);
			V4L2PixelFormat fmt;
			if (fmts.size())
				fmt = fmts[0];

			if (stream == &streamFrame_) {
				devFormatFrame.size = cfg.size;
				devFormatFrame.fourcc = fmt;
			} else if (stream == &streamIr_) {
				devFormatIr.size = cfg.size;
				devFormatIr.fourcc = fmt;
			}
		}

		NeoDevice::PipeConfig pipeConfig = {};
		pipeConfig.topLines = embeddedTopLines_;

		ret = neo_->configure(pipeConfig,
				      &devFormatInput0_, &devFormatInput1_,
				      &devFormatFrame, &devFormatIr);
		if (ret)
			return ret;
	}

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
		return pipeInput0_->exportBuffers(count, buffers);

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
	 * Start the Neo and ISI video devices, buffers will be queued to the
	 * Neo frame and IR outputs when requests will be queued.
	 * INPUT0 ISI stream is mandatory, INPUT1 and Embedded Data are optional
	 */

	ret = neo_->start();
	if (ret)
		goto error;

	if (cameraInfo_->hasStreamInput1()) {
		ret = pipeInput1_->start();
		if (ret)
			goto error;
	}

	if (cameraInfo_->hasStreamEmbedded()) {
		ret = pipeEmbedded_->start();
		if (ret)
			goto error;
	}

	ret = pipeInput0_->start();
	if (ret)
		goto error;

	return 0;

error:
	neo_->stop();

	pipeInput0_->stop();
	if (cameraInfo_->hasStreamInput1())
		pipeInput1_->stop();
	if (cameraInfo_->hasStreamEmbedded())
		pipeEmbedded_->stop();

	ipa_->stop();
	freeBuffers();
	LOG(NxpNeoPipe, Error) << "Failed to start camera " << cameraName();

	return ret;
}

void NxpNeoCameraData::stopDevice()
{
	int ret = 0;

	LOG(NxpNeoPipe, Debug) << "Stop device " << cameraName();

	cancelPendingRequests();

	ipa_->stop();

	ret |= pipeInput0_->stop();
	if (cameraInfo_->hasStreamInput1())
		ret |= pipeInput1_->stop();
	if (cameraInfo_->hasStreamEmbedded())
		ret |= pipeEmbedded_->stop();

	ret |= neo_->stop();

	if (ret)
		LOG(NxpNeoPipe, Warning) << "Failed to stop camera " << cameraName();

	freeBuffers();
}

void NxpNeoCameraData::queuePendingRequests()
{
	FrameBuffer *reqRawBuffer;
	NxpNeoFrames::Info *info;
	int ret;

	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();

		reqRawBuffer = request->findBuffer(&streamRaw_);
		info = frameInfos_.create(request, rawStreamOnly_, reqRawBuffer);
		if (!info)
			break;

		V4L2VideoDevice *dev;
		FrameBuffer *buffer;
		buffer = info->input0Buffer;
		dev = pipeInput0_->output_.get();
		buffer->_d()->setRequest(request);
		ret = dev->queueBuffer(buffer);
		if (cameraInfo_->hasStreamInput1()) {
			buffer = info->input1Buffer;
			dev = pipeInput1_->output_.get();
			buffer->_d()->setRequest(request);
			ret |= dev->queueBuffer(buffer);
		}
		if (cameraInfo_->hasStreamEmbedded()) {
			buffer = info->embeddedBuffer;
			dev = pipeEmbedded_->output_.get();
			buffer->_d()->setRequest(request);
			ret |= dev->queueBuffer(buffer);
		}
		if (ret) {
			LOG(NxpNeoPipe, Error)
				<< "Failed to queue buffers, unbalanced queues";
			pipe()->cancelRequest(request);
			frameInfos_.remove(info);
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

void NxpNeoCameraData::cancelPendingRequests()
{
	processingRequests_ = {};

	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();
		pipe()->cancelRequest(request);
		pendingRequests_.pop();
	}
}

/**
 * \brief Initialize sensor, frontend, IPA and callbacks
 * \return 0 on success or a negative error code otherwise.
 */
int NxpNeoCameraData::init()
{
	int ret;

	unsigned rawCode = getRawMediaBusFormat();
	if (!rawCode) {
		LOG(NxpNeoPipe, Warning) << "No supported format for " << cameraName();
		return -EINVAL;
	}

	ret = loadIPA();
	if (ret)
		return ret;

	/* Initialize the camera properties. */
	properties_ = sensor_->properties();
	ret = initControls();
	if (ret)
		return ret;

	neo_->isp_->frameStart.connect(this, &NxpNeoCameraData::frameStart);

	/* Convert the sensor rotation to a transformation */
	const auto &rotation = properties_.get(properties::Rotation);
	if (!rotation)
		LOG(NxpNeoPipe, Warning) << "Rotation control not exposed by "
				     << cameraName()
				     << ". Assume rotation 0";

	/*
	 * Connect video devices' 'bufferReady' signals to their
	 * slot to implement the image processing pipeline.
	 *
	 * Frames produced by the ISI unit are passed to the
	 * associated NEO inputs where they get processed and
	 * returned through the NEO main and IR outputs.
	 */
	ret = setupCameraIsiPipes();
	if (ret)
		return ret;

	pipeInput0_->bufferReady().connect(
		this, &NxpNeoCameraData::isiInput0BufferReady);

	if (cameraInfo_->hasStreamInput1()) {
		pipeInput1_->bufferReady().connect(
			this, &NxpNeoCameraData::isiInput1BufferReady);
	}
	if (cameraInfo_->hasStreamEmbedded()) {
		pipeEmbedded_->bufferReady().connect(
			this, &NxpNeoCameraData::isiEdBufferReady);
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
 * \brief Get raw media bus format compatible with the sensor, frontend and ISP
 * \param[out] pixelFormat The associated pixel format for a raw stream
 *
 * Look for a raw mbus code supported by the sensor, that is compatible with
 * frontend (ISI) and ISP format capabilities.
 * If several formats are possible, report a format with the largest bit depth.
 *
 * \return The corresponding media bus format, or zero if none is found.
 */
unsigned int NxpNeoCameraData::getRawMediaBusFormat(PixelFormat *pixelFormat) const
{
	std::vector<unsigned int> mbusCodes = sensor_->mbusCodes();

	unsigned int sensorCode = 0;
	const std::map<uint32_t, V4L2PixelFormat> &isiFormats =
		ISIDevice::mediaBusToPixelFormats();
	const std::vector<V4L2PixelFormat> &neoPixelFormats =
		NeoDevice::input0Formats();

	unsigned int maxDepth = 0;
	sensorCode = 0;
	if (pixelFormat)
		*pixelFormat = {};

	for (unsigned int code : mbusCodes) {
		/* Make sure the media bus format is RAW Bayer. */
		const BayerFormat &bayerFormat = BayerFormat::fromMbusCode(code);
		if (!bayerFormat.isValid())
			continue;

		/* Make sure the media format is supported by ISI. */
		if (std::find_if(isiFormats.begin(),
				 isiFormats.end(),
				 [&](auto isiFormat) {
					 return isiFormat.first == code;
				 }) == isiFormats.end())
			continue;

		/* Make sure the media format is supported by NEO. */
		if (std::find_if(neoPixelFormats.begin(),
				 neoPixelFormats.end(),
				 [&](auto neoPixelFormat) {
					 return neoPixelFormat.fourcc() ==
						isiFormats.at(code).fourcc();
				 }) == neoPixelFormats.end())
			continue;

		/* Pick the one with the largest bit depth. */
		if (bayerFormat.bitDepth > maxDepth) {
			maxDepth = bayerFormat.bitDepth;
			sensorCode = code;
			if (pixelFormat)
				*pixelFormat = isiFormats.at(code).toPixelFormat();
		}
	}

	if (!sensorCode)
		LOG(NxpNeoPipe, Debug) << "Cannot find a supported RAW format";

	return sensorCode;
}

/**
 * \brief Configure the front end media controller device
 * \param[in] sensorFormat The sensor subdevice format
 * \param[in] transform The sensor transform
 * \param[out] vdFormatInput0 The input0 front end's capture video device format
 * \param[out] vdFormatInput1 The input1 front end's capture video device format
 * \param[out] vdFormatEd The embedded data front end's capture video device
 * format
 *
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::configureFrontEndFormat(const V4L2SubdeviceFormat &sensorFormat,
					      Transform transform)
{
	int ret;
	CameraSensor *sensor = this->sensor();

	/* Configure entities media links */
	ret = configureFrontEndLinks();
	if (ret)
		return ret;

	/* Configure entities pad formats for each stream present */
	V4L2SubdeviceFormat sdFormatInput0 = sensorFormat;
	ret = sensor->setFormat(&sdFormatInput0, transform);
	if (ret)
		return ret;

	const CameraMediaStream *streamInput0 = cameraInfo_->getStreamInput0();
	ASSERT(streamInput0);
	const std::vector<CameraMediaStream::StreamLink> &streamLinksInput0 =
		streamInput0->streamLinks();
	ret = configureFrontEndStream(streamLinksInput0, sdFormatInput0);
	if (ret)
		return ret;

	V4L2SubdeviceFormat sdFormatInput1 = {};
	if (cameraInfo_->hasStreamInput1()) {
		const CameraMediaStream *streamInput1 =
			cameraInfo_->getStreamInput1();
		ASSERT(streamInput1);
		/*
		 * \todo Retrieve input1 format from sensor when we support
		 * camera stream api.
		 */
		unsigned int code = streamInput1->mbusCode();
		sdFormatInput1.code = code;
		sdFormatInput1.size = sensorFormat.size;
		const std::vector<CameraMediaStream::StreamLink> &streamLinksInput1 =
			streamInput1->streamLinks();
		ret = configureFrontEndStream(streamLinksInput1, sdFormatInput1);
		if (ret)
			return ret;
	}

	V4L2SubdeviceFormat sdFormatEd = {};
	if (cameraInfo_->hasStreamEmbedded()) {
		const CameraMediaStream *streamEmbedded =
			cameraInfo_->getStreamEmbedded();
		ASSERT(streamEmbedded);
		/*
		 * \todo Retrieve embedded format from sensor when we support
		 * camera stream api.
		 */
		unsigned int code =
			streamEmbedded->mbusCode();
		sdFormatEd.code = code;
		unsigned int lines =
			streamEmbedded->embeddedLines();
		sdFormatEd.size = Size(sensorFormat.size.width, lines);
		const std::vector<CameraMediaStream::StreamLink> &streamLinksEmbedded =
			streamEmbedded->streamLinks();
		ret = configureFrontEndStream(streamLinksEmbedded, sdFormatEd);
		if (ret)
			return ret;
	}

	/* Configure ISI capture video devices */
	devFormatInput0_ = {};
	devFormatInput1_ = {};
	devFormatEmbedded_ = {};

	ret = pipeInput0_->configure(sdFormatInput0, &devFormatInput0_);
	if (cameraInfo_->hasStreamInput1())
		ret |= pipeInput1_->configure(sdFormatInput1, &devFormatInput1_);
	if (cameraInfo_->hasStreamEmbedded())
		ret |= pipeEmbedded_->configure(sdFormatEd, &devFormatEmbedded_);

	return ret;
}

/**
 * \brief Initialize the camera controls
 *
 * Initialize the camera controls by calculating controls which the pipeline
 * is reponsible for and merge them with the controls computed by the IPA.
 *
 * This function needs data->ipaControls_ to be initialized by the IPA init()
 * function at camera creation time. Always call this function after IPA init().
 *
 * \return 0 on success or a negative error code otherwise.
 */
int NxpNeoCameraData::initControls()
{
	/*
	 * \todo The controls initialized here depend on sensor configuration
	 * and their limits should be updated once the configuration gets
	 * changed.
	 *
	 * Initialize the sensor using its resolution and compute the control
	 * limits.
	 */
	CameraSensor *sensor = this->sensor();
	V4L2SubdeviceFormat sensorFormat = {};
	sensorFormat.size = sensor->resolution();
	int ret = sensor->setFormat(&sensorFormat);
	if (ret)
		return ret;

	return updateControls();
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
 * \return 0 on success or a negative error code otherwise.
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
	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor->model() },
			 hwRevision,
			 sensorInfo, sensor->controls(),
			 &ipaControls_,
			 &sensorConfig);

	if (ret) {
		LOG(NxpNeoPipe, Error) << "Failed to initialise the NxpNeo IPA";
		return ret;
	}

	sensorIsRgbIr_ = sensorConfig.rgbIr;
	embeddedTopLines_ = sensorConfig.embeddedTopLines;

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

	/*
	 * DelayedControls parameters come from prior IPA init().
	 */
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
 * \return 0 in case of success or a negative error code.
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
	const std::vector<std::unique_ptr<FrameBuffer>> emptyBufferVector;

	for (const std::unique_ptr<FrameBuffer> &buffer : neo_->paramsBuffers_) {
		buffer->setCookie(ipaBufferId++);
		ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
	}

	for (const std::unique_ptr<FrameBuffer> &buffer : neo_->statsBuffers_) {
		buffer->setCookie(ipaBufferId++);
		ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
	}

	ISIPipe *pipeInput0 = pipeInput0_.get();
	ret = prepareISIPipeBuffers(pipeInput0, bufferCount, ipaBufferId);
	if (ret) {
		freeBuffers();
		return ret;
	}

	ISIPipe *pipeInput1 = pipeInput1_.get();
	if (cameraInfo_->hasStreamInput1()) {
		ret = prepareISIPipeBuffers(pipeInput1, bufferCount, ipaBufferId);
		if (ret) {
			freeBuffers();
			return ret;
		}
	}

	ISIPipe *pipeEmbedded = pipeEmbedded_.get();
	if (cameraInfo_->hasStreamEmbedded()) {
		ret = prepareISIPipeBuffers(pipeEmbedded, bufferCount, ipaBufferId);
		if (ret) {
			freeBuffers();
			return ret;
		}
	}

	ipa_->mapBuffers(ipaBuffers_);

	const std::vector<std::unique_ptr<FrameBuffer>> &input0Buffers =
		pipeInput0->buffers();

	const std::vector<std::unique_ptr<FrameBuffer>> &input1Buffers =
		cameraInfo_->hasStreamInput1() ?
		pipeInput1->buffers() :
		emptyBufferVector;

	const std::vector<std::unique_ptr<FrameBuffer>> &embeddedBuffers =
		cameraInfo_->hasStreamEmbedded() ?
		pipeEmbedded->buffers() :
		emptyBufferVector;

	frameInfos_.init(input0Buffers, input1Buffers,
			 embeddedBuffers,
			 neo_->paramsBuffers_, neo_->statsBuffers_);

	frameInfos_.bufferAvailable.connect(
		this, &NxpNeoCameraData::queuePendingRequests);

	return 0;
}

/**
 * \brief Deallocate buffers from ISI and ISP
 * \return 0 in case of success or a negative error code.
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

	ISIPipe *pipeInput0 = pipeInput0_.get();
	pipeInput0->freeBuffers();

	if (cameraInfo_->hasStreamInput1()) {
		ISIPipe *pipeInput1 = pipeInput1_.get();
		pipeInput1->freeBuffers();
	}

	if (cameraInfo_->hasStreamEmbedded()) {
		ISIPipe *pipeEmbedded = pipeEmbedded_.get();
		pipeEmbedded->freeBuffers();
	}

	return 0;
}

/**
 * \brief Acquire ISI pipes that have been reserved at enumeration time
 *
 * \return 0 in case of success or a negative error code.
 */

int NxpNeoCameraData::setupCameraIsiPipes()
{
	ISIDevice *isi = pipe()->isiDevice();
	unsigned int pipeIndex;
	auto isiPipeDeleter =
		[=](ISIPipe *_pipe) { isi->releasePipe(_pipe->index()); };

	const CameraMediaStream *streamInput0 =
		cameraInfo_->getStreamInput0();
	ASSERT(streamInput0);
	pipeIndex = streamInput0->pipe();
	pipeInput0_ = std::shared_ptr<ISIPipe>(isi->getPipeByIndex(pipeIndex),
					       isiPipeDeleter);
	if (!pipeInput0_.get())
		return -ENODEV;

	if (cameraInfo_->hasStreamInput1()) {
		const CameraMediaStream *streamInput1 =
			cameraInfo_->getStreamInput1();
		pipeIndex = streamInput1->pipe();
		pipeInput1_ = std::shared_ptr<ISIPipe>(isi->getPipeByIndex(pipeIndex),
						       isiPipeDeleter);
		if (!pipeInput1_.get())
			return -ENODEV;
	}

	if (cameraInfo_->hasStreamEmbedded()) {
		const CameraMediaStream *streamEmbedded =
			cameraInfo_->getStreamEmbedded();
		pipeIndex = streamEmbedded->pipe();
		pipeEmbedded_ = std::shared_ptr<ISIPipe>(isi->getPipeByIndex(pipeIndex),
							 isiPipeDeleter);
		if (!pipeEmbedded_.get())
			return -ENODEV;
	}

	return 0;
}

/**
 * \brief Configure the graph format for a stream of the camera
 * \param[in] streamLinks Vector of media links and streams.
 * \param[in] sdFormat The subdevice format used for the stream.
 *
 * The pad/stream involved in the camera stream graph are configured with the
 * specified format.
 *
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::configureFrontEndStream(
	const std::vector<CameraMediaStream::StreamLink> &streamLinks,
	V4L2SubdeviceFormat &sdFormat)
{
	const MediaDevice *media = pipe()->isiMedia();
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
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::configureFrontEndLinks() const
{
	int ret;

	/*
	 * Enable graph media links
	 */
	auto EnableLink = [](const CameraMediaStream *_stream,
			     std::string streamName) -> int {
		int res;
		ASSERT(_stream);
		std::vector<CameraMediaStream::StreamLink> links = _stream->streamLinks();
		for (auto &streamLink : links) {
			MediaLink *link = streamLink.mediaLink_;
			MediaPad *sourceMPad = link->source();
			MediaPad *sinkMPad = link->sink();
			std::string source = sourceMPad->entity()->name();
			std::string sink = sinkMPad->entity()->name();
			unsigned int sourcePad = sourceMPad->index();
			unsigned int sinkPad = sinkMPad->index();

			LOG(NxpNeoPipe, Debug)
				<< "Enable link stream " << streamName
				<< " source "
				<< source << "/" << sourcePad
				<< " sink "
				<< sink << "/" << sinkPad;

			res = link->setEnabled(true);
			if (res) {
				LOG(NxpNeoPipe, Error) << "Failed to enable Link";
				return res;
			}
		}

		return 0;
	};

	ret = EnableLink(cameraInfo_->getStreamInput0(), "input0");
	if (ret)
		return ret;

	if (cameraInfo_->hasStreamInput1()) {
		ret = EnableLink(cameraInfo_->getStreamInput1(), "input1");
		if (ret)
			return ret;
	}

	if (cameraInfo_->hasStreamEmbedded()) {
		ret = EnableLink(cameraInfo_->getStreamEmbedded(), "embedded");
		if (ret)
			return ret;
	}

	return ret;
}

/**
 * \brief Handle receipt of cancelled video buffer
 * \param[in] buffer The buffer received on capture node
 * \param[in] info Frame info associated to the buffer
 *
 * When camera is stopped, video devices involved in the pipeline are stopped
 * using stopDevice() method. Buffers queued on those video nodes are released
 * through bufferReady() signal with a status set to FrameCancelled.
 * Such buffers shall be detected so that every other buffer bundled to
 * the same frame and associated request can be marked as cancelled and
 * completed.
 */
bool NxpNeoCameraData::screenCancelledBuffer(FrameBuffer *buffer,
					     NxpNeoFrames::Info *info)
{
	Request *request = info->request;

	/* If the buffer is cancelled force a complete of the whole request. */
	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		pipe()->cancelRequest(request);

		frameInfos_.remove(info);

		return true;
	}

	return false;
}

/**
 * \brief Allocate buffers from ISI pipe and append to IPA shared buffers list
 * \param[in] pipe The ISI pipe to allocate from
 * \param[in] bufferCount The number of buffers to allocate
 * \param[in] id The start value for the unique identifier of the IPA shared
 * buffer
 *
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::prepareISIPipeBuffers(ISIPipe *pipe,
					    unsigned int bufferCount,
					    unsigned int &id)
{
	int ret;
	ret = pipe->allocateBuffers(bufferCount);
	if (ret < 0)
		return ret;

	for (const std::unique_ptr<FrameBuffer> &buffer : pipe->buffers()) {
		buffer->setCookie(id++);
		ipaBuffers_.emplace_back(buffer->cookie(),
					 buffer->planes());
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Buffer Ready slots
 */

/**
 * \brief Handle INPUT0 buffers availability at the ISI output
 * \param[in] buffer The completed buffer
 *
 * Once params buffer for ISP has been produced by 3A, input buffers are
 * queued to NEO for further processing.
 * Buffers will be returned after being processed by ISP.
 */
void NxpNeoCameraData::isiInput0BufferReady(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	if (screenCancelledBuffer(buffer, info))
		return;

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

	if (request->findBuffer(&streamRaw_))
		pipe()->completeBuffer(request, buffer);

	if (!rawStreamOnly_) {
		/*
		 * INPUT0 frame will be queue into ISP once params buffer for
		 * the frame have been produced by IPA.
		 * \todo: in case of ISP operation with INPUT0 + INPUT1 inputs,
		 * wait for both frames to be available before invoking
		 * ipa_->fillParamsBuffer()
		 */
		ipa_->fillParamsBuffer(info->id,
				       info->paramsBuffer->cookie(),
				       info->input0Buffer->cookie());
	} else {
		if (frameInfos_.tryComplete(info))
			pipe()->completeRequest(request);
	}
}

/**
 * \brief Handle INPUT1 buffers availability at the ISI output
 * \param[in] buffer The completed buffer
 *
 * Once params buffer for ISP has been produced by 3A, input buffers are
 * queue to NEO for further processing.
 * Buffer will be returned after being ingested by ISP.
 */
void NxpNeoCameraData::isiInput1BufferReady(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	if (screenCancelledBuffer(buffer, info))
		return;

	Request *request = info->request;
	(void)request;

	/*
	 * \todo: in case of ISP operation with INPUT0 + INPUT1 inputs, wait for
	 *  both frames to be available before invoking ipa_->fillParamsBuffer()
	 */

	ASSERT(pipeInput1_);
}

/**
 * \brief Handle Embedded Data buffers availability at the ISI output
 * \param[in] buffer The completed buffer
 *
 * Embedded data buffer is to be passed to IPA for 3A algorithms to use
 * along with sensor control info and ISP statistics.
 */
void NxpNeoCameraData::isiEdBufferReady(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	if (screenCancelledBuffer(buffer, info))
		return;

	/*
	 * \todo inform IPA about this buffer
	 */
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

	if (screenCancelledBuffer(buffer, info))
		return;

	Request *request = info->request;
	pipe()->completeBuffer(request, buffer);

	if (frameInfos_.tryComplete(info))
		pipe()->completeRequest(request);
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

	/*
	 * tryComplete() will delete info if it completes the NxpNeoFrame.
	 * In that event, we must have obtained the Request before hand.
	 *
	 * \todo Improve the FrameInfo API to avoid this type of issue
	 */
	Request *request = info->request;

	if (frameInfos_.tryComplete(info))
		pipe()->completeRequest(request);
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

	if (screenCancelledBuffer(buffer, info))
		return;

	ipa_->processStatsBuffer(info->id, info->statsBuffer->cookie(),
				 info->effectiveSensorControls);
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
	processingRequests_.pop();

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
	if (cameraInfo_->hasStreamInput1())
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
	if (frameInfos_.tryComplete(info))
		pipe()->completeRequest(request);
}

void NxpNeoCameraData::ipaSetSensorControls([[maybe_unused]] unsigned int id,
					    const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerNxpNeo, "nxp/neo")

} /* namespace libcamera */
