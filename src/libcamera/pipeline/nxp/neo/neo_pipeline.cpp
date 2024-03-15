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

LOG_DEFINE_CATEGORY(NxpNeo)

using namespace libcamera::nxpneo;

class PipelineHandlerNxpNeo;

class NxpNeoCameraData : public Camera::Private
{
public:
	NxpNeoCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe){};

	int loadIPA();

	void neoInput0BufferReady(FrameBuffer *buffer);
	void neoInput1BufferReady(FrameBuffer *buffer);
	void neoOutputBufferReady(FrameBuffer *buffer);
	void isiInput0BufferReady(FrameBuffer *buffer);
	void isiInput1BufferReady(FrameBuffer *buffer);
	void isiEdBufferReady(FrameBuffer *buffer);

	void neoParamsBufferReady(FrameBuffer *buffer);
	void neoStatsBufferReady(FrameBuffer *buffer);
	void queuePendingRequests();
	void cancelPendingRequests();
	void frameStart(uint32_t sequence);

	bool sensorIsRgbIr() const;

	void ipaMetadataReady(unsigned int id, const ControlList &metadata);
	void ipaParamsBufferReady(unsigned int id);
	void ipaSetSensorControls(unsigned int id, const ControlList &sensorControls);

	unsigned int getRawMediaBusFormat(PixelFormat *pixelFormat) const;
	int setupCameraFormat();
	int setupCameraIsiReserve();
	int setupCameraStream(const std::vector<StreamLink> &streamLinks,
			      V4L2SubdeviceFormat &sdFormat);
	int setupCameraStreams(unsigned int mbusCodeInput0,
			       Size sizeInput0,
			       V4L2SubdeviceFormat *sdFormatInput0,
			       V4L2SubdeviceFormat *sdFormatInput1,
			       V4L2SubdeviceFormat *sdFormatEd);
	int setupCameraEnableLinks() const;
	int setupCameraConfigureIsi(
		const V4L2SubdeviceFormat &sdFormatInput0,
		const V4L2SubdeviceFormat &sdFormatInput1,
		const V4L2SubdeviceFormat &sdFormatEd,
		V4L2DeviceFormat *vdFormatInput0,
		V4L2DeviceFormat *vdFormatInput1,
		V4L2DeviceFormat *vdFormatEd);

	bool screenCancelledBuffer(FrameBuffer *buffer, NxpNeoFrames::Info *info);

	int prepareISIPipeBuffers(ISIPipe *pipe,
				  unsigned int bufferCount,
				  unsigned int &id);
	int allocateBuffers();
	int freeBuffers();

	std::string cameraName() const
	{
		return sensor_.get()->entity()->name();
	}

private:
	friend class PipelineHandlerNxpNeo;
	friend class NxpNeoCameraConfiguration;

	PipelineHandlerNxpNeo *pipe();

	const CameraInfo *cameraInfo_;

	std::unique_ptr<NeoDevice> neo_;

	Stream streamFrame_;
	Stream streamIr_;
	Stream streamRaw_;

	std::unique_ptr<DelayedControls> delayedCtrls_;
	NxpNeoFrames frameInfos_;

	std::unique_ptr<ipa::nxpneo::IPAProxyNxpNeo> ipa_;

	/* Requests for which no buffer has been queued to the ISI device yet. */
	std::queue<Request *> pendingRequests_;
	/* Requests queued to the ISI device but not yet processed by the NEO. */
	std::queue<Request *> processingRequests_;

	ControlInfoMap ipaControls_;
	std::unordered_map<uint32_t, DelayedControls::ControlParams>
		delayedControlsParams_;

	std::unique_ptr<CameraSensor> sensor_;

	std::shared_ptr<ISIPipe> pipeInput0_;
	std::shared_ptr<ISIPipe> pipeInput1_;
	std::shared_ptr<ISIPipe> pipeEmbedded_;

	std::vector<IPABuffer> ipaBuffers_;

	unsigned int mbusCode_ = 0;
	unsigned int sequence_ = 0;
	bool rawStreamOnly_ = false;
	PixelFormat rawPixelFormat_;
	V4L2SubdeviceFormat sdFormatInput0_;
	V4L2SubdeviceFormat sdFormatInput1_;
	V4L2SubdeviceFormat sdFormatEmbedded_;
	V4L2DeviceFormat vdFormatInput0_;
	V4L2DeviceFormat vdFormatInput1_;
	V4L2DeviceFormat vdFormatEmbedded_;
};

class NxpNeoCameraConfiguration : public CameraConfiguration
{
public:
	/* \todo get number of buffers from configuration file */
	static constexpr unsigned int kBufferCount = 6;
	static constexpr unsigned int kMaxStreams = 3;

	NxpNeoCameraConfiguration(NxpNeoCameraData *data);

	Status validate() override;

	/* Cache the combinedTransform_ that will be applied to the sensor */
	Transform combinedTransform_;

private:
	/*
	 * The NxpNeoCameraData instance is guaranteed to be valid as long as the
	 * corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	NxpNeoCameraData *data_;
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

	unsigned int numCameras() const
	{
		return manager_->cameras().size();
	}

	bool multiCamera() const
	{
		return (numCameras() > 1);
	}

	std::unique_ptr<ISIDevice> isi_;
	MediaDevice *isiMedia_ = nullptr;

private:
	NxpNeoCameraData *cameraData(Camera *camera)
	{
		return static_cast<NxpNeoCameraData *>(camera->_d());
	}

	int initControls(NxpNeoCameraData *data);
	int updateControls(NxpNeoCameraData *data);

	int createCamera(MediaEntity *sensorEntity, MediaDevice *neoMedia,
			 unsigned int neoInstance);

	int setupRouting() const;
	int setupCameraGraphs();
	int loadPipelineConfig();
	PipelineConfig pipelineConfig_;
};

PipelineHandlerNxpNeo *NxpNeoCameraData::pipe()
{
	PipelineHandler *pipe = Camera::Private::pipe();
	return static_cast<PipelineHandlerNxpNeo *>(pipe);
}

NxpNeoCameraConfiguration::NxpNeoCameraConfiguration(NxpNeoCameraData *data)
	: CameraConfiguration()
{
	data_ = data;
}

CameraConfiguration::Status NxpNeoCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	/*
	 * Validate the requested transform against the sensor capabilities and
	 * rotation and store the final combined transform that configure() will
	 * need to apply to the sensor to save us working it out again.
	 */
	Orientation requestedOrientation = orientation;
	combinedTransform_ = data_->sensor_->computeTransform(&orientation);
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
			LOG(NxpNeo, Debug) << "Unknown config pixel format";
			return Invalid;
		}
	}

	if (yuvRgbCount > 1) {
		LOG(NxpNeo, Debug) << "Multiple rgb/yuv streams not supported";
		return Invalid;
	} else if (rawCount > 1) {
		LOG(NxpNeo, Debug) << "Multiple raw streams not supported";
		return Invalid;
	} else if (irCount > 1) {
		LOG(NxpNeo, Debug) << "Multiple Ir streams not supported";
		return Invalid;
	} else if ((irCount > 1) && !data_->sensorIsRgbIr()) {
		LOG(NxpNeo, Debug) << "Sensor has no RGB-Ir support";
		return Invalid;
	}

	/*
	 * All streams shall use the same size and that size has to be
	 * supported by the sensor. First stream with valid resolution has its
	 * size used as reference for the other streams.
	 * If no valid size is found in any stream, or in multi camera mode,
	 * then fall back to the sensor max supported size.
	 */
	CameraSensor *sensor = data_->sensor_.get();
	std::vector<Size> sizes = sensor->sizes(data_->mbusCode_);
	Size size = sizes.back();
	bool multiCamera = data_->pipe()->multiCamera();
	if (!multiCamera) {
		auto iter = std::find_if(config_.begin(),
					 config_.end(),
					 [&sizes](auto &cfg) {
						return std::find(sizes.begin(),
								 sizes.end(),
								 cfg.size) !=
							sizes.end();
					 });
		if (iter != config_.end())
			size = iter->size;
	}

	for (unsigned int i = 0; i < config_.size(); ++i) {
		const StreamConfiguration originalCfg = config_[i];
		StreamConfiguration *cfg = &config_[i];

		bool isFrame = (streamFrame == cfg->stream());
		bool isIr = (streamIr == cfg->stream());
		bool isRaw = (streamRaw == cfg->stream());

		LOG(NxpNeo, Debug)
			<< "Stream " << i << " to validate cfg " << cfg->toString();

		if (isFrame || isIr) {
			const std::vector<V4L2PixelFormat> &formats =
				isFrame ? NeoDevice::frameFormats() : NeoDevice::irFormats();
			if (std::find_if(formats.begin(),
					 formats.end(),
					 [&](auto &format) {
						 return format.toPixelFormat() ==
							cfg->pixelFormat;
					 }) == formats.end())
				cfg->pixelFormat = formats[0].toPixelFormat();
			cfg->size = size;
			const PixelFormatInfo &info =
				PixelFormatInfo::info(cfg->pixelFormat);
			cfg->stride = info.stride(cfg->size.width, 0, 1);
			cfg->frameSize = info.frameSize(cfg->size, 1);

			LOG(NxpNeo, Debug) << "Assigned " << cfg->toString()
					   << " to the "
					   << (isFrame ? "frame" : "ir")
					   << " stream";
		} else if (isRaw) {
			cfg->pixelFormat = data_->rawPixelFormat_;
			cfg->size = size;
			const PixelFormatInfo &info =
				PixelFormatInfo::info(cfg->pixelFormat);
			cfg->stride = info.stride(cfg->size.width, 0, 64);
			cfg->frameSize = info.frameSize(cfg->size, 64);

			LOG(NxpNeo, Debug) << "Assigned " << cfg->toString()
					   << " to the raw stream";
		} else {
			LOG(NxpNeo, Error) << "Unknown configuration stream";
			return Invalid;
		}

		cfg->bufferCount = NxpNeoCameraConfiguration::kBufferCount;

		if (cfg->pixelFormat != originalCfg.pixelFormat ||
		    cfg->size != originalCfg.size) {
			status = Adjusted;
		}

		LOG(NxpNeo, Debug)
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
		std::make_unique<NxpNeoCameraConfiguration>(data);

	LOG(NxpNeo, Debug) << "Generate configuration " << data->cameraName();

	if (roles.empty())
		return config;

	bool frameOutputAvailable = true;
	bool irOutputAvailable = data->sensorIsRgbIr();
	bool rawOutputAvailable = true;

	CameraSensor *sensor = data->sensor_.get();
	std::vector<Size> sizes = sensor->sizes(data->mbusCode_);
	Size &maxSize = sizes.back();

	/* Size configuration is possible only with a single camera */
	std::vector<SizeRange> ranges;
	if (!multiCamera()) {
		for (Size size : sizes)
			ranges.emplace_back(size);
	} else {
		ranges.emplace_back(maxSize);
	}

	for (const StreamRole role : roles) {
		std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
		PixelFormat pixelFormat;

		switch (role) {
		case StreamRole::StillCapture:
		case StreamRole::Viewfinder:
		case StreamRole::VideoRecording: {
			/*
			 * Propose resolutions supported by sensor with output
			 * formats supported by the ISP.
			 */
			if (frameOutputAvailable) {
				const std::vector<V4L2PixelFormat> &formats =
					NeoDevice::frameFormats();
				pixelFormat = formats[0].toPixelFormat();
				for (const V4L2PixelFormat &format : formats)
					streamFormats[format.toPixelFormat()] = ranges;
				frameOutputAvailable = false;
			} else if (irOutputAvailable) {
				const std::vector<V4L2PixelFormat> &formats =
					NeoDevice::irFormats();
				pixelFormat = formats[0].toPixelFormat();
				for (const V4L2PixelFormat &format : formats)
					streamFormats[format.toPixelFormat()] = ranges;
				irOutputAvailable = false;
			} else {
				LOG(NxpNeo, Error) << "Too many yuv/rgb streams";
				return nullptr;
			}

			break;
		}

		case StreamRole::Raw:
			/*
			 * Propose resolutions supported by sensor with raw
			 * format selected for the sensor.
			 */
			if (!rawOutputAvailable) {
				LOG(NxpNeo, Error) << "Too many raw streams";
				return nullptr;
			}
			pixelFormat = data->rawPixelFormat_;
			streamFormats[pixelFormat] = ranges;
			rawOutputAvailable = false;
			break;

		default:
			LOG(NxpNeo, Error)
				<< "Requested stream role not supported: " << role;
			return nullptr;
		}

		StreamFormats formats(streamFormats);
		StreamConfiguration cfg(formats);
		cfg.size = maxSize;
		cfg.pixelFormat = pixelFormat;
		cfg.bufferCount = NxpNeoCameraConfiguration::kBufferCount;

		config->addConfiguration(cfg);
		LOG(NxpNeo, Debug)
			<< "Generated configuration " << cfg.toString()
			<< " for role " << role;
	}

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

int PipelineHandlerNxpNeo::configure(Camera *camera, CameraConfiguration *c)
{
	NxpNeoCameraConfiguration *config =
		static_cast<NxpNeoCameraConfiguration *>(c);
	NxpNeoCameraData *data = cameraData(camera);
	int ret;

	LOG(NxpNeo, Debug) << "Configure " << data->cameraName();

	/*
	 * Configurations have been through validate() so they can be applied
	 * directly.
	 */

	V4L2SubdeviceFormat *sdFormatInput0 = &data->sdFormatInput0_;
	V4L2SubdeviceFormat *sdFormatInput1 = &data->sdFormatInput1_;
	V4L2SubdeviceFormat *sdFormatEd = &data->sdFormatEmbedded_;

	V4L2DeviceFormat *vdFormatInput0 = &data->vdFormatInput0_;
	V4L2DeviceFormat *vdFormatInput1 = &data->vdFormatInput1_;
	V4L2DeviceFormat *vdFormatEd = &data->vdFormatEmbedded_;

	/*
	 * Camera graph media streams graph reconfiguration
	 * (single-camera pipeline only)
	 */
	if (!multiCamera()) {
		Size size = (*config)[0].size;
		unsigned int mbusCode = data->mbusCode_;

		/* Apply config format to each media pad streams */
		ret = data->setupCameraStreams(mbusCode, size, sdFormatInput0,
					       sdFormatInput1, sdFormatEd);
		if (ret)
			return ret;

		/* ISI video video devices configuration */
		ret = data->setupCameraConfigureIsi(
			*sdFormatInput0, *sdFormatInput1,
			*sdFormatEd,
			vdFormatInput0, vdFormatInput1,
			vdFormatEd);
		if (ret)
			return ret;
	}

	/*
	 * ISP configuration
	 */

	/* Bypass ISP configuration in raw-only mode of operation */
	V4L2DeviceFormat devFormatFrame = {};
	V4L2DeviceFormat devFormatIr = {};

	data->rawStreamOnly_ = ((config->size() == 1) &&
				((*config)[0].stream() == &data->streamRaw_));
	if (!data->rawStreamOnly_) {
		for (unsigned int i = 0; i < config->size(); ++i) {
			StreamConfiguration &cfg = (*config)[i];
			Stream *stream = cfg.stream();

			const auto fmts =
				V4L2PixelFormat::fromPixelFormat(cfg.pixelFormat);
			V4L2PixelFormat fmt;
			if (fmts.size())
				fmt = fmts[0];

			if (stream == &data->streamFrame_) {
				devFormatFrame.size = cfg.size;
				devFormatFrame.fourcc = fmt;
			} else if (stream == &data->streamIr_) {
				devFormatIr.size = cfg.size;
				devFormatIr.fourcc = fmt;
			}
		}

		ret = data->neo_->configure(*vdFormatInput0, *vdFormatInput1,
					    devFormatFrame, devFormatIr);
		if (ret)
			return ret;
	}

	/*
	 * Sensor transform configuration
	 */
	CameraSensor *sensor = data->sensor_.get();
	ret = sensor->setFormat(sdFormatInput0, config->combinedTransform_);
	if (ret)
		return ret;

	/*
	 * IPA configuration
	 */
	IPACameraSensorInfo sensorInfo;
	ret = sensor->sensorInfo(&sensorInfo);
	if (ret)
		return ret;

	std::map<unsigned int, IPAStream> streamConfig;

	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = (*config)[i];
		Stream *stream = cfg.stream();

		if (stream == &data->streamFrame_)
			streamConfig[0] = IPAStream(cfg.pixelFormat, cfg.size);
		else if (stream == &data->streamIr_)
			streamConfig[1] = IPAStream(cfg.pixelFormat, cfg.size);
	}

	ipa::nxpneo::IPAConfigInfo configInfo;
	configInfo.sensorControls = sensor->controls();
	configInfo.sensorInfo = sensorInfo;

	ret = data->ipa_->configure(configInfo, streamConfig, &data->ipaControls_);
	if (ret) {
		LOG(NxpNeo, Error) << "Failed to configure IPA: "
				   << strerror(-ret);
		return ret;
	}

	return updateControls(data);
}

int PipelineHandlerNxpNeo::exportFrameBuffers(Camera *camera, Stream *stream,
					      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	NxpNeoCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &data->streamFrame_) {
		return data->neo_->frame_->exportBuffers(count, buffers);
	} else if (stream == &data->streamIr_) {
		return data->neo_->ir_->exportBuffers(count, buffers);
	} else if (stream == &data->streamRaw_) {
		return data->pipeInput0_->exportBuffers(count, buffers);
	}

	return -EINVAL;
}

int PipelineHandlerNxpNeo::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	NxpNeoCameraData *data = cameraData(camera);
	NeoDevice *neo = data->neo_.get();
	const CameraInfo *cameraInfo = data->cameraInfo_;
	int ret;

	LOG(NxpNeo, Debug) << "Start " << data->cameraName();

	/* Allocate buffers for internal pipeline usage. */
	ret = data->allocateBuffers();
	if (ret)
		return ret;

	ret = data->ipa_->start();
	if (ret)
		goto error;

	data->delayedCtrls_->reset();

	/*
	 * Start the Neo and ISI video devices, buffers will be queued to the
	 * Neo frame and IR outputs when requests will be queued.
	 * INPUT0 ISI stream is mandatory, INPUT1 and Embedded Data are optional
	 */

	ret = neo->start();
	if (ret)
		goto error;

	if (cameraInfo->hasStreamInput1()) {
		ret = data->pipeInput1_->start();
		if (ret)
			goto error;
	}

	if (cameraInfo->hasStreamEmbedded()) {
		ret = data->pipeEmbedded_->start();
		if (ret)
			goto error;
	}

	ret = data->pipeInput0_->start();
	if (ret)
		goto error;

	return 0;

error:
	neo->stop();

	data->pipeInput0_->stop();
	if (cameraInfo->hasStreamInput1())
		data->pipeInput1_->stop();
	if (cameraInfo->hasStreamEmbedded())
		data->pipeEmbedded_->stop();

	data->ipa_->stop();
	data->freeBuffers();
	LOG(NxpNeo, Error) << "Failed to start camera " << camera->id();

	return ret;
}

void PipelineHandlerNxpNeo::stopDevice(Camera *camera)
{
	NxpNeoCameraData *data = cameraData(camera);
	NeoDevice *neo = data->neo_.get();
	const CameraInfo *cameraInfo = data->cameraInfo_;
	int ret = 0;

	LOG(NxpNeo, Debug) << "Stop device " << data->cameraName();

	data->cancelPendingRequests();

	data->ipa_->stop();

	ret |= data->pipeInput0_->stop();
	if (cameraInfo->hasStreamInput1())
		ret |= data->pipeInput1_->stop();
	if (cameraInfo->hasStreamEmbedded())
		ret |= data->pipeEmbedded_->stop();

	ret |= neo->stop();

	if (ret)
		LOG(NxpNeo, Warning) << "Failed to stop camera " << camera->id();

	data->freeBuffers();
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
			LOG(NxpNeo, Error)
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

int PipelineHandlerNxpNeo::queueRequestDevice(Camera *camera, Request *request)
{
	NxpNeoCameraData *data = cameraData(camera);

	data->pendingRequests_.push(request);
	data->queuePendingRequests();

	return 0;
}

/**
 * \brief Cancel a \a Request.
 * \request[in] The request to be cancelled
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

/**
 * \brief Get a Bayer media bus format compatible with the the \a pixelFormat.
 * \param[inout] pixelFormat requested pixelFormat, may be adjusted
 *
 * Checks if the \a pixelFormat can be output by sensor via ISI and is also
 * compatible with NEO capabilities.
 * If not compatible, \a pixelFormat is adjusted to meet above requirements.
 *
 * \return The media bus format corresponding to the requested \a pixelFormat
 * or the adjusted one, and a value of zero if no alternative is found.
 */
unsigned int NxpNeoCameraData::getRawMediaBusFormat(PixelFormat *pixelFormat) const
{
	std::vector<unsigned int> mbusCodes = sensor_->mbusCodes();

	unsigned int sensorCode = 0;
	const std::map<uint32_t, V4L2PixelFormat> &isiFormats =
		ISIDevice::mediaBusToPixelFormats();
	const std::vector<V4L2PixelFormat> &neoPixelFormats =
		NeoDevice::input0Formats();

	/*
	 * Make sure the requested PixelFormat is supported by sensor, ISI and
	 * NEO devices
	 */
	bool isiSupport = false;

	auto it = std::find_if(isiFormats.begin(), isiFormats.end(),
			       [&](auto &isiFormat) {
				       return isiFormat.second.toPixelFormat() == *pixelFormat;
			       });
	if (it != isiFormats.end()) {
		isiSupport = true;
		sensorCode = it->first;
	}
	if (!isiSupport)
		LOG(NxpNeo, Debug) << *pixelFormat
				   << " bayer format not supported by ISI";

	bool sensorSupport = false;
	if (std::find(mbusCodes.begin(),
		      mbusCodes.end(), sensorCode) != mbusCodes.end())
		sensorSupport = true;
	else
		LOG(NxpNeo, Debug) << sensorCode
				   << " mbus format not supported by sensor";

	bool neoSupport = false;
	if (std::find_if(neoPixelFormats.begin(),
			 neoPixelFormats.end(),
			 [&](auto &neoPixelFormat) {
				 return neoPixelFormat.toPixelFormat() == *pixelFormat;
			 }) != neoPixelFormats.end())
		neoSupport = true;
	else
		LOG(NxpNeo, Warning) << *pixelFormat
				     << " bayer format not supported by NEO";

	if (isiSupport && neoSupport && sensorSupport) {
		LOG(NxpNeo, Debug)
			<< "Format is supported by sensor code " << sensorCode
			<< " pixel format " << pixelFormat->toString();
		return sensorCode;
	}

	/*
	 * The desired pixel format cannot be produced. Adjust it to the one
	 * corresponding to the raw media bus format with the largest bit-depth
	 * the sensor provides.
	 */
	unsigned int maxDepth = 0;
	*pixelFormat = {};
	sensorCode = 0;

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
					 return neoPixelFormat.toPixelFormat() ==
						isiFormats.at(code).toPixelFormat();
				 }) == neoPixelFormats.end())
			continue;

		/* Pick the one with the largest bit depth. */
		if (bayerFormat.bitDepth > maxDepth) {
			maxDepth = bayerFormat.bitDepth;
			sensorCode = code;
			*pixelFormat = isiFormats.at(code).toPixelFormat();
		}
	}

	if (!pixelFormat->isValid())
		LOG(NxpNeo, Error) << "Cannot find a supported RAW format";

	LOG(NxpNeo, Debug)
		<< "Format adjusted for sensor code " << sensorCode
		<< " pixel format " << pixelFormat->toString();

	return sensorCode;
}

/**
 * \brief Handle ISI channels reservation for the camera sensor
 *
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::setupCameraIsiReserve()
{
	ISIDevice *isi = pipe()->isi_.get();
	unsigned int pipeIndex;
	auto isiPipeDeleter = [=](ISIPipe *_pipe) { isi->freePipe(_pipe); };

	const CameraMediaStream *streamInput0 =
		cameraInfo_->getStreamInput0();
	ASSERT(streamInput0);
	pipeIndex = streamInput0->pipe();
	pipeInput0_ = std::shared_ptr<ISIPipe>
		(isi->allocPipe(pipeIndex), isiPipeDeleter);
	if (!pipeInput0_.get())
		return -ENODEV;

	if (cameraInfo_->hasStreamInput1()) {
		const CameraMediaStream *streamInput1 =
			cameraInfo_->getStreamInput1();
		pipeIndex = streamInput1->pipe();
		pipeInput1_ = std::shared_ptr<ISIPipe>
			(isi->allocPipe(pipeIndex), isiPipeDeleter);
		if (!pipeInput1_.get())
			return -ENODEV;
	}

	if (cameraInfo_->hasStreamEmbedded()) {
		const CameraMediaStream *streamEmbedded =
			cameraInfo_->getStreamEmbedded();
		pipeIndex = streamEmbedded->pipe();
		pipeEmbedded_ = std::shared_ptr<ISIPipe>
			(isi->allocPipe(pipeIndex), isiPipeDeleter);
		if (!pipeEmbedded_.get())
			return -ENODEV;
	}

	return 0;
}

/**
 * \brief Configure camera ISI pipes subdevices and nodes formats
 * \param[in] sdFormatInput0 subdevice format for input0 channel
 * \param[in] sdFormatInput1 subdevice format for input1 channel
 * \param[in] sdFormatEd subdevice format for embedded data channel
 * \param[out] vdFormatInput0 video device format for input0 channel
 * \param[out] vdFormatInput1 video device format for input1 channel
 * \param[out] vdFormatEd video device format for embedded data channel
 *
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::setupCameraConfigureIsi(
	const V4L2SubdeviceFormat &sdFormatInput0,
	const V4L2SubdeviceFormat &sdFormatInput1,
	const V4L2SubdeviceFormat &sdFormatEd,
	V4L2DeviceFormat *vdFormatInput0,
	V4L2DeviceFormat *vdFormatInput1,
	V4L2DeviceFormat *vdFormatEd)
{
	*vdFormatInput0 = {};
	*vdFormatInput1 = {};
	*vdFormatEd = {};

	int ret = pipeInput0_->configure(sdFormatInput0, vdFormatInput0);
	if (cameraInfo_->hasStreamInput1())
		ret |= pipeInput1_->configure(sdFormatInput1, vdFormatInput1);
	if (cameraInfo_->hasStreamEmbedded())
		ret |= pipeEmbedded_->configure(sdFormatEd, vdFormatEd);

	return ret;
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
int NxpNeoCameraData::setupCameraStream(
	const std::vector<StreamLink> &streamLinks,
	V4L2SubdeviceFormat &sdFormat)
{
	const MediaDevice *media = pipe()->isiMedia_;
	std::unique_ptr<V4L2Subdevice> subDev;
	int ret = 0;

	for (const auto &streamLink : streamLinks) {
		const MediaLink *mediaLink = std::get<0>(streamLink);
		const MediaPad *sourceMediaPad = mediaLink->source();
		const MediaPad *sinkMediaPad = mediaLink->sink();
		std::string sourceName = sourceMediaPad->entity()->name();
		std::string sinkName = sinkMediaPad->entity()->name();
		unsigned int sourcePad = sourceMediaPad->index();
		unsigned int sinkPad = sinkMediaPad->index();
		unsigned int sourceStream = std::get<1>(streamLink);
		unsigned int sinkStream = std::get<2>(streamLink);

		LOG(NxpNeo, Debug)
			<< "Set format " << sdFormat.toString()
			<< " source " << sourceName << " "
			<< sourcePad << "/" << sourceStream
			<< " sink " << sinkName << " "
			<< sinkPad << "/" << sinkStream;

		subDev = V4L2Subdevice::fromEntityName(media, sourceName);
		ret = subDev->open();
		if (ret) {
			LOG(NxpNeo, Warning)
				<< "Error opening subdev " << sourceName;
			return ret;
		}
		ret = subDev->setFormat(sourcePad, sourceStream, &sdFormat);
		if (ret) {
			LOG(NxpNeo, Warning)
				<< "Error setting format " << sourceName;
			return ret;
		}

		/* Stop at capture video node */
		if (sinkMediaPad->entity()->function() == MEDIA_ENT_T_V4L2_VIDEO) {
			LOG(NxpNeo, Debug)
				<< "Configuration completed at video device "
				<< sinkName;
			return 0;
		}

		subDev = V4L2Subdevice::fromEntityName(media, sinkName);
		ret = subDev->open();
		if (ret) {
			LOG(NxpNeo, Warning)
				<< "Error opening subdev " << sinkName;
			return ret;
		}
		ret = subDev->setFormat(sinkPad, sinkStream, &sdFormat);
		if (ret) {
			LOG(NxpNeo, Warning)
				<< "Error setting format " << sinkName;
			return ret;
		}
	}

	return 0;
}

/**
 * \brief Setup format for each media pad stream belonging to a camera
 * \param[in] mbusCodeInput0 The mbus code to set on input0 media stream
 * \param[in] SizeInput0 The size to set on input0 media stream
 * \param[out] sdFormatInput0 The resulting V4L2 format applied on input0 pads
 * \param[out] sdFormatInput1 The resulting V4L2 format applied on input1 pads
 * \param[out] sdFormatEd The resulting V4L2 format applied on embedded pads

 * Format for input0 media stream is defined by parameters as it can be
 * defined by application. Formats for input1 and embedded media streams
 * are derived from both the input0 values and configuration file.
 *
 * \return 0 on success or a negative error code otherwise.
 */
int NxpNeoCameraData::setupCameraStreams(
	unsigned int mbusCodeInput0, Size sizeInput0,
	V4L2SubdeviceFormat *sdFormatInput0,
	V4L2SubdeviceFormat *sdFormatInput1,
	V4L2SubdeviceFormat *sdFormatEd)
{
	int ret;

	/* All streams share the same size */
	*sdFormatInput0 = {};
	sdFormatInput0->mbus_code = mbusCodeInput0;
	sdFormatInput0->size = sizeInput0;
	const CameraMediaStream *streamInput0 = cameraInfo_->getStreamInput0();
	ASSERT(streamInput0);
	const std::vector<StreamLink> &streamLinksInput0 =
		streamInput0->streamLinks();
	ret = setupCameraStream(streamLinksInput0, *sdFormatInput0);
	if (ret)
		return ret;

	*sdFormatInput1 = {};
	if (cameraInfo_->hasStreamInput1()) {
		const CameraMediaStream *streamInput1 =
			cameraInfo_->getStreamInput1();
		ASSERT(streamInput1);
		unsigned int code = streamInput1->mbusCode();
		sdFormatInput1->mbus_code = code;
		sdFormatInput1->size = sizeInput0;
		const std::vector<StreamLink> &streamLinksInput1 =
			streamInput1->streamLinks();
		ret = setupCameraStream(streamLinksInput1, *sdFormatInput1);
		if (ret)
			return ret;
	}

	*sdFormatEd = {};
	if (cameraInfo_->hasStreamEmbedded()) {
		const CameraMediaStream *streamEmbedded =
			cameraInfo_->getStreamEmbedded();
		unsigned int code =
			streamEmbedded->mbusCode();
		sdFormatEd->mbus_code = code;
		unsigned int lines =
			streamEmbedded->embeddedLines();
		sdFormatEd->size = Size(sizeInput0.width, lines);
		const std::vector<StreamLink> &streamLinksEmbedded =
			streamEmbedded->streamLinks();
		ret = setupCameraStream(streamLinksEmbedded, *sdFormatEd);
		if (ret)
			return ret;
	}

	return ret;
}

/**
 * \brief Enable media links from the camera graph
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::setupCameraEnableLinks() const
{
	int ret;

	/*
	 * Enable graph media links
	 */
	auto EnableLink = [](const CameraMediaStream *_stream,
			     std::string streamName) -> int {
		int res;
		ASSERT(_stream);
		std::vector<StreamLink> links = _stream->streamLinks();
		for (auto &streamLink : links) {
			MediaLink *link = std::get<0>(streamLink);
			MediaPad *sourceMPad = link->source();
			MediaPad *sinkMPad = link->sink();
			std::string source = sourceMPad->entity()->name();
			std::string sink = sinkMPad->entity()->name();
			unsigned int sourcePad = sourceMPad->index();
			unsigned int sinkPad = sinkMPad->index();

			LOG(NxpNeo, Debug)
				<< "Enable link stream " << streamName
				<< " source "
				<< source << "/" << sourcePad
				<< " sink "
				<< sink << "/" << sinkPad;

			res = link->setEnabled(true);
			if (res) {
				LOG(NxpNeo, Error) << "Failed to enable Link";
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
 * \brief Retrieve camera sensor format and sizes
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::setupCameraFormat()
{
	/*
	 * Arbitrary Bayer format with high resolution requested.
	 * Actual best format supported by the sensor will be adjusted.
	 **/
	rawPixelFormat_ = formats::SBGGR16;
	mbusCode_ = getRawMediaBusFormat(&rawPixelFormat_);
	if (!mbusCode_)
		return -ENODEV;

	std::vector<Size> sizes = sensor_->sizes(mbusCode_);
	if (!sizes.size())
		return -ENODEV;

	std::string sizesString;
	for (Size size : sizes)
		sizesString += size.toString() + " ";
	LOG(NxpNeo, Debug)
		<< "Sensor mbus code " << mbusCode_
		<< " pixel format " << rawPixelFormat_.toString()
		<< " sizes " << sizesString;

	return 0;
}

/**
 * \brief Report sensor RGB-Ir capability
 *
 * \return True if sensor has RGB-Ir support.
 */
bool NxpNeoCameraData::sensorIsRgbIr() const
{
	/* \todo get information from IPA about IR support */
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
	NeoDevice *neo = neo_.get();
	unsigned int bufferCount;
	int ret;

	bufferCount = std::max({
		streamFrame_.configuration().bufferCount,
		streamIr_.configuration().bufferCount,
		streamRaw_.configuration().bufferCount,
	});

	/* Allocate and map ISP buffers */
	ret = neo->allocateBuffers(bufferCount);
	if (ret < 0)
		return ret;

	unsigned int ipaBufferId = 1;
	const std::vector<std::unique_ptr<FrameBuffer>> emptyBufferVector;

	for (const std::unique_ptr<FrameBuffer> &buffer : neo->paramsBuffers_) {
		buffer->setCookie(ipaBufferId++);
		ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
	}

	for (const std::unique_ptr<FrameBuffer> &buffer : neo->statsBuffers_) {
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
			 neo->paramsBuffers_, neo->statsBuffers_);

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
 * \brief Probe, configure and register camera sensor
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineHandlerNxpNeo::createCamera(MediaEntity *sensorEntity,
					MediaDevice *neoMedia,
					unsigned int neoInstance)
{
	int ret;
	std::unique_ptr<NxpNeoCameraData> data =
		std::make_unique<NxpNeoCameraData>(this);

	std::set<Stream *> streams = {
		&data->streamFrame_,
		&data->streamIr_,
		&data->streamRaw_,
	};

	data->sensor_ = std::make_unique<CameraSensor>(sensorEntity);
	CameraSensor *sensor = data->sensor_.get();
	ret = sensor->init();
	if (ret)
		return ret;

	std::string name = sensorEntity->name();
	data->cameraInfo_ = pipelineConfig_.getCameraInfo(name);
	if (!data->cameraInfo_) {
		LOG(NxpNeo, Warning) << "No CameraInfo for " << name;
		return -EINVAL;
	}

	ret = data->setupCameraFormat();
	if (ret)
		return ret;

	data->neo_ = std::make_unique<NeoDevice>(neoInstance);
	ret = data->neo_->init(neoMedia);
	if (ret)
		return ret;

	ret = data->loadIPA();
	if (ret)
		return ret;

	/* Initialize the camera properties. */
	data->properties_ = sensor->properties();
	ret = initControls(data.get());
	if (ret)
		return ret;

	/*
	 * DelayedControls parameters come from prior IPA init().
	 */
	data->delayedCtrls_ =
		std::make_unique<DelayedControls>(sensor->device(),
						  data->delayedControlsParams_);
	data->neo_->isp_->frameStart.connect(data.get(),
					     &NxpNeoCameraData::frameStart);

	/* Convert the sensor rotation to a transformation */
	const auto &rotation = data->properties_.get(properties::Rotation);
	if (!rotation)
		LOG(NxpNeo, Warning) << "Rotation control not exposed by "
				     << data->sensor_->id()
				     << ". Assume rotation 0";

	/*
	 * Connect video devices' 'bufferReady' signals to their
	 * slot to implement the image processing pipeline.
	 *
	 * Frames produced by the ISI unit are passed to the
	 * associated NEO inputs where they get processed and
	 * returned through the NEO main and IR outputs.
	 */
	const CameraInfo *cameraInfo = data->cameraInfo_;

	ret = data->setupCameraIsiReserve();
	if (ret)
		return ret;

	data->pipeInput0_->bufferReady().connect(
		data.get(),
		&NxpNeoCameraData::isiInput0BufferReady);

	if (cameraInfo->hasStreamInput1()) {
		data->pipeInput1_->bufferReady().connect(
			data.get(),
			&NxpNeoCameraData::isiInput1BufferReady);
	}
	if (cameraInfo->hasStreamEmbedded()) {
		data->pipeEmbedded_->bufferReady().connect(
			data.get(),
			&NxpNeoCameraData::isiEdBufferReady);
	}

	data->neo_->input0_->bufferReady.connect(
		data.get(),
		&NxpNeoCameraData::neoInput0BufferReady);
	data->neo_->input1_->bufferReady.connect(
		data.get(),
		&NxpNeoCameraData::neoInput1BufferReady);
	data->neo_->frame_->bufferReady.connect(
		data.get(),
		&NxpNeoCameraData::neoOutputBufferReady);
	data->neo_->ir_->bufferReady.connect(
		data.get(),
		&NxpNeoCameraData::neoOutputBufferReady);
	data->neo_->params_->bufferReady.connect(
		data.get(),
		&NxpNeoCameraData::neoParamsBufferReady);
	data->neo_->stats_->bufferReady.connect(
		data.get(),
		&NxpNeoCameraData::neoStatsBufferReady);

	/* Create and register the Camera instance. */
	const std::string &cameraId = sensor->id();
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

	for (const auto &[name, routing] : routingMap) {
		LOG(NxpNeo, Debug)
			<< "Configure routing for entity " << name
			<< " routing " << routing.toString();

		std::unique_ptr<V4L2Subdevice> sdev =
			V4L2Subdevice::fromEntityName(isiMedia_, name);
		if (!sdev.get()) {
			LOG(NxpNeo, Error) << "Subdevice does not exist " << name;
			return -EINVAL;
		}

		ret = sdev->open();
		if (ret) {
			LOG(NxpNeo, Error)
				<< "Error opening entity " << name;
			return -EINVAL;
		}

		V4L2Subdevice::Routing _routing = routing;
		ret = sdev->setRouting(&_routing, V4L2Subdevice::ActiveFormat);
		if (ret) {
			LOG(NxpNeo, Error)
				<< "Error setting routing for entity " << name;
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * \brief Initialize the camera graphs in the media device
 *
 * Cameras managed by the pipeline operate on different streams of the media
 * device. Those streams share subdevice pads common to multiple cameras.
 * A given stream to be started requires a valid format to be configured
 * for every other streams sharing the same media device pads.
 * Thus, a default config is applied at startup to all media devices streams
 * used by every camera, so that any selected stream can be started later.
 *
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineHandlerNxpNeo::setupCameraGraphs()
{
	int ret = 0;

	for (auto const &camera : manager_->cameras()) {
		NxpNeoCameraData *data = cameraData(camera.get());
		LOG(NxpNeo, Debug)
			<< "Setup graph for camera " << data->cameraName();

		V4L2SubdeviceFormat *sdFormatInput0 = &data->sdFormatInput0_;
		V4L2SubdeviceFormat *sdFormatInput1 = &data->sdFormatInput1_;
		V4L2SubdeviceFormat *sdFormatEd = &data->sdFormatEmbedded_;

		V4L2DeviceFormat *vdFormatInput0 = &data->vdFormatInput0_;
		V4L2DeviceFormat *vdFormatInput1 = &data->vdFormatInput1_;
		V4L2DeviceFormat *vdFormatEd = &data->vdFormatEmbedded_;

		/* Enable media link between entities */
		ret = data->setupCameraEnableLinks();
		if (ret)
			return ret;

		/* Apply default format to each media pad streams */
		unsigned int mbusCode = data->mbusCode_;
		CameraSensor *sensor = data->sensor_.get();
		std::vector<Size> sizes = sensor->sizes(mbusCode);
		Size size = sizes.back();
		ret = data->setupCameraStreams(mbusCode, size, sdFormatInput0,
					       sdFormatInput1, sdFormatEd);
		if (ret)
			return ret;

		/* ISI video video devices configuration */
		ret = data->setupCameraConfigureIsi(*sdFormatInput0, *sdFormatInput1,
						    *sdFormatEd,
						    vdFormatInput0, vdFormatInput1,
						    vdFormatEd);
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

	ret = pipelineConfig_.load(file, isiMedia_);

	return ret;
}

bool PipelineHandlerNxpNeo::match(DeviceEnumerator *enumerator)
{
	int ret;
	constexpr unsigned int kMaxNeoDevices = 4;

	/*
	 * Prerequisite for pipeline operation is that ISI media device is
	 * present.
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
	 * Discover Neo ISP media devices
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
	 * Discover camera entities from the ISI media device
	 * Bind each camera to an ISP entity
	 */
	unsigned int numCameras = 0;
	for (MediaEntity *entity : isiMedia_->entities()) {
		if (entity->function() != MEDIA_ENT_F_CAM_SENSOR)
			continue;

		if (!neos.size())
			break;

		ret = createCamera(entity, neos.front(), numCameras);
		if (ret) {
			LOG(NxpNeo, Warning) << "Failed to probe camera "
					     << entity->name() << ": " << ret;
		} else {
			numCameras++;
			neos.pop();
		}
	}

	if (numCameras < 1)
		return false;

	/*
	 * Apply global routing and setup camera media device streams
	 */
	ret = setupRouting();
	if (ret)
		return ret;

	ret = setupCameraGraphs();
	if (ret)
		return ret;

	return true;
}

/**
 * \brief Initialize the camera controls
 * \param[in] data The camera data
 *
 * Initialize the camera controls by calculating controls which the pipeline
 * is reponsible for and merge them with the controls computed by the IPA.
 *
 * This function needs data->ipaControls_ to be initialized by the IPA init()
 * function at camera creation time. Always call this function after IPA init().
 *
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineHandlerNxpNeo::initControls(NxpNeoCameraData *data)
{
	/*
	 * \todo The controls initialized here depend on sensor configuration
	 * and their limits should be updated once the configuration gets
	 * changed.
	 *
	 * Initialize the sensor using its resolution and compute the control
	 * limits.
	 */
	CameraSensor *sensor = data->sensor_.get();
	V4L2SubdeviceFormat sensorFormat = {};
	sensorFormat.size = sensor->resolution();
	int ret = sensor->setFormat(&sensorFormat);
	if (ret)
		return ret;

	return updateControls(data);
}

/**
 * \brief Update the camera controls
 * \param[in] data The camera data
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
int PipelineHandlerNxpNeo::updateControls(NxpNeoCameraData *data)
{
	ControlInfoMap::Map controls = {};

	/* Add the IPA registered controls to list of camera controls. */
	for (const auto &ipaControl : data->ipaControls_)
		controls[ipaControl.first] = ipaControl.second;

	data->controlInfo_ = ControlInfoMap(std::move(controls),
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

	/*
	 * Pass the sensor info to the IPA to initialize controls.
	 *
	 * \todo Find a way to initialize IPA controls without basing their
	 * limits on a particular sensor mode. We currently pass sensor
	 * information corresponding to the largest sensor resolution, and the
	 * IPA uses this to compute limits for supported controls. There's a
	 * discrepancy between the need to compute IPA control limits at init
	 * time, and the fact that those limits may depend on the sensor mode.
	 * Research is required to find out to handle this issue.
	 */
	CameraSensor *sensor = this->sensor_.get();
	V4L2SubdeviceFormat sensorFormat = {};
	sensorFormat.size = sensor->resolution();
	int ret = sensor->setFormat(&sensorFormat);
	if (ret)
		return ret;

	IPACameraSensorInfo sensorInfo{};
	ret = sensor->sensorInfo(&sensorInfo);
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
		LOG(NxpNeo, Error) << "Failed to initialise the NxpNeo IPA";
		return ret;
	}

	std::map<int32_t, ipa::nxpneo::DelayedControlsParams> &ipaDelayParams =
		sensorConfig.delayedControlsParams;
	for (const auto &kv : ipaDelayParams) {
		auto k = kv.first;
		auto v = kv.second;
		DelayedControls::ControlParams params = { v.delay, v.priorityWrite };
		delayedControlsParams_.emplace(k, params);
	}

	return 0;
}

void NxpNeoCameraData::ipaSetSensorControls([[maybe_unused]] unsigned int id,
					    const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
}

void NxpNeoCameraData::ipaParamsBufferReady(unsigned int id)
{
	NxpNeoFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

	/* Queue all buffers from the request aimed for the Neo. */
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

/* -----------------------------------------------------------------------------
 * Buffer Ready slots
 */

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
		LOG(NxpNeo, Warning)
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
		 * INPUT0 frame will be queue into ISP once params buffer for the frame
		 * have been produced by IPA.
		 * \todo: in case of ISP operation with INPUT0 + INPUT1 inputs, wait for both
		 * frames to be available before invoking ipa_->fillParamsBuffer()
		 */

		ipa_->fillParamsBuffer(info->id, info->paramsBuffer->cookie());
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
		LOG(NxpNeo, Error)
			<< "Failed to set test pattern mode: " << ret;
		return;
	}

	request->metadata().set(controls::draft::TestPatternMode,
				*testPatternMode);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerNxpNeo)

} /* namespace libcamera */
