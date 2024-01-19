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

class PipelineHandlerNxpNeo;

class NxpNeoCameraData : public Camera::Private
{
public:
	NxpNeoCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe) {};

	int loadIPA();

	void neoOutputBufferReady(FrameBuffer *buffer);
	void isiInput0BufferReady(FrameBuffer *buffer);
	void isiInput1BufferReady(FrameBuffer *buffer);
	void isiEdBufferReady(FrameBuffer *buffer);
	void neoInput0BufferConsumed(FrameBuffer *buffer);
	void neoInput1BufferConsumed(FrameBuffer *buffer);

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
	int setupCamera();
	int setupCameraDiscoverLinks(std::vector<MediaLink *> *links);
	int setupCameraIsiReserve();
	int setupCameraConfigureGraph(std::vector<MediaLink *> &links,
				      V4L2SubdeviceFormat &sdFormat);
	int setupCameraConfigureIsi(
		V4L2SubdeviceFormat &sdFormatInput0, V4L2SubdeviceFormat &sdFormatInput1,
		V4L2SubdeviceFormat &sdFormatEd,
		V4L2DeviceFormat *vdFormatInput0, V4L2DeviceFormat *vdFormatInput1,
		V4L2DeviceFormat *vdFormatEd);

	bool screenCancelledBuffer(FrameBuffer *buffer, NxpNeoFrames::Info *info);

private:
	friend class PipelineHandlerNxpNeo;
	friend class NxpNeoCameraConfiguration;

	PipelineHandlerNxpNeo *pipe();

	NxpNeoSensorProperties sensorProperties_;
	std::vector<MediaLink *> graphLinks_;

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

	std::unique_ptr<CameraSensor> sensor_;

	std::shared_ptr<ISIPipe> pipeInput0_;
	std::shared_ptr<ISIPipe> pipeInput1_;
	std::shared_ptr<ISIPipe> pipeEd_;

	std::vector<IPABuffer> ipaBuffers_;

	unsigned int mbusCode_ = 0;
	unsigned int sequence_ = 0;
	bool rawStreamOnly_ = false;
	PixelFormat rawPixelFormat_;
};

class NxpNeoCameraConfiguration : public CameraConfiguration
{
public:
	static constexpr unsigned int kBufferCount = 4;
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
	const NxpNeoCameraData *data_;
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

	int loadSensorProperties(const CameraSensor *sensor,
				 NxpNeoSensorProperties *sensorProperties);

	std::unique_ptr<ISIDevice> isi_;
	MediaDevice *isiMedia_ = nullptr;
	V4L2Subdevice::Routing isiRouting_;

private:
	NxpNeoCameraData *cameraData(Camera *camera)
	{
		return static_cast<NxpNeoCameraData *>(camera->_d());
	}

	int initControls(NxpNeoCameraData *data);
	int updateControls(NxpNeoCameraData *data);

	int allocateBuffers(Camera *camera);
	int freeBuffers(Camera *camera);

	int createCamera(MediaEntity *sensorEntity, MediaDevice *neoMedia,
			 unsigned int neoInstance);

	int configureRoutes();
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
		} else if ((info.colourEncoding == PixelFormatInfo::ColourEncodingYUV)
			   && (info.planes[0].bytesPerGroup <= 2)
			   && (info.planes[1].bytesPerGroup == 0)) {
			/*  pixel formats Rn detection (grey/Yn) */
			irCount++;
			cfg.setStream(streamIr);
		} else if ((info.colourEncoding == PixelFormatInfo::ColourEncodingYUV)
			   || (info.colourEncoding == PixelFormatInfo::ColourEncodingRGB)) {
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
	 * If no valid size is found in any stream, then we default to sensor
	 * max supported size.
	 */
	CameraSensor *sensor = data_->sensor_.get();
	unsigned int mbusCode_ = data_->mbusCode_;
	std::vector<Size> sizes = sensor->sizes(mbusCode_);
	Size &maxSize = sizes.back();
	Size size = maxSize;

	auto iter = std::find_if(config_.begin(), config_.end(),
		[&sizes](auto &cfg) {
			return (std::find(sizes.begin(), sizes.end(), cfg.size)
				!= sizes.end());
		} );
	if (iter != config_.end())
		size = iter->size;

	for (unsigned int i = 0; i < config_.size(); ++i) {

		const StreamConfiguration originalCfg = config_[i];
		StreamConfiguration *cfg = &config_[i];

		bool isFrame = (streamFrame == cfg->stream());
		bool isIr = (streamIr == cfg->stream());
		bool isRaw = (streamRaw == cfg->stream());

		LOG(NxpNeo, Debug)
			<< "Stream " << i << " to validate cfg " << cfg->toString();

		if (isFrame || isIr) {
			const std::vector<V4L2PixelFormat> &formats = isFrame
						? NeoDevice::frameFormats()
						: NeoDevice::irFormats();
			if (std::find_if(formats.begin(), formats.end(),
					[&](auto &format) {
						return format.toPixelFormat()
							== cfg->pixelFormat;
					}) == formats.end())
				cfg->pixelFormat = formats[0].toPixelFormat();
			cfg->size = size;
			cfg->bufferCount = NxpNeoCameraConfiguration::kBufferCount;
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
			cfg->bufferCount = ISIPipe::kBufferCount;
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
PipelineHandlerNxpNeo::generateConfiguration(Camera *camera, Span<const StreamRole> roles)
{
	NxpNeoCameraData *data = cameraData(camera);
	std::unique_ptr<NxpNeoCameraConfiguration> config =
		std::make_unique<NxpNeoCameraConfiguration>(data);

	if (roles.empty())
		return config;

	bool frameOutputAvailable = true;
	bool irOutputAvailable = data->sensorIsRgbIr();
	bool rawOutputAvailable = true;

	CameraSensor *sensor = data->sensor_.get();

	std::vector<Size> sizes = sensor->sizes(data->mbusCode_);
	Size &maxSize = sizes.back();

	std::vector<SizeRange> ranges;
	for (Size size : sizes)
		ranges.emplace_back(size);

	for (const StreamRole role : roles) {
		std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
		unsigned int bufferCount;
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

			bufferCount = NxpNeoCameraConfiguration::kBufferCount;
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
			bufferCount = ISIPipe::kBufferCount;
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
		cfg.bufferCount = bufferCount;

		config->addConfiguration(cfg);
		LOG(NxpNeo, Debug)
			<< "Generated configuration " << cfg.toString()
			<< " for role " << role ;
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

	/*
	 * Configurations have been through validate() so they can be applied
	 * directly.
	 */

	NxpNeoSensorProperties *sensorProps = &data->sensorProperties_;
	V4L2SubdeviceFormat sdFormatInput0 = {}, sdFormatInput1 = {},  sdFormatEd = {};
	V4L2DeviceFormat vdFormatInput0 = {}, vdFormatInput1 = {}, vdFormatEd = {};

	/* All streams share the same size */
	Size size = (*config)[0].size;
	sdFormatInput0.mbus_code = data->mbusCode_;
	sdFormatInput0.size = size;

	if (sensorProps->hasInput1()) {
		sdFormatInput1.mbus_code = sensorProps->input1Channel_.mbusFormat;
		sdFormatInput1.size = size;
	}

	if (sensorProps->hasEmbedded()) {
		sdFormatEd.mbus_code = sensorProps->edChannel_.mbusFormat;
		sdFormatEd.size = Size(size.width,
				       sensorProps->edChannel_.lines);
	}

	ret = data->setupCameraConfigureIsi(
			sdFormatInput0, sdFormatInput1, sdFormatEd,
			&vdFormatInput0, &vdFormatInput1, &vdFormatEd);
	if (ret)
		return ret;

	ret = data->setupCameraConfigureGraph(data->graphLinks_, sdFormatInput0);
	if (ret)
		return ret;


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

		ret = data->neo_->configure(vdFormatInput0, vdFormatInput1,
					    devFormatFrame, devFormatIr);
		if (ret)
			return ret;
	}

	CameraSensor *sensor = data->sensor_.get();

	IPACameraSensorInfo sensorInfo;
	ret = sensor->sensorInfo(&sensorInfo);
	if (ret)
		return ret;

	ret = sensor->setFormat(&sdFormatInput0, config->combinedTransform_);
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

int PipelineHandlerNxpNeo::allocateBuffers(Camera *camera)
{
	NxpNeoCameraData *data = cameraData(camera);
	NeoDevice *neo = data->neo_.get();
	unsigned int bufferCount;
	int ret;

	bufferCount = std::max({
		data->streamFrame_.configuration().bufferCount,
		data->streamIr_.configuration().bufferCount,
		data->streamRaw_.configuration().bufferCount,
	});

	ret = neo->allocateBuffers(bufferCount);
	if (ret < 0)
		return ret;

	/* Map buffers to the IPA. */
	unsigned int ipaBufferId = 1;

	for (const std::unique_ptr<FrameBuffer> &buffer : neo->paramsBuffers_) {
		buffer->setCookie(ipaBufferId++);
		data->ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
	}

	for (const std::unique_ptr<FrameBuffer> &buffer : neo->statsBuffers_) {
		buffer->setCookie(ipaBufferId++);
		data->ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
	}

	data->ipa_->mapBuffers(data->ipaBuffers_);

	data->frameInfos_.init(neo->paramsBuffers_, neo->statsBuffers_);
	data->frameInfos_.bufferAvailable.connect(
		data, &NxpNeoCameraData::queuePendingRequests);

	return 0;
}

int PipelineHandlerNxpNeo::freeBuffers(Camera *camera)
{
	NxpNeoCameraData *data = cameraData(camera);

	data->frameInfos_.clear();

	std::vector<unsigned int> ids;
	for (IPABuffer &ipabuf : data->ipaBuffers_)
		ids.push_back(ipabuf.id);

	data->ipa_->unmapBuffers(ids);
	data->ipaBuffers_.clear();

	data->neo_->freeBuffers();

	return 0;
}

int PipelineHandlerNxpNeo::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	NxpNeoCameraData *data = cameraData(camera);
	NeoDevice *neo = data->neo_.get();
	NxpNeoSensorProperties *sensorProps = &data->sensorProperties_;
	int ret;

	/* Allocate buffers for internal pipeline usage. */
	ret = allocateBuffers(camera);
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

	if (sensorProps->hasInput1()) {
		ret = data->pipeInput1_->start();
		if (ret)
			goto error;
	}

	if (sensorProps->hasEmbedded()) {
		ret = data->pipeEd_->start();
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
	if (sensorProps->hasInput1())
		data->pipeInput1_->stop();
	if (sensorProps->hasEmbedded())
		data->pipeEd_->stop();

	data->ipa_->stop();
	freeBuffers(camera);
	LOG(NxpNeo, Error) << "Failed to start camera " << camera->id();

	return ret;
}

void PipelineHandlerNxpNeo::stopDevice(Camera *camera)
{
	NxpNeoCameraData *data = cameraData(camera);
	NeoDevice *neo = data->neo_.get();
	NxpNeoSensorProperties *sensorProps = &data->sensorProperties_;

	int ret = 0;

	data->cancelPendingRequests();

	data->ipa_->stop();

	ret |= data->pipeInput0_->stop();
	if (sensorProps->hasInput1())
		ret |= data->pipeInput1_->stop();
	if (sensorProps->hasEmbedded())
		ret |= data->pipeEd_->stop();

	ret |= neo->stop();

	if (ret)
		LOG(NxpNeo, Warning) << "Failed to stop camera " << camera->id();

	freeBuffers(camera);
}

void NxpNeoCameraData::cancelPendingRequests()
{
	processingRequests_ = {};

	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();

		for (auto it : request->buffers()) {
			FrameBuffer *buffer = it.second;
			buffer->_d()->cancel();
			pipe()->completeBuffer(request, buffer);
		}

		pipe()->completeRequest(request);
		pendingRequests_.pop();
	}
}

void NxpNeoCameraData::queuePendingRequests()
{
	FrameBuffer *reqRawBuffer;
	FrameBuffer *rawBuffer;
	FrameBuffer *input1Buffer;
	FrameBuffer *edBuffer;
	NxpNeoFrames::Info *info;
	int ret;

	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();

		info = frameInfos_.create(request, rawStreamOnly_);
		if (!info)
			break;

		rawBuffer = nullptr;
		input1Buffer = nullptr;
		edBuffer = nullptr;

		/*
		 * Queue a INPUT0 buffer on the ISI, using the raw stream buffer
		 * provided in the request, if any, or a ISI internal buffer
		 * otherwise.
		 */
		reqRawBuffer = request->findBuffer(&streamRaw_);
		if (!reqRawBuffer) {
			rawBuffer = pipeInput0_->popAvailableBuffer();
			if (!rawBuffer)
				goto error;
			rawBuffer->_d()->setRequest(request);
		} else {
			rawBuffer = reqRawBuffer;
		}

		/* Queue optional INPUT1 buffer to ISI channel */
		if (sensorProperties_.hasInput1()) {
			input1Buffer = pipeInput1_->popAvailableBuffer();
			if (!input1Buffer)
				goto error;
			input1Buffer->_d()->setRequest(request);
		}

		/* Queue optional Embedded Data buffer to ISI channel */
		if (sensorProperties_.hasEmbedded()) {
			edBuffer = pipeEd_->popAvailableBuffer();
			if (!edBuffer)
				goto error;
			edBuffer->_d()->setRequest(request);
		}

		ret = pipeInput0_->queueBuffer(rawBuffer);
		if (ret)
			goto error;
		if (sensorProperties_.hasInput1())
			ret = pipeInput1_->queueBuffer(input1Buffer);
		if (sensorProperties_.hasEmbedded())
			ret |= pipeEd_->queueBuffer(edBuffer);

		if (ret) {
			LOG(NxpNeo, Error) << "queuePendingRequests error - unbalanced";
			return;
		}

		info->rawBuffer = rawBuffer;
		info->input1Buffer = input1Buffer;
		info->edBuffer = edBuffer;

		ipa_->queueRequest(info->id, request->controls());

		pendingRequests_.pop();
		processingRequests_.push(request);
	}
	return;

error:
	LOG(NxpNeo, Warning) << "queuePendingRequests error ";

	if (edBuffer)
		pipeInput0_->tryReturnBuffer(edBuffer);
	if (input1Buffer)
		pipeInput1_->tryReturnBuffer(input1Buffer);
	if (!reqRawBuffer && rawBuffer)
		pipeEd_->tryReturnBuffer(rawBuffer);

	/*
	 * \todo If queueBuffer fails in queuing a buffer to the device,
	 * report the request as error by cancelling the request and
	 * calling PipelineHandler::completeRequest().
	 */
	frameInfos_.remove(info);
}

int PipelineHandlerNxpNeo::queueRequestDevice(Camera *camera, Request *request)
{
	NxpNeoCameraData *data = cameraData(camera);

	data->pendingRequests_.push(request);
	data->queuePendingRequests();

	return 0;
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
	if (std::find_if(neoPixelFormats.begin(), neoPixelFormats.end(),
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
		if (std::find_if(isiFormats.begin(), isiFormats.end(),
				[&](auto isiFormat){
					return isiFormat.first == code;
				}) == isiFormats.end())
			continue;

		/* Make sure the media format is supported by NEO. */
		if (std::find_if(neoPixelFormats.begin(), neoPixelFormats.end(),
				[&](auto neoPixelFormat){
					return neoPixelFormat.toPixelFormat()
						== isiFormats.at(code).toPixelFormat();
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
 * \brief Load sensor properties
 * \param[out] sensor The targeted sensor
 * \param[out] sensorProps Sensor properties container
 *
 * Retrieve the properties of the targeted sensor, describing its channels
 * composition and other specific attributes.
 * \todo read those values from configuration file - default values for now.
 *
 * \return 0 in case of success or a negative error code.
 */
int PipelineHandlerNxpNeo::loadSensorProperties(const CameraSensor *sensor,
						NxpNeoSensorProperties *sensorProps)
{
	*sensorProps = NxpNeoSensorProperties();

	LOG(NxpNeo, Debug)
		<< " Sensor properties entity " << sensor->entity()->name()
		<< " id " << sensor->id()
		<< " model " << sensor->model() << std::endl
		<< " input0 " << sensorProps->input0Channel_.toString() << std::endl
		<< " input1 " << sensorProps->input1Channel_.toString() << std::endl
		<< " ed " << sensorProps->edChannel_.toString();

	return 0;
}

/**
 * \brief Discover and record graph links from sensor to ISI crossbar.
 * \param[out] links Vector of media links involved in the graph
 *
 * Simplistic walkthrough: only first link of first source pad of every
 * entity is considered as a possible path at the moment.
 * \todo introduce optional path description in pipeline configuration file to
 * give the option to define more complex graph paths.
 *
 * \return 0 on success or a negative error code for error or if no camera
 * has been created
 * \retval -ENODEV no camera has been created
 */
int NxpNeoCameraData::setupCameraDiscoverLinks(std::vector<MediaLink *> *links)
{
	constexpr unsigned int kGraphDepthMax = 8;
	ISIDevice *isi = pipe()->isi_.get();

	const MediaEntity *sensorEntity = sensor_->entity();
	const MediaEntity *isiCrossbarEntity = isi->crossbar()->entity();
	const MediaEntity *entity = sensorEntity;

	for (unsigned int i = 0; i < kGraphDepthMax; i++) {
		MediaPad *sourcePad = nullptr;
		MediaPad *sinkPad;

		for (auto pad : entity->pads()) {
			if (pad->flags() & MEDIA_PAD_FL_SOURCE) {
				sourcePad = pad;
				break;
			}
		}

		MediaLink *link;
		if (sourcePad && sourcePad->links().size()) {
			link = sourcePad->links()[0];
		} else {
			LOG(NxpNeo, Debug)
				<< "Sensor " << sensorEntity->name()
				<< " entity " << entity->name()
				<< " has no source pad with links";
			return -ENOENT;
		}

		links->push_back(link);

		sinkPad = link->sink();
		LOG(NxpNeo, Debug)
			<< "Sensor " << sensorEntity->name()
			<< " graph, added link "
			<< " source entity " << sourcePad->entity()->name()
			<< " pad " << sourcePad->index()
			<< " -> sink entity " << sinkPad->entity()->name()
			<< " pad " << sinkPad->index();

		entity = sinkPad->entity();
		if (entity->name() == isiCrossbarEntity->name())
			break;
	}

	if (entity->name() != isiCrossbarEntity->name()) {
		LOG(NxpNeo, Error)
			<< "Sensor " << sensorEntity->name()
			<< " has no default path to ISI crossbar";
		return -ENOENT;
	}
	return 0;
}


/**
 * \brief Handle ISI reservations for the camera sensor
 *
 * Necessary reservations for a the camera sensor are:
 * - Channel(s) allocation(s) according to sensor properties
 * - ISI crossbar routes provision for later usage
 *
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::setupCameraIsiReserve()
{
	NxpNeoSensorProperties *sensorProps = &sensorProperties_;

	ISIDevice *isi = pipe()->isi_.get();

	unsigned int isiPipePadBase = isi->crossbarFirstSourcePad();

	V4L2Subdevice::Routing routing = {};
	uint32_t sinkPad;
	uint32_t sinkStream;
	uint32_t sourcePad;
	uint32_t sourceStream;

	auto channelRecordRoute = [&routing]
			(unsigned int _sinkPad, unsigned int _sinkStream,
			 unsigned int _sourcePad, unsigned int _sourceStream) {
		struct v4l2_subdev_route _route = {
			.sink_pad = _sinkPad,
			.sink_stream = _sinkStream,
			.source_pad = _sourcePad,
			.source_stream = _sourceStream,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
			.reserved = {}
		};

		routing.push_back(_route);
	};

	auto isiPipeDeleter = [=](ISIPipe *ipipe) { isi->freePipe(ipipe); };

	pipeInput0_ = std::shared_ptr<ISIPipe>(isi->allocPipe(), isiPipeDeleter);
	if (!pipeInput0_.get())
		return -ENODEV;

	sinkPad = static_cast<uint32_t>
			(sensorProps->input0Channel_.xbarPad_);
	sinkStream = static_cast<uint32_t>
			(sensorProps->input0Channel_.xbarPadStream_);
	sourcePad = static_cast<uint32_t>
			(isiPipePadBase + pipeInput0_->index());
	sourceStream = 0;

	channelRecordRoute(sinkPad, sinkStream, sourcePad, sourceStream);


	if (sensorProps->hasInput1()) {
		pipeInput1_ = std::shared_ptr<ISIPipe>
				(isi->allocPipe(), isiPipeDeleter);
		if (!pipeInput1_.get())
			return -ENODEV;

		sinkPad = static_cast<uint32_t>
				(sensorProps->input1Channel_.xbarPad_);
		sinkStream = static_cast<uint32_t>
				(sensorProps->input1Channel_.xbarPadStream_);
		sourcePad = static_cast<uint32_t>
				(isiPipePadBase + pipeInput1_->index());
		sourceStream = 0;

		channelRecordRoute(sinkPad, sinkStream, sourcePad, sourceStream);
	}

	if (sensorProps->hasEmbedded()) {
		pipeEd_ = std::shared_ptr<ISIPipe>
				(isi->allocPipe(), isiPipeDeleter);
		if (!pipeEd_.get())
			return -ENODEV;

		sinkPad = static_cast<uint32_t>
				(sensorProps->edChannel_.xbarPad_);
		sinkStream = static_cast<uint32_t>
				(sensorProps->edChannel_.xbarPadStream_);
		sourcePad = static_cast<uint32_t>
				(isiPipePadBase + pipeEd_->index());
		sourceStream = 0;

		channelRecordRoute(sinkPad, sinkStream, sourcePad, sourceStream);
	}

	LOG(NxpNeo, Debug)
		<< "Merge ISI routes for " << sensor_->id() << std::endl
		<< routing.toString();

	for (auto &route : routing)
		pipe()->isiRouting_.push_back(route);

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
	V4L2SubdeviceFormat &sdFormatInput0, V4L2SubdeviceFormat &sdFormatInput1,
	V4L2SubdeviceFormat &sdFormatEd,
	V4L2DeviceFormat *vdFormatInput0, V4L2DeviceFormat *vdFormatInput1,
	V4L2DeviceFormat *vdFormatEd)
{
	NxpNeoSensorProperties *sensorProps = &sensorProperties_;

	int ret = pipeInput0_->configure(sdFormatInput0, vdFormatInput0);
	if (sensorProps->hasInput1())
		ret|= pipeInput1_->configure(sdFormatInput1, vdFormatInput1);
	if (sensorProps->hasEmbedded())
		ret|= pipeEd_->configure(sdFormatEd, vdFormatEd);

	return ret;
}


/**
 * \brief Configure links and pads of the media graph
 * \param[in] links Vector of media links involved in camera graph.
 * \param[in] sdFormat The subdevice format to be along the graph.
 *
 * The media links involved in the sensor media graph are enabled.
 * Pads involved in the links are individually configured with the sensor
 * output format.
 * \todo add stream support to pads configuration to cater for INPUT1 and embedded
 * data channels.
 *
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::setupCameraConfigureGraph(std::vector<MediaLink *> &links,
						V4L2SubdeviceFormat &sdFormat)
{
	MediaDevice *media = pipe()->isiMedia_;

	for (auto &link : links) {
		const std::string sinkName = link->sink()->entity()->name();
		const std::string sourceName = link->source()->entity()->name();
		unsigned int sinkPad = link->sink()->index();
		unsigned int sourcePad = link->source()->index();

		/* Log warning and continue in case of error */
		int ret = link->setEnabled(true);
		if (ret)
			LOG(NxpNeo, Warning)
				<< "Error enabling link sink " << sinkName
				<< " source " << sourceName;

		LOG(NxpNeo, Debug)
			<< "Set format sink " << sinkName << " pad " << sinkPad
			<< " source " << sourceName << " pad " << sourcePad
			<< " format " << sdFormat.toString();

		std::unique_ptr<V4L2Subdevice> subDev;

		subDev = V4L2Subdevice::fromEntityName(media, sinkName);
		ret = subDev->open();
		if (ret) {
			LOG(NxpNeo, Warning)
				<< "Error opening subdev " << sinkName;
		}
		ret = subDev->setFormat(sinkPad, &sdFormat);
		if (ret) {
			LOG(NxpNeo, Warning)
				<< "Error setting format " << sinkName;
		}

		subDev = V4L2Subdevice::fromEntityName(media, sourceName);
		ret = subDev->open();
		if (ret) {
			LOG(NxpNeo, Warning)
				<< "Error opening subdev " << sourceName;
		}
		ret = subDev->setFormat(sourcePad, &sdFormat);
		if (ret) {
			LOG(NxpNeo, Warning)
				<< "Error setting format " << sourceName;
		}
	}

	return 0;
}

/**
 * \brief Early configuration of the sensor and associated subdevices
 *
 * \return 0 in case of success or a negative error code.
 */
int NxpNeoCameraData::setupCamera()
{
	int ret = setupCameraDiscoverLinks(&graphLinks_);
	if (ret)
		return ret;

	ret = setupCameraIsiReserve();
	if (ret)
		return ret;

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
	/* \todo check sensor mbus codes for IR when those are defined */
	return false;
}

/**
 * \brief Probe, configure and register camera sensor if applicable
 *
 * \return 0 on success or a negative error code for error or if no camera
 * has been created
 * \retval -ENODEV no camera has been created
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

	ret = loadSensorProperties(data->sensor_.get(),
				   &data->sensorProperties_);
	if (ret)
		return ret;

	ret = data->setupCamera();
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
	 * \todo Read delay values from the sensor itself or from a
	 * a sensor database. For now use generic values taken from
	 * the Raspberry Pi and listed as 'generic values'.
	 */
	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { 1, false } },
		{ V4L2_CID_EXPOSURE, { 2, false } },
	};
	data->delayedCtrls_ =
		std::make_unique<DelayedControls>(sensor->device(), params);
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
	NxpNeoSensorProperties *sensorProps = &data->sensorProperties_;

	data->pipeInput0_->bufferReady().connect(data.get(),
			&NxpNeoCameraData::isiInput0BufferReady);
	data->pipeInput0_->bufferAvailable.connect(data.get(),
			&NxpNeoCameraData::queuePendingRequests);
	if (sensorProps->hasInput1()) {
		data->pipeInput1_->bufferReady().connect(data.get(),
				&NxpNeoCameraData::isiInput1BufferReady);
		data->pipeInput1_->bufferAvailable.connect(data.get(),
				&NxpNeoCameraData::queuePendingRequests);
	}
	if (sensorProps->hasEmbedded()) {
		data->pipeEd_->bufferReady().connect(data.get(),
				&NxpNeoCameraData::isiEdBufferReady);
	}

	data->neo_->input0_->bufferReady.connect(data->pipeInput0_.get(),
			&ISIPipe::tryReturnBuffer);
	data->neo_->input1_->bufferReady.connect(data->pipeInput1_.get(),
			&ISIPipe::tryReturnBuffer);
	data->neo_->frame_->bufferReady.connect(data.get(),
			&NxpNeoCameraData::neoOutputBufferReady);
	data->neo_->ir_->bufferReady.connect(data.get(),
			&NxpNeoCameraData::neoOutputBufferReady);
	data->neo_->params_->bufferReady.connect(data.get(),
			&NxpNeoCameraData::neoParamsBufferReady);
	data->neo_->stats_->bufferReady.connect(data.get(),
			&NxpNeoCameraData::neoStatsBufferReady);

	/* Create and register the Camera instance. */
	const std::string &cameraId = sensor->id();
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), cameraId, streams);

	registerCamera(std::move(camera));

	return 0;
}

/**
 * \brief Configure the global V4L2 subdevices routes
 *
 * Configure the subdevices routes in the system. As route configuration does
 * reset the subdevice, it has to be done once at system discovery time.
 *
 * \return 0 on success, or a negative error code for error
 */
int PipelineHandlerNxpNeo::configureRoutes()
{
	int ret;
	V4L2Subdevice::Routing *isiRoutes = &isiRouting_;

	LOG(NxpNeo, Debug)
		<< "Configure aggregated ISI routes " << std::endl
		<< isiRoutes->toString();

	ret = isi_->crossbar()
		      ->setRouting(isiRoutes, V4L2Subdevice::ActiveFormat);
	if (ret) {
		LOG(NxpNeo, Error) << "Error setting ISI routes";
		return ret;
	}

	return 0;
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
			LOG(NxpNeo, Error) << "Failed to probe camera "
					   << entity->name() << ": " << ret;
		} else {
			numCameras++;
			neos.pop();
		}
	}

	/*
	 * Cameras have been probed and configured, so global routing
	 * configuration can be applied now.
	 */
	ret = configureRoutes();
	if (ret)
		return ret;

	return numCameras > 0;
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
 * \return 0 on success or a negative error code otherwise
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
 * \return 0 on success or a negative error code otherwise
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
	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor->model() },
			 hwRevision,
			 sensorInfo, sensor->controls(), &ipaControls_);
	if (ret) {
		LOG(NxpNeo, Error) << "Failed to initialise the NxpNeo IPA";
		return ret;
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
	neo_->input0_->queueBuffer(info->rawBuffer);
	if (sensorProperties_.hasInput1())
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
 * the same frame can be marked as cancelled and completed.
 */
bool NxpNeoCameraData::screenCancelledBuffer(FrameBuffer *buffer,
					     NxpNeoFrames::Info *info)
{
	Request *request = info->request;

	/* If the buffer is cancelled force a complete of the whole request. */
	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		for (auto it : request->buffers()) {
			FrameBuffer *b = it.second;
			b->_d()->cancel();
			pipe()->completeBuffer(request, b);
		}

		frameInfos_.remove(info);
		pipe()->completeRequest(request);

		return true;
	}

	return false;
}

/**
 * \brief Handle buffers completion at the NEO output
 * \param[in] buffer The completed buffer
 *
 * Buffers completed from the NEO output are directed to the application.
 * This callback is common to main (frame) and IR ISP outputs
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
	 * \todo: in case of ISP operation with INPUT0 + INPUT1 inputs, wait for both
	 * frames to be available before invoking ipa_->fillParamsBuffer()
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

	Request *request = info->request;

	/*
	 * \todo: make something with that buffer - recycle for now
	 */
	pipe()->completeBuffer(request, buffer);
	ASSERT(pipeEd_);
	pipeEd_->tryReturnBuffer(buffer);
}

/**
 * \brief Handle INPUT0 buffers consumed by ISP
 * \param[in] buffer The completed buffer
 *
 * Once buffer has been consumed by ISP it may be recycled to pipe buffer pool
 */
void NxpNeoCameraData::neoInput0BufferConsumed(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	Request *request = info->request;

	pipe()->completeBuffer(request, buffer);
	pipeInput0_->tryReturnBuffer(buffer);
}

/**
 * \brief Handle INPUT1 buffers consumed by ISP
 * \param[in] buffer The completed buffer
 *
 * Once buffer has been consumed by ISP it may be recycled to pipe buffer pool
 */
void NxpNeoCameraData::neoInput1BufferConsumed(FrameBuffer *buffer)
{
	NxpNeoFrames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	Request *request = info->request;

	pipe()->completeBuffer(request, buffer);
	ASSERT(pipeInput1_);
	pipeInput1_->tryReturnBuffer(buffer);
}

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
