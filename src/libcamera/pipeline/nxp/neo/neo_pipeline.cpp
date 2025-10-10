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
#include <libcamera/orientation.h>
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

#include "isi_device.h"
#include "neo_device.h"
#include "neo_utils.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoPipe)

using namespace libcamera::nxpneo;

class PipelineHandlerNxpNeo;
class NxpNeoCameraData;

namespace nxpneo {

/*
 * Those enums are mirrored in neo mojom file definitions in order to keep
 * both interfaces self-contained though keeping trivial enums conversion.
 */

enum BufferType {
	BufferTypeImage0,
	BufferTypeImage1,
	BufferTypeEData,
	BufferTypeParams,
	BufferTypeStats,
	BufferTypeFrame,
	BufferTypeIr,
};

enum ContextType {
	ContextTypeRgb,
	ContextTypeIr,
};

enum ModeType {
	ModeTypeStandard,
	ModeTypeHdrMerge,
	ModeTypeRgbIr,
	ModeTypeRgbIrDual,
};

} /* namespace nxpneo */

/**
 * \class NxpNeoFrames
 * \brief Frames control class to handle the active libcamera::Request
 *
 * In order to process a libcamera::Request queued by the application, the
 * pipeline handler has to bundle and track the buffers necessary to process
 * this request. Application may provide in the libcamera::Request the streams
 * buffers to store the processed images and/or the raw image. Additional
 * buffers that remain internal to the pipeline handler are also necessary to
 * process a libcamera::Request: the ones used by front-end, the ISP params and
 * stats buffers exchanged between the pipeline handler and the IPA.
 * NxpNeoFrames class supports the handling of the libcamera::Request
 * concurrently active at a point of time in the pipeline handler, by
 * maintaining a NxpNeoFrames::Info instance for each request in progress until
 * its completion.
 */

/**
 * \struct NxpNeoFrames::InfoContext
 * \brief Frame context descriptor
 *
 * Some sensors have specific modes of operation where they maintain multiple
 * banks (or contexts) of internal registers values that will be applied
 * in sequence in order to produce successive raw images.
 * Those multiple images are used by the pipeline handler as the basis of
 * to produce different streams buffers for the application, that all belong to
 * the same libcamera::Request.
 * A NxpNeoFrames::InfoContext instance is associated to each image context.
 */

/**
 * \var NxpNeoFrames::InfoContext::buffers
 * \brief Buffers and status associated to the context image
 *
 * Each element of the map is a std::pair<BufferFrame *, bool> representing
 * for each buffer type:
 *  - The buffer itself represented by a FrameBuffer
 *  - The buffer receipt status - true if pending, false once complete
 * The buffers associated to a context are:
 *  - The front end buffers usually allocated from internal buffer pools but
 *    may also come from the application when a raw stream is mapped
 *  - The ISP params and stats buffers exchanged between the pipeline handler
 *    and the IPA, allocated from internal buffer pools
 *  - The buffers for the images decoded by the ISP, usually provided by the
 *    application as the buffers associated to the streams.
 * Buffers allocation and mapping is done at NxpNeoFrames::Info creation time.
 *
 * \var NxpNeoFrames::InfoContext::paramDequeued
 * \brief Indicates that the params buffer has been consumed by the ISP
 *
 * \var NxpNeoFrames::InfoContext::metadataProcessed
 * \brief Indicates that the IPA produced the metadata from the ISP stats buffer
 */

/**
 * \struct NxpNeoFrames::Info
 * \brief Frame context descriptor
 *
 * A NxpNeoFrames::Info represents an active libcamera::Request in the pipeline
 * for the whole duration of its processing. It is associated to usually one but
 * possibly more camera contexts, each context being represented by an instance
 * of NxpNeoFrames::InfoContext.
 * Such a frame instance essentially bundles a libcamera::Request with the
 * different buffers involved for its completion.
 */

/**
 * \var NxpNeoFrames::Info::id
 * \brief Corresponds to the libcamera::Request sequence number
 *
 * \var NxpNeoFrames::Info::request
 * \brief The libcamera::Request bundled to that frame
 *
 * \var NxpNeoFrames::Info::rawStreamBuffer
 * \brief The application raw Stream buffer from the request - null if none
 *
 *  \var NxpNeoFrames::Info::frameStreamBuffer
 * \brief The application frame Stream buffer from the request - null if none
 *
 *  \var NxpNeoFrames::Info::irStreamBuffer
 * \brief The application IR Stream buffer from the request - null if none
 *
 *  \var NxpNeoFrames::Info::contexts
 * \brief Map of one or more context instances associated to this frame
 */

class NxpNeoFrames
{
public:
	struct InfoContext {
		std::map<BufferType, std::pair<FrameBuffer *, bool>> buffers;

		bool paramDequeued;
		bool metadataProcessed;
	};

	struct Info {
		unsigned int id;
		Request *request;

		FrameBuffer *rawStreamBuffer;
		FrameBuffer *frameStreamBuffer;
		FrameBuffer *irStreamBuffer;

		std::map<ContextType, InfoContext> contexts;
	};

	NxpNeoFrames(NxpNeoCameraData *data);

	int destroy(unsigned int id);
	void clear();

	Info *create(Request *request);
	Info *createRaw(Request *request);
	Info *createYuv(Request *request);

	Info *find(unsigned int id) const;
	std::pair<Info *, ContextType> find(FrameBuffer *buffer) const;
	Info *find(Request *request) const;

	int completeBuffer(Info *info, ContextType context,
			   const FrameBuffer *buffer) const;
	bool isBufferPending(Info *info, ContextType context,
			     const std::vector<BufferType> &bufferTypes) const;
	bool isContextComplete(Info *info, ContextType context) const;
	bool isFrameComplete(Info *info) const;
	FrameBuffer *buffer(Info *info, ContextType context,
			    BufferType, bool expected) const;

	Signal<> bufferAvailable;

private:
	FrameBuffer *allocBuffer(BufferType bufferType);

	NxpNeoCameraData *data_;
	std::map<unsigned int, std::unique_ptr<Info>> frameInfo_;
};

class NxpNeoCameraData : public Camera::Private
{
public:
	NxpNeoCameraData(PipelineHandler *pipe,
			 std::unique_ptr<CameraSensor> sensor,
			 const CameraInfo *cameraInfo)
		: Camera::Private(pipe),
		  sensor_(std::move(sensor)),
		  cameraInfo_(cameraInfo),
		  frameInfos_(this){};

	int configure(CameraConfiguration *c);
	int configureRaw(CameraConfiguration *c);
	int configureYuv(CameraConfiguration *c);
	int exportFrameBuffers(Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	int start(const ControlList *controls);
	void stopDevice();

	void queuePendingRequests();

	int init(DeviceEnumerator *enumerator);
	PipelineHandlerNxpNeo *pipe();

	bool sensorIsRgbIr() const { return sensorIsRgbIr_; }
	void adjustTopLinesSize(Size *size) const;
	int configureFrontEndFormat(V4L2SubdeviceFormat &sensorFormat,
				    Transform transform);

	CameraSensor *sensor() const { return sensor_.get(); }
	NeoDevice *neoDevice() const { return neo_.get(); }
	std::map<StreamType, ISIPipe *> &isiPipes() { return pipes_; };
	const std::string &cameraName() const { return sensor_->entity()->name(); }
	bool multiCamera() const { return cameraInfo_->cameraProperties().multiCamera; }
	bool isRawCamera() const { return isRawCamera_; };
	const std::map<Size, std::vector<unsigned int>> &
	formatsSizeToCodes() const { return formatsSizeToCodes_; }
	const std::map<unsigned int, std::vector<Size>> &
	formatsCodeToSizes() const { return formatsCodeToSizes_; }
	const std::optional<Orientation> &defaultOrientation() const { return defaultOrientation_; }

	bool rawStreamOnly_ = false;

	Stream streamFrame_;
	Stream streamIr_;
	Stream streamRaw_;

	/* Requests for which no buffer has been queued to the frontend  device yet */
	std::queue<Request *> pendingRequests_;
	/* Requests queued to the frontend device but not yet processed by the ISP */
	std::queue<Request *> processingRequests_;

private:
	friend NxpNeoFrames;

	int updateControls();
	int loadIPA();

	int allocateBuffers();
	int allocateBuffersRaw();
	int allocateBuffersYuv();
	int freeBuffers();
	int freeBuffersRaw();
	int freeBuffersYuv();

	int configureFrontEndStream(const std::vector<CameraMediaStream::StreamLink> &streamLinks,
				    V4L2SubdeviceFormat &sdFormat);
	int configureFrontEndLinks() const;

	int enumerateFormatsRaw();
	int enumerateFormatsYuv();

	void clearRequest(NxpNeoFrames::Info *info);
	void cancelCompleteRequest(NxpNeoFrames::Info *info);
	void tryCompleteRequest(NxpNeoFrames::Info *info);

	void isiInputBufferReady(NxpNeoFrames::Info *info, ContextType context);
	void isiImage0BufferReady(FrameBuffer *buffer);
	void isiImage1BufferReady(FrameBuffer *buffer);
	void isiEmbeddedDataBufferReady(FrameBuffer *buffer);

	void neoInput0BufferReady(FrameBuffer *buffer);
	void neoInput1BufferReady(FrameBuffer *buffer);
	void neoOutputBufferReady(FrameBuffer *buffer);
	void neoParamsBufferReady(FrameBuffer *buffer);
	void neoStatsBufferReady(FrameBuffer *buffer);

	void ipaParamsComputed(unsigned int id, ipa::nxpneo::IPAContextType context,
			       unsigned int bytesused);
	void ipaMetadataReady(unsigned int id, ipa::nxpneo::IPAContextType context,
			      const ControlList &metadata);
	void ipaSetSensorControls(unsigned int id, ipa::nxpneo::IPAContextType context,
				  const ControlList &sensorControls);
	unsigned int contextCount() { return mode_ == ModeTypeRgbIrDual ? 2 : 1; };

	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<NeoDevice> neo_;
	const CameraInfo *cameraInfo_;
	std::optional<Orientation> defaultOrientation_;
	std::map<Size, std::vector<unsigned int>> formatsSizeToCodes_;
	std::map<unsigned int, std::vector<Size>> formatsCodeToSizes_;

	/* Front end pipes and subdevice formats - maps per stream */
	std::map<StreamType, ISIPipe *> pipes_;
	std::map<StreamType, V4L2SubdeviceFormat> pipesSubDevFormats_;

	NxpNeoFrames frameInfos_;
	bool alternatedRawStream_ = false;

	std::unique_ptr<ipa::nxpneo::IPAProxyNxpNeo> ipa_;
	ControlInfoMap ipaControls_;
	std::vector<IPABuffer> ipaBuffers_;
	std::map<ContextType, std::unique_ptr<DelayedControls>> delayedCtrls_;

	unsigned int sequence_ = 0;
	bool sensorIsRgbIr_ = false;
	unsigned int embeddedTopLines_ = 0;
	bool isRawCamera_ = false;

	ModeType mode_ = ModeTypeStandard;

	std::map<BufferType, std::queue<FrameBuffer *>> availableBuffersMap_;
	std::vector<std::unique_ptr<FrameBuffer>> frameBuffersPool_;
	std::vector<std::unique_ptr<FrameBuffer>> irBuffersPool_;
};

class NxpNeoCameraConfiguration : public CameraConfiguration
{
public:
	NxpNeoCameraConfiguration(Camera *camera, NxpNeoCameraData *data);

	Status validate() override;
	Status validateRaw();
	Status validateYuv();

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
	std::unique_ptr<CameraConfiguration> generateRawConfiguration(Camera *camera,
								      Span<const StreamRole> roles);
	std::unique_ptr<CameraConfiguration> generateYuvConfiguration(Camera *camera,
								      Span<const StreamRole> roles);

	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

	bool acquireDevice(Camera *camera) override;
	void releaseDevice(Camera *camera) override;

	ISIDevice *isiDevice() const { return isi_.get(); }
	const PipelineConfig *pipelineConfig() { return &pipelineConfig_; }

private:
	friend NxpNeoCameraData;

	NxpNeoCameraData *cameraData(Camera *camera)
	{
		return static_cast<NxpNeoCameraData *>(camera->_d());
	}

	int createCamera(MediaEntity *sensorEntity, DeviceEnumerator *enumerator);

	int setupRouting() const;
	int setupCameraGraphs();
	int loadPipelineConfig();

	unsigned int numCameras() const { return numCamerasRaw_ + numCamerasYuv_; }
	unsigned int numCamerasRaw() const { return numCamerasRaw_; }
	unsigned int numCamerasYuv() const { return numCamerasYuv_; }

	PipelineConfig pipelineConfig_;

	unsigned int numCamerasRaw_ = 0;
	unsigned int numCamerasYuv_ = 0;
	unsigned int acquireCount_ = 0;
	std::shared_ptr<ISIDevice> isi_;
};

namespace {

const std::map<StreamType, BufferType> streamToBufferType = {
	{ StreamTypeImage0, BufferTypeImage0 },
	{ StreamTypeImage1, BufferTypeImage1 },
	{ StreamTypeEData, BufferTypeEData },
};

}

NxpNeoFrames::NxpNeoFrames(NxpNeoCameraData *data)
	: data_(data)
{
}

int NxpNeoFrames::destroy(unsigned int id)
{
	Info *info = find(id);
	if (!info)
		return -ENOENT;

	/* Return internal buffers for reuse. */
	for (const auto &[context, infoContext] : info->contexts) {
		for (const auto &[bufferType, bufferDesc] : infoContext.buffers) {
			FrameBuffer *buffer = bufferDesc.first;
			if (buffer == info->rawStreamBuffer ||
			    buffer == info->frameStreamBuffer ||
			    buffer == info->irStreamBuffer)
				continue;
			data_->availableBuffersMap_[bufferType].push(buffer);
		}
	}

	/* Delete the extended frame information. */
	frameInfo_.erase(info->id);

	bufferAvailable.emit();

	return 0;
}

void NxpNeoFrames::clear()
{
	frameInfo_.clear();
}

NxpNeoFrames::Info *NxpNeoFrames::create(Request *request)
{
	if (data_->isRawCamera())
		return createRaw(request);
	else
		return createYuv(request);
}

NxpNeoFrames::Info *NxpNeoFrames::createRaw(Request *request)
{
	unsigned int id = request->sequence();

	/*
	 * First make sure there is a sufficient number of internal buffers
	 * available to populate the NxpNeoFrames::Info.
	 * In RGBIr context switch mode, one buffer per frame context is needed
	 * for embedded data as well as for ISP params and stats.
	 */
	unsigned int _contextCount = data_->contextCount();
	for (const auto &[type, availableBuffers] : data_->availableBuffersMap_) {
		unsigned int count =
			type == BufferTypeEData || type == BufferTypeParams || type == BufferTypeStats
				? _contextCount
				: 1;
		if (availableBuffers.size() < count) {
			LOG(NxpNeoPipe, Warning) << " buffers underrun type " << type;
			return nullptr;
		}
	}

	/* \todo Remove the dynamic allocation of Info */
	std::unique_ptr<Info> info = std::make_unique<Info>();

	info->id = id;
	info->request = request;
	info->rawStreamBuffer = request->findBuffer(&data_->streamRaw_);
	info->frameStreamBuffer = request->findBuffer(&data_->streamFrame_);
	info->irStreamBuffer = request->findBuffer(&data_->streamIr_);
	info->contexts.insert({ ContextTypeRgb, {} });
	if (data_->mode_ == ModeTypeRgbIrDual)
		info->contexts.insert({ ContextTypeIr, {} });

	bool evenRequest = (id % 2 == 0);
	for (auto &[context, infoContext] : info->contexts) {
		/*
		 * Map the ISP input buffers that are typically internal buffers
		 * unless the raw stream is active in which case the raw buffer
		 * is provided by the application.
		 * Alternate mapping of the raw stream has special cases:
		 * - RGBIr context switch: we may want to alternate capture on
		 *   the different contexts RGB and IR
		 * - HDR merge mode, we may want to alternate capture on long
		 *   and short frames (image0 and image 1)
		 * In other cases, the raw stream is unconditionally mapped to
		 * the image0. If there is no raw stream enabled and so no
		 * dedicated buffer provided by the application, then internal
		 * buffers are used for image0 and image1 active pipes.
		 */
		FrameBuffer *image0Buffer = nullptr;
		FrameBuffer *image1Buffer = nullptr;

		unsigned int mode = data_->mode_;
		bool hasImage0 = (mode != ModeTypeRgbIrDual || context == ContextTypeRgb);
		bool hasImage1 = (mode == ModeTypeHdrMerge ||
				  (mode == ModeTypeRgbIrDual && context == ContextTypeIr));

		bool alternatedRawStream = data_->alternatedRawStream_;
		if (info->rawStreamBuffer) {
			if (alternatedRawStream && mode == ModeTypeRgbIrDual) {
				unsigned int rawContext = evenRequest ? ContextTypeRgb : ContextTypeIr;
				if (context == rawContext) {
					if (hasImage0)
						image0Buffer = info->rawStreamBuffer;
					else
						image1Buffer = info->rawStreamBuffer;
				}
			} else if (alternatedRawStream && mode == ModeTypeHdrMerge) {
				if (evenRequest)
					image0Buffer = info->rawStreamBuffer;
				else
					image1Buffer = info->rawStreamBuffer;

			} else {
				if (hasImage0)
					image0Buffer = info->rawStreamBuffer;
			}
		}

		auto &buffersMap = infoContext.buffers;
		if (hasImage0 && !image0Buffer)
			image0Buffer = allocBuffer(BufferTypeImage0);
		if (image0Buffer)
			buffersMap.insert({ BufferTypeImage0, { image0Buffer, true } });

		if (hasImage1 && !image1Buffer)
			image1Buffer = allocBuffer(BufferTypeImage1);
		if (image1Buffer)
			buffersMap.insert({ BufferTypeImage1, { image1Buffer, true } });

		bool hasEmbeddedData =
			data_->availableBuffersMap_.count(BufferTypeEData);
		if (hasEmbeddedData) {
			FrameBuffer *edataBuffer = allocBuffer(BufferTypeEData);
			buffersMap.insert({ BufferTypeEData, { edataBuffer, true } });
		}

		/* Map the ISP params / stats internal buffers */
		FrameBuffer *paramsBuffer = allocBuffer(BufferTypeParams);
		buffersMap.insert({ BufferTypeParams, { paramsBuffer, true } });
		FrameBuffer *statsBuffer = allocBuffer(BufferTypeStats);
		buffersMap.insert({ BufferTypeStats, { statsBuffer, true } });

		infoContext.paramDequeued = false;
		infoContext.metadataProcessed = false;

		/*
		 * Map the ISP frame and infrared output buffer of the ISP. They
		 * are provided by the application in the libcamera:Request as
		 * streams buffers. The exception is the RGBIr context switch
		 * where some dummy internal buffer have to be provided for the
		 * ISP decoded output that are discarded.
		 */
		FrameBuffer *frameBuffer = nullptr;
		FrameBuffer *irBuffer = nullptr;

		switch (mode) {
		case ModeTypeRgbIrDual:
			ASSERT(context == ContextTypeRgb || context == ContextTypeIr);
			if (context == ContextTypeRgb) {
				if (data_->rawStreamOnly_)
					frameBuffer = allocBuffer(BufferTypeFrame);
				else
					frameBuffer = info->frameStreamBuffer;
				if (info->irStreamBuffer)
					irBuffer = allocBuffer(BufferTypeIr);
			} else {
				irBuffer = info->irStreamBuffer;
				if ((info->frameStreamBuffer) || (data_->rawStreamOnly_))
					frameBuffer = allocBuffer(BufferTypeFrame);
			}
			break;
		case ModeTypeRgbIr:
		case ModeTypeStandard:
		case ModeTypeHdrMerge:
		default:
			ASSERT(context == ContextTypeRgb);
			if (data_->rawStreamOnly_)
				frameBuffer = allocBuffer(BufferTypeFrame);
			else
				frameBuffer = info->frameStreamBuffer;
			irBuffer = info->irStreamBuffer;
			break;
		}

		if (frameBuffer)
			buffersMap.insert({ BufferTypeFrame, { frameBuffer, true } });
		if (irBuffer)
			buffersMap.insert({ BufferTypeIr, { irBuffer, true } });
	}

	frameInfo_[id] = std::move(info);

	return frameInfo_[id].get();
}

NxpNeoFrames::Info *NxpNeoFrames::createYuv(Request *request)
{
	unsigned int id = request->sequence();

	std::unique_ptr<Info> info = std::make_unique<Info>();

	/* Single context, the output buffer is provided by application. */
	info->id = id;
	info->request = request;
	info->rawStreamBuffer = request->findBuffer(&data_->streamRaw_);
	info->frameStreamBuffer = nullptr;
	info->irStreamBuffer = nullptr;

	info->contexts.insert({ ContextTypeRgb, {} });
	InfoContext &infoContext = info->contexts.at(ContextTypeRgb);
	auto &buffersMap = infoContext.buffers;
	buffersMap.insert({ BufferTypeImage0, { info->rawStreamBuffer, true } });

	/* IPA-related operations are bypassed */
	infoContext.paramDequeued = true;
	infoContext.metadataProcessed = true;

	frameInfo_[id] = std::move(info);

	return frameInfo_[id].get();
}

NxpNeoFrames::Info *NxpNeoFrames::find(unsigned int id) const
{
	const auto &itInfo = frameInfo_.find(id);

	if (itInfo != frameInfo_.end())
		return itInfo->second.get();

	LOG(NxpNeoPipe, Debug) << "Can't find tracking information for frame " << id;

	return nullptr;
}

std::pair<NxpNeoFrames::Info *, ContextType> NxpNeoFrames::find(FrameBuffer *buffer) const
{
	for (auto &itInfo : frameInfo_) {
		Info *info = itInfo.second.get();
		for (auto &[context, infoContext] : info->contexts)
			for (const auto &[bufferType, bufferDesc] : infoContext.buffers)
				if (bufferDesc.first == buffer)
					return { info, context };

	}

	LOG(NxpNeoPipe, Debug) << "Can't find tracking information from buffer";

	return { nullptr, ContextTypeRgb };
}

NxpNeoFrames::Info *NxpNeoFrames::find(Request *request) const
{
	for (const auto &itInfo : frameInfo_) {
		Info *info = itInfo.second.get();
		if (info->request == request)
			return info;
	}

	LOG(NxpNeoPipe, Debug) << "Can't find tracking information from request";

	return nullptr;
}

int NxpNeoFrames::completeBuffer(Info *info, ContextType context,
				 const FrameBuffer *buffer) const
{
	auto itContext = info->contexts.find(context);
	if (itContext == info->contexts.end()) {
		LOG(NxpNeoPipe, Error) << "Context does not exist " << context;
		return -EINVAL;
	}
	InfoContext &infoContext = itContext->second;
	for (auto &[bufferType, bufferDesc] : infoContext.buffers) {
		if (bufferDesc.first != buffer)
			continue;
		if (!bufferDesc.second) {
			LOG(NxpNeoPipe, Error) << "Buffer already completed";
			return -EINVAL;
		}
		bufferDesc.second = false;
		return 0;
	}

	LOG(NxpNeoPipe, Error) << "Buffer to complete not found in Info";

	return -ENOENT;
}

bool NxpNeoFrames::isBufferPending(Info *info, ContextType context,
				   const std::vector<BufferType> &bufferTypes) const
{
	auto itContext = info->contexts.find(context);
	if (itContext == info->contexts.end()) {
		LOG(NxpNeoPipe, Error) << "Context does not exist " << context;
		return false;
	}
	const InfoContext &infoContext = itContext->second;
	for (BufferType bufferType : bufferTypes) {
		auto it = infoContext.buffers.find(bufferType);
		if (it == infoContext.buffers.end())
			continue;
		const auto &bufferDesc = it->second;
		if (bufferDesc.second)
			return true;
	}

	return false;
}

bool NxpNeoFrames::isContextComplete(Info *info, ContextType context) const
{
	auto itContext = info->contexts.find(context);
	if (itContext == info->contexts.end()) {
		LOG(NxpNeoPipe, Error) << "Context does not exist " << context;
		return false;
	}
	const InfoContext &infoContext = itContext->second;

	const std::vector<BufferType> allBufferTypes = {
		BufferTypeImage0,
		BufferTypeImage1,
		BufferTypeEData,
		BufferTypeParams,
		BufferTypeStats,
		BufferTypeFrame,
		BufferTypeIr,
	};
	bool buffersComplete = !isBufferPending(info, context, allBufferTypes);
	bool complete = buffersComplete &&
			infoContext.metadataProcessed && infoContext.paramDequeued;

	return complete;
}

bool NxpNeoFrames::isFrameComplete(Info *info) const
{
	for (const auto &[context, infoContext] : info->contexts) {
		if (!isContextComplete(info, context))
			return false;
	}

	return true;
}

FrameBuffer *NxpNeoFrames::buffer(Info *info, ContextType context,
				  BufferType bufferType, bool expected) const
{
	auto itContext = info->contexts.find(context);
	if (itContext == info->contexts.end()) {
		LOG(NxpNeoPipe, Error) << "Context does not exist " << context;
		return nullptr;
	}
	const InfoContext &infoContext = itContext->second;

	FrameBuffer *buffer = nullptr;
	auto it = infoContext.buffers.find(bufferType);
	if (it != infoContext.buffers.end()) {
		auto &bufferDesc = it->second;
		buffer = bufferDesc.first;
	}

	if (expected && !buffer)
		LOG(NxpNeoPipe, Error)
			<< "Expected buffer type " << bufferType;
	return buffer;
}

FrameBuffer *NxpNeoFrames::allocBuffer(BufferType bufferType)
{
	auto &buffersMap = data_->availableBuffersMap_;
	auto it = buffersMap.find(bufferType);
	if (it == buffersMap.end()) {
		LOG(NxpNeoPipe, Error) << " No buffers for type " << bufferType;
		return nullptr;
	}

	std::queue<FrameBuffer *> &queue = it->second;
	ASSERT(!queue.empty());
	FrameBuffer *buffer = queue.front();
	queue.pop();
	return buffer;
}

NxpNeoCameraConfiguration::NxpNeoCameraConfiguration(Camera *camera,
						     NxpNeoCameraData *data)
	: CameraConfiguration()
{
	camera_ = camera->shared_from_this();
	data_ = data;
}

CameraConfiguration::Status NxpNeoCameraConfiguration::validate()
{
	if (data_->isRawCamera())
		return validateRaw();
	else
		return validateYuv();
}

CameraConfiguration::Status NxpNeoCameraConfiguration::validateRaw()
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
	if (data_->defaultOrientation().has_value())
		orientation = data_->defaultOrientation().value();
	combinedTransform_ = sensor->computeTransform(&orientation);
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
		data_->formatsCodeToSizes();
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
			data_->formatsSizeToCodes();
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

CameraConfiguration::Status NxpNeoCameraConfiguration::validateYuv()
{
	Status status = Valid;
	CameraSensor *sensor = data_->sensor();

	if (config_.empty())
		return Invalid;

	if (config_.size() > 1) {
		config_.resize(1);
		status = Adjusted;
	}

	Stream *streamRaw = const_cast<Stream *>(&data_->streamRaw_);
	StreamConfiguration &cfg = config_[0];
	const StreamConfiguration originalCfg = cfg;
	cfg.setStream(streamRaw);

	Orientation requestedOrientation = orientation;
	if (data_->defaultOrientation().has_value())
		orientation = data_->defaultOrientation().value();
	combinedTransform_ = sensor->computeTransform(&orientation);
	if (orientation != requestedOrientation)
		status = Adjusted;

	/*
	 * Make sure that the configuration size matches one resolution provided
	 * by the sensor. Also verify that the stream pixel format belongs to
	 * the list of processed formats from ISI.
	 * \todo Add support for channel-based rescaling
	 */
	const std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		data_->formatsSizeToCodes();
	if (sizeToCodes.find(cfg.size) == sizeToCodes.end())
		cfg.size = sizeToCodes.rbegin()->first;

	const std::vector<PixelFormat> &pixelFormats =
		ISIPipe::pixelFormatsProcessed();
	auto it = std::find(pixelFormats.begin(), pixelFormats.end(), cfg.pixelFormat);
	if (it == pixelFormats.end())
		cfg.pixelFormat = pixelFormats[0];

	/*
	 * Cache sensor format for later usage by configure().
	 * Look for a sensor format with the same color encoding as the stream.
	 */
	const PixelFormatInfo &pixelInfo = PixelFormatInfo::info(cfg.pixelFormat);
	PixelFormatInfo::ColourEncoding encoding = pixelInfo.colourEncoding;
	const std::vector<unsigned int> &codes = sizeToCodes.at(cfg.size);
	auto itCode = std::find_if(codes.begin(), codes.end(),
				   [encoding](unsigned int code) {
					   const MediaBusFormatInfo &mbInfo =
						   MediaBusFormatInfo::info(code);
					   return mbInfo.colourEncoding == encoding;
				   });
	sensorFormat_ = {};
	if (itCode != codes.end())
		sensorFormat_.code = *itCode;
	else
		sensorFormat_.code = codes[0];
	sensorFormat_.size = cfg.size;
	LOG(NxpNeoPipe, Debug) << "Sensor format " << sensorFormat_.toString();

	/* Acquire stride and color space from the pipe device */
	std::map<StreamType, ISIPipe *> pipes = data_->isiPipes();
	V4L2VideoDevice *pipeDevice = pipes[StreamTypeImage0]->output_.get();
	V4L2DeviceFormat devFormat = {};
	devFormat.size = cfg.size;
	devFormat.fourcc = pipeDevice->toV4L2PixelFormat(cfg.pixelFormat);
	devFormat.colorSpace = cfg.colorSpace;
	pipeDevice->tryFormat(&devFormat);

	cfg.colorSpace = devFormat.colorSpace;
	cfg.stride = devFormat.planes[0].bpl;
	cfg.frameSize = devFormat.planes[0].size;

	const GlobalInfo &globalInfo =
		data_->pipe()->pipelineConfig()->globalInfo();
	cfg.bufferCount = globalInfo.bufferCount;

	if (cfg.pixelFormat != originalCfg.pixelFormat ||
	    cfg.size != originalCfg.size) {
		status = Adjusted;
	}

	if (originalCfg.colorSpace.has_value() &&
	    cfg.colorSpace != originalCfg.colorSpace) {
		status = Adjusted;
	}

	return status;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerNxpNeo::generateConfiguration(Camera *camera,
					     Span<const StreamRole> roles)
{
	NxpNeoCameraData *data = cameraData(camera);
	if (data->isRawCamera())
		return generateRawConfiguration(camera, roles);
	else
		return generateYuvConfiguration(camera, roles);
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerNxpNeo::generateRawConfiguration(Camera *camera,
						Span<const StreamRole> roles)
{
	NxpNeoCameraData *data = cameraData(camera);
	std::unique_ptr<NxpNeoCameraConfiguration> config =
		std::make_unique<NxpNeoCameraConfiguration>(camera, data);

	LOG(NxpNeoPipe, Debug) << "Generate raw configuration " << data->cameraName();

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
		data->formatsSizeToCodes();
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
				data->formatsCodeToSizes();
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

	if (data->defaultOrientation().has_value())
		config->orientation = data->defaultOrientation().value();

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerNxpNeo::generateYuvConfiguration(Camera *camera,
						Span<const StreamRole> roles)
{
	NxpNeoCameraData *data = cameraData(camera);
	std::unique_ptr<NxpNeoCameraConfiguration> config =
		std::make_unique<NxpNeoCameraConfiguration>(camera, data);

	LOG(NxpNeoPipe, Debug) << "Generate YUV configuration " << data->cameraName();

	if (roles.empty())
		return config;

	/* \todo: Add multistream support */
	if (roles.size() > 1) {
		LOG(NxpNeoPipe, Error) << "Single stream supported";
		return nullptr;
	}

	const StreamRole &role = roles[0];
	std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
	const std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		data->formatsSizeToCodes();
	const std::vector<Size> sensorSizes = utils::map_keys(sizeToCodes);

	switch (role) {
	case StreamRole::StillCapture:
	case StreamRole::Viewfinder:
	case StreamRole::VideoRecording: {
		/*
		 * Sensor formats were screened to be compatible with ISI
		 * channel processed mode. Provide all the RGB/YUV processed
		 * formats with the sensor native size.
		 * \todo Add channel rescaling support
		 */
		std::vector<SizeRange> sensorRanges;
		for (const Size &size : sensorSizes)
			sensorRanges.emplace_back(size);

		std::vector<PixelFormat> pixelFormats =
			ISIPipe::pixelFormatsProcessed();
		for (const PixelFormat &format : pixelFormats)
			streamFormats[format] = sensorRanges;
		break;
	}

	default:
		LOG(NxpNeoPipe, Error) << "Stream role not supported " << role;
		return nullptr;
	}

	StreamFormats formats(streamFormats);
	StreamConfiguration cfg(formats);
	cfg.pixelFormat = formats::YUYV;
	cfg.size = sensorSizes.back();

	const GlobalInfo &globalInfo =
		data->pipe()->pipelineConfig()->globalInfo();
	cfg.bufferCount = globalInfo.bufferCount;

	config->addConfiguration(cfg);
	LOG(NxpNeoPipe, Debug)
		<< "Generated configuration " << cfg.toString()
		<< " for role " << role;

	if (data->defaultOrientation().has_value())
		config->orientation = data->defaultOrientation().value();

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

	/* Discover camera entities from the frontend media controller device. */
	std::vector<MediaEntity *> sensorsEntities = locateSensors(isiMedia);
	for (MediaEntity *entity : sensorsEntities) {
		ret = createCamera(entity, enumerator);
		if (ret)
			LOG(NxpNeoPipe, Warning) << "Failed to probe camera "
						 << entity->name() << ": " << ret;
	}

	if (numCameras() < 1)
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
					DeviceEnumerator *enumerator)
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

	/* CameraData instance creation */
	std::unique_ptr<NxpNeoCameraData> data =
		std::make_unique<NxpNeoCameraData>(this, std::move(sensor), cameraInfo);

	ret = data->init(enumerator);
	if (ret)
		return ret;

	if (data->isRawCamera())
		numCamerasRaw_++;
	else
		numCamerasYuv_++;

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
 * streaming. Thus, a default graph configuration is necessary for each camera
 * of the set before streaming operation is started on another camera. This is
 * done when the frontend media device is locked.
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
		const std::map<Size, std::vector<unsigned int>> &
			sizeToCodes = data->formatsSizeToCodes();
		ASSERT(sizeToCodes.size() == 1);
		sensorFormat.size = sizeToCodes.begin()->first;
		const std::vector<unsigned int> &codes = sizeToCodes.begin()->second;
		ASSERT(codes.size() == 1);
		sensorFormat.code = codes.back();

		ASSERT(data->defaultOrientation().has_value());
		Orientation orientation = data->defaultOrientation().value();
		Transform transform = data->sensor()->computeTransform(&orientation);
		ret = data->configureFrontEndFormat(sensorFormat, transform);

		if (ret)
			return ret;
	}

	return ret;
}

/**
 * \brief Load the pipeline configuration
 *
 * Load the pipeline configuration that consists in:
 *  - The parameters configured in the pipeline handler configuration file
 *  - The pipeline graphs that are dynamically discovered from the media device
 *
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
	if (isRawCamera())
		return configureRaw(c);
	else
		return configureYuv(c);
}

int NxpNeoCameraData::configureRaw(CameraConfiguration *c)
{
	NxpNeoCameraConfiguration *config =
		static_cast<NxpNeoCameraConfiguration *>(c);
	int ret = 0;

	LOG(NxpNeoPipe, Debug) << "Configure " << cameraName();

	/*
	 * Camera front-end graph reconfiguration is only applicable to the
	 * single camera case. For multi-camera case, front-end was statically
	 * configured at camera acquisition time.
	 * Only the ISI pipes at the very end of the front-end can be
	 * configured without multi-camera consideration. The pipe subdevice
	 * format comes from the front-end configuration.
	 */
	if (!multiCamera()) {
		V4L2SubdeviceFormat sensorFormat = config->sensorFormat();
		ret = configureFrontEndFormat(sensorFormat,
					      config->combinedTransform());
		if (ret)
			return ret;
	}
	std::map<StreamType, V4L2DeviceFormat> pipesDevFormats;
	for (auto [stream, pipe] : pipes_) {
		V4L2SubdeviceFormat &subdevFormat = pipesSubDevFormats_[stream];
		V4L2DeviceFormat &deviceFormat = pipesDevFormats[stream];
		deviceFormat = {};
		ret |= pipe->configure(subdevFormat, deviceFormat);
	}
	if (ret)
		return ret;

	/* ISP configuration. */
	V4L2DeviceFormat devFormatFrame = {};
	V4L2DeviceFormat devFormatIr = {};

	V4L2DeviceFormat &devFormatInput0 =
		pipesDevFormats[StreamTypeImage0];
	V4L2DeviceFormat devFormatInput1None = {};
	V4L2DeviceFormat &devFormatInput1 =
		mode_ == ModeTypeHdrMerge
			? pipesDevFormats[StreamTypeImage1]
			: devFormatInput1None;

	rawStreamOnly_ = ((config->size() == 1) &&
			  ((*config)[0].stream() == &streamRaw_));

	if (!rawStreamOnly_) {
		for (unsigned int i = 0; i < config->size(); ++i) {
			StreamConfiguration &cfg = (*config)[i];
			Stream *stream = cfg.stream();

			if (stream == &streamRaw_)
				continue;

			V4L2DeviceFormat *deviceFormat;
			V4L2VideoDevice *videoDevice;
			if (stream == &streamFrame_) {
				deviceFormat = &devFormatFrame;
				videoDevice = neo_->frame_.get();
			} else {
				deviceFormat = &devFormatIr;
				videoDevice = neo_->ir_.get();
			}

			V4L2PixelFormat pixelFormat =
				videoDevice->toV4L2PixelFormat(cfg.pixelFormat);
			deviceFormat->fourcc = pixelFormat;

			deviceFormat->size = cfg.size;

			/*
			 * Use libcamera sYCC colorspace definition that maps
			 * to a v4l2 sRGB colorspace equivalent.
			 */
			std::optional<ColorSpace> colorSpace = cfg.colorSpace;
			if (colorSpace && colorSpace.value() == ColorSpace::Srgb)
				colorSpace = ColorSpace::Sycc;
			deviceFormat->colorSpace = colorSpace;
		}
	} else {
		/*
		 * ISP driver requires at least one of the output video nodes to
		 * be enabled. During raw-stream only mode of operation, there
		 * is no buffer provided by the application for the outputs.
		 * Thus, configure the frame output with an arbitrary format.
		 * Internal buffers will be allocated and provided to the ISP.
		 */
		devFormatFrame.size = devFormatInput0.size;
		adjustTopLinesSize(&devFormatFrame.size);
		devFormatFrame.fourcc = V4L2PixelFormat(V4L2_PIX_FMT_NV12);
		devFormatFrame.colorSpace = ColorSpace::Sycc;
	}

	NeoDevice::PipeConfig pipeConfig = {};
	pipeConfig.topLines = embeddedTopLines_;
	ret = neo_->configure(pipeConfig,
			      &devFormatInput0, &devFormatInput1,
			      &devFormatFrame, &devFormatIr);
	if (ret)
		return ret;

	/*
	 * Raw stream is mapped alternately on image0 and image1 for cases
	 *  - RGBIr dual context to capture both contexts
	 *  - HDR Merge to capture long and short images if they share the same
	 *    format
	 */
	if (mode_ == ModeTypeRgbIrDual)
		alternatedRawStream_ = true;
	else if (mode_ == ModeTypeHdrMerge)
		alternatedRawStream_ =
			(devFormatInput0.fourcc == devFormatInput1.fourcc &&
			 devFormatInput0.size == devFormatInput1.size);
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

	ColorSpace colorSpace = ColorSpace::Raw;
	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = (*config)[i];
		Stream *stream = cfg.stream();

		if (stream == &streamFrame_) {
			streamConfig[ipa::nxpneo::IPAStreamTypeFrame] = IPAStream(cfg.pixelFormat,
										  cfg.size);
			/*
			 * Take color space from the frame if it exists,
			 * or default to raw (IR only stream case).
			 */
			colorSpace = cfg.colorSpace.value_or(ColorSpace::Raw);
		} else if (stream == &streamIr_) {
			streamConfig[ipa::nxpneo::IPAStreamTypeIr] = IPAStream(cfg.pixelFormat,
									       cfg.size);
		}
	}

	ipa::nxpneo::IPAConfigInfo configInfo;
	std::vector<uint32_t> ids = utils::map_keys(sensor_->controls().idmap());
	configInfo.sensorControls = sensor_->controls();
	configInfo.sensorControlList = sensor_->getControls(ids);
	configInfo.sensorInfo = sensorInfo;

	ipa::nxpneo::IPAColorSpace IPAcolorSpace = ipa::nxpneo::IPAColorSpace(
		static_cast<ipa::nxpneo::IPAPrimaries>(colorSpace.primaries),
		static_cast<ipa::nxpneo::IPATransferFunction>(colorSpace.transferFunction),
		static_cast<ipa::nxpneo::IPAYcbcrEncoding>(colorSpace.ycbcrEncoding),
		static_cast<ipa::nxpneo::IPARange>(colorSpace.range));

	ret = ipa_->configure(configInfo, streamConfig,
			      static_cast<ipa::nxpneo::IPAModeType>(mode_),
			      IPAcolorSpace, &ipaControls_);
	if (ret) {
		LOG(NxpNeoPipe, Error) << "Failed to configure IPA: "
				       << strerror(-ret);
		return ret;
	}

	ret = updateControls();
	return ret;
}

int NxpNeoCameraData::configureYuv(CameraConfiguration *c)
{
	NxpNeoCameraConfiguration *config =
		static_cast<NxpNeoCameraConfiguration *>(c);
	int ret;
	LOG(NxpNeoPipe, Debug) << "Configure " << cameraName();

	/*
	 * Camera front-end graph reconfiguration is only applicable to the
	 * single camera case. For multi-camera case, front-end was statically
	 * configured at camera acquisition time.
	 * Only the ISI pipes at the very end of the front-end can be
	 * configured without multi-camera consideration. The pipe subdevice
	 * format comes from the front-end configuration.
	 */
	if (!multiCamera()) {
		V4L2SubdeviceFormat sensorFormat = config->sensorFormat();
		ret = configureFrontEndFormat(sensorFormat,
					      config->combinedTransform());
		if (ret)
			return ret;
	}
	/* \todo support multistream */
	ISIPipe *pipe = pipes_.at(StreamTypeImage0);
	StreamConfiguration &streamConfig = c->at(0);

	V4L2SubdeviceFormat &subdevFormat = pipesSubDevFormats_[StreamTypeImage0];
	V4L2DeviceFormat deviceFormat = {};
	V4L2VideoDevice *videoDevice = pipe->output_.get();
	deviceFormat.fourcc =
		videoDevice->toV4L2PixelFormat(streamConfig.pixelFormat);
	deviceFormat.size = streamConfig.size;
	deviceFormat.colorSpace = streamConfig.colorSpace;

	ret = pipe->configure(subdevFormat, deviceFormat);
	return ret;
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
		return pipes_[StreamTypeImage0]->exportBuffers(count, buffers);

	return -EINVAL;
}

int NxpNeoCameraData::start([[maybe_unused]] const ControlList *controls)
{
	int ret;

	LOG(NxpNeoPipe, Debug) << "Start " << cameraName();
	sequence_ = 0;
	const std::array<ContextType, 2> allContexts = { ContextTypeRgb, ContextTypeIr };

	/* Allocate buffers for internal pipeline usage. */
	ret = allocateBuffers();
	if (ret)
		return ret;

	if (isRawCamera()) {
		ret = ipa_->start();
		if (ret)
			goto error;

		for (const auto context : allContexts)
			delayedCtrls_[context]->reset();

		ret = neo_->start();
		if (ret)
			goto error;
	}

	/*
	 * Start the Neo and ISI video devices.
	 * ISI secondary streams are started first, then the primary stream
	 */
	for (auto [stream, pipe] : pipes_) {
		if (stream == StreamTypeImage0)
			continue;
		ret = pipes_[stream]->start();
		if (ret)
			goto error;
	}

	ret = pipes_[StreamTypeImage0]->start();
	if (ret)
		goto error;

	return 0;

error:
	pipes_[StreamTypeImage0]->stop();
	for (auto [stream, pipe] : pipes_) {
		if (stream == StreamTypeImage0)
			continue;
		pipes_[stream]->stop();
	}

	if (isRawCamera()) {
		neo_->stop();
		ipa_->stop();
	}

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

	ret = pipes_[StreamTypeImage0]->stop();
	for (auto [stream, pipe] : pipes_) {
		if (stream == StreamTypeImage0)
			continue;
		ret |= pipes_[stream]->stop();
	}

	if (isRawCamera()) {
		ipa_->stop();
		ret |= neo_->stop();
	}

	freeBuffers();

	if (ret)
		LOG(NxpNeoPipe, Warning) << "Failed to stop camera " << cameraName();
}

void NxpNeoCameraData::queuePendingRequests()
{
	NxpNeoFrames::Info *info;
	int ret = 0;

	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();

		info = frameInfos_.create(request);
		if (!info)
			break;

		for (const auto context : utils::map_keys(info->contexts)) {
			for (auto [stream, pipe] : pipes_) {
				V4L2VideoDevice *dev = pipe->output_.get();
				BufferType bufferType = streamToBufferType.at(stream);
				FrameBuffer *buffer =
					frameInfos_.buffer(info, context, bufferType, false);
				if (!buffer)
					continue;
				ret |= dev->queueBuffer(buffer);
			}
		}

		if (ret) {
			LOG(NxpNeoPipe, Error)
				<< "Failed to queue buffers, unbalanced queues";
			pipe()->cancelRequest(request);
			frameInfos_.destroy(info->id);
			pendingRequests_.pop();
			return;
		}

		if (isRawCamera())
			ipa_->queueRequest(info->id, request->controls());

		pendingRequests_.pop();
		processingRequests_.push(request);
	}

	return;
}

/**
 * \brief Initialize sensor, frontend, IPA and callbacks
 * \param[in] enumerator The media devices enumerator
 *
 * \return 0 on success or a negative error code otherwise
 */
int NxpNeoCameraData::init(DeviceEnumerator *enumerator)
{
	/* Detect compatible raw or RGB/YUV formats */
	isRawCamera_ = true;
	int ret = enumerateFormatsRaw();
	if (ret) {
		ret = enumerateFormatsYuv();
		isRawCamera_ = false;
	}

	if (ret) {
		LOG(NxpNeoPipe, Debug) << "No supported format for " << cameraName();
		return -EINVAL;
	}

	if (isRawCamera_) {
		DeviceMatch isp(NeoDevice::kDriverName());
		isp.add(NeoDevice::kSDevNeoEntityName());
		isp.add(NeoDevice::kVDevInput0EntityName());
		isp.add(NeoDevice::kVDevInput1EntityName());
		isp.add(NeoDevice::kVDevEntityParamsName());
		isp.add(NeoDevice::kVDevEntityFrameName());
		isp.add(NeoDevice::kVDevEntityIrName());
		isp.add(NeoDevice::kVDevEntityStatsName());

		MediaDevice *neoDevice =
			pipe()->acquireMediaDevice(enumerator, isp);
		if (!neoDevice)
			return -EINVAL;

		/*
		 * The ISP instance number corresponds to the number of raw
		 * cameras already detected.
		 */
		neo_ = std::make_unique<NeoDevice>(pipe()->numCamerasRaw());
		ret = neo_->init(neoDevice);
		if (ret)
			return ret;

		ret = loadIPA();
		if (ret)
			return ret;

		updateControls();

		if (!cameraInfo_->hasStream(StreamTypeImage1))
			mode_ = sensorIsRgbIr() ? ModeTypeRgbIr : ModeTypeStandard;
		else
			mode_ = sensorIsRgbIr() ? ModeTypeRgbIrDual : ModeTypeHdrMerge;
	}

	/* Initialize the camera properties. */
	properties_ = sensor_->properties();

	/*
	 * A default orientation may be defined for a camera in the pipeline
	 * config file. For multi-camera case, when not defined in the config
	 * file, the camera mounting orientation is selected as default
	 * orientation to be used for the camera preconfiguration.
	 */
	std::optional<Orientation> configOrientation =
		cameraInfo_->cameraProperties().orientation;
	if (configOrientation.has_value()) {
		Orientation tryOrientation = configOrientation.value();
		sensor_->computeTransform(&tryOrientation);
		if (tryOrientation != configOrientation.value()) {
			LOG(NxpNeoPipe, Warning)
				<< "Configured orientation " << configOrientation.value()
				<< " not supported by sensor";
			return -EINVAL;
		}
		defaultOrientation_ = tryOrientation;
	}
	if (multiCamera() && !defaultOrientation_.has_value()) {
		const auto &rotation = properties_.get(properties::Rotation);
		Orientation mountingOrientation =
			orientationFromRotation(rotation.value_or(0));
		defaultOrientation_ = mountingOrientation;
	}

	/*
	 * Connect video devices' 'bufferReady' signals to their
	 * slot to implement the image processing pipeline.
	 *
	 * Frames produced by the ISI unit are passed to the
	 * associated NEO inputs where they get processed and
	 * returned through the NEO main and IR outputs.
	 */

	if (!cameraInfo_->stream(StreamTypeImage0)) {
		LOG(NxpNeoPipe, Error)
			<< "Mandatory stream image0 is missing for " << cameraName();
		return -ENODEV;
	}

	const std::map<StreamType, void (NxpNeoCameraData::*)(FrameBuffer *)> pipeReadyFuncs{
		{ StreamTypeImage0, &NxpNeoCameraData::isiImage0BufferReady },
		{ StreamTypeImage1, &NxpNeoCameraData::isiImage1BufferReady },
		{ StreamTypeEData, &NxpNeoCameraData::isiEmbeddedDataBufferReady },
	};

	ISIDevice *isi = pipe()->isiDevice();
	for (StreamType stream : kStreamTypes) {
		const CameraMediaStream *cameraMediaStream = cameraInfo_->stream(stream);
		if (!cameraMediaStream)
			continue;
		unsigned int pipeIndex = cameraMediaStream->pipe();
		pipes_[stream] = isi->getPipeByIndex(pipeIndex);

		auto it = pipeReadyFuncs.find(stream);
		ASSERT(it != pipeReadyFuncs.end());
		pipes_[stream]->bufferReady().connect(this, it->second);
	}

	if (neo_) {
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
	}

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
 * \brief Configure the front-end media controller device for a camera
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
		bool enable = pipes_.count(StreamTypeImage1);
		ret = sensor->setAuxiliaryEnabled(enable);
		if (ret && enable) {
			LOG(NxpNeoPipe, Warning)
				<< "Auxiliary stream configuration failed"
				<< " [" << enable << "]";
			return ret;
		}
	}

	if (sensor->embeddedDataStream().has_value()) {
		bool enable = pipes_.count(StreamTypeEData);
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
	pipesSubDevFormats_.clear();
	for (auto [stream, pipe] : pipes_) {
		const CameraMediaStream *cameraInfoStream = cameraInfo_->stream(stream);
		ASSERT(cameraInfoStream);
		const std::vector<CameraMediaStream::StreamLink> &streamLinks =
			cameraInfoStream->streamLinks();

		V4L2SubdeviceFormat &subdevFormat = pipesSubDevFormats_[stream];
		if (stream == StreamTypeImage0) {
			subdevFormat = sensorFormat;
		} else if (stream == StreamTypeImage1) {
			subdevFormat = sensor->auxiliaryFormat();
		} else if (stream == StreamTypeEData) {
			subdevFormat = sensor->embeddedDataFormat();
		} else {
			LOG(NxpNeoPipe, Error) << "Invalid stream " << stream;
			continue;
		};

		ret = configureFrontEndStream(streamLinks, subdevFormat);
		if (ret)
			return ret;
	}

	return ret;
}

/**
 * \brief Update the camera controls
 *
 * Compute the camera controls by calculating controls which the pipeline
 * is responsible for and merge them with the controls computed by the IPA.
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
	ipa_->paramsComputed.connect(this, &NxpNeoCameraData::ipaParamsComputed);
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

	uint32_t hwRevision = neo_->media()->hwRevision();
	ipa::nxpneo::SensorConfig sensorConfig;
	const MediaEntity *entity = sensor->entity();
	std::vector<uint32_t> ids = utils::map_keys(sensor_->controls().idmap());
	ipa::nxpneo::InitParams initParams = { hwRevision, neo_->apiVersion(),
					       entity->name(), sensorInfo,
					       sensor->controls(),
					       sensor_->getControls(ids) };
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
	std::unordered_map<uint32_t, DelayedControls::ControlParams> delayParams;
	for (const auto &[k, v] : ipaDelayParams) {
		DelayedControls::ControlParams controlParams = { v.delay, v.priorityWrite };
		delayParams.emplace(k, controlParams);
	}
	if (!delayParams.size()) {
		const CameraSensorProperties::SensorDelays &delays =
			sensor->sensorDelays();
		delayParams = {
			{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
			{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
		};
	}

	V4L2Subdevice *device = sensor->device();
	const std::array<ContextType, 2> allContexts = { ContextTypeRgb, ContextTypeIr };
	for (const auto &context : allContexts) {
		delayedCtrls_.emplace(
			context, std::make_unique<DelayedControls>(device, delayParams));
	}

	return 0;
}

/**
 * \brief Allocate buffers from ISI and ISP
 *
 * Internal buffers are allocated for ISI active channels and ISP params and
 * statistics buffers. Those buffers are aggregated into the list of shared
 * buffers between the pipeline and the IPA.
 * Those buffers are registered into separates pools that will be accessed by
 * the NxpNeoFrames objects to pick buffers for the different pipeline stages.
 *
 * \return 0 in case of success or a negative error code
 */
int NxpNeoCameraData::allocateBuffers()
{
	int ret;
	if (isRawCamera())
		ret = allocateBuffersRaw();
	else
		ret = allocateBuffersYuv();

	if (ret)
		return ret;

	frameInfos_.bufferAvailable.connect(
		this, &NxpNeoCameraData::queuePendingRequests);
	return 0;
}

int NxpNeoCameraData::allocateBuffersRaw()
{
	unsigned int bufferCount;
	bufferCount = std::max({
		streamFrame_.configuration().bufferCount,
		streamIr_.configuration().bufferCount,
		streamRaw_.configuration().bufferCount,
	});

	unsigned int ipaBufferId = 1;
	auto registerPoolBuffers =
		[&](std::vector<std::unique_ptr<FrameBuffer>> *_pool, BufferType _bufferType) {
			for (const std::unique_ptr<FrameBuffer> &buffer : *_pool) {
				buffer->setCookie(ipaBufferId++);
				ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
				availableBuffersMap_[_bufferType].push(buffer.get());
			}
		};

	/*
	 * RGBIr dual context switch has some peculiarities related to frames
	 * buffer allocation:
	 *  - Some buffers are instantiated per context so their pool size has
	 *    to be sized accordingly. That is the case for embedded data, ISP
	 *    params and stats buffers.
	 *  - Throw-away buffers for ISP frame and infrared outputs have to be
	 *    allocated. They are used as temporary storage for the ISP
	 *    decoded buffers not delivered to the application.
	 * Raw-only operation also has the specificity that ISP capture buffer
	 * for frame output is not provided in the libcamera::Request so it has
	 * to be allocated internally. For RGBIr raw-only operation, a frame
	 * buffer for the frame output is to be provided for both contexts.
	 * \todo replace full size output buffers by short dummy buffers when
	 * that is supported by the ISP driver.
	 */
	int ret = 0;
	unsigned int _contextCount = contextCount();

	const std::map<BufferType, std::pair<std::vector<std::unique_ptr<FrameBuffer>> *,
					     V4L2VideoDevice *>>
		ispOutputPools = {
			{ BufferTypeFrame, { &frameBuffersPool_, neo_->frame_.get() } },
			{ BufferTypeIr, { &irBuffersPool_, neo_->ir_.get() } },
		};
	if (rawStreamOnly_) {
		unsigned int count = bufferCount * _contextCount;
		const auto &[pool, device] = ispOutputPools.at(BufferTypeFrame);
		int res = device->exportBuffers(count, pool);
		ret |= res == static_cast<int>(count) ? 0 : -ENOMEM;
		registerPoolBuffers(pool, BufferTypeFrame);
	} else if (mode_ == ModeTypeRgbIrDual) {
		for (const auto &[bufferType, pair] : ispOutputPools) {
			std::vector<std::unique_ptr<FrameBuffer>> *pool = pair.first;
			V4L2VideoDevice *device = pair.second;

			int res = device->exportBuffers(bufferCount, pool);
			ret |= res == static_cast<int>(bufferCount) ? 0 : -ENOMEM;

			registerPoolBuffers(pool, bufferType);
		}
	}

	/* Allocate and map stats and params buffers. */
	const std::map<BufferType, std::vector<std::unique_ptr<FrameBuffer>> *>
		ispMetaPools = {
			{ BufferTypeParams, &neo_->paramsBuffers_ },
			{ BufferTypeStats, &neo_->statsBuffers_ },
		};
	ret |= neo_->allocateBuffers(bufferCount * _contextCount);

	for (const auto [bufferType, pool] : ispMetaPools)
		registerPoolBuffers(pool, bufferType);

	/* ISI pipe buffers for images and edata streams */
	for (const auto [stream, pipe] : pipes_) {
		unsigned int count = stream == StreamTypeEData
					     ? bufferCount * _contextCount
					     : bufferCount;
		ret |= pipe->allocateBuffers(count);

		BufferType bufferType = streamToBufferType.at(stream);
		registerPoolBuffers(&pipe->buffers(), bufferType);
	}

	if (ret) {
		freeBuffers();
		return ret;
	}

	ipa_->mapBuffers(ipaBuffers_);

	return 0;
}

int NxpNeoCameraData::allocateBuffersYuv()
{
	unsigned int bufferCount = streamRaw_.configuration().bufferCount;
	int ret = 0;

	/* Application exports buffers imported by the ISI pipe devices. */
	for (const auto [stream, pipe] : pipes_)
		ret |= pipe->importBuffers(bufferCount);

	return ret;
}

/**
 * \brief Deallocate buffers from ISI and ISP
 * \return 0 in case of success or a negative error code
 */

int NxpNeoCameraData::freeBuffers()
{
	frameInfos_.bufferAvailable.disconnect(
		this, &NxpNeoCameraData::queuePendingRequests);
	frameInfos_.clear();

	int ret;
	if (isRawCamera())
		ret = freeBuffersRaw();
	else
		ret = freeBuffersYuv();

	return ret;
}

int NxpNeoCameraData::freeBuffersRaw()
{
	std::vector<unsigned int> ids;
	for (IPABuffer &ipabuf : ipaBuffers_)
		ids.push_back(ipabuf.id);

	ipa_->unmapBuffers(ids);
	ipaBuffers_.clear();

	for (auto [bufferType, availableBuffers] : availableBuffersMap_) {
		while (!availableBuffers.empty())
			availableBuffers.pop();
	}
	availableBuffersMap_.clear();

	neo_->freeBuffers();

	for (auto [stream, pipe] : pipes_)
		pipe->freeBuffers();

	frameBuffersPool_.clear();
	irBuffersPool_.clear();

	return 0;
}

int NxpNeoCameraData::freeBuffersYuv()
{
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
	for (StreamType stream : kStreamTypes) {
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

/**
 * \brief Enumerate the compatible sizes and mbus-codes for the raw sensor
 *
 * Enumerate the sizes and associated mbus-codes provided by the sensor modes
 * compatible with the pipeline. Two maps are stored in the class for later
 * usage:
 * - All sizes associated to a given mbus code
 *   This map can be later accessed via getter formatsCodeToSizes()
 * - All mbus codes associated to a given size
 *   This map can be later accessed via getter formatsSizeToCodes()
 * Mbus codes selected have to be Bayer formats supported by the frontend and
 * the ISP. Also, size widths selected must be within ISP supported range.
 * In case of multi-camera condition, the set of available formats is limited
 * to a single default value that will be used for the graph preconfiguration.
 *
 * \return 0 in case of success or a negative error code
 */
int NxpNeoCameraData::enumerateFormatsRaw()
{
	std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		formatsSizeToCodes_;
	std::map<unsigned int, std::vector<Size>> &codeToSizes =
		formatsCodeToSizes_;

	const std::vector<unsigned int> &mbusCodes = sensor_->mbusCodes();
	const std::vector<unsigned int> &bayerCodes = ISIPipe::bayerMbusCodes();
	const std::vector<V4L2PixelFormat> &neoPixelFormats =
		NeoDevice::input0Formats();

	/*  Camera formats filtering may be defined in the config file */
	std::optional<unsigned int> bppFilter =
		cameraInfo_->cameraProperties().formatBpp;
	std::optional<Size> sizeFilter =
		cameraInfo_->cameraProperties().formatSize;

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

			if (sizeFilter && size != sizeFilter.value())
				continue;

			sizeToCodes[size].push_back(code);
			codeToSizes[code].push_back(size);
		}
	}

	/* Make sure there is at least one compatible size and code */
	if (!sizeToCodes.size() || !sizeToCodes.begin()->second.size()) {
		LOG(NxpNeoPipe, Debug)
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
 * \brief Enumerate the compatible sizes and mbus-codes for the smart sensor
 *
 * Enumerate the sizes and associated mbus-codes provided by the sensor modes
 * compatible with the pipeline. Two maps are stored in the class for later
 * usage:
 * - All sizes associated to a given mbus code
 *   This map can be later accessed via getter formatsCodeToSizes()
 * - All mbus codes associated to a given size
 *   This map can be later accessed via getter formatsSizeToCodes()
 * Mbus codes selected have to be formats supported by the frontend. Also, size
 * widths selected must be within front-end supported range.
 * In case of multi-camera condition, the set of available formats is limited
 * to a single default value that will be used for the graph preconfiguration.
 *
 * \return 0 in case of success or a negative error code
 */
int NxpNeoCameraData::enumerateFormatsYuv()
{
	std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		formatsSizeToCodes_;
	std::map<unsigned int, std::vector<Size>> &codeToSizes =
		formatsCodeToSizes_;

	const std::vector<unsigned int> &mbusCodes = sensor_->mbusCodes();
	const std::vector<uint32_t> &sinkCodes = ISIPipe::sinkMbusCodesProcessed();

	/*  Camera formats filtering may be defined in the config file */
	std::optional<unsigned int> bppFilter =
		cameraInfo_->cameraProperties().formatBpp;
	std::optional<Size> sizeFilter =
		cameraInfo_->cameraProperties().formatSize;

	for (unsigned int code : mbusCodes) {
		if (std::find(sinkCodes.begin(), sinkCodes.end(), code) == sinkCodes.end())
			continue;

		const MediaBusFormatInfo &info = MediaBusFormatInfo::info(code);
		if (bppFilter && info.bitsPerPixel != bppFilter.value())
			continue;

		std::vector<Size> sizes = sensor_->sizes(code);
		/*
		 * This is a workaround for external ISP that advertise a single
		 * (min, max) size range corresponding to its full rescaling
		 * capability. The CameraSensor::size() function reports only
		 * the max value of the range that may fall out of ISI support.
		 * For that case, make sure that at least the sensor native
		 * resolution is considered.
		 */
		Size resolution = sensor_->resolution();
		if (sizes.size() == 1 &&
		    std::find(sizes.begin(), sizes.end(), resolution) == sizes.end()) {
			sizes.push_back(std::move(resolution));
		}

		for (const Size &size : sizes) {
			if (size.width > ISIDevice::kChainedWidthMax)
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

	/*
	 * For multi-camera, default configuration is set arbitrarily to the
	 * the highest size/bitdepth.
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
	oss << "Yuv formats size [ mbuscodes ] ";
	for (const auto &[size, codes] : sizeToCodes) {
		oss << size.toString() << " [ ";
		for (const unsigned int code : codes)
			oss << utils::hex(code) << " ";
		oss << size.toString() << "] ";
	}
	LOG(NxpNeoPipe, Debug) << oss.str();

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

	if (!frameInfos_.isFrameComplete(info))
		return;

	pipe()->completeRequest(request);

	clearRequest(info);
}

/* -----------------------------------------------------------------------------
 * Buffer Ready slots
 */

/**
 * \brief Handle buffers availability of the ISI pipes buffers
 * \param[in] info The frame info associated to ongoing request
 * \param[in] context The frame context relevant to the buffer received
 *
 * In case all front-end buffers associated to the request have been received,
 * the IPA can be invoked to retrieve ISP parameters.
 */
void NxpNeoCameraData::isiInputBufferReady(NxpNeoFrames::Info *info, ContextType context)
{
	const std::vector<BufferType>
		inputBufferTypes = { BufferTypeImage0, BufferTypeImage1, BufferTypeEData };
	if (frameInfos_.isBufferPending(info, context, inputBufferTypes))
		return;

	std::map<uint32_t, uint32_t> bufferIds;

	FrameBuffer *image0Buffer =
		frameInfos_.buffer(info, context, BufferTypeImage0, false);
	if (image0Buffer)
		bufferIds[ipa::nxpneo::IPABufferTypeImage0] = image0Buffer->cookie();

	FrameBuffer *image1Buffer =
		frameInfos_.buffer(info, context, BufferTypeImage1, false);
	if (image1Buffer)
		bufferIds[ipa::nxpneo::IPABufferTypeImage1] = image1Buffer->cookie();

	FrameBuffer *edataBuffer =
		frameInfos_.buffer(info, context, BufferTypeEData, false);
	if (edataBuffer)
		bufferIds[ipa::nxpneo::IPABufferTypeEData] = edataBuffer->cookie();

	FrameBuffer *paramsBuffer =
		frameInfos_.buffer(info, context, BufferTypeParams, true);
	ASSERT(paramsBuffer);
	bufferIds[ipa::nxpneo::IPABufferTypeParams] = paramsBuffer->cookie();

	ipa_->computeParams(info->id,
			    static_cast<ipa::nxpneo::IPAContextType>(context),
			    bufferIds);
}

/**
 * \brief Handle IMAGE0 buffers availability at the ISI output
 * \param[in] buffer The completed buffer
 */
void NxpNeoCameraData::isiImage0BufferReady(FrameBuffer *buffer)
{
	auto [info, context] = frameInfos_.find(buffer);
	if (!info)
		return;
	frameInfos_.completeBuffer(info, context, buffer);

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	unsigned int seq = buffer->metadata().sequence;
	if (seq != sequence_)
		LOG(NxpNeoPipe, Warning)
			<< "Image0 frame loss! expected " << sequence_
			<< " received " << seq;
	sequence_ = seq + 1;

	/* Record the sensor's timestamp in the request metadata. */
	Request *request = info->request;
	request->metadata().set(controls::SensorTimestamp,
				buffer->metadata().timestamp);

	if (request->findBuffer(&streamRaw_) == buffer)
		pipe()->completeBuffer(request, buffer);

	if (isRawCamera()) {
		isiInputBufferReady(info, context);

		delayedCtrls_[ContextTypeRgb]->applyControls(info->id);
	} else {
		tryCompleteRequest(info);
	}
}

/**
 * \brief Handle IMAGE1 buffers availability at the ISI output
 * \param[in] buffer The completed buffer
 */
void NxpNeoCameraData::isiImage1BufferReady(FrameBuffer *buffer)
{
	auto [info, context] = frameInfos_.find(buffer);
	if (!info)
		return;
	frameInfos_.completeBuffer(info, context, buffer);

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	Request *request = info->request;
	(void)request;

	if (request->findBuffer(&streamRaw_) == buffer)
		pipe()->completeBuffer(request, buffer);

	if (mode_ == ModeTypeHdrMerge &&
	    frameInfos_.isBufferPending(info, context, { BufferTypeImage0 }))
		LOG(NxpNeoPipe, Warning) << "Out of order input frame receipt";

	isiInputBufferReady(info, context);

	if (mode_ == ModeTypeRgbIrDual)
		delayedCtrls_[ContextTypeIr]->applyControls(info->id);
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
	auto [info, context] = frameInfos_.find(buffer);
	if (!info)
		return;
	frameInfos_.completeBuffer(info, context, buffer);

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	isiInputBufferReady(info, context);
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
	auto [info, context] = frameInfos_.find(buffer);
	if (!info)
		return;
	frameInfos_.completeBuffer(info, context, buffer);

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	Request *request = info->request;
	auto streamBuffers = request->buffers();
	auto it = std::find_if(streamBuffers.begin(), streamBuffers.end(),
			       [buffer](auto &kv) {
				       return kv.second == buffer;
			       });
	if (it != streamBuffers.end())
		pipe()->completeBuffer(request, buffer);

	tryCompleteRequest(info);
}

/**
 * \brief Handle params buffers consumed by ISP
 * \param[in] buffer The consumed buffer
 */
void NxpNeoCameraData::neoParamsBufferReady(FrameBuffer *buffer)
{
	auto [info, context] = frameInfos_.find(buffer);
	if (!info)
		return;
	frameInfos_.completeBuffer(info, context, buffer);

	auto it = info->contexts.find(context);
	ASSERT(it != info->contexts.end());
	NxpNeoFrames::InfoContext &infoContext = it->second;
	if (infoContext.paramDequeued)
		LOG(NxpNeoPipe, Error) << "Params buffer already dequeued ";
	infoContext.paramDequeued = true;

	tryCompleteRequest(info);
}

/**
 * \brief Handle stats buffers produced by ISP
 * \param[in] buffer The produced buffer
 */
void NxpNeoCameraData::neoStatsBufferReady(FrameBuffer *buffer)
{
	auto [info, context] = frameInfos_.find(buffer);
	if (!info)
		return;
	frameInfos_.completeBuffer(info, context, buffer);

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	std::map<uint32_t, uint32_t> bufferIds = {
		{ ipa::nxpneo::IPABufferTypeStats, buffer->cookie() },
	};

	unsigned int sequence = info->id;
	ipa_->processStats(sequence,
			   static_cast<ipa::nxpneo::IPAContextType>(context),
			   bufferIds,
			   delayedCtrls_[context]->get(sequence));

	tryCompleteRequest(info);
}

void NxpNeoCameraData::ipaParamsComputed(unsigned int id,
					 ipa::nxpneo::IPAContextType context,
					 unsigned int bytesused)
{
	NxpNeoFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

	ContextType _context = static_cast<ContextType>(context);

	int ret = 0;
	/* Queue buffers ISP output buffers */
	FrameBuffer *frameBuffer =
		frameInfos_.buffer(info, _context, BufferTypeFrame, false);
	if (frameBuffer)
		ret |= neo_->frame_->queueBuffer(frameBuffer);
	FrameBuffer *irBuffer =
		frameInfos_.buffer(info, _context, BufferTypeIr, false);
	if (irBuffer)
		ret |= neo_->ir_->queueBuffer(irBuffer);

	/* Queue ISP params and stats buffers */
	FrameBuffer *paramsBuffer =
		frameInfos_.buffer(info, _context, BufferTypeParams, true);
	if (paramsBuffer) {
		paramsBuffer->_d()->metadata().planes()[0].bytesused = bytesused;
		ret |= neo_->params_->queueBuffer(paramsBuffer);
	}
	FrameBuffer *statsBuffer =
		frameInfos_.buffer(info, _context, BufferTypeStats, true);
	if (statsBuffer)
		ret |= neo_->stats_->queueBuffer(statsBuffer);

	/* Queue ISP input buffers */
	FrameBuffer *image0Buffer =
		frameInfos_.buffer(info, _context, BufferTypeImage0, false);
	FrameBuffer *image1Buffer =
		frameInfos_.buffer(info, _context, BufferTypeImage1, false);
	if (image0Buffer)
		ret |= neo_->input0_->queueBuffer(image0Buffer);
	if (image1Buffer) {
		if (mode_ == ModeTypeHdrMerge)
			ret |= neo_->input1_->queueBuffer(image1Buffer);
		else if (mode_ == ModeTypeRgbIrDual)
			ret |= neo_->input0_->queueBuffer(image1Buffer);
		else
			LOG(NxpNeoPipe, Error) << "Unexpected image1 in mode " << mode_;
	}

	if (ret)
		LOG(NxpNeoPipe, Error) << "Failed to queue ISP buffers";
}

void NxpNeoCameraData::ipaMetadataReady(unsigned int id,
					ipa::nxpneo::IPAContextType context,
					const ControlList &metadata)
{
	NxpNeoFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

	Request *request = info->request;
	request->metadata().merge(metadata);

	auto it = info->contexts.find(static_cast<ContextType>(context));
	if (it == info->contexts.end()) {
		LOG(NxpNeoPipe, Error) << "Invalid context " << context;
		return;
	}
	NxpNeoFrames::InfoContext &infoContext = it->second;
	if (infoContext.metadataProcessed)
		LOG(NxpNeoPipe, Error) << "Metadata already processed";
	infoContext.metadataProcessed = true;
	tryCompleteRequest(info);
}

void NxpNeoCameraData::ipaSetSensorControls([[maybe_unused]] unsigned int id,
					    ipa::nxpneo::IPAContextType context,
					    const ControlList &sensorControls)
{
	delayedCtrls_[static_cast<ContextType>(context)]->push(sensorControls);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerNxpNeo, "nxp/neo")

} /* namespace libcamera */
