/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Pipeline handler for Neo ISP
 *
 * Based on Pipeline handler for Intel IPU3
 *     src/libcamera/pipeline/ipu3/ipu3.cpp
 * Copyright (C) 2019, Google Inc.
 */

#include <algorithm>
#include <iomanip>
#include <memory>
#include <queue>
#include <regex>
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

#include "front_end.h"
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

/**
 * \enum BufferType
 * \brief Types of buffers used in the Neo pipeline
 *
 * This enumeration defines the different buffer types that are tracked and
 * managed throughout the pipeline processing stages.
 *
 * \var BufferType::Image0
 * \brief Primary image input buffer from the front-end (or raw stream buffer)
 *
 * \var BufferType::Image1
 * \brief Secondary image input buffer from the front-end (used in HDR merge or
 * RGBIr dual mode)
 *
 * \var BufferType::EData
 * \brief Embedded data buffer containing sensor metadata
 *
 * \var BufferType::Params
 * \brief ISP parameters buffer exchanged between pipeline handler and IPA
 *
 * \var BufferType::Stats
 * \brief ISP statistics buffer exchanged between pipeline handler and IPA
 *
 * \var BufferType::Frame
 * \brief ISP output buffer for the main processed frame stream
 *
 * \var BufferType::Ir
 * \brief ISP output buffer for the infrared stream (RGBIr sensors)
 */
enum class BufferType {
	Image0,
	Image1,
	EData,
	Params,
	Stats,
	Frame,
	Ir,
};

/**
 * \enum CameraContext
 * \brief Camera context types for the Neo pipeline
 *
 * Some sensors support multiple register banks (contexts) that can be
 * configured independently and applied in sequence to produce different
 * types of image outputs. This enumeration defines the context types used in
 * the pipeline to distinguish between RGB and infrared processing paths.
 *
 * \var CameraContext::Rgb
 * \brief RGB context for standard color image processing
 *
 * \var CameraContext::Ir
 * \brief Infrared context for IR image processing (RGBIr sensors)
 */
enum class CameraContext {
	Rgb,
	Ir,
};

/**
 * \enum PipelineMode
 * \brief Operating modes for the Neo ISP pipeline
 *
 * This enumeration defines the different operating modes supported by the
 * Neo pipeline, which determine how sensor data is processed and which streams
 * are available.
 *
 * \var PipelineMode::Standard
 * \brief Standard single-frame processing mode
 *
 * Standard mode processes a single raw image from the sensor through the ISP to
 * produce RGB/YUV output streams.
 *
 * \var PipelineMode::HdrMerge
 * \brief High Dynamic Range merge mode
 *
 * HDR merge mode combines multiple exposures (long and short frames) from the
 * sensor to produce a single high dynamic range output image.
 *
 * \var PipelineMode::RgbIr
 * \brief RGB and Infrared processing mode
 *
 * RGBIr mode processes data from sensors with both RGB and infrared pixels,
 * producing both RGB and IR output streams from a single context.
 *
 * \var PipelineMode::RgbIrDual
 * \brief Dual-context RGB and Infrared mode
 *
 * RGBIr dual mode uses multiple sensor contexts to separately capture and
 * process RGB and infrared optimized pixels data, producing independent RGB and
 * IR output streams.
 */
enum class PipelineMode {
	Standard,
	HdrMerge,
	RgbIr,
	RgbIrDual,
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
 * maintaining a NxpNeoFrames::Info instance for each Request in progress until
 * its completion.
 */

/**
 * \var NxpNeoFrames::data_
 * \brief Pointer to the NxpNeoCameraData instance
 *
 * \var NxpNeoFrames::frameInfo_
 * \brief Map of active frame Info instances indexed by libcamera::Request id
 */

/**
 * \class NxpNeoFrames::InfoContext
 * \brief Frame context descriptor
 *
 * Some sensors have specific modes of operation where they maintain multiple
 * banks (or contexts) of internal registers values that will be applied
 * in sequence in order to produce successive raw images.
 * Those multiple images are used by the pipeline handler to produce different
 * stream buffers for the application, that all belong to the same
 * libcamera::Request.
 * A NxpNeoFrames::InfoContext instance is associated to each camera context.
 */

/**
 * \var NxpNeoFrames::InfoContext::paramDequeued_
 * \brief Indicates that the params buffer has been consumed by the ISP
 *
 * \var NxpNeoFrames::InfoContext::metadataProcessed_
 * \brief Indicates that the IPA produced the metadata from the ISP stats buffer

 * \var NxpNeoFrames::InfoContext::buffers_
 * \brief Buffers and status associated with the context image
 *
 * Each element of the map is a std::pair<FrameBuffer *, bool> holding for each
 * buffer type:
 *  - The buffer itself represented by a FrameBuffer
 *  - The buffer processing status - true if pending, false once complete
 * The types of buffer associated to a context are:
 *  - The front end buffers, usually allocated from internal buffer pools but
 *    may also come from the application when a raw stream exists
 *  - The ISP params and stats buffers exchanged between the pipeline handler
 *    and the IPA, allocated from internal buffer pools
 *  - The buffers for the images decoded by the ISP, usually provided by the
 *    application as the buffers associated to the streams.
 * Buffers allocation and mapping for each buffer type is done at
 * NxpNeoFrames::Info creation time.
 *
 */

/**
 * \class NxpNeoFrames::Info
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
 * \var NxpNeoFrames::Info::id_
 * \brief Corresponds to the libcamera::Request sequence number
 *
 * \var NxpNeoFrames::Info::request_
 * \brief The libcamera::Request bundled to that frame
 *
 * \var NxpNeoFrames::Info::contexts_
 * \brief Map of one or more context instances associated to this frame
 *
 * \var NxpNeoFrames::Info::rawStreamBuffer_
 * \brief The application raw Stream buffer from the request - null if none
 *
 *  \var NxpNeoFrames::Info::frameStreamBuffer_
 * \brief The application frame Stream buffer from the request - null if none
 *
 *  \var NxpNeoFrames::Info::irStreamBuffer_
 * \brief The application IR Stream buffer from the request - null if none
 */

class NxpNeoFrames
{
public:
	class InfoContext
	{
	public:
		int resolveBuffer(BufferType bufferType);
		bool isBufferPending(const std::vector<BufferType> &bufferTypes) const;
		bool isContextComplete() const;
		FrameBuffer *getBuffer(BufferType bufferType) const;

		bool paramDequeued_;
		bool metadataProcessed_;

	private:
		friend NxpNeoFrames;

		std::map<BufferType, std::pair<FrameBuffer *, bool>> buffers_;
	};

	class Info
	{
	public:
		bool isFrameComplete() const;
		InfoContext *getContext(CameraContext context);

		unsigned int id_;
		Request *request_;

	private:
		friend NxpNeoFrames;

		std::map<CameraContext, InfoContext> contexts_;

		FrameBuffer *rawStreamBuffer_;
		FrameBuffer *frameStreamBuffer_;
		FrameBuffer *irStreamBuffer_;
	};

	NxpNeoFrames(NxpNeoCameraData *data);

	int destroy(unsigned int id);
	void clear();
	Info *create(Request *request);

	Info *find(unsigned int id);
	Info *find(Request *request);
	std::tuple<Info *, InfoContext *, CameraContext>
	find(const FrameBuffer *buffer, BufferType bufferType);

private:
	FrameBuffer *reserveBuffer(BufferType bufferType);
	void releaseBuffer(BufferType bufferType, FrameBuffer *buffer);
	Info *createRaw(Request *request);
	Info *createYuv(Request *request);

	NxpNeoCameraData *data_;
	std::map<unsigned int, Info> frameInfo_;
};

class NxpNeoCameraData : public Camera::Private
{
public:
	NxpNeoCameraData(PipelineHandler *pipe,
			 FrontEndHandler *frontEnd,
			 FrontEndCamera *feCamera)
		: Camera::Private(pipe),
		  frameInfos_(this),
		  frontEnd_(frontEnd),
		  feCamera_(feCamera) {}

	int configure(CameraConfiguration *c);
	int exportFrameBuffers(Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	int start(const ControlList *controls);
	void stopDevice();

	int queueRequestDevice(Request *request);

	int init();
	PipelineHandlerNxpNeo *pipe();

	void adjustTopLinesSize(Size *size) const;

	/*
	 * Accessors to camera data used by the pipeline handler class
	 * in order to handle the CameraConfiguration ops.
	 */
	const FrontEndCamera *feCamera() const { return feCamera_; };
	const CameraSensor *sensor() const { return feCamera_->sensor(); }
	const std::string &cameraName() const { return feCamera_->name(); }
	const FrontEndCamera::Attributes &feAttributes() const { return feCamera_->attributes(); }

	bool sensorIsRgbIr() const { return feAttributes().rgbIrCfa; }
	bool isRawCamera() const { return !feAttributes().ispBypass; }

	const std::map<Size, std::vector<unsigned int>> &
	formatsSizeToCodes() const { return feCamera_->formats().sizeMbusCodesMap; }
	const std::map<unsigned int, std::vector<Size>> &
	formatsCodeToSizes() const { return feCamera_->formats().mbusCodeSizesMap; }
	const std::map<unsigned int, std::vector<PixelFormat>> &
	formatsCodeToPixelFormats() const
	{
		return feCamera_->formats().mbusCodePixelFormatsMap;
	}

	NeoDevice *neoDevice(CameraContext context = CameraContext::Rgb) const;
	FrontEndHandler *frontEnd() const { return frontEnd_; }

	bool rawStreamOnly_ = false;

	Stream streamFrame_;
	Stream streamIr_;
	Stream streamRaw_;

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

	int configureRaw(CameraConfiguration *c);
	int configureYuv(CameraConfiguration *c);

	void cancelCompleteRequest(NxpNeoFrames::Info *info);
	void tryCompleteRequest(NxpNeoFrames::Info *info);

	void feInputBufferReady(NxpNeoFrames::Info *info, CameraContext context);
	void feImage0BufferReady(FrameBuffer *buffer);
	void feImage1BufferReady(FrameBuffer *buffer);
	void feEDataBufferReady(FrameBuffer *buffer);
	void applySensorControls(NxpNeoFrames::Info *info);

	void neoInput0BufferReady(FrameBuffer *buffer);
	void neoInput1BufferReady(FrameBuffer *buffer);
	void neoOutputBufferReady(FrameBuffer *buffer, BufferType bufferType);
	void neoFrameBufferReady(FrameBuffer *buffer);
	void neoIrBufferReady(FrameBuffer *buffer);
	void neoParamsBufferReady(FrameBuffer *buffer);
	void neoStatsBufferReady(FrameBuffer *buffer);

	void ipaParamsComputed(unsigned int id,
			       ipa::nxpneo::IPACameraContext context,
			       unsigned int bytesused);
	void ipaMetadataReady(unsigned int id,
			      ipa::nxpneo::IPACameraContext context,
			      const ControlList &metadata);
	void ipaSetSensorControls(unsigned int id,
				  const ControlList &sensorControls);
	void ipaSetLensControls(const ControlList &lensControls);

	const std::vector<CameraContext> &contexts() const { return contexts_; }

	NxpNeoFrames frameInfos_;
	bool alternatedRawStream_ = false;

	std::unique_ptr<ipa::nxpneo::IPAProxyNxpNeo> ipa_;
	ControlInfoMap ipaControls_;
	std::vector<IPABuffer> ipaBuffers_;
	std::unique_ptr<DelayedControls> delayedCtrls_;

	unsigned int sequence_ = 0;
	unsigned int embeddedTopLines_ = 0;

	FrontEndHandler *frontEnd_;
	FrontEndCamera *feCamera_;
	std::map<CameraContext, NeoDevice *> neoDevices_;

	std::vector<CameraContext> contexts_;
	std::map<FEStream, std::vector<std::unique_ptr<FrameBuffer>>> feBufferPools_;
	std::map<FEStream, V4L2DeviceFormat> feVDevFormats_;

	PipelineMode mode_ = PipelineMode::Standard;

	std::map<BufferType, std::queue<FrameBuffer *>> availableBuffersMap_;

	std::unique_ptr<Timer> controlsTimer_;
};

class NxpNeoCameraConfiguration : public CameraConfiguration
{
public:
	NxpNeoCameraConfiguration(Camera *camera, NxpNeoCameraData *data);

	Status validate() override;

	const V4L2SubdeviceFormat &sensorFormat() { return sensorFormat_; }
	const Transform &combinedTransform() { return combinedTransform_; }

private:
	Status validateRaw();
	Status validateYuv();

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

namespace {

/*
 * Maximum number of requests that shall be queued into the pipeline to keep
 * the regulation fast.
 */
static constexpr unsigned int kNeoIspMaxQueuedRequests = 4;

/*
 * This many internal buffers (or rather parameter and statistics buffer
 * pairs) ensures that the pipeline runs smoothly, without frame drops.
 */
static constexpr unsigned int kNeoIspMinBufferCount = 4;

} /* namespace */

class PipelineHandlerNxpNeo : public PipelineHandler
{
public:
	PipelineHandlerNxpNeo(CameraManager *manager)
		: PipelineHandler(manager, kNeoIspMaxQueuedRequests) {}

	std::unique_ptr<CameraConfiguration> generateConfiguration(
		Camera *camera, Span<const StreamRole> roles) override;

	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

	bool acquireDevice(Camera *camera) override;

	const PipelineConfig *pipelineConfig() { return &pipelineConfig_; }

private:
	friend NxpNeoCameraData;

	NxpNeoCameraData *cameraData(Camera *camera)
	{
		return static_cast<NxpNeoCameraData *>(camera->_d());
	}

	int createCamera(FrontEndHandler *fe, FrontEndCamera *feCamera);

	std::unique_ptr<CameraConfiguration> generateConfigurationRaw(
		Camera *camera, Span<const StreamRole> roles);
	std::unique_ptr<CameraConfiguration> generateConfigurationYuv(
		Camera *camera, Span<const StreamRole> roles);

	int loadPipelineConfig();

	PipelineConfig pipelineConfig_;

	std::vector<std::unique_ptr<FrontEndHandler>> frontEnds_;
	std::unique_ptr<NeoDeviceAllocator> neoAllocator_;
};

namespace {

const std::map<FEStream, BufferType> streamToBufferType = {
	{ FEStream::Image0, BufferType::Image0 },
	{ FEStream::Image1, BufferType::Image1 },
	{ FEStream::EData, BufferType::EData },
};

}

/**
 * \brief Mark a buffer as completed for this context
 * \param[in] bufferType The type of buffer to mark as complete
 *
 * This function marks a buffer of the specified type as no longer pending,
 * indicating that its processing has completed.
 *
 * \return 0 on success, -ENOENT if the buffer type is not found, or -EINVAL
 * if the buffer was already resolved
 */
int NxpNeoFrames::InfoContext::resolveBuffer(BufferType bufferType)
{
	auto it = buffers_.find(bufferType);
	if (it == buffers_.end()) {
		LOG(NxpNeoPipe, Error) << "Buffer type to retire not found";
		return -ENOENT;
	}

	auto &[buffer, pending] = it->second;
	if (!pending) {
		LOG(NxpNeoPipe, Error) << "Buffer already retired";
		return -EINVAL;
	}

	pending = false;
	return 0;
}

/**
 * \brief Check if any buffer of the specified types is still pending
 * \param[in] bufferTypes Vector of buffer types to check
 *
 * \return True if at least one buffer of the specified types is still pending,
 * false otherwise
 */
bool NxpNeoFrames::InfoContext::isBufferPending(
	const std::vector<BufferType> &bufferTypes) const
{
	for (BufferType bufferType : bufferTypes) {
		const auto it = buffers_.find(bufferType);
		if (it == buffers_.end())
			continue;
		const auto &[buffer, pending] = it->second;
		if (pending)
			return true;
	}

	return false;
}

/**
 * \brief Check if all processing for this context is complete
 *
 * A context is considered complete when all its buffers have been processed,
 * the params buffer has been dequeued by the ISP, and the IPA has produced
 * the metadata from the stats buffer.
 *
 * \return True if the context processing is complete, false otherwise
 */
bool NxpNeoFrames::InfoContext::isContextComplete() const
{
	bool buffersComplete = true;
	for (const auto &[type, bufferDesc] : buffers_) {
		const auto &[buffer, pending] = bufferDesc;
		if (pending) {
			buffersComplete = false;
			break;
		}
	}

	bool complete = buffersComplete &&
			metadataProcessed_ && paramDequeued_;

	return complete;
}

/**
 * \brief Retrieve a buffer of the specified type from this context
 * \param[in] bufferType The type of buffer to retrieve
 *
 * \return Pointer to the FrameBuffer if found, nullptr otherwise
 */
FrameBuffer *NxpNeoFrames::InfoContext::getBuffer(BufferType bufferType) const
{
	auto it = buffers_.find(bufferType);
	if (it == buffers_.end())
		return nullptr;

	const auto &[buffer, pending] = it->second;
	return buffer;
}

/**
 * \brief Check if all processing for this frame is complete
 *
 * A frame is considered complete when all its associated contexts have
 * completed their processing.
 *
 * \return True if the frame processing is complete, false otherwise
 */
bool NxpNeoFrames::Info::isFrameComplete() const
{
	for (const auto &[context, infoContext] : contexts_) {
		if (!infoContext.isContextComplete())
			return false;
	}

	return true;
}

/**
 * \brief Retrieve the InfoContext for a specific camera context
 * \param[in] context The camera context to retrieve
 *
 * \return Pointer to the InfoContext if found, nullptr otherwise
 */
NxpNeoFrames::InfoContext *
NxpNeoFrames::Info::getContext(CameraContext context)
{
	auto it = contexts_.find(context);
	if (it == contexts_.end()) {
		LOG(NxpNeoPipe, Error) << "Frame info for context not found";
		return nullptr;
	}

	return &it->second;
}

NxpNeoFrames::NxpNeoFrames(NxpNeoCameraData *data)
	: data_(data)
{
}

/**
 * \brief Destroy a frame Info instance and release its resources
 * \param[in] id The frame sequence number identifying the Info to destroy
 *
 * This function removes the frame Info associated with the given id from the
 * active frames map and returns all internal buffers to their respective pools
 * for reuse.
 *
 * \return 0 on success, -ENOENT if the frame Info is not found
 */
int NxpNeoFrames::destroy(unsigned int id)
{
	Info *info = find(id);
	if (!info) {
		LOG(NxpNeoPipe, Error) << "Info frame could not be destroyed";
		return -ENOENT;
	}

	/* Return internal buffers for reuse. */
	for (const auto &[context, infoContext] : info->contexts_) {
		for (const auto &[bufferType, bufferDesc] : infoContext.buffers_) {
			const auto &[buffer, pending] = bufferDesc;
			if (buffer == info->rawStreamBuffer_ ||
			    buffer == info->frameStreamBuffer_ ||
			    buffer == info->irStreamBuffer_)
				continue;
			releaseBuffer(bufferType, buffer);
		}
	}

	/* Delete the extended frame information. */
	frameInfo_.erase(info->id_);

	return 0;
}

/**
 * \brief Clear all active frame Info instances
 *
 * This function destroys all frame Info instances currently tracked, releasing
 * all associated resources and returning buffers to their pools.
 */
void NxpNeoFrames::clear()
{
	while (!frameInfo_.empty())
		destroy(frameInfo_.begin()->first);
}

/**
 * \brief Create a new frame Info instance for a libcamera::Request
 * \param[in] request The libcamera Request to associate with the frame Info
 *
 * This function creates and initializes a new Info instance that bundles the
 * Request with all necessary buffers for processing through the pipeline.
 *
 * \return Pointer to the created Info instance, or nullptr on failure
 */
NxpNeoFrames::Info *NxpNeoFrames::create(Request *request)
{
	if (data_->isRawCamera())
		return createRaw(request);
	else
		return createYuv(request);
}

/**
 * \brief Find a frame Info instance by libcamera::Request sequence number
 * \param[in] id The frame sequence number to search for
 *
 * \return Pointer to the Info instance if found, nullptr otherwise
 */
NxpNeoFrames::Info *NxpNeoFrames::find(unsigned int id)
{
	const auto it = frameInfo_.find(id);
	if (it != frameInfo_.end())
		return &it->second;

	LOG(NxpNeoPipe, Debug) << "Can't find tracking information for frame " << id;

	return nullptr;
}

/**
 * \brief Find a frame Info instance by libcamera::Request pointer
 * \param[in] request The Request to search for
 *
 * \return Pointer to the Info instance if found, nullptr otherwise
 */
NxpNeoFrames::Info *NxpNeoFrames::find(Request *request)
{
	for (auto &[id, info] : frameInfo_) {
		if (info.request_ == request)
			return &info;
	}

	LOG(NxpNeoPipe, Debug) << "Can't find tracking information from request";

	return nullptr;
}

/**
 * \brief Find frame Info, context, and camera context by buffer
 * \param[in] buffer The FrameBuffer to search for
 * \param[in] bufferType The type of buffer being searched
 *
 * This function searches through all active frame Info instances and their
 * contexts to locate the specified buffer of the given type.
 *
 * \return A tuple containing pointers to the Info, InfoContext, and the
 * CameraContext. Returns nullptr values if not found.
 */
std::tuple<NxpNeoFrames::Info *, NxpNeoFrames::InfoContext *, CameraContext>
NxpNeoFrames::find(const FrameBuffer *buffer, BufferType bufferType)
{
	for (auto &[id, info] : frameInfo_) {
		for (auto &[context, infoContext] : info.contexts_) {
			auto it = infoContext.buffers_.find(bufferType);
			if (it != infoContext.buffers_.end() &&
			    it->second.first == buffer)
				return { &info, &infoContext, context };
		}
	}

	LOG(NxpNeoPipe, Debug) << "Can't find frame info from buffer";
	return { nullptr, nullptr, CameraContext::Rgb };
}

FrameBuffer *NxpNeoFrames::reserveBuffer(BufferType bufferType)
{
	auto &buffersMap = data_->availableBuffersMap_;
	auto it = buffersMap.find(bufferType);
	if (it == buffersMap.end()) {
		LOG(NxpNeoPipe, Error)
			<< "No buffer pool type " << static_cast<int>(bufferType);
		return nullptr;
	}

	std::queue<FrameBuffer *> &queue = it->second;
	if (queue.empty()) {
		LOG(NxpNeoPipe, Error)
			<< "Buffer pool empty type " << static_cast<int>(bufferType);
		return nullptr;
	}
	FrameBuffer *buffer = queue.front();
	queue.pop();
	return buffer;
}

void NxpNeoFrames::releaseBuffer(BufferType bufferType, FrameBuffer *buffer)
{
	auto &buffersMap = data_->availableBuffersMap_;
	auto it = buffersMap.find(bufferType);
	if (it == buffersMap.end()) {
		LOG(NxpNeoPipe, Error)
			<< "No buffer pool type " << static_cast<int>(bufferType);
		return;
	}
	auto &[type, queue] = *it;
	queue.push(buffer);
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
	unsigned int contextCount = data_->contexts().size();
	for (const auto &[type, availableBuffers] : data_->availableBuffersMap_) {
		unsigned int count =
			type == BufferType::EData ||
					type == BufferType::Params ||
					type == BufferType::Stats
				? contextCount
				: 1;
		if (availableBuffers.size() < count) {
			LOG(NxpNeoPipe, Warning)
				<< " buffers underrun type " << static_cast<int>(type);
			return nullptr;
		}
	}

	Info &info = frameInfo_[id];
	info.id_ = id;
	info.request_ = request;
	info.rawStreamBuffer_ = request->findBuffer(&data_->streamRaw_);
	info.frameStreamBuffer_ = request->findBuffer(&data_->streamFrame_);
	info.irStreamBuffer_ = request->findBuffer(&data_->streamIr_);
	info.contexts_.emplace(CameraContext::Rgb, InfoContext{});
	if (data_->mode_ == PipelineMode::RgbIrDual)
		info.contexts_.emplace(CameraContext::Ir, InfoContext{});

	bool evenRequest = (id % 2 == 0);
	for (auto &[context, infoContext] : info.contexts_) {
		/*
		 * Map the ISP input buffers that are typically internal buffers
		 * unless the raw stream is active in which case the raw buffer
		 * is provided by the application.
		 * Alternate mapping of the raw stream has special cases:
		 * - RGBIr context switch: we may want to alternate capture on
		 *   the different contexts RGB and IR
		 * - HDR merge mode, we may want to alternate capture on long
		 *   and short frames (image0 and image1)
		 * In other cases, the raw stream is unconditionally mapped to
		 * the image0. If there is no raw stream enabled and so no
		 * dedicated buffer provided by the application, then internal
		 * buffers are used for image0 and image1 active pipes.
		 */
		FrameBuffer *image0Buffer = nullptr;
		FrameBuffer *image1Buffer = nullptr;

		PipelineMode mode = data_->mode_;
		bool hasImage0 = (mode != PipelineMode::RgbIrDual ||
				  context == CameraContext::Rgb);
		bool hasImage1 = (mode == PipelineMode::HdrMerge ||
				  (mode == PipelineMode::RgbIrDual &&
				   context == CameraContext::Ir));

		bool alternatedRawStream = data_->alternatedRawStream_;
		if (info.rawStreamBuffer_) {
			if (alternatedRawStream &&
			    mode == PipelineMode::RgbIrDual) {
				CameraContext rawContext =
					evenRequest
						? CameraContext::Rgb
						: CameraContext::Ir;
				if (context == rawContext) {
					if (hasImage0)
						image0Buffer = info.rawStreamBuffer_;
					else
						image1Buffer = info.rawStreamBuffer_;
				}
			} else if (alternatedRawStream &&
				   mode == PipelineMode::HdrMerge) {
				if (evenRequest)
					image0Buffer = info.rawStreamBuffer_;
				else
					image1Buffer = info.rawStreamBuffer_;

			} else {
				if (hasImage0)
					image0Buffer = info.rawStreamBuffer_;
			}
		}

		auto &buffersMap = infoContext.buffers_;
		if (hasImage0 && !image0Buffer)
			image0Buffer = reserveBuffer(BufferType::Image0);
		if (image0Buffer)
			buffersMap.emplace(BufferType::Image0,
					   std::make_pair(image0Buffer, true));

		if (hasImage1 && !image1Buffer)
			image1Buffer = reserveBuffer(BufferType::Image1);
		if (image1Buffer)
			buffersMap.emplace(BufferType::Image1,
					   std::make_pair(image1Buffer, true));

		bool hasEmbeddedData =
			data_->availableBuffersMap_.count(BufferType::EData);
		if (hasEmbeddedData) {
			FrameBuffer *edataBuffer = reserveBuffer(BufferType::EData);
			buffersMap.emplace(BufferType::EData,
					   std::make_pair(edataBuffer, true));
		}

		/* Map the ISP params / stats internal buffers */
		FrameBuffer *paramsBuffer = reserveBuffer(BufferType::Params);
		buffersMap.emplace(BufferType::Params,
				   std::make_pair(paramsBuffer, true));
		FrameBuffer *statsBuffer = reserveBuffer(BufferType::Stats);
		buffersMap.emplace(BufferType::Stats,
				   std::make_pair(statsBuffer, true));

		infoContext.paramDequeued_ = false;
		infoContext.metadataProcessed_ = false;

		if (data_->rawStreamOnly_)
			continue;

		/*
		 * Map the ISP frame and infrared output buffer of the ISP. They
		 * are provided by the application in the libcamera:Request as
		 * stream buffers.
		 */
		FrameBuffer *frameBuffer = nullptr;
		FrameBuffer *irBuffer = nullptr;

		switch (mode) {
		case PipelineMode::RgbIrDual:
			if (context == CameraContext::Rgb)
				frameBuffer = info.frameStreamBuffer_;
			else
				irBuffer = info.irStreamBuffer_;
			break;
		case PipelineMode::RgbIr:
		case PipelineMode::Standard:
		case PipelineMode::HdrMerge:
		default:
			frameBuffer = info.frameStreamBuffer_;
			irBuffer = info.irStreamBuffer_;
			break;
		}

		if (frameBuffer)
			buffersMap.emplace(BufferType::Frame,
					   std::make_pair(frameBuffer, true));
		if (irBuffer)
			buffersMap.emplace(BufferType::Ir,
					   std::make_pair(irBuffer, true));
	}

	return &info;
}

NxpNeoFrames::Info *NxpNeoFrames::createYuv(Request *request)
{
	unsigned int id = request->sequence();

	/* Single context, the output buffer is provided by application. */
	Info &info = frameInfo_[id];
	info.id_ = id;
	info.request_ = request;
	info.rawStreamBuffer_ = request->findBuffer(&data_->streamRaw_);
	info.frameStreamBuffer_ = nullptr;
	info.irStreamBuffer_ = nullptr;

	auto [it, inserted] =
		info.contexts_.emplace(CameraContext::Rgb, InfoContext{});
	InfoContext &infoContext = it->second;
	auto &buffersMap = infoContext.buffers_;
	buffersMap.emplace(BufferType::Image0,
			   std::make_pair(info.rawStreamBuffer_, true));

	/* IPA-related operations are bypassed */
	infoContext.paramDequeued_ = true;
	infoContext.metadataProcessed_ = true;

	return &info;
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
	const CameraSensor *sensor = data_->sensor();

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
	orientation = data_->feCamera()->validateOrientation(orientation);
	combinedTransform_ = sensor->computeTransform(&orientation);
	if (orientation != requestedOrientation)
		status = Adjusted;

	/*
	 * Work out the sensor format to be used. When a raw stream is specified
	 * its pixel output format defines explicitly the sensor bit depth and
	 * size. Thus, the raw stream configuration is checked first to find a
	 * possible match with the sensor format and size capabilities.
	 * If sensor format has not been resolved from the raw stream, check for
	 * every stream configured if the requested size can be provided by the
	 * sensor, then derive a working code for that size.
	 * If none of the streams is configured with a size supported with that
	 * sensor, fall back onto selecting arbitrarily the highest size and the
	 * associated mbus code with the highest bit depth.
	 */
	const std::map<unsigned int, std::vector<Size>> &codeToSizes =
		data_->formatsCodeToSizes();
	const std::vector<unsigned int> sensorCodes = utils::map_keys(codeToSizes);

	std::optional<V4L2SubdeviceFormat> sensorFormat;
	const auto rawConfigIt = std::find_if(
		config_.begin(), config_.end(),
		[=](StreamConfiguration &cfg) { return cfg.stream() == streamRaw; });
	if (rawConfigIt != config_.end()) {
		/*
		 * There is a raw stream: look for a corresponding mbus code
		 * that matches that pixel format and size.
		 */
		const StreamConfiguration &rawConfig = *rawConfigIt;
		const BayerFormat &bayerConfig =
			BayerFormat::fromPixelFormat(rawConfig.pixelFormat);
		auto rawCodeIt = std::find_if(
			sensorCodes.begin(), sensorCodes.end(),
			[&bayerConfig](unsigned int code) {
				const BayerFormat &bayerCode =
					BayerFormat::fromMbusCode(code);
				return bayerCode == bayerConfig;
			});
		if (rawCodeIt != sensorCodes.end()) {
			unsigned int code = *rawCodeIt;
			const std::vector<Size> &sizes = codeToSizes.at(code);
			if (std::find(sizes.begin(), sizes.end(),
				      rawConfig.size) != sizes.end()) {
				sensorFormat = {
					.code = code,
					.size = rawConfig.size,
					.colorSpace = std::nullopt,
				};
			}
		}
	}

	const std::map<Size, std::vector<unsigned int>> &sizeToCodes =
		data_->formatsSizeToCodes();
	const std::vector<Size> sensorSizes = utils::map_keys(sizeToCodes);

	if (!sensorFormat.has_value()) {
		/*
		 * Look for a sensor format that matches any StreamConfig size.
		 * For non-raw streams, the sensor format is adjusted to crop
		 * the embedded top lines if any, before comparing the sensor
		 * and the config sizes.
		 */
		for (const auto &config : config_) {
			bool isRaw = config.stream() == streamRaw;
			auto sizeIt = std::find_if(
				sensorSizes.begin(), sensorSizes.end(),
				[this, &config, isRaw](Size size) {
					if (!isRaw)
						data_->adjustTopLinesSize(&size);
					return size == config.size;
				});

			if (sizeIt == sensorSizes.end())
				continue;

			const Size &size = *sizeIt;
			if (sizeToCodes.at(size).empty())
				continue;

			unsigned int code = sizeToCodes.at(size).back();
			sensorFormat = {
				.code = code,
				.size = size,
				.colorSpace = std::nullopt,
			};
			break;
		}
	}

	if (!sensorFormat.has_value()) {
		/*
		 * Fallback: select the highest resolution and the associated
		 * mbus code with the largest bit depth.
		 */
		if (sizeToCodes.empty() || sizeToCodes.rbegin()->second.empty()) {
			LOG(NxpNeoPipe, Error) << "No sensor formats available";
			return Invalid;
		}
		const auto &[size, codes] = *sizeToCodes.rbegin();
		unsigned int bpp = 0;
		unsigned int code = 0;
		for (unsigned int c : codes) {
			const BayerFormat &bayerFormat =
				BayerFormat::fromMbusCode(c);
			if (bayerFormat.bitDepth > bpp) {
				bpp = bayerFormat.bitDepth;
				code = c;
			}
		}

		sensorFormat = {
			.code = code,
			.size = size,
			.colorSpace = std::nullopt,
		};
	}

	/* Cache sensor format for later usage by configure(). */
	sensorFormat_ = sensorFormat.value();
	LOG(NxpNeoPipe, Debug) << "Sensor format " << sensorFormat_.toString();

	Size ispStreamSize(sensorFormat_.size);
	data_->adjustTopLinesSize(&ispStreamSize);

	for (unsigned int i = 0; i < config_.size(); ++i) {
		const StreamConfiguration originalCfg = config_[i];
		StreamConfiguration *cfg = &config_[i];

		bool isFrame = (streamFrame == cfg->stream());
		bool isIr = (streamIr == cfg->stream());
		bool isRaw = (streamRaw == cfg->stream());

		LOG(NxpNeoPipe, Debug)
			<< "Stream " << i << " to validate cfg " << cfg->toString();

		if (isFrame || isIr) {
			/* Check format, default on YUYV (frame) and R8 (IR). */
			const NeoDevice *neo = data_->neoDevice();
			V4L2VideoDevice *device;
			if (isFrame) {
				const std::vector<PixelFormat> &pixelFormats =
					neo->capturePixelFormats(NeoDevice::VideoDevice::Frame);
				if (std::find(pixelFormats.begin(), pixelFormats.end(),
					      cfg->pixelFormat) == pixelFormats.end())
					cfg->pixelFormat = formats::YUYV;
				device = neo->frame_.get();
			} else {
				const std::vector<PixelFormat> &pixelFormats =
					neo->capturePixelFormats(NeoDevice::VideoDevice::Ir);
				if (std::find(pixelFormats.begin(), pixelFormats.end(),
					      cfg->pixelFormat) == pixelFormats.end())
					cfg->pixelFormat = formats::R8;
				device = neo->ir_.get();
			}
			cfg->size = ispStreamSize;

			V4L2DeviceFormat format = {};
			format.size = cfg->size;
			format.fourcc = device->toV4L2PixelFormat(cfg->pixelFormat);
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

			device->tryFormat(&format);
			cfg->stride = format.planes[0].bpl;
			cfg->frameSize = format.planes[0].size;
			cfg->colorSpace = isFrame ? format.colorSpace : ColorSpace::Raw;

			LOG(NxpNeoPipe, Debug) << "Assigned " << cfg->toString()
					       << " to the "
					       << (isFrame ? "frame" : "ir")
					       << " stream";
		} else if (isRaw) {
			const BayerFormat &bayerFormat =
				BayerFormat::fromMbusCode(sensorFormat_.code);
			cfg->pixelFormat = bayerFormat.toPixelFormat();
			cfg->size = sensorFormat_.size;
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

		if (cfg->bufferCount < kNeoIspMinBufferCount) {
			cfg->bufferCount = kNeoIspMinBufferCount;
			status = Adjusted;
		}

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
	const CameraSensor *sensor = data_->sensor();

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
	orientation = data_->feCamera()->validateOrientation(orientation);
	combinedTransform_ = sensor->computeTransform(&orientation);
	if (orientation != requestedOrientation)
		status = Adjusted;

	/*
	 * Make sure that the configuration size matches one resolution provided
	 * by the sensor. Also verify that the stream pixel format belongs to
	 * the list of processed formats supported by the front-end.
	 * Also, cache the corresponding sensor configuration for later usage
	 * during configure().
	 */
	const std::map<unsigned int, std::vector<Size>> &codeToSizes =
		data_->formatsCodeToSizes();
	const std::map<unsigned int, std::vector<PixelFormat>> &codeToPixelFormats =
		data_->formatsCodeToPixelFormats();

	sensorFormat_ = {};
	for (const auto &[code, pixformats] : codeToPixelFormats) {
		const auto itFormats = std::find(pixformats.begin(), pixformats.end(),
						 cfg.pixelFormat);
		if (itFormats == pixformats.end())
			continue;
		const auto itSizes = codeToSizes.find(code);
		if (itSizes == codeToSizes.end()) {
			LOG(NxpNeoPipe, Error) << "No sizes found for code " << code;
			continue;
		}
		const std::vector<Size> &sizes = itSizes->second;
		const auto itSize = std::find(sizes.begin(), sizes.end(), cfg.size);
		if (itSize == sizes.end())
			continue;
		sensorFormat_.code = code;
		sensorFormat_.size = cfg.size;
		break;
	}

	/* No matching sensor configuration. Fallback to any valid config. */
	if (!sensorFormat_.code || sensorFormat_.size.isNull()) {
		if (codeToPixelFormats.empty())
			return Invalid;
		auto itFormats = codeToPixelFormats.begin();
		unsigned int code = itFormats->first;
		cfg.pixelFormat = itFormats->second[0];
		const auto itSizes = codeToSizes.find(code);
		if (itSizes == codeToSizes.end())
			return Invalid;
		const std::vector<Size> &sizes = itSizes->second;
		if (sizes.empty())
			return Invalid;
		auto itSize = std::find(sizes.begin(), sizes.end(), cfg.size);
		if (itSize == sizes.end())
			cfg.size = sizes[0];

		sensorFormat_.code = code;
		sensorFormat_.size = cfg.size;
	}

	V4L2VideoDevice *vdev = data_->feCamera()->videoDevice(FEStream::Image0);
	if (!vdev)
		return Invalid;

	/* Acquire stride and color space from the front-end device */
	V4L2DeviceFormat devFormat = {};
	devFormat.size = cfg.size;
	devFormat.fourcc = vdev->toV4L2PixelFormat(cfg.pixelFormat);
	devFormat.colorSpace = cfg.colorSpace;
	vdev->tryFormat(&devFormat);

	cfg.colorSpace = devFormat.colorSpace;
	cfg.stride = devFormat.planes[0].bpl;
	cfg.frameSize = devFormat.planes[0].size;

	if (cfg.bufferCount < kNeoIspMinBufferCount) {
		cfg.bufferCount = kNeoIspMinBufferCount;
		status = Adjusted;
	}

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
		return generateConfigurationRaw(camera, roles);
	else
		return generateConfigurationYuv(camera, roles);
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
	return data->queueRequestDevice(request);
}

bool PipelineHandlerNxpNeo::match(DeviceEnumerator *enumerator)
{
	int ret = loadPipelineConfig();
	if (ret)
		return false;

	/*
	 * Prerequisite for pipeline operation is that front-end media
	 * controller device is present. Media device is acquired by the NEO
	 * context allocator.
	 */
	neoAllocator_ = std::make_unique<NeoDeviceAllocator>(this, enumerator);
	if (!neoAllocator_->isValid()) {
		LOG(NxpNeoPipe, Debug) << "Neo device allocator not found";
		return false;
	}

	unsigned int totalCount = 0;
	FrontEndHandler::MatchParams feMatchParams{
		.pipeline = this,
		.enumerator = enumerator,
		.neoAllocator = neoAllocator_.get(),
		.pipelineConfig = &pipelineConfig_,
	};

	/* Match all the registered front-end handlers. */
	const std::vector<FrontEndHandlerFactoryBase *> &factories =
		FrontEndHandlerFactoryBase::factories();
	for (const FrontEndHandlerFactoryBase *factory : factories) {
		LOG(NxpNeoPipe, Debug)
			<< "Found registered front-end '"
			<< factory->name() << "'";

		std::unique_ptr<FrontEndHandler> fe = factory->create();
		if (!fe->match(feMatchParams))
			continue;
		unsigned int cameraCount = 0;
		for (auto const &feCamera : fe->cameras()) {
			ret = createCamera(fe.get(), feCamera);
			if (ret)
				continue;
			cameraCount++;
		}
		if (cameraCount) {
			LOG(NxpNeoPipe, Debug)
				<< "Front-end " << fe->name()
				<< " registered " << cameraCount << " cameras";
			frontEnds_.push_back(std::move(fe));
		}
		totalCount += cameraCount;
	}

	return !!totalCount;
}

bool PipelineHandlerNxpNeo::acquireDevice(Camera *camera)
{
	NxpNeoCameraData *data = cameraData(camera);

	LOG(NxpNeoPipe, Debug) << "acquireDevice " << data->cameraName()
			       << " count " << useCount();

	const FrontEndCamera *feCamera = data->feCamera();
	FrontEndHandler *frontEnd = data->frontEnd();
	int ret = frontEnd->acquireDevice(feCamera->name());
	return !ret;
}

int PipelineHandlerNxpNeo::createCamera(FrontEndHandler *fe,
					FrontEndCamera *feCamera)
{
	/* CameraData instance creation */
	std::unique_ptr<NxpNeoCameraData> data =
		std::make_unique<NxpNeoCameraData>(this, fe, feCamera);

	int ret = data->init();
	if (ret)
		return ret;

	/* Create and register the Camera instance. */
	std::set<Stream *> streams = {
		&data->streamFrame_,
		&data->streamIr_,
		&data->streamRaw_,
	};
	const std::string &cameraId = feCamera->sensor()->id();
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), cameraId, streams);

	registerCamera(std::move(camera));

	return 0;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerNxpNeo::generateConfigurationRaw(Camera *camera,
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
			 * all output formats supported by the ISP. Gray formats
			 * reported apply to both RGB and IR pixels, when
			 * applicable to the sensor, as there is no dedicated IR
			 * role for now.
			 */
			const NeoDevice *neo = data->neoDevice();
			if (!neo)
				return nullptr;
			NeoDevice::VideoDevice device = NeoDevice::VideoDevice::Frame;
			for (const PixelFormat &format : neo->capturePixelFormats(device))
				streamFormats[format] = pixelRanges;
			if (data->sensorIsRgbIr()) {
				device = NeoDevice::VideoDevice::Ir;
				for (const PixelFormat &format : neo->capturePixelFormats(device))
					streamFormats[format] = pixelRanges;
			}

			/*
			 * Select only one format per ISP capture node so that
			 * the resulting stream configuration passes validate()
			 * check.
			 */
			if (frameOutputAvailable) {
				pixelFormat = formats::YUYV;
				colorSpace = ColorSpace::Sycc;
				frameOutputAvailable = false;
			} else if (irOutputAvailable) {
				pixelFormat = formats::R8;
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

	Orientation mountingOrientation = data->sensor()->mountingOrientation();
	config->orientation =
		data->feCamera()->validateOrientation(mountingOrientation);

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerNxpNeo::generateConfigurationYuv(Camera *camera,
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
	const std::map<unsigned int, std::vector<PixelFormat>> &codeToPixelFormats =
		data->formatsCodeToPixelFormats();

	switch (role) {
	case StreamRole::StillCapture:
	case StreamRole::Viewfinder:
	case StreamRole::VideoRecording: {
		/*
		 * This is a smart camera, so we assume that all sizes are
		 * available for all the available codes.
		 * Likewise, we assume that front-end can provide all pixel
		 * formats for all the sensor codes and sizes available.
		 */
		std::vector<SizeRange> sensorRanges;
		for (const Size &size : sensorSizes)
			sensorRanges.emplace_back(size);

		if (codeToPixelFormats.empty())
			return nullptr;

		for (const auto &pixelFormat : codeToPixelFormats.begin()->second)
			streamFormats[pixelFormat] = sensorRanges;
		break;
	}

	default:
		LOG(NxpNeoPipe, Error) << "Stream role not supported " << role;
		return nullptr;
	}

	if (streamFormats.empty() || streamFormats.begin()->second.empty())
		return nullptr;

	StreamFormats formats(streamFormats);
	StreamConfiguration cfg(formats);
	PixelFormat defaultFormat = formats::YUYV;
	auto it = streamFormats.find(defaultFormat);
	if (it == streamFormats.end())
		it = streamFormats.begin();
	cfg.pixelFormat = it->first;
	const std::vector<SizeRange> &sizeRanges = it->second;
	cfg.size = sizeRanges.back().max;

	const GlobalInfo &globalInfo =
		data->pipe()->pipelineConfig()->globalInfo();
	cfg.bufferCount = globalInfo.bufferCount;

	config->addConfiguration(cfg);
	LOG(NxpNeoPipe, Debug)
		<< "Generated configuration " << cfg.toString()
		<< " for role " << role;

	Orientation mountingOrientation = data->sensor()->mountingOrientation();
	config->orientation =
		data->feCamera()->validateOrientation(mountingOrientation);

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
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

	ret = pipelineConfig_.load(file);

	return ret;
}

int NxpNeoCameraData::configure(CameraConfiguration *c)
{
	if (isRawCamera())
		return configureRaw(c);
	else
		return configureYuv(c);
}

int NxpNeoCameraData::exportFrameBuffers(Stream *stream,
					 std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &streamFrame_) {
		NeoDevice *neoRgb = neoDevice(CameraContext::Rgb);
		if (!neoRgb)
			return -EINVAL;
		return neoRgb->frame_->exportBuffers(count, buffers);
	} else if (stream == &streamIr_) {
		CameraContext context =
			mode_ != PipelineMode::RgbIrDual
				? CameraContext::Rgb
				: CameraContext::Ir;
		NeoDevice *neo = neoDevice(context);
		if (!neo)
			return -EINVAL;
		return neo->ir_->exportBuffers(count, buffers);
	} else if (stream == &streamRaw_) {
		V4L2VideoDevice *vdev = feCamera_->videoDevice(FEStream::Image0);
		if (!vdev)
			return -EINVAL;
		return vdev->exportBuffers(count, buffers);
	}

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

	/* Start the IPA and the ISP instances for a raw camera. */
	if (isRawCamera()) {
		ret = ipa_->start();
		if (ret)
			goto error;

		if (delayedCtrls_)
			delayedCtrls_->reset();

		ret = 0;
		for (auto &[context, neo] : neoDevices_)
			ret |= neo->start();

		if (ret)
			goto error;
	}

	/* Start the front-end devices. */
	for (const auto stream : feCamera_->streams()) {
		V4L2VideoDevice *vdev = feCamera_->videoDevice(stream);
		if (!vdev)
			goto error;
		ret = vdev->streamOn();
		if (ret)
			goto error;
	}

	return 0;

error:
	LOG(NxpNeoPipe, Error) << "Failed to start camera " << cameraName();
	stopDevice();

	return -EINVAL;
}

void NxpNeoCameraData::stopDevice()
{
	int ret = 0;

	LOG(NxpNeoPipe, Debug) << "Stop device " << cameraName();

	for (const auto &stream : feCamera_->streams()) {
		V4L2VideoDevice *vdev = feCamera_->videoDevice(stream);
		if (vdev)
			ret |= vdev->streamOff();
	}

	if (isRawCamera()) {
		ipa_->stop();
		for (auto &[context, neo] : neoDevices_)
			ret |= neo->stop();
	}

	/*
	 * Queued requests with buffers queued in V4L2 devices have been
	 * synchronously cancelled by the stop() calls above. There may be some
	 * requests left that are pending on a IPA operation completion event.
	 * Cancel them now to clean up the pipeline state.
	 */
	while (!queuedRequests_.empty()) {
		Request *request = queuedRequests_.front();
		NxpNeoFrames::Info *frameInfo = frameInfos_.find(request);
		ASSERT(frameInfo);
		cancelCompleteRequest(frameInfo);
	}

	frameInfos_.clear();

	freeBuffers();

	if (controlsTimer_.get())
		controlsTimer_->stop();

	if (ret)
		LOG(NxpNeoPipe, Warning) << "Failed to stop camera " << cameraName();
}

int NxpNeoCameraData::queueRequestDevice(Request *request)
{
	NxpNeoFrames::Info *info;
	int ret = 0;

	info = frameInfos_.create(request);
	if (!info)
		return -EAGAIN;

	for (const CameraContext context : contexts()) {
		NxpNeoFrames::InfoContext *infoContext = info->getContext(context);
		if (!infoContext)
			continue;
		for (const auto &stream : feCamera_->streams()) {
			V4L2VideoDevice *vdev = feCamera_->videoDevice(stream);
			if (!vdev)
				return -ENODEV;
			BufferType bufferType = streamToBufferType.at(stream);
			FrameBuffer *buffer = infoContext->getBuffer(bufferType);
			if (!buffer)
				continue;
			ret |= vdev->queueBuffer(buffer);
		}
	}

	if (ret) {
		/* Request will be cancelled as an error is returned. */
		LOG(NxpNeoPipe, Error)
			<< "Failed to queue buffers, unbalanced queues";
		frameInfos_.destroy(info->id_);
		return -EIO;
	}

	if (isRawCamera())
		ipa_->queueRequest(info->id_, request->controls());

	return 0;
}

/**
 * \brief Initialize sensor, frontend, IPA and callbacks
 * \param[in] enumerator The media devices enumerator
 *
 * \return 0 on success or a negative error code otherwise
 */
int NxpNeoCameraData::init()
{
	int ret = 0;

	/* Pipeline mode selection. */
	FrontEndCamera::Attributes attributes;
	if (isRawCamera()) {
		if (feCamera_->hasStream(FEStream::Image1))
			mode_ = sensorIsRgbIr()
					? PipelineMode::RgbIrDual
					: PipelineMode::HdrMerge;
		else
			mode_ = sensorIsRgbIr()
					? PipelineMode::RgbIr
					: PipelineMode::Standard;
	} else {
		mode_ = PipelineMode::Standard;
	}

	/*
	 * Memory-to-memory operation: connect the front-end buffer ready slots
	 * of the different streams to their respective signals.
	 */
	if (!feCamera_->hasStream(FEStream::Image0)) {
		LOG(NxpNeoPipe, Error)
			<< "Mandatory stream image0 is missing for " << feCamera_->name();
		return -ENODEV;
	}

	const std::map<FEStream, void (NxpNeoCameraData::*)(FrameBuffer *)> feReadyFuncs{
		{ FEStream::Image0, &NxpNeoCameraData::feImage0BufferReady },
		{ FEStream::Image1, &NxpNeoCameraData::feImage1BufferReady },
		{ FEStream::EData, &NxpNeoCameraData::feEDataBufferReady },
	};

	for (const FEStream &stream : feCamera_->streams()) {
		auto it = feReadyFuncs.find(stream);
		if (it == feReadyFuncs.end()) {
			LOG(NxpNeoPipe, Error)
				<< "No buffer ready callback for stream " << static_cast<int>(stream);
			return -EINVAL;
		}
		V4L2VideoDevice *vdev = feCamera_->videoDevice(stream);
		if (!vdev)
			return -ENODEV;
		const auto &[_ignored, slot] = *it;
		vdev->bufferReady.connect(this, slot);
	}

	properties_ = sensor()->properties();

	static const std::vector<CameraContext>
		singleContext{ CameraContext::Rgb };
	static const std::vector<CameraContext>
		dualContext{ CameraContext::Rgb, CameraContext::Ir };
	contexts_ = mode_ == PipelineMode::RgbIrDual ? dualContext : singleContext;

	/*
	 * Further initializations are relevant to the ISP and IPA. For the
	 * camera pipelines that do not use the ISP, there is nothing more to
	 * do.
	 */
	if (!isRawCamera())
		return 0;

	const std::vector<NeoDevice *> &neoDevices = feCamera_->neoDevices();
	if (contexts_.size() != neoDevices.size()) {
		LOG(NxpNeoPipe, Error)
			<< "Number of contexts does not match number of neo devices";
		return -ENODEV;
	}
	for (const auto &[i, context] : utils::enumerate(contexts_))
		neoDevices_.insert({ context, neoDevices[i] });

	for (auto &[context, neo] : neoDevices_) {
		neo->input0_->bufferReady.connect(
			this, &NxpNeoCameraData::neoInput0BufferReady);
		neo->input1_->bufferReady.connect(
			this, &NxpNeoCameraData::neoInput1BufferReady);
		neo->frame_->bufferReady.connect(
			this, &NxpNeoCameraData::neoFrameBufferReady);
		neo->ir_->bufferReady.connect(
			this, &NxpNeoCameraData::neoIrBufferReady);
		neo->params_->bufferReady.connect(
			this, &NxpNeoCameraData::neoParamsBufferReady);
		neo->stats_->bufferReady.connect(
			this, &NxpNeoCameraData::neoStatsBufferReady);
	}

	/*
	 * IPA and controls initializations.
	 */
	ret = loadIPA();
	if (ret)
		return ret;
	updateControls();

	controlsTimer_ = feAttributes().controlsDelay.has_value()
				 ? std::make_unique<Timer>()
				 : nullptr;

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
 * \brief Get the NeoDevice object bound to a camera context
 * \param[in] context The targeted context
 *
 * \return The NeoDevice object for the context or nullptr if it does not exist
 */
NeoDevice *NxpNeoCameraData::neoDevice(CameraContext context) const
{
	auto it = neoDevices_.find(context);
	if (it != neoDevices_.end())
		return neoDevices_.at(context);
	else {
		LOG(NxpNeoPipe, Error) << "NeoDevice not found for context";
		return nullptr;
	}
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
	ipa_ = pipe()->createIPA<ipa::nxpneo::IPAProxyNxpNeo>(1, 1);
	if (!ipa_)
		return -ENOENT;

	ipa_->setSensorControls.connect(this, &NxpNeoCameraData::ipaSetSensorControls);
	ipa_->setLensControls.connect(this, &NxpNeoCameraData::ipaSetLensControls);
	ipa_->paramsComputed.connect(this, &NxpNeoCameraData::ipaParamsComputed);
	ipa_->metadataReady.connect(this, &NxpNeoCameraData::ipaMetadataReady);

	IPACameraSensorInfo sensorInfo{};
	CameraSensor *sensor = feCamera_->sensor();
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

	NeoDevice *neo = neoDevice();
	if (!neo)
		return -ENODEV;
	uint32_t hwRevision = neo->media()->hwRevision();
	ipa::nxpneo::SensorConfig sensorConfig;
	std::vector<uint32_t> ids = utils::map_keys(sensor->controls().idmap());
	ipa::nxpneo::InitParams initParams = {
		hwRevision,
		neo->hwCapabilities(),
		neo->supportedParamsBlocks(),
		feCamera_->name(),
		sensorInfo,
		sensor->controls(),
		sensor->getControls(ids),
		!!sensor->focusLens(),
	};

	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor->model() },
			 initParams, &ipaControls_, &sensorConfig);
	if (ret) {
		LOG(NxpNeoPipe, Error) << "Failed to initialise the NxpNeo IPA";
		return ret;
	}

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

	V4L2Subdevice *subdev = sensor->device();
	if (subdev) {
		delayedCtrls_ = std::make_unique<DelayedControls>(
			subdev, delayedControlsParams);
	}

	return 0;
}

/**
 * \brief Allocate buffers from front-end and ISP devices
 *
 * Internal buffers are allocated for front-end channels and ISP params and
 * statistics buffers. Those buffers are aggregated into the list of shared
 * buffers between the pipeline and the IPA.
 * Those buffers are registered into separates pools that will be accessed by
 * the NxpNeoFrames objects to pick buffers for the different pipeline stages.
 *
 * \return 0 in case of success or a negative error code
 */
int NxpNeoCameraData::allocateBuffers()
{
	if (isRawCamera())
		return allocateBuffersRaw();
	else
		return allocateBuffersYuv();
}

int NxpNeoCameraData::allocateBuffersRaw()
{
	unsigned int bufferCount = kNeoIspMaxQueuedRequests;
	unsigned int ipaBufferId = 1;

	auto registerPoolBuffers =
		[&](std::vector<std::unique_ptr<FrameBuffer>> *_pool, BufferType _bufferType) {
			for (const std::unique_ptr<FrameBuffer> &buffer : *_pool) {
				Span<const FrameBuffer::Plane> planes = buffer->planes();
				buffer->setCookie(ipaBufferId++);
				ipaBuffers_.emplace_back(buffer->cookie(),
							 std::vector<FrameBuffer::Plane>{ planes.begin(),
											  planes.end() });
				availableBuffersMap_[_bufferType].push(buffer.get());
			}
		};

	/*
	 * RGBIr dual context switch has a peculiarity related to frame buffers
	 * allocation: ISP params and stats as well as buffers for the embedded
	 * data streams are instantiated per context so their pools have to be
	 * sized accordingly.
	 * Raw-only operation has the specificity that no buffers are provided
	 * by the application for the ISP outputs. Therefore the ISP capture
	 * video devices are disabled and no buffers have to be provided to
	 * them.
	 */
	int ret = 0;

	/* Allocate and map stats and params buffers. */
	for (auto &[context, neo] : neoDevices_) {
		const std::map<BufferType, std::vector<std::unique_ptr<FrameBuffer>> *>
			ispMetaPools = {
				{ BufferType::Params, &neo->paramsBuffers_ },
				{ BufferType::Stats, &neo->statsBuffers_ },
			};
		ret |= neo->allocateBuffers(bufferCount);
		for (const auto [bufferType, pool] : ispMetaPools)
			registerPoolBuffers(pool, bufferType);
	}

	/* Allocate front-end devices buffers for images and edata streams. */
	unsigned int contextCount = contexts().size();
	for (const FEStream &stream : feCamera_->streams()) {
		unsigned int count = stream == FEStream::EData
					     ? bufferCount * contextCount
					     : bufferCount;
		V4L2VideoDevice *vdev = feCamera_->videoDevice(stream);
		if (!vdev) {
			ret |= -ENODEV;
			continue;
		}
		std::vector<std::unique_ptr<FrameBuffer>> &pool = feBufferPools_[stream];
		int res = vdev->exportBuffers(count, &pool);
		if (res < 0 || static_cast<unsigned int>(res) != count)
			ret |= -ENOMEM;
		ret |= vdev->importBuffers(count);

		BufferType bufferType = streamToBufferType.at(stream);
		registerPoolBuffers(&pool, bufferType);
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

	/* Application exports buffers imported by the front-end devices. */
	for (const FEStream &stream : feCamera_->streams()) {
		V4L2VideoDevice *vdev = feCamera_->videoDevice(stream);
		if (!vdev) {
			ret = -ENODEV;
			continue;
		}
		ret |= vdev->importBuffers(bufferCount);
	}

	if (ret)
		freeBuffers();

	return ret;
}

/**
 * \brief Deallocate buffers from front-end and ISP
 * \return 0 in case of success or a negative error code
 */

int NxpNeoCameraData::freeBuffers()
{
	if (isRawCamera())
		return freeBuffersRaw();
	else
		return freeBuffersYuv();
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

	for (auto &[context, neo] : neoDevices_)
		neo->freeBuffers();

	int ret = 0;
	for (const FEStream &stream : feCamera_->streams()) {
		V4L2VideoDevice *vdev = feCamera_->videoDevice(stream);
		if (!vdev) {
			ret |= -ENODEV;
			continue;
		}
		vdev->releaseBuffers();
	}
	feBufferPools_.clear();

	return ret;
}

int NxpNeoCameraData::freeBuffersYuv()
{
	int ret = 0;
	for (const FEStream &stream : feCamera_->streams()) {
		V4L2VideoDevice *vdev = feCamera_->videoDevice(stream);
		if (!vdev) {
			ret |= -ENODEV;
			continue;
		}
		vdev->releaseBuffers();
	}

	return ret;
}

int NxpNeoCameraData::configureRaw(CameraConfiguration *c)
{
	NxpNeoCameraConfiguration *config =
		static_cast<NxpNeoCameraConfiguration *>(c);
	int ret = 0;

	LOG(NxpNeoPipe, Debug) << "Configure " << cameraName();

	CameraSensor *sensor = feCamera_->sensor();

	/* Front-end configuration */
	V4L2SubdeviceFormat sensorFormat = config->sensorFormat();
	std::map<FEStream, V4L2DeviceFormat> feVDevFormats;
	ret = feCamera_->configure(sensorFormat, config->combinedTransform(),
				   &feVDevFormats, nullptr);
	if (ret)
		return ret;

	/* ISP configuration. */
	V4L2DeviceFormat devFormatFrame{};
	V4L2DeviceFormat devFormatIr{};

	V4L2DeviceFormat &devFormatInput0 = feVDevFormats.at(FEStream::Image0);
	V4L2DeviceFormat devFormatInput1None{};
	V4L2DeviceFormat &devFormatInput1 =
		mode_ == PipelineMode::HdrMerge
			? feVDevFormats.at(FEStream::Image1)
			: devFormatInput1None;

	rawStreamOnly_ = ((config->size() == 1) &&
			  ((*config)[0].stream() == &streamRaw_));

	NeoDevice *neoRgb = neoDevice(CameraContext::Rgb);
	if (!neoRgb)
		return -EINVAL;
	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = (*config)[i];
		Stream *stream = cfg.stream();

		if (stream == &streamRaw_)
			continue;

		V4L2DeviceFormat *deviceFormat;
		V4L2VideoDevice *videoDevice;
		if (stream == &streamFrame_) {
			deviceFormat = &devFormatFrame;
			videoDevice = neoRgb->frame_.get();
		} else {
			deviceFormat = &devFormatIr;
			videoDevice = neoRgb->ir_.get();
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

	NeoDevice::PipeConfig pipeConfig = {};
	pipeConfig.topLines = embeddedTopLines_;
	std::map<NeoDevice::VideoDevice, V4L2DeviceFormat *> ispFormatsMap;
	ispFormatsMap[NeoDevice::VideoDevice::Input0] = &devFormatInput0;
	if (devFormatInput1.fourcc.isValid())
		ispFormatsMap[NeoDevice::VideoDevice::Input1] = &devFormatInput1;
	if (mode_ != PipelineMode::RgbIrDual) {
		if (devFormatFrame.fourcc.isValid())
			ispFormatsMap[NeoDevice::VideoDevice::Frame] = &devFormatFrame;
		if (devFormatIr.fourcc.isValid())
			ispFormatsMap[NeoDevice::VideoDevice::Ir] = &devFormatIr;
		ret = neoRgb->configure(pipeConfig, ispFormatsMap);
	} else {
		if (devFormatFrame.fourcc.isValid())
			ispFormatsMap[NeoDevice::VideoDevice::Frame] = &devFormatFrame;
		ret = neoRgb->configure(pipeConfig, ispFormatsMap);

		ispFormatsMap.erase(NeoDevice::VideoDevice::Frame);
		if (devFormatIr.fourcc.isValid())
			ispFormatsMap[NeoDevice::VideoDevice::Ir] = &devFormatIr;
		NeoDevice *neoIr = neoDevice(CameraContext::Ir);
		if (!neoIr)
			return -EINVAL;
		ret |= neoIr->configure(pipeConfig, ispFormatsMap);
	}
	if (ret)
		return ret;

	/*
	 * Raw stream is mapped alternately on image0 and image1 for cases
	 *  - RGBIr dual context to capture both contexts
	 *  - HDR Merge to capture long and short images if they share the same
	 *    format
	 */
	if (mode_ == PipelineMode::RgbIrDual)
		alternatedRawStream_ = true;
	else if (mode_ == PipelineMode::HdrMerge)
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
	ret = sensor->sensorInfo(&sensorInfo);
	if (ret)
		return ret;
	adjustTopLinesSize(&sensorInfo.outputSize);

	std::map<ipa::nxpneo::IPAStreamType, IPAStream> streamConfig;

	ColorSpace colorSpace = ColorSpace::Raw;
	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = (*config)[i];
		Stream *stream = cfg.stream();

		if (stream == &streamFrame_) {
			streamConfig[ipa::nxpneo::IPAStreamType::Frame] =
				IPAStream{ cfg.pixelFormat, cfg.size };
			/*
			 * Take color space from the frame if it exists,
			 * or default to raw (IR-only stream case).
			 */
			colorSpace = cfg.colorSpace.value_or(ColorSpace::Raw);
		} else if (stream == &streamIr_) {
			streamConfig[ipa::nxpneo::IPAStreamType::Ir] =
				IPAStream{ cfg.pixelFormat, cfg.size };
		}
	}

	ipa::nxpneo::IPAConfigInfo configInfo;
	std::vector<uint32_t> ids = utils::map_keys(sensor->controls().idmap());
	configInfo.sensorControls = sensor->controls();
	configInfo.sensorControlList = sensor->getControls(ids);
	if (sensor->focusLens())
		configInfo.lensControls = sensor->focusLens()->controls();

	configInfo.sensorInfo = sensorInfo;

	configInfo.colorSpace = ipa::nxpneo::IPAColorSpace(
		static_cast<ipa::nxpneo::IPAPrimaries>(colorSpace.primaries),
		static_cast<ipa::nxpneo::IPATransferFunction>(colorSpace.transferFunction),
		static_cast<ipa::nxpneo::IPAYcbcrEncoding>(colorSpace.ycbcrEncoding),
		static_cast<ipa::nxpneo::IPARange>(colorSpace.range));

	configInfo.mode = static_cast<ipa::nxpneo::IPAPipelineMode>(mode_);

	const PixelFormatInfo &pixelformatInfo =
		PixelFormatInfo::info(devFormatInput1.fourcc);
	configInfo.bitsPerPixelAuxiliary =
		pixelformatInfo.isValid() ? pixelformatInfo.bitsPerPixel : 0;

	ret = ipa_->configure(configInfo, streamConfig, &ipaControls_);
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

	/* Front-end configuration */
	V4L2SubdeviceFormat sensorFormat = config->sensorFormat();
	StreamConfiguration &streamConfig = c->at(0);
	std::map<FEStream, V4L2DeviceFormat> feProcessedVDevFormats;
	V4L2DeviceFormat &deviceFormat =
		feProcessedVDevFormats[FEStream::Image0];
	V4L2VideoDevice *videoDevice = feCamera_->videoDevice(FEStream::Image0);
	if (!videoDevice)
		return -ENODEV;
	deviceFormat.fourcc =
		videoDevice->toV4L2PixelFormat(streamConfig.pixelFormat);
	deviceFormat.size = streamConfig.size;
	deviceFormat.colorSpace = streamConfig.colorSpace;
	ret = feCamera_->configure(sensorFormat, config->combinedTransform(),
				   nullptr, &feProcessedVDevFormats);

	return ret;
}

/* -----------------------------------------------------------------------------
 * Buffer Handling
 */

/**
 * \brief Cancel an active request in the pipeline
 * \param[in] request The frame Info associated with the request to be cancelled
 */
void NxpNeoCameraData::cancelCompleteRequest(NxpNeoFrames::Info *info)
{
	Request *request = info->request_;

	frameInfos_.destroy(info->id_);

	pipe()->cancelRequest(request);
}

/**
 * \brief Complete an active request if no longer in use by the pipeline
 * \param[in] info The frame Info associated to the request to be completed
 */
void NxpNeoCameraData::tryCompleteRequest(NxpNeoFrames::Info *info)
{
	Request *request = info->request_;

	if (!info->isFrameComplete())
		return;

	frameInfos_.destroy(info->id_);

	pipe()->completeRequest(request);
}

/* -----------------------------------------------------------------------------
 * Buffer Ready slots
 */

/**
 * \brief Handle stream buffer availability from the front-end video device
 * \param[in] info The frame info associated to ongoing request
 * \param[in] context The frame context relevant to the buffer received
 *
 * In case all front-end buffers associated to the request have been received,
 * the IPA can be invoked to retrieve ISP parameters.
 */
void NxpNeoCameraData::feInputBufferReady(NxpNeoFrames::Info *info, CameraContext context)
{
	static const std::vector<BufferType> inputBufferTypes = {
		BufferType::Image0, BufferType::Image1, BufferType::EData
	};
	NxpNeoFrames::InfoContext *infoContext = info->getContext(context);
	if (!infoContext)
		return;
	if (infoContext->isBufferPending(inputBufferTypes))
		return;

	std::map<ipa::nxpneo::IPABufferType, uint32_t> bufferIds;

	FrameBuffer *image0Buffer = infoContext->getBuffer(BufferType::Image0);
	if (image0Buffer)
		bufferIds[ipa::nxpneo::IPABufferType::Image0] = image0Buffer->cookie();

	FrameBuffer *image1Buffer = infoContext->getBuffer(BufferType::Image1);
	if (image1Buffer)
		bufferIds[ipa::nxpneo::IPABufferType::Image1] = image1Buffer->cookie();

	FrameBuffer *edataBuffer = infoContext->getBuffer(BufferType::EData);
	if (edataBuffer)
		bufferIds[ipa::nxpneo::IPABufferType::EData] = edataBuffer->cookie();

	FrameBuffer *paramsBuffer = infoContext->getBuffer(BufferType::Params);
	if (!paramsBuffer) {
		LOG(NxpNeoPipe, Error) << "Params buffer not available";
		return;
	}
	bufferIds[ipa::nxpneo::IPABufferType::Params] = paramsBuffer->cookie();

	ipa_->computeParams(info->id_,
			    static_cast<ipa::nxpneo::IPACameraContext>(context),
			    bufferIds);
}

/**
 * \brief Handle image0 stream buffer availability from front-end video device
 * \param[in] buffer The completed buffer
 */
void NxpNeoCameraData::feImage0BufferReady(FrameBuffer *buffer)
{
	BufferType bufferType = BufferType::Image0;
	auto [info, infoContext, context] = frameInfos_.find(buffer, bufferType);
	if (!info || !infoContext)
		return;

	if (infoContext->resolveBuffer(bufferType))
		return;

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
	Request *request = info->request_;
	request->_d()->metadata().set(controls::SensorTimestamp,
				      buffer->metadata().timestamp);

	if (request->findBuffer(&streamRaw_) == buffer)
		pipe()->completeBuffer(request, buffer);

	if (isRawCamera()) {
		feInputBufferReady(info, context);

		applySensorControls(info);
	} else {
		tryCompleteRequest(info);
	}
}

/**
 * \brief Handle image1 stream buffer availability from front-end video device
 * \param[in] buffer The completed buffer
 */
void NxpNeoCameraData::feImage1BufferReady(FrameBuffer *buffer)
{
	BufferType bufferType = BufferType::Image1;
	auto [info, infoContext, context] = frameInfos_.find(buffer, bufferType);
	if (!info || !infoContext)
		return;

	if (infoContext->resolveBuffer(bufferType))
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	Request *request = info->request_;
	if (request->findBuffer(&streamRaw_) == buffer)
		pipe()->completeBuffer(request, buffer);

	static const std::vector<BufferType> image0BufferType = { BufferType::Image0 };
	if (mode_ == PipelineMode::HdrMerge &&
	    infoContext->isBufferPending(image0BufferType))
		LOG(NxpNeoPipe, Info) << "Out of order input frame receipt";

	feInputBufferReady(info, context);
}

/**
 * \brief Handle edata stream buffer availability from front-end video device
 * \param[in] buffer The completed buffer
 *
 * Embedded data buffer is to be passed to IPA for 3A algorithms to use
 * along with sensor control info and ISP statistics.
 */
void NxpNeoCameraData::feEDataBufferReady(FrameBuffer *buffer)
{
	BufferType bufferType = BufferType::EData;
	auto [info, infoContext, context] = frameInfos_.find(buffer, bufferType);
	if (!info || !infoContext)
		return;

	if (infoContext->resolveBuffer(bufferType))
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	feInputBufferReady(info, context);
}

/**
 * \brief Apply the sensor controls update for the current request
 * \param[in] info The frame Info object associated to the request
 */
void NxpNeoCameraData::applySensorControls(NxpNeoFrames::Info *info)
{
	if (!isRawCamera())
		return;

	if (!delayedCtrls_)
		return;

	unsigned int id = info->id_;
	DelayedControls *delayedControls = delayedCtrls_.get();

	auto apply = [id, delayedControls]() {
		delayedControls->applyControls(id);
	};

	const std::optional<utils::Duration> &controlsDelay =
		feAttributes().controlsDelay;
	if (!controlsDelay.has_value()) {
		apply();
		return;
	}

	Timer *timer = controlsTimer_.get();
	if (timer->isRunning()) {
		LOG(NxpNeoPipe, Debug) << "Controls timer is running";
		timer->stop();
	}

	timer->timeout.disconnect();
	timer->timeout.connect(this, apply);

	std::chrono::milliseconds delay =
		std::chrono::duration_cast<std::chrono::milliseconds>(controlsDelay.value());
	timer->start(delay);
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
void NxpNeoCameraData::neoOutputBufferReady(FrameBuffer *buffer,
					    BufferType bufferType)
{
	auto [info, infoContext, context] =
		frameInfos_.find(buffer, bufferType);
	if (!info || !infoContext)
		return;

	if (infoContext->resolveBuffer(bufferType))
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	Request *request = info->request_;
	auto streamBuffers = request->buffers();
	auto it = std::find_if(streamBuffers.begin(), streamBuffers.end(),
			       [buffer](auto &kv) {
				       return kv.second == buffer;
			       });
	if (it != streamBuffers.end())
		pipe()->completeBuffer(request, buffer);

	tryCompleteRequest(info);
}

void NxpNeoCameraData::neoFrameBufferReady(FrameBuffer *buffer)
{
	neoOutputBufferReady(buffer, BufferType::Frame);
}

void NxpNeoCameraData::neoIrBufferReady(FrameBuffer *buffer)
{
	neoOutputBufferReady(buffer, BufferType::Ir);
}

/**
 * \brief Handle params buffers consumed by ISP
 * \param[in] buffer The consumed buffer
 */
void NxpNeoCameraData::neoParamsBufferReady(FrameBuffer *buffer)
{
	BufferType bufferType = BufferType::Params;
	auto [info, infoContext, context] = frameInfos_.find(buffer, bufferType);
	if (!info || !infoContext)
		return;

	if (infoContext->resolveBuffer(bufferType))
		return;

	if (infoContext->paramDequeued_)
		LOG(NxpNeoPipe, Error) << "Params buffer already dequeued ";
	infoContext->paramDequeued_ = true;

	tryCompleteRequest(info);
}

/**
 * \brief Handle stats buffers produced by ISP
 * \param[in] buffer The produced buffer
 */
void NxpNeoCameraData::neoStatsBufferReady(FrameBuffer *buffer)
{
	BufferType bufferType = BufferType::Stats;
	auto [info, infoContext, context] = frameInfos_.find(buffer, bufferType);
	if (!info || !infoContext)
		return;

	if (infoContext->resolveBuffer(bufferType))
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		cancelCompleteRequest(info);
		return;
	}

	std::map<ipa::nxpneo::IPABufferType, uint32_t> bufferIds = {
		{ ipa::nxpneo::IPABufferType::Stats, buffer->cookie() },
	};

	unsigned int sequence = info->id_;
	ControlList sensorControls;
	if (delayedCtrls_) {
		sensorControls = delayedCtrls_->get(sequence);
	} else {
		CameraSensor *sensor = feCamera_->sensor();
		std::vector<uint32_t> ids =
			utils::map_keys(sensor->controls().idmap());
		sensorControls = sensor->getControls(ids);
	}

	ipa_->processStats(sequence,
			   static_cast<ipa::nxpneo::IPACameraContext>(context),
			   bufferIds,
			   sensorControls);

	tryCompleteRequest(info);
}

void NxpNeoCameraData::ipaParamsComputed(unsigned int id,
					 ipa::nxpneo::IPACameraContext context,
					 unsigned int bytesused)
{
	NxpNeoFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

	CameraContext _context = static_cast<CameraContext>(context);
	NxpNeoFrames::InfoContext *infoContext = info->getContext(_context);
	if (!infoContext)
		return;

	int ret = 0;
	/* Queue ISP output buffers */
	NeoDevice *neo = neoDevice(_context);
	if (!neo)
		return;
	FrameBuffer *frameBuffer = infoContext->getBuffer(BufferType::Frame);
	if (frameBuffer)
		ret |= neo->frame_->queueBuffer(frameBuffer);
	FrameBuffer *irBuffer = infoContext->getBuffer(BufferType::Ir);
	if (irBuffer)
		ret |= neo->ir_->queueBuffer(irBuffer);

	/* Queue ISP params and stats buffers */
	FrameBuffer *paramsBuffer = infoContext->getBuffer(BufferType::Params);
	if (paramsBuffer) {
		paramsBuffer->_d()->metadata().planes()[0].bytesused = bytesused;
		ret |= neo->params_->queueBuffer(paramsBuffer);
	}
	FrameBuffer *statsBuffer = infoContext->getBuffer(BufferType::Stats);
	if (statsBuffer)
		ret |= neo->stats_->queueBuffer(statsBuffer);

	/* Queue ISP input buffers */
	FrameBuffer *image0Buffer = infoContext->getBuffer(BufferType::Image0);
	FrameBuffer *image1Buffer = infoContext->getBuffer(BufferType::Image1);
	if (image0Buffer)
		ret |= neo->input0_->queueBuffer(image0Buffer);
	if (image1Buffer) {
		if (mode_ == PipelineMode::HdrMerge)
			ret |= neo->input1_->queueBuffer(image1Buffer);
		else if (mode_ == PipelineMode::RgbIrDual)
			ret |= neo->input0_->queueBuffer(image1Buffer);
		else
			LOG(NxpNeoPipe, Error)
				<< "Unexpected image1 in mode " << static_cast<int>(mode_);
	}

	if (ret)
		LOG(NxpNeoPipe, Error) << "Failed to queue ISP buffers";
}

void NxpNeoCameraData::ipaMetadataReady(unsigned int id,
					ipa::nxpneo::IPACameraContext context,
					const ControlList &metadata)
{
	NxpNeoFrames::Info *info = frameInfos_.find(id);
	if (!info)
		return;

	Request *request = info->request_;
	request->_d()->metadata().merge(metadata);

	CameraContext _context = static_cast<CameraContext>(context);
	NxpNeoFrames::InfoContext *infoContext = info->getContext(_context);
	if (!infoContext)
		return;

	if (infoContext->metadataProcessed_)
		LOG(NxpNeoPipe, Error) << "Metadata already processed";
	infoContext->metadataProcessed_ = true;
	tryCompleteRequest(info);
}

void NxpNeoCameraData::ipaSetSensorControls([[maybe_unused]] unsigned int id,
					    const ControlList &sensorControls)
{
	if (delayedCtrls_)
		delayedCtrls_->push(sensorControls);
}

void NxpNeoCameraData::ipaSetLensControls(const ControlList &lensControls)
{
	CameraLens *lens = feCamera_->sensor()->focusLens();

	if (lens && lensControls.contains(V4L2_CID_FOCUS_ABSOLUTE)) {
		ControlValue const &focusValue = lensControls.get(V4L2_CID_FOCUS_ABSOLUTE);
		lens->setFocusPosition(focusValue.get<int32_t>());
	}
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerNxpNeo, "nxp/neo")

} /* namespace libcamera */
