/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 Image Processing Algorithms
 *     src/ipa/rkisp1.cpp
 * Copyright (C) 2019, Google Inc.
 *
 * neo_ipa.cpp - NXP NEO Image Processing Algorithms
 * Copyright 2024-2025 NXP
 */

#include <algorithm>
#include <math.h>
#include <queue>
#include <sstream>
#include <stdint.h>
#include <string.h>

#include <linux/nxp_neoisp.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/request.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/nxpneo_ipa_interface.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/algorithm.h"

#include "ipa_context.h"
#include "neo_ipa_version.h"
#include "params.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoIPA)
LOG_DEFINE_CATEGORY(NxpNeoControlList)

using namespace std::literals::chrono_literals;
using namespace libcamera::nxp;

namespace ipa::nxpneo {

/* Maximum number of frame contexts to be held */
static constexpr uint32_t kMaxFrameContexts = 16;

class IPANxpNeo : public IPANxpNeoInterface, public Module
{
public:
	IPANxpNeo();

	int init(const IPASettings &settings, const InitParams &params,
		 ControlInfoMap *ipaControls,
		 SensorConfig *sensorConfig) override;
	int start() override;
	void stop() override;

	int configure(const IPAConfigInfo &ipaConfig,
		      const std::map<uint32_t, IPAStream> &streamConfig,
		      ControlInfoMap *ipaControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void queueRequest(const uint32_t frame, const ControlList &controls) override;
	void computeParams(const uint32_t frame, const IPAContextType context,
			   const std::map<uint32_t, uint32_t> &bufferIds) override;
	void processStats(const uint32_t frame, const IPAContextType context,
			  const std::map<uint32_t, uint32_t> &bufferIds,
			  const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateSensorConfig(const IPACameraSensorInfo &sensorInfo,
				const ControlInfoMap &sensorControls);
	void updateControls(const IPACameraSensorInfo &sensorInfo,
			    const ControlInfoMap &sensorControls,
			    ControlInfoMap *ipaControls);
	void setControls(unsigned int frame, IPAContextType context);
	std::string controlListToString(const ControlList *ctrls) const;
	std::string logSensorParams(const unsigned int frame,
				    const ControlList *ctrlsApplied,
				    const ControlList *ctrlsToApply) const;

	static const std::map<const IPAModeType, SensorStreamModes> kSensorStreamModeMap;
	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, MappedFrameBuffer> mappedBuffers_;

	ControlInfoMap sensorControls_;
	ControlList sensorControlList_;

	/* Local parameter storage */
	struct IPAContext context_;
};

const std::map<const IPAModeType, SensorStreamModes> IPANxpNeo::kSensorStreamModeMap = {
	{ IPAModeTypeStandard, SensorStreamStandard },
	{ IPAModeTypeHdrMerge, SensorStreamHdr },
	{ IPAModeTypeRgbIr, SensorStreamRgbIr },
	{ IPAModeTypeRgbIrDual, SensorStreamDualContext },
};

namespace {

/* List of controls handled by the NeoNxp IPA */
const ControlInfoMap::Map nxpneoControls{
	{ &controls::AeEnable, ControlInfo(false, true) },
	{ &controls::AwbEnable, ControlInfo(false, true) },
	{ &controls::ColourGains, ControlInfo(0.0f, 32.0f) },
	{ &controls::Gamma, ControlInfo(0.5f, 10.0f, 2.2f) },
};

} /* namespace */

IPANxpNeo::IPANxpNeo()
	: context_(kMaxFrameContexts)
{
}

std::string IPANxpNeo::logPrefix() const
{
	return "nxpneo";
}

int IPANxpNeo::init(const IPASettings &settings, const InitParams &params,
		    ControlInfoMap *ipaControls,
		    SensorConfig *sensorConfig)
{
	LOG(NxpNeoIPA, Info) << "IPANxpNeo NXPNEO_IPA_" << IpaVersion::version();

	LOG(NxpNeoIPA, Debug) << "Hardware revision is " << params.hwRevision;
	LOG(NxpNeoIPA, Debug) << "Sensor entity: " << params.sensorEntity;
	LOG(NxpNeoIPA, Debug) << "API version is " << params.apiVersion;

	/* Set the hardware-related block for the algorithms. */
	context_.hw.apiVersion = params.apiVersion;
	context_.hw.hwRevision = params.hwRevision;

	context_.camHelper = CameraHelperFactoryBase::create(settings.sensorModel);
	if (!context_.camHelper) {
		LOG(NxpNeoIPA, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	/* Load the tuning data file. */
	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(NxpNeoIPA, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<libcamera::YamlObject> data = YamlParser::parse(file);
	if (!data) {
		LOG(NxpNeoIPA, Error) << "Failed to parse configuration file";
		return -EINVAL;
	}

	unsigned int version = (*data)["version"].get<uint32_t>(0);
	if (version != 1) {
		LOG(NxpNeoIPA, Error)
			<< "Invalid tuning file version " << version;
		return -EINVAL;
	}

	if (!data->contains("algorithms")) {
		LOG(NxpNeoIPA, Error)
			<< "Tuning file doesn't contain any algorithm";
		return -EINVAL;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret) {
		LOG(NxpNeoIPA, Error) << "Failed to create algorithms";
		return ret;
	}

	context_.configuration = {};
	sensorControlList_ = params.sensorControlList;

	/* Initialize the IPA context. */
	updateSensorConfig(params.sensorInfo, params.sensorControls);
	/* Initialize controls. */
	updateControls(params.sensorInfo, params.sensorControls, ipaControls);

	/* Initialize SensorConfig parameters */
	const CameraHelper::Attributes *attributes = context_.camHelper->attributes();
	const std::map<int32_t, std::pair<uint32_t, bool>> &camHelperDelayParams =
		attributes->delayedControlParams;

	ControlList ctrls(params.sensorControls);
	auto idMap = ctrls.idMap();
	std::map<int32_t, ipa::nxpneo::DelayedControlsParams> &ipaDelayParams =
		sensorConfig->delayedControlsParams;
	for (const auto &kv : camHelperDelayParams) {
		auto k = kv.first;
		auto v = kv.second;
		if (idMap->find(kv.first) != idMap->end())
			ipaDelayParams.emplace(std::piecewise_construct,
					       std::forward_as_tuple(k),
					       std::forward_as_tuple(v.first, v.second));
		else
			LOG(NxpNeoIPA, Warning)
				<< "The sensor control list doesn't support the control ID "
				<< utils::hex(kv.first);
	}

	sensorConfig->embeddedTopLines = attributes->mdParams.topLines;
	sensorConfig->rgbIr = attributes->rgbIr;

	/* Set the camera helper with sensor control values. */
	context_.camHelper->setControls(&params.sensorControlList);

	return 0;
}

int IPANxpNeo::start()
{
	const std::array<IPAContextType, 2> allContexts = { IPAContextTypeRgb, IPAContextTypeIr };
	for (const auto &context : allContexts)
		setControls(0, context);

	return 0;
}

void IPANxpNeo::stop()
{
	context_.frameContexts.clear();
}

int IPANxpNeo::configure(const IPAConfigInfo &ipaConfig,
			 const std::map<uint32_t, IPAStream> &streamConfig,
			 ControlInfoMap *ipaControls)
{
	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	context_.configuration.pipelineMode = ipaConfig.mode;

	const IPACameraSensorInfo &info = ipaConfig.sensorInfo;
	sensorControlList_ = ipaConfig.sensorControlList;

	/* Update the IPA context using the new sensor settings. */
	updateSensorConfig(info, ipaConfig.sensorControls);
	/* Update the camera controls using the new sensor settings. */
	updateControls(info, ipaConfig.sensorControls, ipaControls);

	uint32_t bpp0 = ipaConfig.sensorInfo.bitsPerPixel;
	uint32_t bpp1 = ipaConfig.bitsPerPixelAuxiliary;
	context_.configuration.sensor.bpps = { bpp0, bpp1 };

	/* Active streams */
	std::map<IPAStreamType, IPAStream> &streams = context_.configuration.streams;
	for (const auto &[streamType, config] : streamConfig)
		streams[static_cast<IPAStreamType>(streamType)] = config;

	context_.configuration.colorSpace = ipaConfig.colorSpace;

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		if (algo->disabled_)
			continue;

		int ret = algo->configure(context_, info);
		if (ret)
			return ret;
	}

	return 0;
}

void IPANxpNeo::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto elem = buffers_.emplace(std::piecewise_construct,
					     std::forward_as_tuple(buffer.id),
					     std::forward_as_tuple(buffer.planes));
		const FrameBuffer &fb = elem.first->second;

		MappedFrameBuffer mappedBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite);
		if (!mappedBuffer.isValid()) {
			LOG(NxpNeoIPA, Fatal) << "Failed to mmap buffer: "
					      << strerror(mappedBuffer.error());
		}

		mappedBuffers_.emplace(buffer.id, std::move(mappedBuffer));
	}
}

void IPANxpNeo::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		const auto fb = buffers_.find(id);
		if (fb == buffers_.end())
			continue;

		mappedBuffers_.erase(id);
		buffers_.erase(id);
	}
}

void IPANxpNeo::queueRequest(const uint32_t frame, const ControlList &controls)
{
	IPAFrameContext &frameContext = context_.frameContexts.alloc(frame);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());
		if (algo->disabled_)
			continue;
		algo->queueRequest(context_, frame, frameContext, controls);
	}
}

void IPANxpNeo::computeParams(const uint32_t frame, const IPAContextType context,
			      const std::map<uint32_t, uint32_t> &bufferIds)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	/*
	 * Metadata parsing is done either from image pixel data top lines, or
	 * from a separate camera stream in a dedicated buffer.
	 * A necessary condition for the pixel data top lines parsing to be
	 * possible is that the raw buffer has been mapped in the IPA beforehand
	 * with the mapBuffer() call.
	 * However, when a raw stream is active concurrently with a decoded
	 * stream, the raw buffers used by the pipeline are provided by
	 * the application instead of being internally allocated. Thus, raw
	 * buffers are not known in advance by the pipeline, so they can not
	 * be mapped in the IPA. In that case, embedded data parsing is not
	 * doable.
	 */

	ControlList &controls = frameContext.sensor.mdControls;
	controls = ControlList(md::controlIdMap);
	frameContext.sensor.metaDataValid = false;

	uint8_t *metaData = nullptr;
	size_t metaSize = 0;

	/*
	 * Look for metadata availability, either from the camera embedded data
	 * stream or from the pixel data top lines.
	 */
	auto eDataIt = bufferIds.find(IPABufferTypeEData);
	unsigned int eDataBufferId =
		eDataIt != bufferIds.end() ? eDataIt->second : 0;
	if (eDataBufferId && mappedBuffers_.count(eDataBufferId)) {
		const MappedBuffer::Plane &plane =
			mappedBuffers_.at(eDataBufferId).planes()[0];
		metaData = plane.data();
		metaSize = plane.size_bytes();
	} else {
		auto input0It = bufferIds.find(IPABufferTypeImage0);
		unsigned int rawBufferId =
			input0It != bufferIds.end() ? input0It->second : 0;
		if (rawBufferId && mappedBuffers_.count(rawBufferId)) {
			const MappedBuffer::Plane &plane =
				mappedBuffers_.at(rawBufferId).planes()[0];
			metaData = plane.data();
			uint32_t topLines =
				context_.camHelper->attributes()->mdParams.topLines;
			std::array<uint32_t, 2> &bpps =
				context_.configuration.sensor.bpps;
			size_t bytepp =
				bpps[0] <= 8 ? sizeof(uint8_t) : sizeof(uint16_t);
			unsigned int width =
				context_.configuration.sensor.size.width;
			metaSize = topLines * width * bytepp;
		}
	}

	if (metaSize) {
		Span<uint8_t> mdBuffer(metaData, metaSize);
		if (!context_.camHelper->parseEmbedded(mdBuffer, &controls))
			frameContext.sensor.metaDataValid = true;
	}

	/* Prepare parameters buffer. */
	auto paramsIter = bufferIds.find(IPABufferTypeParams);
	unsigned int paramsBufferId =
		paramsIter != bufferIds.end() ? paramsIter->second : 0;
	ASSERT(mappedBuffers_.count(paramsBufferId));

	NxpNeoParams params(context_.hw.apiVersion,
			    mappedBuffers_.at(paramsBufferId).planes()[0]);

	for (auto const &algo : algorithms())
		algo->prepare(context_, frame, frameContext, &params);

	paramsComputed.emit(frame, context, params.size());
}

void IPANxpNeo::processStats(const uint32_t frame, const IPAContextType context,
			     const std::map<uint32_t, uint32_t> &bufferIds,
			     const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	auto statsIter = bufferIds.find(IPABufferTypeStats);
	unsigned int statsBufferId =
		statsIter != bufferIds.end() ? statsIter->second : 0;
	ASSERT(mappedBuffers_.count(statsBufferId));
	const NxpNeoStats stats(context_.hw.apiVersion,
				mappedBuffers_.at(statsBufferId).planes()[0]);

	ControlList &mdControls = frameContext.sensor.mdControls;

	if (!frameContext.sensor.metaDataValid) {
		mdControls = ControlList(md::controlIdMap);
		context_.camHelper->sensorControlsToMetaData(&sensorControls, &mdControls);
	}

	Duration exposure;
	if (mdControls.contains(md::Exposure.id())) {
		const ControlValue &exposureValue =
			mdControls.get(md::Exposure.id());
		Span<const float> exposuresSpan =
			exposureValue.get<Span<const float>>();
		exposure = exposuresSpan[0] * 1.0s;
	} else {
		LOG(NxpNeoIPA, Warning) << "No exposure metadata";
		exposure = context_.configuration.sensor.minExposureTime;
	}

	frameContext.sensor.exposure = context_.camHelper->exposureLines(
		exposure,
		context_.configuration.sensor.lineDuration);

	float aGain = 1.0f;
	if (mdControls.contains(md::AnalogueGain.id())) {
		const ControlValue &aGainValue =
			mdControls.get(md::AnalogueGain.id());
		Span<const float> aGainsSpan =
			aGainValue.get<Span<const float>>();
		aGain = aGainsSpan[0];
	} else {
		LOG(NxpNeoIPA, Warning) << "No analog gain metadata";
	}

	float dGain = 1.0f;
	if (mdControls.contains(md::DigitalGain.id())) {
		const ControlValue &dGainValue =
			mdControls.get(md::DigitalGain.id());
		Span<const float> dGainsSpan =
			dGainValue.get<Span<const float>>();
		dGain = dGainsSpan[0];
	} else {
		LOG(NxpNeoIPA, Warning) << "No digital gain metadata";
	}

	frameContext.sensor.gain = aGain * dGain;

	ControlList metadata(controls::controls);
	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());
		if (algo->disabled_)
			continue;
		algo->process(context_, frame, frameContext, &stats, metadata);
	}

	/*
	 * \todo Create IR-specific controls and have relevant algorithms to
	 *       use them during RGBIr context processing. For now just clear
	 *       the RGBIr context metadata to avoid merge conflict of the 2
	 *       contexts metadata being populated with the same controls.
	 */
	if (context == IPAContextTypeIr)
		metadata.clear();

	setControls(frame, context);
	metadataReady.emit(frame, context, metadata);
}

void IPANxpNeo::updateSensorConfig(const IPACameraSensorInfo &sensorInfo,
				   const ControlInfoMap &sensorControls)
{
	CameraMode cameraMode;
	cameraMode.pixelRate = sensorInfo.pixelRate;
	cameraMode.bitdepth = sensorInfo.bitsPerPixel;
	cameraMode.width = sensorInfo.outputSize.width;
	cameraMode.height = sensorInfo.outputSize.height;
	cameraMode.hblank = sensorControlList_.get(V4L2_CID_HBLANK).get<int32_t>();
	cameraMode.vblank = sensorControlList_.get(V4L2_CID_VBLANK).get<int32_t>();
	auto iter = kSensorStreamModeMap.find(context_.configuration.pipelineMode);
	if (iter != kSensorStreamModeMap.end()) {
		cameraMode.streamMode = iter->second;
	} else {
		cameraMode.streamMode = SensorStreamStandard;
		LOG(NxpNeoIPA, Warning)
			<< "No sensor stream mode found for pipeline mode: "
			<< context_.configuration.pipelineMode
			<< " - Default mode is used: " << cameraMode.streamMode;
	}
	context_.camHelper->setCameraMode(cameraMode);

	sensorControls_ = sensorControls;

	/*
	 * Compute exposure time limits from the exposure control limits and
	 * the line duration.
	 */
	std::vector<Duration> vMinExposure, vMaxExposure, vDefExposure;
	context_.camHelper->controlInfoMapGetExposureRange(
		&sensorControls, &vMinExposure, &vMaxExposure, &vDefExposure);

	/* Compute the analogue gain limits. */
	std::vector<double> vMinGain, vMaxGain, vDefGain;
	context_.camHelper->controlInfoMapGetAnalogGainRange(
		&sensorControls, &vMinGain, &vMaxGain, &vDefGain);

 	const ControlInfo &v4l2VBlank = sensorControls.find(V4L2_CID_VBLANK)->second;

	LOG(NxpNeoIPA, Debug)
		<< "Exposure: [" << vMinExposure[0] << ", " << vMaxExposure[0]
		<< "], gain: [" << vMinGain[0] << ", " << vMaxGain[0] << "]";

	/*
	 * When the AGC computes the new exposure values for a frame, it needs
	 * to know the limits for exposure time and analogue gain.
	 * As it depends on the sensor, update it with the controls.
	 *
	 * \todo take VBLANK into account for maximum exposure time
	 */
	context_.configuration.sensor.minExposureTime = vMinExposure[0];
	context_.configuration.sensor.maxExposureTime = vMaxExposure[0];
	context_.configuration.sensor.defExposureTime = vDefExposure[0];

	context_.configuration.sensor.minAnalogueGain = vMinGain[0];
	context_.configuration.sensor.maxAnalogueGain = vMaxGain[0];
	context_.configuration.sensor.defAnalogueGain = vDefGain[0];

	/* Update IPA context with sensor vblank, output size and line duration. */
	context_.configuration.sensor.defVBlank = v4l2VBlank.def().get<int32_t>();
	context_.configuration.sensor.size = sensorInfo.outputSize;
	context_.configuration.sensor.lineDuration = context_.camHelper->hblankToLineLength(
		cameraMode.hblank);
}

void IPANxpNeo::updateControls(const IPACameraSensorInfo &sensorInfo,
			       const ControlInfoMap &sensorControls,
			       ControlInfoMap *ipaControls)
{
	ControlInfoMap::Map ctrlMap = nxpneoControls;
	auto &sensorConfig = context_.configuration.sensor;

	/* ExposureTime range is in microseconds */
	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&controls::ExposureTime),
			std::forward_as_tuple(
				static_cast<int32_t>(sensorConfig.minExposureTime / 1.0us),
				static_cast<int32_t>(sensorConfig.maxExposureTime / 1.0us),
				static_cast<int32_t>(sensorConfig.defExposureTime / 1.0us)));

	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&controls::AnalogueGain),
			std::forward_as_tuple(
				static_cast<float>(sensorConfig.minAnalogueGain),
				static_cast<float>(sensorConfig.maxAnalogueGain),
				static_cast<float>(sensorConfig.defAnalogueGain)));

	/*
	 * Compute the frame duration limits.
	 *
	 * The frame length is computed assuming a fixed line length combined
	 * with the vertical frame sizes.
	 */
	const ControlInfo &v4l2HBlank = sensorControls.find(V4L2_CID_HBLANK)->second;
	uint32_t hblank = v4l2HBlank.def().get<int32_t>();
	uint32_t lineLength = sensorInfo.outputSize.width + hblank;

	const ControlInfo &v4l2VBlank = sensorControls.find(V4L2_CID_VBLANK)->second;
	std::array<uint32_t, 3> frameHeights{
		v4l2VBlank.min().get<int32_t>() + sensorInfo.outputSize.height,
		v4l2VBlank.max().get<int32_t>() + sensorInfo.outputSize.height,
		v4l2VBlank.def().get<int32_t>() + sensorInfo.outputSize.height,
	};

	std::array<int64_t, 3> frameDurations;
	for (unsigned int i = 0; i < frameHeights.size(); ++i) {
		uint64_t frameSize = lineLength * frameHeights[i];
		frameDurations[i] = frameSize / (sensorInfo.pixelRate / 1000000U);
	}

	ctrlMap[&controls::FrameDurationLimits] = ControlInfo(frameDurations[0],
							      frameDurations[1],
							      frameDurations[2]);

	ctrlMap.merge(context_.ctrlMap);
	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);
}

void IPANxpNeo::setControls(unsigned int frame, IPAContextType context)
{
	/*
	 * \todo The frame number is most likely wrong here, we need to take
	 * internal sensor delays and other timing parameters into account.
	 */

	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	ControlList ctrls(sensorControls_);

	/*
	 * Skip control setting for frame 0 for which the frame context
	 * doesn't have a relevant configuration for the exposure and the gain.
	 * Indeed the frame context is not initialized at startup.
	 *
	 * This workaround prevents some frames from flashing at startup.
	 * This effect can be addressed later by configuring some startup
	 * frames to be hidden.
	 */
	Duration exposure = context_.camHelper->exposure(frameContext.agc.exposure,
							 context_.configuration.sensor.lineDuration);

	if (frame)
		context_.camHelper->controlListSetAGC(&ctrls, exposure, frameContext.agc.gain);

	LOG(NxpNeoControlList, Debug)
		<< logSensorParams(frame, &frameContext.sensor.mdControls, &ctrls);

	setSensorControls.emit(frame, context, ctrls);
}

std::string IPANxpNeo::controlListToString(const ControlList *ctrls) const
{
	std::stringstream log;
	for (auto it = ctrls->begin(); it != ctrls->end(); ++it) {
		ControlValue value = it->second;
		if (it != ctrls->begin())
			log << "\n";
		log << it->first << ": val=" << value.toString();
	}

	return log.str();
}

std::string IPANxpNeo::logSensorParams(const unsigned int frame,
				       const ControlList *ctrlsApplied,
				       const ControlList *ctrlsToApply) const
{
	std::stringstream log;

	log << "\n--- frame [" << frame << "] meta data:\n"
	    << controlListToString(ctrlsApplied)
	    << "\nupdate:\n"
	    << controlListToString(ctrlsToApply);

	return log.str();
}

} // namespace ipa::nxpneo

/*
 * External IPA module interface
 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"nxp/neo",
	"nxp/neo",
};

IPAInterface *ipaCreate()
{
	return new ipa::nxpneo::IPANxpNeo();
}
}

} /* namespace libcamera */
