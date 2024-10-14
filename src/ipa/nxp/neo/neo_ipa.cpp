/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 Image Processing Algorithms
 *     src/ipa/rkisp1.cpp
 * Copyright (C) 2019, Google Inc.
 *
 * neo_ipa.cpp - NXP NEO Image Processing Algorithms
 * Copyright 2024 NXP
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
#include "nxp/cam_helper/camera_helper.h"

#include "ipa_context.h"

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

	int init(const IPASettings &settings, unsigned int hwRevision,
		 const IPACameraSensorInfo &sensorInfo,
		 const ControlInfoMap &sensorControls,
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
	void fillParamsBuffer(const uint32_t frame,
			      const uint32_t paramsBufferId,
			      const uint32_t rawBufferId) override;
	void processStatsBuffer(const uint32_t frame, const uint32_t bufferId,
				const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateControls(const IPACameraSensorInfo &sensorInfo,
			    const ControlInfoMap &sensorControls,
			    ControlInfoMap *ipaControls);
	void setControls(unsigned int frame);
	std::string controlListToString(const ControlList *ctrls) const;
	std::string logSensorParams(const unsigned int frame,
				    const ControlList *ctrlsApplied,
				    const ControlList *ctrlsToApply) const;

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, MappedFrameBuffer> mappedBuffers_;

	ControlInfoMap sensorControls_;

	/* revision-specific data */
	uint32_t hwRevision_;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraHelper> camHelper_;

	/* Local parameter storage */
	struct IPAContext context_;
};

namespace {

/* List of controls handled by the NeoNxp IPA */
const ControlInfoMap::Map nxpneoControls{
	{ &controls::AeEnable, ControlInfo(false, true) },
	{ &controls::AwbEnable, ControlInfo(false, true) },
	{ &controls::ColourGains, ControlInfo(0.0f, 32.0f) },
};

} /* namespace */

IPANxpNeo::IPANxpNeo()
	: context_({ {}, {}, { kMaxFrameContexts }, {} })
{
}

std::string IPANxpNeo::logPrefix() const
{
	return "nxpneo";
}

int IPANxpNeo::init(const IPASettings &settings, unsigned int hwRevision,
		    const IPACameraSensorInfo &sensorInfo,
		    const ControlInfoMap &sensorControls,
		    ControlInfoMap *ipaControls,
		    SensorConfig *sensorConfig)
{
	LOG(NxpNeoIPA, Debug) << "Hardware revision is " << hwRevision;

	camHelper_ = CameraHelperFactoryBase::create(settings.sensorModel);
	if (!camHelper_) {
		LOG(NxpNeoIPA, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	context_.configuration.sensor.lineDuration = sensorInfo.minLineLength
						   * 1.0s / sensorInfo.pixelRate;

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

	/* Initialize controls. */
	updateControls(sensorInfo, sensorControls, ipaControls);

	/* Initialize SensorConfig parameters */
	const CameraHelper::Attributes *attributes = camHelper_->attributes();
	const std::map<int32_t, std::pair<uint32_t, bool>> &camHelperDelayParams =
		attributes->delayedControlParams;

	std::map<int32_t, ipa::nxpneo::DelayedControlsParams> &ipaDelayParams =
		sensorConfig->delayedControlsParams;
	for (const auto &kv : camHelperDelayParams) {
		auto k = kv.first;
		auto v = kv.second;
		ipaDelayParams.emplace(std::piecewise_construct,
				       std::forward_as_tuple(k),
				       std::forward_as_tuple(v.first, v.second));
	}

	sensorConfig->embeddedTopLines = attributes->mdParams.topLines;
	sensorConfig->rgbIr = attributes->rgbIr;

	return 0;
}

int IPANxpNeo::start()
{
	setControls(0);

	return 0;
}

void IPANxpNeo::stop()
{
	context_.frameContexts.clear();
}

int IPANxpNeo::configure(const IPAConfigInfo &ipaConfig,
			 [[maybe_unused]] const std::map<uint32_t, IPAStream> &streamConfig,
			 ControlInfoMap *ipaControls)
{
	const IPACameraSensorInfo *sensorInfo = &ipaConfig.sensorInfo;

	CameraMode mode;
	mode.pixelRate = sensorInfo->pixelRate;
	mode.minLineLength = sensorInfo->minLineLength;
	mode.maxLineLength = sensorInfo->maxLineLength;
	mode.minFrameLength = sensorInfo->minFrameLength;
	mode.maxFrameLength = sensorInfo->maxFrameLength;
	camHelper_->setCameraMode(mode);

	sensorControls_ = ipaConfig.sensorControls;
	std::vector<double> vMinExposure, vMaxExposure, vDefExposure;
	camHelper_->controlInfoMapGetExposureRange(
		&sensorControls_, &vMinExposure, &vMaxExposure, &vDefExposure);

	std::vector<double> vMinGain, vMaxGain, vDefGain;
	camHelper_->controlInfoMapGetAnalogGainRange(
		&sensorControls_, &vMinGain, &vMaxGain, &vDefGain);

	LOG(NxpNeoIPA, Debug)
		<< "Exposure: [" << vMinExposure[0] << ", " << vMaxExposure[0]
		<< "], gain: [" << vMinGain[0] << ", " << vMaxGain[0] << "]";

	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	/* Set the hardware revision for the algorithms. */
	context_.configuration.hw.revision = hwRevision_;

	const IPACameraSensorInfo &info = ipaConfig.sensorInfo;
	const ControlInfo vBlank = sensorControls_.find(V4L2_CID_VBLANK)->second;
	context_.configuration.sensor.defVBlank = vBlank.def().get<int32_t>();
	context_.configuration.sensor.size = info.outputSize;
	context_.configuration.sensor.lineDuration = info.minLineLength * 1.0s /
						     info.pixelRate;

	/* Update the camera controls using the new sensor settings. */
	updateControls(info, sensorControls_, ipaControls);

	/*
	 * When the AGC computes the new exposure values for a frame, it needs
	 * to know the limits for shutter speed and analogue gain.
	 * As it depends on the sensor, update it with the controls.
	 *
	 * \todo take VBLANK into account for maximum shutter speed
	 */
	context_.configuration.sensor.minShutterSpeed = vMinExposure[0] * 1.0s;
	context_.configuration.sensor.maxShutterSpeed = vMaxExposure[0] * 1.0s;

	context_.configuration.sensor.minAnalogueGain = vMinGain[0];
	context_.configuration.sensor.maxAnalogueGain = vMaxGain[0];

	uint32_t bpp = ipaConfig.sensorInfo.bitsPerPixel;

	context_.configuration.sensor.bpp = bpp;

	/* Embedded metadata size computation */
	size_t bytepp;
	if (bpp <= 8)
		bytepp = sizeof(uint8_t);
	else
		bytepp = sizeof(uint16_t);
	uint32_t topLines = camHelper_->attributes()->mdParams.topLines;
	context_.configuration.sensor.metaDataSize =
		topLines * context_.configuration.sensor.size.width * bytepp;

	/* Active streams */
	std::vector<IPAStream> &streams = context_.configuration.streams;
	for (auto it = streamConfig.begin(); it != streamConfig.end(); it++)
		streams.push_back(it->second);

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

void IPANxpNeo::fillParamsBuffer(const uint32_t frame,
				 const uint32_t paramsBufferId,
				 const uint32_t rawBufferId)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	/*
	 * Metadata parsing is done when CameraHelper implementation for the
	 * sensor reports that some top lines are used for embedded data.
	 * A necessary condition for the top lines parsing to be possible is
	 * that the raw buffer was mapped in the IPA with mapBuffer() call.
	 * However, when a raw stream is active concurrently with a decoded
	 * stream, the raw buffers used by the pipeline are provided by
	 * the application instead of being internally allocated. Thus, raw
	 * buffers are not known in advance by the pipeline, so they can not
	 * be mapped in the IPA. In that case, embedded data parsing is not
	 * doable.
	 */

	const IPASessionConfiguration &sessionConfig = context_.configuration;

	ControlList &controls = frameContext.sensor.mdControls;
	controls = ControlList(md::controlIdMap);

	frameContext.sensor.metaDataValid = false;
	size_t metadataSize = sessionConfig.sensor.metaDataSize;
	if (metadataSize && mappedBuffers_.count(rawBufferId)) {
		uint8_t *metadata = mappedBuffers_.at(rawBufferId).planes()[0].data();
		Span<uint8_t> mdBuffer(metadata, metadataSize);
		int ret = camHelper_->parseEmbedded(mdBuffer, &controls);
		frameContext.sensor.metaDataValid = (ret == 0);
	}

	/* Prepare parameters buffer. */
	neoisp_meta_params_s *params =
		reinterpret_cast<neoisp_meta_params_s *>(
			mappedBuffers_.at(paramsBufferId).planes()[0].data());

	params->frame_id = 0;
	params->features_cfg = {};

	for (auto const &algo : algorithms())
		algo->prepare(context_, frame, frameContext, params);

	paramsBufferReady.emit(frame);
}

void IPANxpNeo::processStatsBuffer(const uint32_t frame, const uint32_t bufferId,
				   const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	const neoisp_meta_stats_s *stats;
	stats = reinterpret_cast<neoisp_meta_stats_s *>(
		mappedBuffers_.at(bufferId).planes()[0].data());

	ControlList &mdControls = frameContext.sensor.mdControls;

	if (!frameContext.sensor.metaDataValid) {
		mdControls = ControlList(md::controlIdMap);
		camHelper_->sensorControlsToMetaData(&sensorControls, &mdControls);
	}

	float exposure = 0.0f;
	if (mdControls.contains(md::Exposure.id())) {
		const ControlValue &exposureValue =
			mdControls.get(md::Exposure.id());
		Span<const float> exposuresSpan =
			exposureValue.get<Span<const float>>();
		exposure = exposuresSpan[0];
	} else {
		LOG(NxpNeoIPA, Warning) << "No exposure metadata";
	}

	double lineDuration =
		context_.configuration.sensor.lineDuration.get<std::ratio<1>>();
	double linesF = exposure / lineDuration;
	uint32_t linesInt = static_cast<uint32_t>(std::round(linesF));
	frameContext.sensor.exposure = linesInt;

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
		algo->process(context_, frame, frameContext, stats, metadata);
	}

	setControls(frame);

	metadataReady.emit(frame, metadata);
}

void IPANxpNeo::updateControls(const IPACameraSensorInfo &sensorInfo,
			       const ControlInfoMap &sensorControls,
			       ControlInfoMap *ipaControls)
{
	ControlInfoMap::Map ctrlMap = nxpneoControls;

	/*
	 * Compute exposure time limits from the exposure control limits and
	 * the line duration.
	 */
	std::vector<double> vMinExposure, vMaxExposure, vDefExposure;
	camHelper_->controlInfoMapGetExposureRange(
		&sensorControls, &vMinExposure, &vMaxExposure, &vDefExposure);
	/* ExposureTime range is in microseconds */
	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&controls::ExposureTime),
			std::forward_as_tuple(
				static_cast<int32_t>(vMinExposure[0] * 1.0e6f),
				static_cast<int32_t>(vMaxExposure[0] * 1.0e6f),
				static_cast<int32_t>(vDefExposure[0] * 1.0e6f)));

	/* Compute the analogue gain limits. */
	std::vector<double> vMinGain, vMaxGain, vDefGain;
	camHelper_->controlInfoMapGetAnalogGainRange(
		&sensorControls, &vMinGain, &vMaxGain, &vDefGain);

	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&controls::AnalogueGain),
			std::forward_as_tuple(
				static_cast<float>(vMinGain[0]),
				static_cast<float>(vMaxGain[0]),
				static_cast<float>(vDefGain[0])));

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

void IPANxpNeo::setControls(unsigned int frame)
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
	double lineDuration =
		context_.configuration.sensor.lineDuration.get<std::ratio<1>>();
	double exposure = frameContext.agc.exposure * lineDuration;
	if (frame)
		camHelper_->controlListSetAGC(&ctrls, exposure, frameContext.agc.gain);

	LOG(NxpNeoControlList, Debug)
		<< logSensorParams(frame, &frameContext.sensor.mdControls, &ctrls);

	setSensorControls.emit(frame, ctrls);
}

std::string IPANxpNeo::controlListToString(const ControlList *ctrls) const
{
	std::stringstream log;
	for (auto it = ctrls->begin(); it != ctrls->end(); ++it) {
		ControlValue value = it->second;
		log << it->first << ": val=" << value.toString() << "\n";
	}

	return log.str();
}

std::string IPANxpNeo::logSensorParams(const unsigned int frame,
				       const ControlList *ctrlsApplied,
				       const ControlList *ctrlsToApply) const
{
	std::stringstream log;

	log << "Sensor parameter status:\n"
	    << "--------------" << frame << "-----------\n"
	    << "Current sensor params:\n"
	    << controlListToString(ctrlsApplied)
	    << "------\n"
	    << "New sensor params to apply:\n"
	    << controlListToString(ctrlsToApply)
	    << "----------------------------\n";

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
