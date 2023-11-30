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
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPANxpNeo)

using namespace std::literals::chrono_literals;

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
		 ControlInfoMap *ipaControls) override;
	int start() override;
	void stop() override;

	int configure(const IPAConfigInfo &ipaConfig,
		      const std::map<uint32_t, IPAStream> &streamConfig,
		      ControlInfoMap *ipaControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void queueRequest(const uint32_t frame, const ControlList &controls) override;
	void fillParamsBuffer(const uint32_t frame, const uint32_t bufferId) override;
	void processStatsBuffer(const uint32_t frame, const uint32_t bufferId,
				const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateControls(const IPACameraSensorInfo &sensorInfo,
			    const ControlInfoMap &sensorControls,
			    ControlInfoMap *ipaControls);
	void setControls(unsigned int frame);

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, MappedFrameBuffer> mappedBuffers_;

	ControlInfoMap sensorControls_;

	/* revision-specific data */
	uint32_t hwRevision_;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Local parameter storage */
	struct IPAContext context_;
};

namespace {

/* List of controls handled by the NeoNxp IPA */
const ControlInfoMap::Map nxpneoControls{
	{ &controls::AeEnable, ControlInfo(false, true) },
	{ &controls::AwbEnable, ControlInfo(false, true) },
};

} /* namespace */

IPANxpNeo::IPANxpNeo()
	: context_({ {}, {}, { kMaxFrameContexts } })
{
}

std::string IPANxpNeo::logPrefix() const
{
	return "nxpneo";
}

int IPANxpNeo::init(const IPASettings &settings, unsigned int hwRevision,
		    const IPACameraSensorInfo &sensorInfo,
		    const ControlInfoMap &sensorControls,
		    ControlInfoMap *ipaControls)
{
	LOG(IPANxpNeo, Debug) << "Hardware revision is " << hwRevision;

	camHelper_ = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!camHelper_) {
		LOG(IPANxpNeo, Error)
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
		LOG(IPANxpNeo, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<libcamera::YamlObject> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;

	unsigned int version = (*data)["version"].get<uint32_t>(0);
	if (version != 1) {
		LOG(IPANxpNeo, Error)
			<< "Invalid tuning file version " << version;
		return -EINVAL;
	}

	if (!data->contains("algorithms")) {
		LOG(IPANxpNeo, Error)
			<< "Tuning file doesn't contain any algorithm";
		return -EINVAL;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret)
		return ret;

	/* Initialize controls. */
	updateControls(sensorInfo, sensorControls, ipaControls);

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
			 const std::map<uint32_t, IPAStream> &streamConfig,
			 ControlInfoMap *ipaControls)
{
	sensorControls_ = ipaConfig.sensorControls;

	/*
	 * \todo Some camera including OX03C10 do not support standard controls
	 * to setup exposure and gains - add camHelper support to handle sensor
	 * specifics here.
	 */
	const auto itExp = sensorControls_.find(V4L2_CID_EXPOSURE);
	int32_t minExposure = itExp->second.min().get<int32_t>();
	int32_t maxExposure = itExp->second.max().get<int32_t>();

	const auto itGain = sensorControls_.find(V4L2_CID_ANALOGUE_GAIN);
	int32_t minGain = itGain->second.min().get<int32_t>();
	int32_t maxGain = itGain->second.max().get<int32_t>();

	LOG(IPANxpNeo, Debug)
		<< "Exposure: [" << minExposure << ", " << maxExposure
		<< "], gain: [" << minGain << ", " << maxGain << "]";

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
	context_.configuration.sensor.lineDuration = info.minLineLength * 1.0s / info.pixelRate;

	/* Update the camera controls using the new sensor settings. */
	updateControls(info, sensorControls_, ipaControls);

	/*
	 * When the AGC computes the new exposure values for a frame, it needs
	 * to know the limits for shutter speed and analogue gain.
	 * As it depends on the sensor, update it with the controls.
	 *
	 * \todo take VBLANK into account for maximum shutter speed
	 */
	context_.configuration.sensor.minShutterSpeed =
		minExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.sensor.maxShutterSpeed =
		maxExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.sensor.minAnalogueGain = camHelper_->gain(minGain);
	context_.configuration.sensor.maxAnalogueGain = camHelper_->gain(maxGain);

	context_.configuration.raw = std::any_of(streamConfig.begin(), streamConfig.end(),
		[](auto &cfg) -> bool {
			PixelFormat pixelFormat{ cfg.second.pixelFormat };
			const PixelFormatInfo &format = PixelFormatInfo::info(pixelFormat);
			return format.colourEncoding == PixelFormatInfo::ColourEncodingRAW;
		});

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		/* Disable algorithms that don't support raw formats. */
		algo->disabled_ = context_.configuration.raw && !algo->supportsRaw_;
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
			LOG(IPANxpNeo, Fatal) << "Failed to mmap buffer: "
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

void IPANxpNeo::fillParamsBuffer(const uint32_t frame, const uint32_t bufferId)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	neoisp_meta_params_s *params =
		reinterpret_cast<neoisp_meta_params_s *>(
			mappedBuffers_.at(bufferId).planes()[0].data());

	/* Prepare parameters buffer. */
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

	/*
	 * In raw capture mode, the ISP is bypassed and no statistics buffer is
	 * provided.
	 */
	const neoisp_meta_stats_s *stats = nullptr;
	if (!context_.configuration.raw)
		stats = reinterpret_cast<neoisp_meta_stats_s *>(
			mappedBuffers_.at(bufferId).planes()[0].data());

	/*
	 * \todo Some camera including OX03C10 do not support standard controls
	 * to setup exposure and gains - add camHelper support to handle sensor
	 * specifics here.
	 */

	/*
	 * \todo Spurious frame loss at stream start() induces invalid
	 * control values reported by pipeline's DelayedControl object.
	 * Occurence is logged by pipeline, workaround the issue to avoid
	 * later assert condition failure.
	 */

	bool invalidControl = false;
	uint32_t exposure = 0;
	const ControlValue &exposureValue
			= sensorControls.get(V4L2_CID_EXPOSURE);
	if (exposureValue.type() != ControlTypeNone)
		exposure = camHelper_->gain(exposureValue.get<int32_t>());
	else
		invalidControl = true;

	uint32_t gain = 0;
	const ControlValue &gainValue
			= sensorControls.get(V4L2_CID_ANALOGUE_GAIN);
	if (gainValue.type() != ControlTypeNone)
		gain = camHelper_->gain(gainValue.get<int32_t>());
	else
		invalidControl = true;

	if (invalidControl)
		LOG(IPANxpNeo, Debug) << "Invalid sensor controls " << frame;

	frameContext.sensor.exposure = exposure;
	frameContext.sensor.gain = camHelper_->gain(gain);

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
	 * \todo Some camera including OX03C10 do not support standard controls
	 * to setup exposure and gains - add camHelper support to handle sensor
	 * specifics here.
	 */

	/*
	 * Compute exposure time limits from the V4L2_CID_EXPOSURE control
	 * limits and the line duration.
	 */
	double lineDuration = context_.configuration.sensor.lineDuration.get<std::micro>();
	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>() * lineDuration;
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>() * lineDuration;
	int32_t defExposure = v4l2Exposure.def().get<int32_t>() * lineDuration;
	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&controls::ExposureTime),
			std::forward_as_tuple(minExposure, maxExposure, defExposure));

	/* Compute the analogue gain limits. */
	const ControlInfo &v4l2Gain = sensorControls.find(V4L2_CID_ANALOGUE_GAIN)->second;
	float minGain = camHelper_->gain(v4l2Gain.min().get<int32_t>());
	float maxGain = camHelper_->gain(v4l2Gain.max().get<int32_t>());
	float defGain = camHelper_->gain(v4l2Gain.def().get<int32_t>());
	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&controls::AnalogueGain),
			std::forward_as_tuple(minGain, maxGain, defGain));

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

	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);
}

void IPANxpNeo::setControls(unsigned int frame)
{
	/*
	 * \todo The frame number is most likely wrong here, we need to take
	 * internal sensor delays and other timing parameters into account.
	 */

	IPAFrameContext &frameContext = context_.frameContexts.get(frame);
	uint32_t exposure = frameContext.agc.exposure;
	uint32_t gain = camHelper_->gainCode(frameContext.agc.gain);

	/*
	 * \todo Some camera including OX03C10 do not support standard controls
	 * to setup exposure and gains - add camHelper support to handle sensor
	 * specifics here.
	 */

	ControlList ctrls(sensorControls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain));

	setSensorControls.emit(frame, ctrls);
}

} // namespace ipa::nxpneo

/*
 * External IPA module interface
 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"PipelineHandlerNxpNeo",
	"nxp/neo",
};

IPAInterface *ipaCreate()
{
	return new ipa::nxpneo::IPANxpNeo();
}
}

} /* namespace libcamera */
