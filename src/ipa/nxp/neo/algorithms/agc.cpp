/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 AGC/AEC mean-based control algorithm
 *     src/ipa/rkisp1/algorithms/agc.cpp
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * agc.cpp - AGC/AEC mean-based control algorithm
 * Copyright 2024 NXP
 */

#include "agc.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/histogram.h"

/**
 * \file agc.h
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::nxpneo::algorithms {

/**
 * \class Agc
 * \brief A mean-based auto-exposure algorithm
 */

LOG_DEFINE_CATEGORY(NxpNeoAgc)

/* Minimum limit for analogue gain value */
static constexpr double kMinAnalogueGain = 1.0;

/* \todo Honour the FrameDurationLimits control instead of hardcoding a limit */
static constexpr utils::Duration kMaxShutterSpeed = 60ms;

/* Number of frames to wait before calculating stats on minimum exposure */
static constexpr uint32_t kNumStartupFrames = 10;

Agc::Agc()
{
}

/**
 * \brief Configure the AGC given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Agc::configure(IPAContext &context, const IPACameraSensorInfo &configInfo)
{
	(void)configInfo;

	/* Configure the default exposure and gain. */
	context.activeState.agc.automatic.gain = context.configuration.sensor.minAnalogueGain;
	context.activeState.agc.automatic.exposure =
		10ms / context.configuration.sensor.lineDuration;
	context.activeState.agc.manual.gain = context.activeState.agc.automatic.gain;
	context.activeState.agc.manual.exposure = context.activeState.agc.automatic.exposure;
	context.activeState.agc.autoEnabled = true;

	context.configuration.agc.enabled = true;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &agc = context.activeState.agc;

	const auto &agcEnable = controls.get(controls::AeEnable);
	if (agcEnable && *agcEnable != agc.autoEnabled) {
		agc.autoEnabled = *agcEnable;

		LOG(NxpNeoAgc, Debug)
			<< (agc.autoEnabled ? "Enabling" : "Disabling")
			<< " AGC";
	}

	const auto &exposure = controls.get(controls::ExposureTime);
	if (exposure && !agc.autoEnabled) {
		agc.manual.exposure = *exposure * 1.0us
				    / context.configuration.sensor.lineDuration;

		LOG(NxpNeoAgc, Debug)
			<< "Set exposure to " << agc.manual.exposure;
	}

	const auto &gain = controls.get(controls::AnalogueGain);
	if (gain && !agc.autoEnabled) {
		agc.manual.gain = *gain;

		LOG(NxpNeoAgc, Debug) << "Set gain to " << agc.manual.gain;
	}

	frameContext.agc.autoEnabled = agc.autoEnabled;

	if (!frameContext.agc.autoEnabled) {
		frameContext.agc.exposure = agc.manual.exposure;
		frameContext.agc.gain = agc.manual.gain;
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Agc::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, neoisp_meta_params_s *params)
{
	(void)params;

	if (frameContext.agc.autoEnabled) {
		frameContext.agc.exposure = context.activeState.agc.automatic.exposure;
		frameContext.agc.gain = context.activeState.agc.automatic.gain;
	}

	if (frame > 0)
		return;
}

/**
 * \brief Apply a filter on the exposure value to limit the speed of changes
 * \param[in] exposureValue The target exposure from the AGC algorithm
 *
 * The speed of the filter is adaptive, and will produce the target quicker
 * during startup, or when the target exposure is within 20% of the most recent
 * filter output.
 *
 * \return The filtered exposure
 */
utils::Duration Agc::filterExposure(utils::Duration exposureValue)
{
	double speed = 0.2;

	/* Adapt instantly if we are in startup phase. */
	if (frameCount_ < kNumStartupFrames)
		speed = 1.0;

	/*
	 * If we are close to the desired result, go faster to avoid making
	 * multiple micro-adjustments.
	 * \todo Make this customisable?
	 */
	if (filteredExposure_ < 1.2 * exposureValue &&
	    filteredExposure_ > 0.8 * exposureValue)
		speed = sqrt(speed);

	filteredExposure_ = speed * exposureValue +
			    filteredExposure_ * (1.0 - speed);

	LOG(NxpNeoAgc, Debug) << "After filtering, exposure " << filteredExposure_;

	return filteredExposure_;
}

/**
 * \brief Estimate the new exposure and gain values
 * \param[inout] context The shared IPA Context
 * \param[in] frameContext The FrameContext for this frame
 * \param[in] yGain The gain calculated on the current brightness level
 * \param[in] iqMeanGain The gain calculated based on the relative luminance target
 */
void Agc::computeExposure(IPAContext &context, IPAFrameContext &frameContext,
			  double yGain, double iqMeanGain)
{
	IPASessionConfiguration &configuration = context.configuration;
	IPAActiveState &activeState = context.activeState;

	/* Get the effective exposure and gain applied on the sensor. */
	uint32_t exposure = frameContext.sensor.exposure;
	double analogueGain = frameContext.sensor.gain;

	/* Use the highest of the two gain estimates. */
	double evGain = std::max(yGain, iqMeanGain);

	utils::Duration minShutterSpeed = configuration.sensor.minShutterSpeed;
	utils::Duration maxShutterSpeed = std::min(configuration.sensor.maxShutterSpeed,
						   kMaxShutterSpeed);

	double minAnalogueGain = std::max(configuration.sensor.minAnalogueGain,
					  kMinAnalogueGain);
	double maxAnalogueGain = configuration.sensor.maxAnalogueGain;

	/* Consider within 1% of the target as correctly exposed. */
	if (utils::abs_diff(evGain, 1.0) < 0.01)
		return;

	/* extracted from Rpi::Agc::computeTargetExposure. */

	/* Calculate the shutter time in seconds. */
	utils::Duration currentShutter = exposure * configuration.sensor.lineDuration;

	/*
	 * Update the exposure value for the next computation using the values
	 * of exposure and gain really used by the sensor.
	 */
	utils::Duration effectiveExposureValue = currentShutter * analogueGain;

	LOG(NxpNeoAgc, Debug) << "Actual total exposure " << currentShutter * analogueGain
			      << " Shutter speed " << currentShutter
			      << " Gain " << analogueGain
			      << " Needed ev gain " << evGain;

	/*
	 * Calculate the current exposure value for the scene as the latest
	 * exposure value applied multiplied by the new estimated gain.
	 */
	utils::Duration exposureValue = effectiveExposureValue * evGain;

	/* Clamp the exposure value to the min and max authorized. */
	utils::Duration maxTotalExposure = maxShutterSpeed * maxAnalogueGain;
	exposureValue = std::min(exposureValue, maxTotalExposure);
	LOG(NxpNeoAgc, Debug) << "Target total exposure " << exposureValue
			      << ", maximum is " << maxTotalExposure;

	/*
	 * Divide the exposure value as new exposure and gain values.
	 * \todo estimate if we need to desaturate
	 */
	exposureValue = filterExposure(exposureValue);

	/*
	 * Push the shutter time up to the maximum first, and only then
	 * increase the gain.
	 */
	utils::Duration shutterTime = std::clamp<utils::Duration>(exposureValue / minAnalogueGain,
								  minShutterSpeed, maxShutterSpeed);
	double stepGain = std::clamp(exposureValue / shutterTime,
				     minAnalogueGain, maxAnalogueGain);
	LOG(NxpNeoAgc, Debug) << "Divided up shutter and gain are "
			      << shutterTime << " and "
			      << stepGain;

	/* Update the estimated exposure and gain. */
	activeState.agc.automatic.exposure = shutterTime / configuration.sensor.lineDuration;
	activeState.agc.automatic.gain = stepGain;
}

void Agc::fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
		       ControlList &metadata)
{
	utils::Duration exposureTime = context.configuration.sensor.lineDuration
				     * frameContext.sensor.exposure;
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());

	/* \todo Use VBlank value calculated from each frame exposure. */
	uint32_t vTotal = context.configuration.sensor.size.height
			+ context.configuration.sensor.defVBlank;
	utils::Duration frameDuration = context.configuration.sensor.lineDuration
				      * vTotal;
	metadata.set(controls::FrameDuration, frameDuration.get<std::micro>());
}

/**
 * \brief Process NxpNeo statistics, and run AGC operations
 * \param[in] context The shared IPA context
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The current frame context
 * \param[in] stats The NEO statistics and ISP results
 * \param[out] metadata Metadata for the frame, to be filled by the algorithm
 *
 * Identify the current image brightness, and use that to estimate the optimal
 * new exposure and gain for the scene.
 */
void Agc::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, const neoisp_meta_stats_s *stats,
		  ControlList &metadata)
{
	if (!stats) {
		fillMetadata(context, frameContext, metadata);
		return;
	}

	/* \todo compute actual values from stats */
	double yGain = 1.0;
	double iqMeanGain = 1.0;

	computeExposure(context, frameContext, yGain, iqMeanGain);
	frameCount_++;

	fillMetadata(context, frameContext, metadata);

	/*
	 * XXX: override exposure and gains with reasonable default values until
	 * functional AGC algorithm is available
	 * \todo remove when dynamically computed values are available
	 */
	IPASessionConfiguration &configuration = context.configuration;
	IPAActiveState &activeState = context.activeState;

	static constexpr float alpha = 0.5;
	utils::Duration _shutterTime =
		alpha * configuration.sensor.minShutterSpeed +
		(1 - alpha) * configuration.sensor.maxShutterSpeed;
	activeState.agc.automatic.exposure =
		_shutterTime / configuration.sensor.lineDuration;
	activeState.agc.automatic.gain =
		alpha * configuration.sensor.minAnalogueGain +
		(1 - alpha) * configuration.sensor.maxAnalogueGain;
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
