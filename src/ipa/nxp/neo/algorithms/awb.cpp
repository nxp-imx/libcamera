/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 AGC/AEC mean-based control algorithm
 *     src/ipa/rkisp1/algorithms/awb.cpp
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * awb.cpp - AWB control algorithm
 * Copyright 2024 NXP
 */

#include "awb.h"

#include <algorithm>
#include <cmath>
#include <iomanip>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file awb.h
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class Awb
 * \brief A Grey world white balance correction algorithm
 */

LOG_DEFINE_CATEGORY(NxpNeoAwb)

/* Minimum mean value below which AWB can't operate. */
constexpr double kMeanMinThreshold = 2.0;

Awb::Awb()
	: rgbMode_(false)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Awb::configure(IPAContext &context,
		   const IPACameraSensorInfo &configInfo)
{
	(void)configInfo;

	context.activeState.awb.gains.manual.red = 1.0;
	context.activeState.awb.gains.manual.blue = 1.0;
	context.activeState.awb.gains.manual.green = 1.0;
	context.activeState.awb.gains.automatic.red = 1.0;
	context.activeState.awb.gains.automatic.blue = 1.0;
	context.activeState.awb.gains.automatic.green = 1.0;
	context.activeState.awb.autoEnabled = true;

	context.configuration.awb.enabled = true;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Awb::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &awb = context.activeState.awb;

	const auto &awbEnable = controls.get(controls::AwbEnable);
	if (awbEnable && *awbEnable != awb.autoEnabled) {
		awb.autoEnabled = *awbEnable;

		LOG(NxpNeoAwb, Debug)
			<< (*awbEnable ? "Enabling" : "Disabling") << " AWB";
	}

	const auto &colourGains = controls.get(controls::ColourGains);
	if (colourGains && !awb.autoEnabled) {
		awb.gains.manual.red = (*colourGains)[0];
		awb.gains.manual.blue = (*colourGains)[1];

		LOG(NxpNeoAwb, Debug)
			<< "Set colour gains to red: " << awb.gains.manual.red
			<< ", blue: " << awb.gains.manual.blue;
	}

	frameContext.awb.autoEnabled = awb.autoEnabled;

	if (!awb.autoEnabled) {
		frameContext.awb.gains.red = awb.gains.manual.red;
		frameContext.awb.gains.green = 1.0;
		frameContext.awb.gains.blue = awb.gains.manual.blue;
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Awb::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, neoisp_meta_params_s *params)
{
	(void)params;

	/*
	 * This is the latest time we can read the active state. This is the
	 * most up-to-date automatic values we can read.
	 */
	if (frameContext.awb.autoEnabled) {
		frameContext.awb.gains.red = context.activeState.awb.gains.automatic.red;
		frameContext.awb.gains.green = context.activeState.awb.gains.automatic.green;
		frameContext.awb.gains.blue = context.activeState.awb.gains.automatic.blue;
	}

	/* If we have already set the AWB measurement parameters, return. */
	if (frame > 0)
		return;
}

uint32_t Awb::estimateCCT(double red, double green, double blue)
{
	/* Convert the RGB values to CIE tristimulus values (XYZ) */
	double X = (-0.14282) * (red) + (1.54924) * (green) + (-0.95641) * (blue);
	double Y = (-0.32466) * (red) + (1.57837) * (green) + (-0.73191) * (blue);
	double Z = (-0.68202) * (red) + (0.77073) * (green) + (0.56332) * (blue);

	/* Calculate the normalized chromaticity values */
	double x = X / (X + Y + Z);
	double y = Y / (X + Y + Z);

	/* Calculate CCT */
	double n = (x - 0.3320) / (0.1858 - y);
	return 449 * n * n * n + 3525 * n * n + 6823.3 * n + 5520.33;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Awb::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const neoisp_meta_stats_s *stats,
		  ControlList &metadata)
{
	(void)stats;

	IPAActiveState &activeState = context.activeState;
	double greenMean;
	double redMean;
	double blueMean;

	/* \todo compute actual values from stats */
	greenMean = 1.0;
	redMean = 1.0;
	blueMean = 1.0;

	activeState.awb.temperatureK = estimateCCT(redMean, greenMean, blueMean);

	/*
	 * Estimate the red and blue gains to apply in a grey world. The green
	 * gain is hardcoded to 1.0. Avoid divisions by zero by clamping the
	 * divisor to a minimum value of 1.0.
	 */
	double redGain = greenMean / std::max(redMean, 1.0);
	double blueGain = greenMean / std::max(blueMean, 1.0);

	/* Filter the values to avoid oscillations. */
	double speed = 0.2;
	redGain = speed * redGain + (1 - speed) * activeState.awb.gains.automatic.red;
	blueGain = speed * blueGain + (1 - speed) * activeState.awb.gains.automatic.blue;

	activeState.awb.gains.automatic.red = redGain;
	activeState.awb.gains.automatic.blue = blueGain;
	activeState.awb.gains.automatic.green = 1.0;

	frameContext.awb.temperatureK = activeState.awb.temperatureK;

	metadata.set(controls::AwbEnable, frameContext.awb.autoEnabled);
	metadata.set(controls::ColourGains, {
			static_cast<float>(frameContext.awb.gains.red),
			static_cast<float>(frameContext.awb.gains.blue)
		});
	metadata.set(controls::ColourTemperature, frameContext.awb.temperatureK);

	LOG(NxpNeoAwb, Debug) << std::showpoint
		<< "Means [" << redMean << ", " << greenMean << ", " << blueMean
		<< "], gains [" << activeState.awb.gains.automatic.red << ", "
		<< activeState.awb.gains.automatic.green << ", "
		<< activeState.awb.gains.automatic.blue << "], temp "
		<< frameContext.awb.temperatureK << "K";
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
