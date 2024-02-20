/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 AGC/AEC mean-based control algorithm
 *     src/ipa/rkisp1/algorithms/agc.cpp
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * Based on IPU3 AGC/AEC mean-based control algorithm
 *     src/ipa/ipu3/algorithms/agc.cpp
 * Copyright (C) 2021, Ideas On Board
 *
 * agc.cpp - AGC/AEC mean-based control algorithm
 * Copyright 2024 NXP
 */

#include "agc.h"
#include "nxp-neoisp-enums.h"

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

LOG_DEFINE_CATEGORY(NxpNeoAlgoAgc)

/* Histogram configuration: This value is used to disable ROI0 */
#define AGC_ROI_INVALID_IMAGE_GEOMETRY 65535

/* Histogram assignment to RGGB channels */
#define AGC_HIST_CFG_RED NEO_HIST0_ID
#define AGC_HIST_CFG_GREEN NEO_HIST1_ID
#define AGC_HIST_CFG_BLUE NEO_HIST2_ID
#define AGC_HIST_MEM_RED NEO_HIST0_OFFSET
#define AGC_HIST_MEM_GREEN NEO_HIST1_OFFSET
#define AGC_HIST_MEM_BLUE NEO_HIST2_OFFSET

/*
 * Scaling (gain) factor for the histogram bin determination.
 * The value specified is in u8.16 format.
 *
 * The default scaling value is calculated with a default 20bits range.
 * Indeed the expected bit range to reach at the HDR merge unit (prior
 * to the STAT unit) is 20bits range.
 *
 * defaultScaleValue = maxBins * 2^16 / 2^20
 *
 */
#define AGC_HIST_SCALE_DEFAULT ((NEO_HIST_BIN_SIZE << 16) >> 20)

Agc::Agc()
{
}

/**
 * \brief Initialise the AGC algorithm from tuning files
 * \param[in] context The shared IPA context
 * \param[in] tuningData The YamlObject containing Agc tuning data
 *
 * This function calls the base class' tuningData parsers to discover which
 * control values are supported.
 *
 * \return 0 on success or errors from the base class
 */
int Agc::init(IPAContext &context, const YamlObject &tuningData)
{
	int ret;

	ret = parseTuningData(tuningData);
	if (ret)
		return ret;

	context.ctrlMap.merge(controls());

	return 0;
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
	/* Configure the default exposure and gain. */
	context.activeState.agc.automatic.gain = context.configuration.sensor.minAnalogueGain;
	context.activeState.agc.automatic.exposure =
		10ms / context.configuration.sensor.lineDuration;
	context.activeState.agc.manual.gain = context.activeState.agc.automatic.gain;
	context.activeState.agc.manual.exposure = context.activeState.agc.automatic.exposure;
	context.activeState.agc.autoEnabled = true;

	/* ROI set to full image size */
	context.configuration.agc.roi.xpos = 0;
	context.configuration.agc.roi.ypos = 0;
	context.configuration.agc.roi.width = configInfo.outputSize.width;
	context.configuration.agc.roi.height = configInfo.outputSize.height;

	context.activeState.agc.constraintMode = constraintModes().begin()->first;
	context.activeState.agc.exposureMode = exposureModeHelpers().begin()->first;

	/* \todo Run this again when FrameDurationLimits is passed in */
	setLimits(context.configuration.sensor.minShutterSpeed,
		  context.configuration.sensor.maxShutterSpeed,
		  context.configuration.sensor.minAnalogueGain,
		  context.configuration.sensor.maxAnalogueGain);
	resetFrameCount();

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

		LOG(NxpNeoAlgoAgc, Debug)
			<< (agc.autoEnabled ? "Enabling" : "Disabling")
			<< " AGC";
	}

	const auto &exposure = controls.get(controls::ExposureTime);
	if (exposure && !agc.autoEnabled) {
		agc.manual.exposure = *exposure * 1.0us
				    / context.configuration.sensor.lineDuration;

		LOG(NxpNeoAlgoAgc, Debug)
			<< "Set exposure to " << agc.manual.exposure;
	}

	const auto &gain = controls.get(controls::AnalogueGain);
	if (gain && !agc.autoEnabled) {
		agc.manual.gain = *gain;

		LOG(NxpNeoAlgoAgc, Debug) << "Set gain to " << agc.manual.gain;
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
	if (frameContext.agc.autoEnabled) {
		frameContext.agc.exposure = context.activeState.agc.automatic.exposure;
		frameContext.agc.gain = context.activeState.agc.automatic.gain;
	}

	if (frame > 0)
		return;

	/* Configure histograms */
	/* Foreground ROI disabled (> Image geometry means invalid ROI) */
	params->regs.stat.roi0.xpos = AGC_ROI_INVALID_IMAGE_GEOMETRY;
	params->regs.stat.roi0.ypos = AGC_ROI_INVALID_IMAGE_GEOMETRY;
	params->regs.stat.roi0.width = AGC_ROI_INVALID_IMAGE_GEOMETRY;
	params->regs.stat.roi0.height = AGC_ROI_INVALID_IMAGE_GEOMETRY;
	/* Background ROI: set to full image */
	params->regs.stat.roi1 = context.configuration.agc.roi;

	/* Histogram control */
	/* HIST for Red */
	neoisp_stat_hist_cfg_s *hist_red = &params->regs.stat.hists[AGC_HIST_CFG_RED];
	hist_red->hist_ctrl_offset = 0;
	hist_red->hist_ctrl_channel = NEO_HIST_CHANNEL_R;
	hist_red->hist_ctrl_pattern = 0;
	hist_red->hist_ctrl_dir_input1_dif = 0;
	hist_red->hist_ctrl_lin_input1_log = 0;
	hist_red->hist_scale_scale = AGC_HIST_SCALE_DEFAULT;
	/* HIST for Gr+Gb */
	neoisp_stat_hist_cfg_s *hist_green = &params->regs.stat.hists[AGC_HIST_CFG_GREEN];
	hist_green->hist_ctrl_offset = 0;
	hist_green->hist_ctrl_channel = NEO_HIST_CHANNEL_GR | NEO_HIST_CHANNEL_GB;
	hist_green->hist_ctrl_pattern = 0;
	hist_green->hist_ctrl_dir_input1_dif = 0;
	hist_green->hist_ctrl_lin_input1_log = 0;
	hist_green->hist_scale_scale = AGC_HIST_SCALE_DEFAULT;
	/* HIST for Blue */
	neoisp_stat_hist_cfg_s *hist_blue = &params->regs.stat.hists[AGC_HIST_CFG_BLUE];
	hist_blue->hist_ctrl_offset = 0;
	hist_blue->hist_ctrl_channel = NEO_HIST_CHANNEL_B;
	hist_blue->hist_ctrl_pattern = 0;
	hist_blue->hist_ctrl_dir_input1_dif = 0;
	hist_blue->hist_ctrl_lin_input1_log = 0;
	hist_blue->hist_scale_scale = AGC_HIST_SCALE_DEFAULT;

	/* Enable the STAT unit parameter update */
	params->features_cfg.stat_cfg = 1;
}

void Agc::fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
		       ControlList &metadata)
{
	utils::Duration exposureTime = context.configuration.sensor.lineDuration *
				       frameContext.sensor.exposure;
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());

	/* \todo Use VBlank value calculated from each frame exposure. */
	uint32_t vTotal = context.configuration.sensor.size.height +
			  context.configuration.sensor.defVBlank;
	utils::Duration frameDuration = context.configuration.sensor.lineDuration *
					vTotal;
	metadata.set(controls::FrameDuration, frameDuration.get<std::micro>());
}

/**
 * \brief Estimate the relative luminance of the frame with a given gain
 * \param[in] gain The gain to apply in estimating luminance
 *
 * This function estimates the average relative luminance of the frame that
 * would be output by the sensor if an additional \a gain was applied.
 *
 * The estimation is based on the AWB statistics for the current frame. Red,
 * green and blue averages for all cells are first multiplied by the gain, and
 * then saturated to approximate the sensor behaviour at high brightness
 * values. The approximation is quite rough, as it doesn't take into account
 * non-linearities when approaching saturation.
 *
 * The relative luminance (Y) is computed from the linear RGB components using
 * the Rec. 601 formula. The values are normalized to the [0.0, 1.0] range,
 * where 1.0 corresponds to a theoretical perfect reflector of 100% reference
 * white.
 *
 * More detailed information can be found in:
 * https://en.wikipedia.org/wiki/Relative_luminance
 *
 * \return The relative luminance
 */
double Agc::estimateLuminance(double gain) const
{
	double redSum = 0, greenSum = 0, blueSum = 0;
	double redMean = 0, greenMean = 0, blueMean = 0;
	uint32_t redPixelsCount = 0, greenPixelsCount = 0, bluePixelsCount = 0;

	for (unsigned int i = 0; i < rgbTriples_.size(); i++) {
		/* Accumulate weighted bin */
		redSum += std::get<0>(rgbTriples_[i]) * gain * i;
		greenSum += std::get<1>(rgbTriples_[i]) * gain * i;
		blueSum += std::get<2>(rgbTriples_[i]) * gain * i;

		redPixelsCount += std::get<0>(rgbTriples_[i]);
		greenPixelsCount += std::get<1>(rgbTriples_[i]);
		bluePixelsCount += std::get<2>(rgbTriples_[i]);
	}

	redMean = std::min(redSum / redPixelsCount,
			   static_cast<double>(NEO_HIST_BIN_SIZE - 1));
	greenMean = std::min(greenSum / greenPixelsCount,
			     static_cast<double>(NEO_HIST_BIN_SIZE - 1));
	blueMean = std::min(blueSum / bluePixelsCount,
			    static_cast<double>(NEO_HIST_BIN_SIZE - 1));

	LOG(NxpNeoAlgoAgc, Debug) << "Mean [R,G,B]: " << redMean
				  << ", " << greenMean << ", " << blueMean
				  << " - gain=" << gain;
	/*
	 * Apply the AWB gains to approximate colours correctly, use the Rec.
	 * 601 formula to calculate the relative luminance, and normalize it.
	 */
	double ySum = redMean * rGain_ * 0.299 +
		      greenMean * gGain_ * 0.587 +
		      blueMean * bGain_ * 0.114;

	return ySum / (NEO_HIST_BIN_SIZE - 1);
}

/**
 * \brief Parse histogram statistics from ISP
 * \param[in] stats Histogram statistics from ISP
 *
 * Store bin values of each channel for further processing from estimateLuminance.
 * This function also provides pointer to histogram used for
 * brightness estimation.
 *
 * \return Histogram used for brightness estimation
 */
Histogram Agc::parseStatistics(const neoisp_meta_stats_s *stats)
{
	const uint32_t *binRed = &(stats->mems.hist.hist_stat[AGC_HIST_MEM_RED]);
	const uint32_t *binGreen = &(stats->mems.hist.hist_stat[AGC_HIST_MEM_GREEN]);
	const uint32_t *binBlue = &(stats->mems.hist.hist_stat[AGC_HIST_MEM_BLUE]);
	Histogram histGreen{ Span<const uint32_t>(binGreen, NEO_HIST_BIN_SIZE) };

	rgbTriples_.clear();

	/* rgbTriples contains the bin value for each channel */
	for (unsigned int i = 0; i < NEO_HIST_BIN_SIZE; i++) {
		rgbTriples_.push_back({
			binRed[i],
			binGreen[i],
			binBlue[i],
		});
	}

	/* return the green histogram for brightness estimation */
	return histGreen;
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

	Histogram hist = parseStatistics(stats);
	rGain_ = context.activeState.awb.gains.automatic.red;
	gGain_ = context.activeState.awb.gains.automatic.blue;
	bGain_ = context.activeState.awb.gains.automatic.green;

	/*
	 * The Agc algorithm needs to know the effective exposure value that was
	 * applied to the sensor when the statistics were collected.
	 */
	utils::Duration exposureTime = context.configuration.sensor.lineDuration *
				       frameContext.sensor.exposure;
	double analogueGain = frameContext.sensor.gain;
	utils::Duration effectiveExposureValue = exposureTime * analogueGain;
	LOG(NxpNeoAlgoAgc, Debug)
		<< "Sensor[Exposure, Gain]= "
		<< exposureTime << ", " << analogueGain << " - frame=" << frame;

	utils::Duration shutterTime;
	double aGain, dGain;
	std::tie(shutterTime, aGain, dGain) =
		calculateNewEv(context.activeState.agc.constraintMode,
			       context.activeState.agc.exposureMode, hist,
			       effectiveExposureValue);

	LOG(NxpNeoAlgoAgc, Debug)
		<< "Divided up shutter, analogue gain and digital gain are "
		<< shutterTime << ", " << aGain << " and " << dGain;

	IPAActiveState &activeState = context.activeState;
	/* Update the estimated exposure and gain. */
	activeState.agc.automatic.exposure = shutterTime / context.configuration.sensor.lineDuration;
	activeState.agc.automatic.gain = aGain;

	fillMetadata(context, frameContext, metadata);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
