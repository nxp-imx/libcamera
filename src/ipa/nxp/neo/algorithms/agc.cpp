/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * NXP NEO AGC/AEC mean-based control algorithm
 *
 * Based on RkISP1 AGC/AEC mean-based control algorithm
 *     src/ipa/rkisp1/algorithms/agc.cpp
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * Based on IPU3 AGC/AEC mean-based control algorithm
 *     src/ipa/ipu3/algorithms/agc.cpp
 * Copyright (C) 2021, Ideas On Board
 */

#include "agc.h"
#include "neoisp-definitions.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tuple>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/colours.h"
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
 *
 * The AGC algorithm is processing separately the RGB channels and
 * the Ir channel.
 * In case of RGBIr Dual mode, the algorithm is executing differently
 * for the RGB and for the Ir context the following operations:
 * - parsing of the statistics
 * - calculating the new exposure value for the frame context
 * It is using for each context a different AgcMeanLuminance instance
 * from libipa.
 * For RGB channels, the statistics come from the STAT block of the ISP.
 * For Ir channel, the statistics come from the RGBIR block of the ISP.
 * In RGBIr Dual mode, the controls provided part of the capture request
 * apply the same to each context (RGB and Ir).
 *
 * Details about AGC dependencies can be found in the:
 * <src/ipa/nxp/neo/Documentation/source/neo_ipa_algorithms.rst>
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoAgc)

const RGB<uint8_t> AgcStatsRgb::kHistIds{ { HistId0, HistId1, HistId2 } };

std::ostream &operator<<(std::ostream &os, IPACameraContext c);

std::ostream &operator<<(std::ostream &os, IPACameraContext c)
{
	switch (c) {
	case IPACameraContext::Rgb:
		return os << "Rgb";
	case IPACameraContext::Ir:
		return os << "Ir";
	default:
		return os << "Unknown";
	}
}

Agc::Agc()
{
	agcs_[IPACameraContext::Rgb] = std::make_unique<AgcStatsRgb>();
	agcs_[IPACameraContext::Ir] = std::make_unique<AgcStatsIr>();

	setIrOps(IrOpAll);
}

/**
 * \brief Initialise the AGC algorithm from tuning file
 * \param[in] context The shared IPA context
 * \param[in] tuningData The ValueNode containing Agc tuning data
 *
 * \return 0 on success or errors from the base class
 */
int Agc::init(IPAContext &context, const ValueNode &tuningData)
{
	for (const auto &[id, agc] : agcs_) {
		int ret = agc->init(context, tuningData);
		if (ret)
			return ret;
	}

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
	/* ROI set to full image size */
	context.configuration.agc.roi.xpos = 0;
	context.configuration.agc.roi.ypos = 0;
	context.configuration.agc.roi.width = configInfo.outputSize.width;
	context.configuration.agc.roi.height = configInfo.outputSize.height;

	for (const auto &ctxt : context.configuration.activeContexts) {
		context.activeState.agcs[ctxt] = {};
		agcs_[ctxt]->configure(context);
	}

	return 0;
}

/**
 * \brief Check and process any AGC controls coming from the user request
 * \param[in] context The shared IPA context
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The current frame context
 * \param[in] controls The controls provided part of the request
 *
 * In RGBIr Dual mode, the requested controls applies the same to each context
 * (RGB and Ir).
 */
void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	/*
	 * Initialize autoEnabled with any context, it is the same
	 * for both of them.
	 */
	bool autoEnabled =
		context.activeState.agcs.at(IPACameraContext::Rgb).autoEnabled;

	const auto &agcEnable = controls.get(controls::AeEnable);
	if (agcEnable && *agcEnable != autoEnabled) {
		autoEnabled = *agcEnable;
		LOG(NxpNeoAlgoAgc, Debug)
			<< (autoEnabled ? "Enabling" : "Disabling")
			<< " AGC";
	}

	const auto &exposure = controls.get(controls::ExposureTime);
	const auto &gain = controls.get(controls::AnalogueGain);

	for (auto &[cameraContext, agc] : context.activeState.agcs) {
		agc.autoEnabled = autoEnabled;
		if (exposure && !autoEnabled) {
			agc.manual.exposure =
				*exposure * 1.0us /
				context.configuration.sensor.lineDuration;
			LOG(NxpNeoAlgoAgc, Debug)
				<< "Context " << cameraContext
				<< " Set exposure to " << agc.manual.exposure;
		}
		if (gain && !autoEnabled) {
			agc.manual.gain = *gain;
			LOG(NxpNeoAlgoAgc, Debug)
				<< "Context " << cameraContext
				<< " Set gain to " << agc.manual.gain;
		}

		frameContext.agcs[cameraContext].autoEnabled = agc.autoEnabled;
		auto &agcFrameContext = frameContext.agcs.at(cameraContext);

		if (!agcFrameContext.autoEnabled) {
			agcFrameContext.exposure = agc.manual.exposure;
			agcFrameContext.gain = agc.manual.gain;
		}
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Agc::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, NxpNeoParams *params)
{
	IPACameraContext cameraContext = frameContext.cameraContext;
	auto agcActiveState = context.activeState.agcs.at(cameraContext).automatic;
	auto &agcFrameContext = frameContext.agcs.at(cameraContext);

	if (agcFrameContext.autoEnabled) {
		agcFrameContext.exposure = agcActiveState.exposure;
		agcFrameContext.gain = agcActiveState.gain;
	}

	if (frame > 0 && !frameContext.hdr.update)
		return;

	agcs_[cameraContext]->setupHistograms(context, frameContext, params);
}

void Agc::fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
		       ControlList &metadata) const
{
	/* Metadata are only filled in RGB context. */
	if (frameContext.cameraContext == IPACameraContext::Ir)
		return;

	auto agcSensor = frameContext.sensor.agcs.at(frameContext.cameraContext);
	utils::Duration exposureTime = context.configuration.sensor.lineDuration *
				       agcSensor.exposure;
	metadata.set(controls::AnalogueGain, agcSensor.gain);
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());

	auto &agcFrameContext = frameContext.agcs.at(frameContext.cameraContext);
	metadata.set(controls::AeEnable, agcFrameContext.autoEnabled);

	/* \todo Use VBlank value calculated from each frame exposure. */
	uint32_t vTotal = context.configuration.sensor.size.height +
			  context.configuration.sensor.defVBlank;
	utils::Duration frameDuration = context.configuration.sensor.lineDuration *
					vTotal;
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
		  IPAFrameContext &frameContext, const NxpNeoStats *stats,
		  ControlList &metadata)
{
	IPACameraContext cameraContext = frameContext.cameraContext;

	if (!stats) {
		fillMetadata(context, frameContext, metadata);
		return;
	}

	agcs_[cameraContext]->parseStatistics(stats);
	agcs_[cameraContext]->setAwbGains(context, frameContext);

	/*
	 * The Agc algorithm needs to know the effective exposure value that was
	 * applied to the sensor when the statistics were collected.
	 */
	auto agcSensor = frameContext.sensor.agcs.at(cameraContext);
	utils::Duration exposureTime = context.configuration.sensor.lineDuration *
				       agcSensor.exposure;
	double analogueGain = agcSensor.gain;
	utils::Duration effectiveExposureValue = exposureTime * analogueGain;
	utils::Duration newExposureTime;
	double aGain, qGain, dGain;
	auto &agcActiveState = context.activeState.agcs.at(cameraContext);

	std::tie(newExposureTime, aGain, qGain, dGain) =
		agcs_[cameraContext]->AgcMeanLuminance::calculateNewEv(
			agcActiveState.constraintMode,
			agcActiveState.exposureMode,
			agcs_[cameraContext]->histogram(),
			effectiveExposureValue);

	LOG(NxpNeoAlgoAgc, Debug)
		<< "Computed AGC frame" << frame << " context " << cameraContext
		<< " exposure time: " << newExposureTime << ", aGain: "
		<< aGain << ", qGain: " << qGain
		<< ", dGain: " << dGain;

	/* Update the estimated exposure and gain. */
	agcActiveState.automatic.exposure = newExposureTime /
					    context.configuration.sensor.lineDuration;
	agcActiveState.automatic.gain = aGain;

	/*
	 * Update the frame context of current frame only for frame 0
	 * to make sure that the frame context has relevant configuration
	 * whilst processing frame 0.
	 * Indeed the frame context for each frame is updated whilst
	 * preparing the next frame.
	 */
	auto &agcFrameContext = frameContext.agcs.at(cameraContext);
	if (!frame && agcFrameContext.autoEnabled) {
		agcFrameContext.exposure = agcActiveState.automatic.exposure;
		agcFrameContext.gain = agcActiveState.automatic.gain;
	}

	fillMetadata(context, frameContext, metadata);
}

void AgcStats::configure(IPAContext &context)
{
	auto &agcActiveState = context.activeState.agcs.at(cameraContext_);

	/* Configure the default exposure and gain. */
	agcActiveState.automatic.gain = context.configuration.sensor.minAnalogueGain;
	agcActiveState.automatic.exposure =
		10ms / context.configuration.sensor.lineDuration;
	agcActiveState.manual.gain = agcActiveState.automatic.gain;
	agcActiveState.manual.exposure = agcActiveState.automatic.exposure;
	agcActiveState.autoEnabled = true;
	agcActiveState.constraintMode =
		static_cast<controls::AeConstraintModeEnum>(
			constraintModes().begin()->first);
	agcActiveState.exposureMode =
		static_cast<controls::AeExposureModeEnum>(
			exposureModeHelpers().begin()->first);

	/* \todo Run this again when FrameDurationLimits is passed in */
	setLimits(context.configuration.sensor.minExposureTime,
		  context.configuration.sensor.maxExposureTime,
		  context.configuration.sensor.minAnalogueGain,
		  context.configuration.sensor.maxAnalogueGain,
		  {});
	resetFrameCount();
}

/**
 * \brief Initialise the AGC RGB instance from tuning file
 * \param[in] context The shared IPA context
 * \param[in] tuningData The ValueNode containing Agc tuning data
 *
 * This function calls the base class' tuningData parsers to discover which
 * control values are supported.
 *
 * \return 0 on success or errors from the base class
 */
int AgcStatsRgb::init(IPAContext &context, const ValueNode &tuningData)
{
	int ret = parseTuningData(tuningData);
	if (ret)
		return ret;

	context.ctrlMap[&controls::AeEnable] = ControlInfo(false, true);
	context.ctrlMap.merge(controls());

	return parseTuningDataRgb(tuningData);
}

/**
 * \brief Parse the tuning data required for the RGB context
 * \param[in] tuningData The ValueNode representing the tuning data
 *
 * \return 0 on success or a negative error code
 */
int AgcStatsRgb::parseTuningDataRgb(const ValueNode &tuningData)
{
	/*
	 * Histogram scale parsing
	 *
	 * The scaling factor of the histogram is configured such that
	 * the targetted value range of the image is covered among the 64 bins
	 * of the linear histogram.
	 * 4 histograms can be used from the STAT unit of the ISP.
	 * The histScale_ list contains the histogram scaling factor to
	 * program for each of the 4 histograms.
	 */
	const ValueNode &obj = tuningData["hist-scale"];
	if (!obj.size()) {
		LOG(NxpNeoAlgoAgc, Debug) << "Use default histogram scaling value: "
					  << HIST_SCALE_DEFAULT;
		for (unsigned int i = 0; i < NEO_STAT_HIST_CNT; ++i) {
			histScale_.push_back(HIST_SCALE_DEFAULT);
		}
		return 0;
	}

	histScale_ = obj.get<std::vector<uint32_t>>().value_or(std::vector<uint32_t>{});
	if (histScale_.size() != NEO_STAT_HIST_CNT) {
		LOG(NxpNeoAlgoAgc, Error)
			<< "histScale_ list size must be " << NEO_STAT_HIST_CNT;
		return -EINVAL;
	}

	userConfig_ = true;

	return 0;
}

/**
 * \brief Update histogram scaling factor according to the pipeline mode
 * \param[in] context The shared IPA context
 * \param[in] frameContext The current frame context
 */
void AgcStatsRgb::configureHistScale(const IPAContext &context,
				     const IPAFrameContext &frameContext)
{
	/*
	 * In HDR mode, the histogram scaling factor is adapted considering
	 * that it should be configured for the long capture and
	 * that HDR merge is rescaling input captures as follow:
	 * - short capture to 20-bits range
	 * - long capture to the range of the short capture divided by the ratio
	 *   between the long and the short captures
	 * Note that HIST_SCALE_DEFAULT is configured for the default 20-bits
	 * scaling format.
	 */
	if (!userConfig_) {
		uint16_t ratioL2S = context.configuration.hdr.ratioLong2Short;
		/*
		 * Image0 is scaled to:
		 * - if HDR is enabled:
		 *     a range corresponding to 20-bit divided by the exposure
		 *     ratio.
		 * - if HDR is disabled:
		 *     20-bit.
		 */
		uint32_t scale = frameContext.hdr.enabled ?
			HIST_SCALE_DEFAULT * ratioL2S :
			HIST_SCALE_DEFAULT;
		histScale_ = { scale, scale, scale, scale };
	}
}

/**
 * \brief Setup the STAT block of the ISP needed for the RGB histograms
 * \param[in] context The shared IPA context
 * \param[in] frameContext The current frame context
 * \param[out] params Params of the ISP to update
 */
void AgcStatsRgb::setupHistograms(const IPAContext &context,
				  const IPAFrameContext &frameContext,
				  NxpNeoParams *params)
{
	/* STAT Histogram configuration for RGB channels */
	auto statConfig = params->block<BlockParamsType::Stat>();
	statConfig.setEnabled(true);

	configureHistScale(context, frameContext);

	/* Foreground ROI disabled (> Image geometry means invalid ROI) */
	statConfig->roi0.xpos = HIST_ROI_INVALID_IMAGE_GEOMETRY;
	statConfig->roi0.ypos = HIST_ROI_INVALID_IMAGE_GEOMETRY;
	statConfig->roi0.width = HIST_ROI_INVALID_IMAGE_GEOMETRY;
	statConfig->roi0.height = HIST_ROI_INVALID_IMAGE_GEOMETRY;
	/* Background ROI: set to full image */
	statConfig->roi1 = context.configuration.agc.roi;

	/* STAT Histogram configuration */
	/* HIST for Red */
	neoisp_stat_hist_cfg_s *histRed = &statConfig->hists[kHistIds.r()];
	histRed->hist_ctrl_offset = 0;
	histRed->hist_ctrl_channel = NEO_HIST_CHANNEL_R;
	histRed->hist_ctrl_pattern = 0;
	histRed->hist_ctrl_dir_input1_dif = 0;
	histRed->hist_ctrl_lin_input1_log = 0;
	histRed->hist_scale_scale = histScale_[kHistIds.r()];
	/* HIST for Gr+Gb */
	neoisp_stat_hist_cfg_s *histGreen = &statConfig->hists[kHistIds.g()];
	histGreen->hist_ctrl_offset = 0;
	histGreen->hist_ctrl_channel = NEO_HIST_CHANNEL_GR | NEO_HIST_CHANNEL_GB;
	histGreen->hist_ctrl_pattern = 0;
	histGreen->hist_ctrl_dir_input1_dif = 0;
	histGreen->hist_ctrl_lin_input1_log = 0;
	histGreen->hist_scale_scale = histScale_[kHistIds.g()];
	/* HIST for Blue */
	neoisp_stat_hist_cfg_s *histBlue = &statConfig->hists[kHistIds.b()];
	histBlue->hist_ctrl_offset = 0;
	histBlue->hist_ctrl_channel = NEO_HIST_CHANNEL_B;
	histBlue->hist_ctrl_pattern = 0;
	histBlue->hist_ctrl_dir_input1_dif = 0;
	histBlue->hist_ctrl_lin_input1_log = 0;
	histBlue->hist_scale_scale = histScale_[kHistIds.b()];
}

/**
 * \brief Store AWB gains needed to adjust the luminance estimation of
 *        the RGB channels
 * \param[in] context The shared IPA context
 * \param[in] frameContext The current frame context
 *
 * The AWB gains computed for the frame context are stored to be used
 * by the luminance estimation of the RGB channels.
 */
void AgcStatsRgb::setAwbGains(IPAContext &context, IPAFrameContext &frameContext)
{
	auto &awb = context.activeState.awb;
	std::array<bool, 3> &awbEnabled = frameContext.awb.colorGainsSet;

	/* If the AWB algorithm is disabled, use 1.0 for the gains. */
	if (std::find(awbEnabled.begin(), awbEnabled.end(), true) != awbEnabled.end())
		awbGains_ = (awb.autoEnabled ? awb.gains.automatic : awb.gains.manual);
	else
		awbGains_ = (RGB<double>{ 1.0 });
}

/**
 * \brief Parse histogram statistics from STAT block of the ISP
 * \param[in] stats Histogram statistics from ISP
 *
 * Store bin values of each channel for further processing from
 * estimateLuminance.
 * This function also updates the histogram provided to libipa for
 * brightness estimation.
 */
void AgcStatsRgb::parseStatistics(const NxpNeoStats *stats)
{
	auto histMemStats = stats->block<BlockStatsType::MHist>();

	const uint32_t *binRed =
		&(histMemStats->hist_stat[GET_HIST_MEM_OFFSET(kHistIds.r(),
							      RoiId1)]);
	const uint32_t *binGreen =
		&(histMemStats->hist_stat[GET_HIST_MEM_OFFSET(kHistIds.g(),
							      RoiId1)]);
	const uint32_t *binBlue =
		&(histMemStats->hist_stat[GET_HIST_MEM_OFFSET(kHistIds.b(),
							      RoiId1)]);
	histogram_ = Histogram(Span<const uint32_t>(binGreen, NEO_HIST_BIN_SIZE));

	rgbTriples_.clear();

	/* rgbTriples contains the bin value for each channel */
	for (unsigned int i = 0; i < NEO_HIST_BIN_SIZE; i++) {
		rgbTriples_.push_back({
			binRed[i],
			binGreen[i],
			binBlue[i],
		});
	}
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
double AgcStatsRgb::estimateLuminance(double gain) const
{
	RGB<double> sums{ 0.0 };
	RGB<double> means{ 0.0 };
	RGB<double> pixelsCounts{ 0.0 };

	for (unsigned int i = 0; i < rgbTriples_.size(); i++) {
		/* Accumulate weighted bin */
		sums.r() += std::get<0>(rgbTriples_[i]) * gain * i;
		sums.g() += std::get<1>(rgbTriples_[i]) * gain * i;
		sums.b() += std::get<2>(rgbTriples_[i]) * gain * i;

		pixelsCounts.r() += std::get<0>(rgbTriples_[i]);
		pixelsCounts.g() += std::get<1>(rgbTriples_[i]);
		pixelsCounts.b() += std::get<2>(rgbTriples_[i]);
	}

	means = sums / pixelsCounts;
	means = means.min(static_cast<double>(NEO_HIST_BIN_SIZE - 1));
	LOG(NxpNeoAlgoAgc, Debug) << "RGB stats means: " << means
				  << " - applied gain: " << gain;
	/*
	 * Apply the AWB gains to approximate colours correctly, use the Rec.
	 * 601 formula to calculate the relative luminance, and normalize it.
	 */
	double ySum = rec601LuminanceFromRGB(means * awbGains_);
	return ySum / (NEO_HIST_BIN_SIZE - 1);
}

/**
 * \brief Initialise the AGC IR instance from tuning file
 * \param[in] context The shared IPA context
 * \param[in] tuningData The ValueNode containing Agc tuning data
 *
 * This function calls the base class' tuningData parsers.
 * The controls discovered by the AgcMeanLuminance parsers are the same
 * for each context (RGB and Ir) and are merged from the RGB context,
 * see AgcStatsIr::init().
 *
 * \return 0 on success or errors from the base class
 */
int AgcStatsIr::init([[maybe_unused]] IPAContext &context,
		     const ValueNode &tuningData)
{
	return parseTuningData(tuningData);
}

/**
 * \brief Setup the RGBIR block of the ISP needed for the Ir channel histogram
 * \param[in] context The shared IPA context
 * \param[in] frameContext The current frame context
 * \param[out] params Params of the ISP to update
 */
void AgcStatsIr::setupHistograms(const IPAContext &context,
				 [[maybe_unused]] const IPAFrameContext &frameContext,
				 NxpNeoParams *params)
{
	/* RGBIR Histogram configuration for the Ir channel*/
	auto rgbirConfig = params->block<BlockParamsType::RgbIr>();

	rgbirConfig->roi[0].xpos = HIST_ROI_INVALID_IMAGE_GEOMETRY;
	rgbirConfig->roi[0].ypos = HIST_ROI_INVALID_IMAGE_GEOMETRY;
	rgbirConfig->roi[0].width = HIST_ROI_INVALID_IMAGE_GEOMETRY;
	rgbirConfig->roi[0].height = HIST_ROI_INVALID_IMAGE_GEOMETRY;
	/* Background ROI: set to full image */
	rgbirConfig->roi[1] = context.configuration.agc.roi;

	neoisp_stat_hist_cfg_s *histIr = &rgbirConfig->hists[kHistId];
	histIr->hist_ctrl_offset = 0;
	histIr->hist_ctrl_channel = kHistChannelIr;
	histIr->hist_ctrl_pattern = 0;
	histIr->hist_ctrl_dir_input1_dif = 0;
	histIr->hist_ctrl_lin_input1_log = 0;
	histIr->hist_scale_scale = HIST_SCALE_DEFAULT;
}

/**
 * \brief Parse histogram statistics from RGBIR block of the ISP
 * \param[in] stats Histogram statistics from ISP
 *
 * Store bin values of the Ir channel for further processing from
 * estimateLuminance.
 * This function also updates the histogram provided to libipa for
 * brightness estimation.
 */
void AgcStatsIr::parseStatistics(const NxpNeoStats *stats)
{
	auto rgbIrMemStats = stats->block<BlockStatsType::MRgbIr>();
	const uint32_t *binChIr =
		&(rgbIrMemStats->rgbir_hist[GET_HIST_MEM_OFFSET(kHistId,
								RoiId1)]);
	histogram_ = Histogram(Span<const uint32_t>(binChIr, NEO_HIST_BIN_SIZE));
}

/**
 * \brief Estimate the relative luminance of the frame with a given gain
 * \param[in] gain The gain to apply in estimating luminance
 *
 * This function estimates the average relative luminance of the frame that
 * would be output by the sensor if an additional \a gain was applied.
 *
 * The estimation is based on the RGBIR statistics for the current frame.
 * The average of the Ir channel histogram is multiplied by the gain and
 * normalized to the [0.0, 1.0] range, where 1.0 corresponds to a
 * theoretical perfect reflector of 100% reference white.
 *
 * \return The relative luminance
 */
double AgcStatsIr::estimateLuminance(double gain) const
{
	double yLevel = std::min(histogram_.interQuantileMean(0, 1) * gain,
				 histogram_.bins() * 1.0);
	LOG(NxpNeoAlgoAgc, Debug) << "Ir stats means: " << yLevel
				  << " - applied gain: " << gain;
	return yLevel / histogram_.bins();
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")
} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
