/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on IPU3 AWB control algorithm
 *     src/ipa/ipu3/algorithms/awb.cpp
 * Copyright (C) 2021, Ideas On Board
 *
 * awb.cpp - AWB control algorithm
 * Copyright 2024-2025 NXP
 */

#include "awb.h"
#include "nxp-neoisp-enums.h"

#include <algorithm>
#include <cmath>
#include <iomanip>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/colours.h"

/**
 * \file awb.h
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class Awb
 * \brief A Grey world white balance correction algorithm
 *
 * The Grey World algorithm assumes that the scene, in average, is neutral grey.
 * Reference: Lam, Edmund & Fung, George. (2008). Automatic White Balancing in
 * Digital Photography. 10.1201/9781420054538.ch10.
 *
 * AWB correction is typically applied in the OBWB2 block which is the default
 * value. However, there is the option to specify in the calibration file that
 * AWB correction should be moved to OBWB0/1 instead.
 * AWB may share usage of the OBWB blocks with BLC, AWB configuring the
 * gains and BLC configuring the offsets. Thus, AWB also configures default offsets
 * to zero in the OBWB blocks if they were not configured beforehand by BLC.
 *
 * Relevant keys in the AWB section of the calibration file:
 * obwb-blocks: the OBWB blocks where AWB gains should apply - optional
 *              valid values: { "obwb0/1", "obwb2"}
 *              default value: "obwb2"
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoAwb)

Awb::Awb()
	: enabled_(false), obwbs_(kObwbMap.at(kDefaultObwb))
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Awb::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	/* Get the OBWB block name from tuning file. */
	const std::string &obwb_name =
		tuningData["obwb-blocks"].get<std::string>().value_or(kDefaultObwb);

	/* Get the OBWB blocks where AWB gains should apply. */
	auto it = kObwbMap.find(obwb_name);
	if (it != kObwbMap.end()) {
		obwbs_ = it->second;
		LOG(NxpNeoAlgoAwb, Debug) << "AWB gains apply in " << it->first;
	} else {
		LOG(NxpNeoAlgoAwb, Warning)
			<< "AWB gains are not applied! Invalid \"" << obwb_name
			<< "\" name from tuning file, should be \"obwb0/1\" or \"obwb2\"";
		enabled_ = false;
		return 0;
	}

	enabled_ = true;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Awb::configure(IPAContext &context,
		   const IPACameraSensorInfo &configInfo)
{
	if (!enabled_)
		return 0;

	context.activeState.awb.gains.manual = RGB<double>{ 1.0 };
	context.activeState.awb.gains.automatic = RGB<double>{ 1.0 };
	context.activeState.awb.autoEnabled = true;

	/*
	 * Configuration for CTEMP Block Statistics.
	 * ROI is defined as the full image size.
	 */
	context.configuration.awb.roi.xpos = 0;
	context.configuration.awb.roi.ypos = 0;
	context.configuration.awb.roi.width = configInfo.outputSize.width;
	context.configuration.awb.roi.height = configInfo.outputSize.height;

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
	if (!enabled_)
		return;

	auto &awb = context.activeState.awb;

	const auto &awbEnable = controls.get(controls::AwbEnable);
	if (awbEnable && *awbEnable != awb.autoEnabled) {
		awb.autoEnabled = *awbEnable;

		LOG(NxpNeoAlgoAwb, Debug)
			<< (*awbEnable ? "Enabling" : "Disabling") << " AWB";
	}

	const auto &colourGains = controls.get(controls::ColourGains);
	if (colourGains && !awb.autoEnabled) {
		awb.gains.manual.r() = (*colourGains)[0];
		awb.gains.manual.b() = (*colourGains)[1];

		LOG(NxpNeoAlgoAwb, Debug)
			<< "Set colour gains to " << awb.gains.manual;
	}

	frameContext.awb.autoEnabled = awb.autoEnabled;

	if (!awb.autoEnabled)
		frameContext.awb.gains = awb.gains.manual;
}

constexpr uint16_t Awb::gainDouble2Param(double gain)
{
	/*
	 * The colour gains applied by the OBWB for the four channels (Gr, R, B
	 * and Gb) are expressed in the parameters structure as 16-bit integers
	 * that store a fixed-point U8.8 value in the range [0, 256[.
	 *
	 * Pout = (Pin * gain) >> 8
	 *
	 * where 'Pin' is the input pixel value, 'Pout' the output pixel value,
	 * and 'gain' the gain in the parameters structure as a 16-bit integer.
	 */
	return std::clamp(gain * 256, 0.0, 65535.0);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Awb::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, neoisp_meta_params_s *params)
{
	if (!enabled_)
		return;

	/*
	 * This is the latest time we can read the active state. This is the
	 * most up-to-date automatic values we can read.
	 */
	if (frameContext.awb.autoEnabled)
		frameContext.awb.gains = context.activeState.awb.gains.automatic;

	for (const uint8_t &obwb : obwbs_) {
		int obpp;
		if (obwb == 0) {
			params->features_cfg.obwb0_cfg = 1;
			obpp = NEO_OBWB_OBPP_20BPP;
		} else if (obwb == 1) {
			params->features_cfg.obwb1_cfg = 1;
			obpp = NEO_OBWB_OBPP_16BPP;
		} else if (obwb == 2) {
			params->features_cfg.obwb2_cfg = 1;
			obpp = NEO_OBWB_OBPP_20BPP;
		} else {
			LOG(NxpNeoAlgoAwb, Warning) << "Invalid OBWB" << +obwb << " block,";
			continue;
		}
		params->regs.obwb[obwb].ctrl_obpp = obpp;
		params->regs.obwb[obwb].r_ctrl_gain =
			gainDouble2Param(frameContext.awb.gains.r());
		params->regs.obwb[obwb].gr_ctrl_gain =
			gainDouble2Param(frameContext.awb.gains.g());
		params->regs.obwb[obwb].gb_ctrl_gain =
			gainDouble2Param(frameContext.awb.gains.g());
		params->regs.obwb[obwb].b_ctrl_gain =
			gainDouble2Param(frameContext.awb.gains.b());

		frameContext.awb.colorGainsSet[obwb] = true;

		/*
		 * When OBWB offsets are not configured by BLC, set some default offsets.
		 * Zero offset values are configured as default (no BLC).
		 */
		if (!frameContext.blc.colorOffsetsSet[obwb]) {
			params->regs.obwb[obwb].r_ctrl_offset = 0;
			params->regs.obwb[obwb].gr_ctrl_offset = 0;
			params->regs.obwb[obwb].gb_ctrl_offset = 0;
			params->regs.obwb[obwb].b_ctrl_offset = 0;
		}
	}

	/* If we have already set the CTEMP measurement parameters, return. */
	if (frame > 0)
		return;

	/* Enable CTEMP measurements */
	params->regs.ctemp.ctrl_enable = 1;
	/* Enable color space correction on the input pixel components
	   before measurements */
	params->regs.ctemp.ctrl_cscon = 1;
	/* size of pixel components: set to default value */
	params->regs.ctemp.ctrl_ibpp = NEO_CTEMP_IBPP_20BPP;

	/* Configure the Block Statistics measurements. */
	params->regs.ctemp.roi = context.configuration.awb.roi;
	/*
	 * The block size should be such that the sum statistics never
	 * exceeds the maximum sum value coded with 28 bits mantissa and
	 * 4 bits exponent.
	 * The maximum sum is reached with ((1U << 28) - 1)) << 15.
	 * For 20bits maximum pixel format, the margin is large enough to not
	 * reach this maximum sum value.
	 */
	params->regs.ctemp.stat_blk_size0_xsize =
		params->regs.ctemp.roi.width / NEO_CTEMP_BLOCK_NB_X;
	params->regs.ctemp.stat_blk_size0_ysize =
		params->regs.ctemp.roi.height / NEO_CTEMP_BLOCK_NB_X;

	/* Enable the CTEMP unit parameter update */
	params->features_cfg.ctemp_cfg = 1;
}

/*
 * Generate an RGB vector with the average values for each block.
 */
void Awb::generateBlocks(const neoisp_meta_stats_s *stats)
{
	neoisp_ctemp_mem_stats_s ctemp = stats->mems.ctemp;

	blocks_.clear();

	for (unsigned int i = 0; i < NEO_CTEMP_BLOCK_NB_X * NEO_CTEMP_BLOCK_NB_Y; i++) {
		/*
		 * A 2x2 area of RGGB pixels is processed at once
		 * and the counter is incremented for the whole 2x2 block by one.
		 * Hence the counted statistics is 4 times smaller than
		 * the programmed block size.
		 */
		double counted = ctemp.ctemp_pix_cnt[i];
		unsigned long sumR, sumG, sumB = 0;
		/*
		 * Each statistics sum has 28 bits mantissa (bit[31:4]) and
		 * 4 bits exponent (bit[3:0])
		 */
		sumR = static_cast<unsigned long>(ctemp.ctemp_r_sum[i] >> 4)
		       << (ctemp.ctemp_r_sum[i] & 0xF);
		sumG = static_cast<unsigned long>(ctemp.ctemp_g_sum[i] >> 4)
		       << (ctemp.ctemp_g_sum[i] & 0xF);
		sumB = static_cast<unsigned long>(ctemp.ctemp_b_sum[i] >> 4)
		       << (ctemp.ctemp_b_sum[i] & 0xF);
		RGB<double> block{ { static_cast<double>(sumR),
				     static_cast<double>(sumG),
				     static_cast<double>(sumB) } };
		block /= counted;
		blocks_.push_back(block);
	}
}

void Awb::awbGreyWorld(IPAActiveState &activeState, IPAFrameContext &frameContext,
		       const uint32_t frame)
{
	LOG(NxpNeoAlgoAwb, Debug) << "Grey world AWB";
	/*
	 * Make a separate list of the derivatives for each of red and blue, so
	 * that we can sort them to exclude the extreme gains.
	 */
	std::vector<RGB<double>> &redDerivative(blocks_);
	std::vector<RGB<double>> blueDerivative(redDerivative);
	std::sort(redDerivative.begin(), redDerivative.end(),
		  [](RGB<double> const &a, RGB<double> const &b) {
			  return a.g() * b.r() < b.g() * a.r();
		  });
	std::sort(blueDerivative.begin(), blueDerivative.end(),
		  [](RGB<double> const &a, RGB<double> const &b) {
			  return a.g() * b.b() < b.g() * a.b();
		  });

	/* Average the middle half of the values. */
	int discard = redDerivative.size() / 4;

	RGB<double> sumRed{ 0.0 };
	RGB<double> sumBlue{ 0.0 };
	for (auto ri = redDerivative.begin() + discard,
		  bi = blueDerivative.begin() + discard;
	     ri != redDerivative.end() - discard; ri++, bi++)
		sumRed += *ri, sumBlue += *bi;

	/*
	 * The ISP computes the AWB measurements after applying the colour gains,
	 * divide by the gains that were used to get the raw means from the
	 * sensor.
	 */
	sumRed /= frameContext.awb.gains;
	sumBlue /= frameContext.awb.gains;

	RGB<double> gains({
		sumRed.g() / (sumRed.r() + 1),
		1.0,
		sumBlue.g() / (sumBlue.b() + 1),
	});

	/*
	 * Color temperature is not relevant in Grey world but
	 * still useful to estimate it :-)
	 */
	double ct = estimateCCT({ { sumRed.r(),
				    sumRed.g(),
				    sumBlue.b() } });

	/*
	 * Clamp the gain values to the hardware, which expresses gains as Q8.8
	 * unsigned integer values. Set the minimum just above zero to avoid
	 * divisions by zero when computing the raw means in subsequent
	 * iterations.
	 */
	gains = gains.max(1.0 / 256).min(65535.0 / 256);

	/*
	 * Filter the values to avoid oscillations.
	 * Adapt instantly if we are in startup phase.
	 */
	double speed = frame < kNumStartupFrames ? 1.0 : 0.2;

	ct = ct * speed + activeState.awb.temperatureK * (1 - speed);
	gains = gains * speed + activeState.awb.gains.automatic * (1 - speed);

	activeState.awb.temperatureK = static_cast<unsigned int>(ct);
	activeState.awb.gains.automatic = gains;
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
	if (!enabled_)
		return;

	IPAActiveState &activeState = context.activeState;

	generateBlocks(stats);
	awbGreyWorld(activeState, frameContext, frame);

	frameContext.awb.temperatureK = activeState.awb.temperatureK;

	metadata.set(controls::AwbEnable, frameContext.awb.autoEnabled);
	metadata.set(controls::ColourGains, { static_cast<float>(frameContext.awb.gains.r()),
					      static_cast<float>(frameContext.awb.gains.b()) });
	metadata.set(controls::ColourTemperature, frameContext.awb.temperatureK);

	LOG(NxpNeoAlgoAwb, Debug)
		<< std::showpoint
		<< "AWB Gains [" << activeState.awb.gains.automatic
		<< ", temp " << frameContext.awb.temperatureK << "K";
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
