/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on IPU3 AWB control algorithm
 *     src/ipa/ipu3/algorithms/awb.cpp
 * Copyright (C) 2021, Ideas On Board
 *
 * awb.cpp - AWB control algorithm
 * Copyright 2024 NXP
 */

#include "awb.h"
#include "nxp-neoisp-enums.h"

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

  /**
 * \class Awb
 * \brief A Grey world white balance correction algorithm
 *
 * The Grey World algorithm assumes that the scene, in average, is neutral grey.
 * Reference: Lam, Edmund & Fung, George. (2008). Automatic White Balancing in
 * Digital Photography. 10.1201/9781420054538.ch10.
 *
 */

Awb::Awb()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Awb::configure(IPAContext &context,
		   const IPACameraSensorInfo &configInfo)
{
	context.activeState.awb.gains.manual.red = 1.0;
	context.activeState.awb.gains.manual.blue = 1.0;
	context.activeState.awb.gains.manual.green = 1.0;
	context.activeState.awb.gains.automatic.red = 1.0;
	context.activeState.awb.gains.automatic.blue = 1.0;
	context.activeState.awb.gains.automatic.green = 1.0;
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
	/*
	 * This is the latest time we can read the active state. This is the
	 * most up-to-date automatic values we can read.
	 */
	if (frameContext.awb.autoEnabled) {
		frameContext.awb.gains.red = context.activeState.awb.gains.automatic.red;
		frameContext.awb.gains.green = context.activeState.awb.gains.automatic.green;
		frameContext.awb.gains.blue = context.activeState.awb.gains.automatic.blue;
	}

	/* Configure OB_WB */
	/* size of pixel components: set to default value */
	/* \todo: check if WB gain should also applied on line path 0 and 1 */
	params->regs.obwb[NEO_OBWB_MERGE_PATH].ctrl_obpp = NEO_OBWB_OBPP_20BPP;

	/* Update the WB gains. */
	params->features_cfg.obwb2_cfg = 1;
	params->regs.obwb[NEO_OBWB_MERGE_PATH].r_ctrl_gain =
					gainDouble2Param(frameContext.awb.gains.red);
	params->regs.obwb[NEO_OBWB_MERGE_PATH].gr_ctrl_gain =
					gainDouble2Param(frameContext.awb.gains.green);
	params->regs.obwb[NEO_OBWB_MERGE_PATH].gb_ctrl_gain =
					gainDouble2Param(frameContext.awb.gains.green);
	params->regs.obwb[NEO_OBWB_MERGE_PATH].b_ctrl_gain =
					gainDouble2Param(frameContext.awb.gains.blue);

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

/*
 * Generate an RGB vector with the average values for each block.
 */
void Awb::generateBlocks(const neoisp_meta_stats_s *stats)
{
	neoisp_ctemp_mem_stats_s ctemp = stats->mems.ctemp;

	blocks_.clear();

	for (unsigned int i = 0; i < NEO_CTEMP_BLOCK_NB_X * NEO_CTEMP_BLOCK_NB_Y; i++) {
		RGB block;
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
		block.R = sumR / counted;
		block.G = sumG / counted;
		block.B = sumB / counted;
		blocks_.push_back(block);
	}
}

void Awb::awbGreyWorld(IPAActiveState &activeState, IPAFrameContext &frameContext)
{
	LOG(NxpNeoAwb, Debug) << "Grey world AWB";
	/*
	 * Make a separate list of the derivatives for each of red and blue, so
	 * that we can sort them to exclude the extreme gains.
	 */
	std::vector<RGB> &redDerivative(blocks_);
	std::vector<RGB> blueDerivative(redDerivative);
	std::sort(redDerivative.begin(), redDerivative.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.R < b.G * a.R;
		  });
	std::sort(blueDerivative.begin(), blueDerivative.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.B < b.G * a.B;
		  });

	/* Average the middle half of the values. */
	int discard = redDerivative.size() / 4;

	RGB sumRed(0, 0, 0);
	RGB sumBlue(0, 0, 0);
	for (auto ri = redDerivative.begin() + discard,
		  bi = blueDerivative.begin() + discard;
	     ri != redDerivative.end() - discard; ri++, bi++)
		sumRed += *ri, sumBlue += *bi;

	/*
	 * The ISP computes the AWB measurements after applying the colour gains,
	 * divide by the gains that were used to get the raw means from the
	 * sensor.
	 */
	sumRed.G /= frameContext.awb.gains.green;
	sumRed.R /= frameContext.awb.gains.red;
	sumBlue.G /= frameContext.awb.gains.green;
	sumBlue.B /= frameContext.awb.gains.blue;

	double redGain = sumRed.G / (sumRed.R + 1),
	       blueGain = sumBlue.G / (sumBlue.B + 1);

	/*
	 * Color temperature is not relevant in Grey world but
	 * still useful to estimate it :-)
	 */
	activeState.awb.temperatureK = estimateCCT(sumRed.R, sumRed.G, sumBlue.B);

	/*
	 * Clamp the gain values to the hardware, which expresses gains as Q8.8
	 * unsigned integer values.
	 */
	redGain = std::clamp(redGain, 0.0, 65535.0 / 256);
	blueGain = std::clamp(blueGain, 0.0, 65535.0 / 256);

	activeState.awb.gains.automatic.red = redGain;
	activeState.awb.gains.automatic.blue = blueGain;
	/* Hardcode the green gain to 1.0. */
	activeState.awb.gains.automatic.green = 1.0;
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
	IPAActiveState &activeState = context.activeState;

	generateBlocks(stats);
	awbGreyWorld(activeState, frameContext);

	frameContext.awb.temperatureK = activeState.awb.temperatureK;

	metadata.set(controls::AwbEnable, frameContext.awb.autoEnabled);
	metadata.set(controls::ColourGains, {
			static_cast<float>(frameContext.awb.gains.red),
			static_cast<float>(frameContext.awb.gains.blue)
		});
	metadata.set(controls::ColourTemperature, frameContext.awb.temperatureK);

	LOG(NxpNeoAwb, Debug) << std::showpoint
		<< "AWB Gains [" << activeState.awb.gains.automatic.red << ", "
		<< activeState.awb.gains.automatic.green << ", "
		<< activeState.awb.gains.automatic.blue << "], temp "
		<< frameContext.awb.temperatureK << "K";
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
