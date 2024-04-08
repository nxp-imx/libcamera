/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * hdr_decomp.cpp - NXP NEO HDR Decompression configuration
 * Copyright 2024 NXP
 */

#include "hdr_decomp.h"

#include <algorithm>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file hdr_decomp.cpp
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class HdrDecomp
 * \brief HDR Decompression configuration
 *
 * This Algorithm configures the HDR Decompression unit.
 * It scales the input pixels to 20-bit precision, with a non-linear transfer
 * function when compression is used by the sensor.
 */

LOG_DEFINE_CATEGORY(NxpNeoHdrDecomp)

HdrDecomp::HdrDecomp()
	: input0_({}), input1_({})
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int HdrDecomp::init([[maybe_unused]] IPAContext &context,
		    const YamlObject &tuningData)
{
	/*
	 * Input0 calibration parsing
	 */

	const YamlObject &obj0 = tuningData["input0"];
	if (!obj0.isDictionary() || (!obj0.size())) {
		LOG(NxpNeoHdrDecomp, Debug) << "input0 not configured";
		return 0;
	}

	input0_.points = obj0["points"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input0_.points.size() != kNumPoints) {
		LOG(NxpNeoHdrDecomp, Error)
			<< "input0 points list size must be " << kNumPoints;
		return -EINVAL;
	}

	input0_.offsets = obj0["offsets"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input0_.offsets.size() != kNumOffsets) {
		LOG(NxpNeoHdrDecomp, Error)
			<< "input0 offsets list size must be " << kNumOffsets;
		return -EINVAL;
	}

	input0_.newpoints = obj0["newpoints"].getList<uint32_t>()
				.value_or(std::vector<uint32_t>{});
	if (input0_.newpoints.size() != kNumNewPoints) {
		LOG(NxpNeoHdrDecomp, Error)
			<< "input0 newpoints list size must be " << kNumNewPoints;
		return -EINVAL;
	}

	input0_.ratios = obj0["ratios"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input0_.ratios.size() != kNumRatios) {
		LOG(NxpNeoHdrDecomp, Error)
			<< "input0 ratios list size must be " << kNumRatios;
		return -EINVAL;
	}

	input0_.enabled = true;

	/*
	 * Input1 calibration parsing
	 */

	const YamlObject &obj1 = tuningData["input1"];
	if (!obj1.isDictionary() || (!obj1.size())) {
		LOG(NxpNeoHdrDecomp, Debug) << "input1 not configured";
		return 0;
	}

	input1_.points = obj1["points"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input1_.points.size() != kNumPoints) {
		LOG(NxpNeoHdrDecomp, Error)
			<< "input1 points list size must be " << kNumPoints;
		return -EINVAL;
	}

	input1_.offsets = obj1["offsets"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input1_.offsets.size() != kNumOffsets) {
		LOG(NxpNeoHdrDecomp, Error)
			<< "input1 offsets list size must be " << kNumOffsets;
		return -EINVAL;
	}

	input1_.newpoints = obj1["newpoints"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input1_.newpoints.size() != kNumNewPoints) {
		LOG(NxpNeoHdrDecomp, Error)
			<< "input1 newpoints list size must be " << kNumNewPoints;
		return -EINVAL;
	}

	input1_.ratios = obj1["ratios"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input1_.ratios.size() != kNumRatios) {
		LOG(NxpNeoHdrDecomp, Error)
			<< "input1 ratios list size must be " << kNumRatios;
		return -EINVAL;
	}

	input1_.enabled = true;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void HdrDecomp::prepare([[maybe_unused]] IPAContext &context, const uint32_t frame,
			[[maybe_unused]] IPAFrameContext &frameContext,
			neoisp_meta_params_s *params)
{
	if (frame > 0)
		return;

	LOG(NxpNeoHdrDecomp, Debug)
		<< "input0/1 enabled " << input0_.enabled << "/" << input1_.enabled;

	if (input0_.enabled) {
		params->features_cfg.hdr_decompress_input0_cfg = 1;

		neoisp_hdr_decompress0_cfg_s *hd0 =
			&params->regs.decompress_input0;
		hd0->ctrl_enable = 1;

		hd0->knee_point1 = input0_.points[0];
		hd0->knee_point2 = input0_.points[1];
		hd0->knee_point3 = input0_.points[2];
		hd0->knee_point4 = input0_.points[3];

		hd0->knee_offset0 = input0_.offsets[0];
		hd0->knee_offset1 = input0_.offsets[1];
		hd0->knee_offset2 = input0_.offsets[2];
		hd0->knee_offset3 = input0_.offsets[3];
		hd0->knee_offset4 = input0_.offsets[4];

		hd0->knee_npoint0 = input0_.newpoints[0];
		hd0->knee_npoint1 = input0_.newpoints[1];
		hd0->knee_npoint2 = input0_.newpoints[2];
		hd0->knee_npoint3 = input0_.newpoints[3];
		hd0->knee_npoint4 = input0_.newpoints[4];

		hd0->knee_ratio0 = input0_.ratios[0];
		hd0->knee_ratio1 = input0_.ratios[1];
		hd0->knee_ratio2 = input0_.ratios[2];
		hd0->knee_ratio3 = input0_.ratios[3];
		hd0->knee_ratio4 = input0_.ratios[4];
	}

	if (input1_.enabled) {
		params->features_cfg.hdr_decompress_input1_cfg = 1;

		neoisp_hdr_decompress1_cfg_s *hd1 =
			&params->regs.decompress_input1;
		hd1->ctrl_enable = 1;

		hd1->knee_point1 = input1_.points[0];
		hd1->knee_point2 = input1_.points[1];
		hd1->knee_point3 = input1_.points[2];
		hd1->knee_point4 = input1_.points[3];

		hd1->knee_offset0 = input1_.offsets[0];
		hd1->knee_offset1 = input1_.offsets[1];
		hd1->knee_offset2 = input1_.offsets[2];
		hd1->knee_offset3 = input1_.offsets[3];
		hd1->knee_offset4 = input1_.offsets[4];

		hd1->knee_npoint0 = input1_.newpoints[0];
		hd1->knee_npoint1 = input1_.newpoints[1];
		hd1->knee_npoint2 = input1_.newpoints[2];
		hd1->knee_npoint3 = input1_.newpoints[3];
		hd1->knee_npoint4 = input1_.newpoints[4];

		hd1->knee_ratio0 = input1_.ratios[0];
		hd1->knee_ratio1 = input1_.ratios[1];
		hd1->knee_ratio2 = input1_.ratios[2];
		hd1->knee_ratio3 = input1_.ratios[3];
		hd1->knee_ratio4 = input1_.ratios[4];
	}
}

REGISTER_IPA_ALGORITHM(HdrDecomp, "HdrDecomp")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
