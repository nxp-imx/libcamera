/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * hdr_decomp.cpp - NXP NEO HDR Decompression configuration
 * Copyright 2025 NXP
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
 * The block can be used to apply a non-linear decompression of the pixel
 * values when the sensor uses compression. It may also be used for simple
 * linear rescaling of the pixel values.
 * The target output pixel format is:
 * - 20-bit on line path 0 (input0)
 * - 16-bit on line path 1 (input1)
 * Input format is either the sensor output bpp or a MSB-aligned shifted version
 * of it - see PIPECONF block LPALIGN0 and LPALIGN1 configurations.
 * HDR DECOMP operation is configured by a number of parameters defined in the
 * sensor calibration file.
 * Using points[] evaluated in increasing order, the conversion logic is:
 * if (pv < points[N])
 *   opv = (pv - offsets[N-1]) * ratios[N-1] + newpoints[N-1]
 * with:
 * - pv: input pixel value
 * - opv: output pixel value
 * - points: KNEE_POINT[1-4] (u16)
 * - offsets: KNEE_NPOINT[0-4] (u16)
 * - newpoints: KNEE_NPOINT[0-4] (u20 for input0 - u16 for input1)
 * - ratios: KNEE_RATIO[0-4] (u7.5)
 * Last entry in the offsets/newpoints/ratios arrays is used as the default case
 * when no value from points[] array matched the condition (pv < points[N]).
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoHdrDecomp)

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
		LOG(NxpNeoAlgoHdrDecomp, Debug) << "input0 not configured";
		return 0;
	}

	input0_.points = obj0["points"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input0_.points.size() != kNumPoints) {
		LOG(NxpNeoAlgoHdrDecomp, Error)
			<< "input0 points list size must be " << kNumPoints;
		return -EINVAL;
	}

	input0_.offsets = obj0["offsets"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input0_.offsets.size() != kNumOffsets) {
		LOG(NxpNeoAlgoHdrDecomp, Error)
			<< "input0 offsets list size must be " << kNumOffsets;
		return -EINVAL;
	}

	input0_.newpoints = obj0["newpoints"].getList<uint32_t>()
				.value_or(std::vector<uint32_t>{});
	if (input0_.newpoints.size() != kNumNewPoints) {
		LOG(NxpNeoAlgoHdrDecomp, Error)
			<< "input0 newpoints list size must be " << kNumNewPoints;
		return -EINVAL;
	}

	input0_.ratios = obj0["ratios"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input0_.ratios.size() != kNumRatios) {
		LOG(NxpNeoAlgoHdrDecomp, Error)
			<< "input0 ratios list size must be " << kNumRatios;
		return -EINVAL;
	}

	input0_.enabled = true;

	/*
	 * Input1 calibration parsing
	 */

	const YamlObject &obj1 = tuningData["input1"];
	if (!obj1.isDictionary() || (!obj1.size())) {
		LOG(NxpNeoAlgoHdrDecomp, Debug) << "input1 not configured";
		return 0;
	}

	input1_.points = obj1["points"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input1_.points.size() != kNumPoints) {
		LOG(NxpNeoAlgoHdrDecomp, Error)
			<< "input1 points list size must be " << kNumPoints;
		return -EINVAL;
	}

	input1_.offsets = obj1["offsets"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input1_.offsets.size() != kNumOffsets) {
		LOG(NxpNeoAlgoHdrDecomp, Error)
			<< "input1 offsets list size must be " << kNumOffsets;
		return -EINVAL;
	}

	input1_.newpoints = obj1["newpoints"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input1_.newpoints.size() != kNumNewPoints) {
		LOG(NxpNeoAlgoHdrDecomp, Error)
			<< "input1 newpoints list size must be " << kNumNewPoints;
		return -EINVAL;
	}

	input1_.ratios = obj1["ratios"].getList<uint16_t>()
				.value_or(std::vector<uint16_t>{});
	if (input1_.ratios.size() != kNumRatios) {
		LOG(NxpNeoAlgoHdrDecomp, Error)
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
			NxpNeoParams *params)
{
	if (frame > 0)
		return;

	LOG(NxpNeoAlgoHdrDecomp, Debug)
		<< "input0/1 enabled " << input0_.enabled << "/" << input1_.enabled;

	if (input0_.enabled) {
		auto hdrdec0Config = params->block<BlockParamsType::HdrDec0>();
		hdrdec0Config.setUpdate(true);

		hdrdec0Config->ctrl_enable = 1;

		hdrdec0Config->knee_point1 = input0_.points[0];
		hdrdec0Config->knee_point2 = input0_.points[1];
		hdrdec0Config->knee_point3 = input0_.points[2];
		hdrdec0Config->knee_point4 = input0_.points[3];

		hdrdec0Config->knee_offset0 = input0_.offsets[0];
		hdrdec0Config->knee_offset1 = input0_.offsets[1];
		hdrdec0Config->knee_offset2 = input0_.offsets[2];
		hdrdec0Config->knee_offset3 = input0_.offsets[3];
		hdrdec0Config->knee_offset4 = input0_.offsets[4];

		hdrdec0Config->knee_npoint0 = input0_.newpoints[0];
		hdrdec0Config->knee_npoint1 = input0_.newpoints[1];
		hdrdec0Config->knee_npoint2 = input0_.newpoints[2];
		hdrdec0Config->knee_npoint3 = input0_.newpoints[3];
		hdrdec0Config->knee_npoint4 = input0_.newpoints[4];

		hdrdec0Config->knee_ratio0 = input0_.ratios[0];
		hdrdec0Config->knee_ratio1 = input0_.ratios[1];
		hdrdec0Config->knee_ratio2 = input0_.ratios[2];
		hdrdec0Config->knee_ratio3 = input0_.ratios[3];
		hdrdec0Config->knee_ratio4 = input0_.ratios[4];
	}

	if (input1_.enabled) {
		auto hdrdec1Config = params->block<BlockParamsType::HdrDec1>();
		hdrdec1Config.setUpdate(true);

		hdrdec1Config->ctrl_enable = 1;

		hdrdec1Config->knee_point1 = input1_.points[0];
		hdrdec1Config->knee_point2 = input1_.points[1];
		hdrdec1Config->knee_point3 = input1_.points[2];
		hdrdec1Config->knee_point4 = input1_.points[3];

		hdrdec1Config->knee_offset0 = input1_.offsets[0];
		hdrdec1Config->knee_offset1 = input1_.offsets[1];
		hdrdec1Config->knee_offset2 = input1_.offsets[2];
		hdrdec1Config->knee_offset3 = input1_.offsets[3];
		hdrdec1Config->knee_offset4 = input1_.offsets[4];

		hdrdec1Config->knee_npoint0 = input1_.newpoints[0];
		hdrdec1Config->knee_npoint1 = input1_.newpoints[1];
		hdrdec1Config->knee_npoint2 = input1_.newpoints[2];
		hdrdec1Config->knee_npoint3 = input1_.newpoints[3];
		hdrdec1Config->knee_npoint4 = input1_.newpoints[4];

		hdrdec1Config->knee_ratio0 = input1_.ratios[0];
		hdrdec1Config->knee_ratio1 = input1_.ratios[1];
		hdrdec1Config->knee_ratio2 = input1_.ratios[2];
		hdrdec1Config->knee_ratio3 = input1_.ratios[3];
		hdrdec1Config->knee_ratio4 = input1_.ratios[4];
	}
}

REGISTER_IPA_ALGORITHM(HdrDecomp, "HdrDecomp")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
