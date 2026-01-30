/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO HDR Decompression configuration
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
 *
 * Input format to the HDR Decompression block reflects the PIPECONF block
 * LPALIGN0/1 configurations as described in:
 * <src/ipa/nxp/neo/Documentation/source/neo_ipa_algorithms.rst>.
 * When a non-linear decompression is to be applied the input pixel format of
 * the block should be limited to 16-bit bit depth to be able to configure some
 * knee-points. In that case the LPALIGN0/1 configuration should typically be
 * set to zero explicitly in the calibration file to avoid rescaling the native
 * bit depth.
 *
 * Output pixel bit depths of the HDR Decompression block are the ones of the
 * HDR-merge block inputs, as described in the Neo IPA algorithms documentation
 * found in:
 * <src/ipa/nxp/neo/Documentation/source/neo_ipa_algorithms.rst>,
 *
 * When the HDR Decompression block is explicitly configured in the calibration
 * file, those values are applied with priority.
 * For explicit user-defined configuration of the block from the configuration
 * file, a number of parameters are defined in the sensor calibration file.
 * Using points[] evaluated in increasing order, the block conversion logic is:
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
{
	setIrOps(IrOpPrepare);
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int HdrDecomp::init([[maybe_unused]] IPAContext &context,
		    const ValueNode &tuningData)
{
	/*
	 * Input0 calibration parsing
	 */

	const ValueNode &obj0 = tuningData["input0"];
	if (obj0.isDictionary() || obj0.size()) {
		std::optional<std::vector<uint16_t>> points =
			obj0["points"].get<std::vector<uint16_t>>();
		if (points && points->size() != kNumPoints) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input0 points list size must be "
				<< kNumPoints;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> offsets =
			obj0["offsets"].get<std::vector<uint16_t>>();
		if (offsets && offsets->size() != kNumOffsets) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input0 offsets list size must be "
				<< kNumOffsets;
			return -EINVAL;
		}

		std::optional<std::vector<uint32_t>> newpoints =
			obj0["newpoints"].get<std::vector<uint32_t>>();
		if (newpoints && newpoints->size() != kNumNewPoints) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input0 newpoints list size must be "
				<< kNumNewPoints;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> ratios =
			obj0["ratios"].get<std::vector<uint16_t>>();
		if (ratios && ratios->size() != kNumRatios) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input0 ratios list size must be "
				<< kNumRatios;
			return -EINVAL;
		}

		if (points && offsets && newpoints && ratios) {
			input0_.points = std::move(points.value());
			input0_.offsets = std::move(offsets.value());
			input0_.newpoints = std::move(newpoints.value());
			input0_.ratios = std::move(ratios.value());
			input0_.userConfig = true;
		}
	}

	if (!input0_.userConfig) {
		/* Configure block as bypass - unitary gain in u7.5 format. */
		input0_.points = { 0, 0, 0, 0 };
		input0_.offsets = { 0, 0, 0, 0, 0 };
		input0_.newpoints = { 0, 0, 0, 0, 0 };
		input0_.ratios = { 0, 0, 0, 0, (1 << 5) };
	}

	/*
	 * Input1 calibration parsing
	 */

	const ValueNode &obj1 = tuningData["input1"];
	if (obj1.isDictionary() || obj1.size()) {
		std::optional<std::vector<uint16_t>> points =
			obj1["points"].get<std::vector<uint16_t>>();
		if (points && points->size() != kNumPoints) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input1 points list size must be "
				<< kNumPoints;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> offsets =
			obj1["offsets"].get<std::vector<uint16_t>>();
		if (offsets && offsets->size() != kNumOffsets) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input1 offsets list size must be "
				<< kNumOffsets;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> newpoints =
			obj1["newpoints"].get<std::vector<uint16_t>>();
		if (newpoints && newpoints->size() != kNumNewPoints) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input1 newpoints list size must be "
				<< kNumNewPoints;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> ratios =
			obj1["ratios"].get<std::vector<uint16_t>>();
		if (ratios && ratios->size() != kNumRatios) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input1 ratios list size must be "
				<< kNumRatios;
			return -EINVAL;
		}

		if (points && offsets && newpoints && ratios) {
			input1_.points = std::move(points.value());
			input1_.offsets = std::move(offsets.value());
			input1_.newpoints = std::move(newpoints.value());
			input1_.ratios = std::move(ratios.value());
			input1_.userConfig = true;
		}
	}

	if (!input1_.userConfig) {
		/* Configure block as bypass - unitary gain in u7.5 format. */
		input1_.points = { 0, 0, 0, 0 };
		input1_.offsets = { 0, 0, 0, 0, 0 };
		input1_.newpoints = { 0, 0, 0, 0, 0 };
		input1_.ratios = { 0, 0, 0, 0, (1 << 5) };
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int HdrDecomp::configure(IPAContext &context,
			 [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	/*
	 * The algorithm considerations and hardware constraints can be found in:
	 * <src/ipa/nxp/neo/Documentation/source/neo_ipa_algorithms.rst>.
	 */
	IPAPipelineMode &mode = context.configuration.pipelineMode;
	std::array<uint32_t, 2> &bpps = context.configuration.sensor.bpps;

	/* Special cases: update ratio[4] to amend the unitary gain (u7.5). */
	if (!input0_.userConfig && bpps[0] == 12) {
		if (mode != IPAPipelineMode::HdrMerge)
			input0_.ratios[4] = (1 << 5) * 16;
		else
			input0_.ratios[4] = (1 << 5) / 16;
	}

	if (!input1_.userConfig && bpps[1] == 12) {
		if (mode == IPAPipelineMode::HdrMerge)
			input1_.ratios[4] = (1 << 5) / 16;
	}

	if (!input0_.userConfig && bpps[0] == 10) {
		if (mode != IPAPipelineMode::HdrMerge)
			input0_.ratios[4] = (1 << 5);
		else
			input0_.ratios[4] = (1 << 5) * 4;
	}

	if (!input1_.userConfig && bpps[1] == 10) {
		if (mode != IPAPipelineMode::HdrMerge)
			input1_.ratios[4] = (1 << 5);
		else
			input1_.ratios[4] = (1 << 5) * 4;
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void HdrDecomp::prepare([[maybe_unused]] IPAContext &context,
			const uint32_t frame,
			[[maybe_unused]] IPAFrameContext &frameContext,
			NxpNeoParams *params)
{
	if (frame > 0)
		return;

	LOG(NxpNeoAlgoHdrDecomp, Debug)
		<< "input0/1 user config "
		<< input0_.userConfig << "/" << input1_.userConfig;

	auto hdrdec0Config = params->block<BlockParamsType::HdrDec0>();
	hdrdec0Config.setEnabled(true);

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

	auto hdrdec1Config = params->block<BlockParamsType::HdrDec1>();
	hdrdec1Config.setEnabled(true);

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

REGISTER_IPA_ALGORITHM(HdrDecomp, "HdrDecomp")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
