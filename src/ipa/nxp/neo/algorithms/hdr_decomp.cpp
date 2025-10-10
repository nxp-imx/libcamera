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
 *
 *       input0              input1
 *     AXI IN0 DMA         AXI IN1 DMA
 *          │                   │
 *  ┌───────▼───────────────────▼───────┐
 *  │ PIPECONF                          │
 *  │    LPALIGN0             LPALIGN1  │
 *  │    INALIGN0             INALIGN1  │
 *  └───────┬───────────────────┬───────┘
 *  ┌───────▼───────┐   ┌───────▼───────┐
 *  │      HC0      │   │      HC1      │
 *  └───────┬───────┘   └───────┬───────┘
 *  ┌───────▼───────┐   ┌───────▼───────┐
 *  │  HDR Decomp0  │   │  HDR Decomp1  │
 *  └───────┬───────┘   └───────┬───────┘
 *  ┌───────▼───────┐   ┌───────▼───────┐
 *  │     OBWB0     │   │     OBWB1     │
 *  └───────┬───────┘   └───────┬───────┘
 *  ┌───────▼───────────────────▼───────┐
 *  │             HDR Merge             │
 *  └─────────────────┬─────────────────┘
 *  ┌─────────────────▼─────────────────┐
 *  │               RGBIR               │
 *  └───────┬───────────────────┬───────┘
 *  ┌───────▼───────┐           │
 *  │     OBWB2     │           │
 *  └───────┬───────┘           │
 *          ▼                   ▼
 *      to RGB Path        to IR path
 *
 * Input format to the HDR Decompression block is either the native sensor
 * output pixel format or a MSB-aligned shifted version of it - see PIPECONF
 * block LPALIGN0/1 configurations.
 * When a non-linear decompression is to be applied the input pixel format of
 * the block should be limited to 16-bit format to be able to configure some
 * knee-points. In that case the LPALIGN0/1 configuration should typically be
 * set to zero explicitly in the calibration file to avoid rescaling the native
 * pixel format.
 *
 * If the HDR merge block is not used, the target pixel format at the output of
 * the HDR Decompression block is:
 * - 20-bit on line path 0 (input0)
 * - 16-bit on line path 1 (input1)
 * When the HDR block is used, the target pixel format at the output of the HDR
 * Decompression blocks is the same as the native sensor format, with a minimum
 * of 12-bit to have support for the saturation in the subsequent OBWB blocks.
 * Thus, the HDR block is configured either as bypass or, for 10-bit HDR merge
 * mode, with the necessary gain to convert from 10-bit to 12-bit format.
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
 *
 * If no block configuration is present in the calibration file, the algorithm
 * falls back on a default configuration logic, and the block will be configured
 * as simple linear gain or bypass, without decompression.
 * If pipeline is not in HDR-merge mode, it is assumed that PIPECONF.LPALIGN0/1
 * is set to 1 meaning that camera pixel native format has been rescaled. Most
 * of the time block can be used in bypass, as the input0 and input1 formats
 * already match the HDR Decompression block targeted output format that is
 * 20-bit for input0 and 16-bit for input1.
 * In HDR-merge mode of operation, it is assumed that PIPECONF.LPALIGN0/1 has
 * been set to zero to avoid rescaling. Native sensor format is expected at the
 * output of the HDR Decompression block so it can be configured in bypass mode.
 * See the PipeConf algorithm for PIPECONF block rescaling logic and exceptions,
 * depending on PIPECONF.LPALIGN0/1, the sensor native pixel format and the ISP
 * hardware revision.
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoHdrDecomp)

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
	if (obj0.isDictionary() || (obj0.size())) {
		std::optional<std::vector<uint16_t>> points =
			obj0["points"].getList<uint16_t>();
		if (points && points->size() != kNumPoints) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input0 points list size must be " << kNumPoints;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> offsets =
			obj0["offsets"].getList<uint16_t>();
		if (offsets && offsets->size() != kNumOffsets) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input0 offsets list size must be " << kNumOffsets;
			return -EINVAL;
		}

		std::optional<std::vector<uint32_t>> newpoints =
			obj0["newpoints"].getList<uint32_t>();
		if (newpoints && newpoints->size() != kNumNewPoints) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input0 newpoints list size must be " << kNumNewPoints;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> ratios =
			obj0["ratios"].getList<uint16_t>();
		if (ratios && ratios->size() != kNumRatios) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input0 ratios list size must be " << kNumRatios;
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

	const YamlObject &obj1 = tuningData["input1"];
	if (obj1.isDictionary() || (obj1.size())) {
		std::optional<std::vector<uint16_t>> points =
			obj1["points"].getList<uint16_t>();
		if (points && points->size() != kNumPoints) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input1 points list size must be " << kNumPoints;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> offsets =
			obj1["offsets"].getList<uint16_t>();
		if (offsets && offsets->size() != kNumOffsets) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input1 offsets list size must be " << kNumOffsets;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> newpoints =
			obj1["newpoints"].getList<uint16_t>();
		if (newpoints && newpoints->size() != kNumNewPoints) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input1 newpoints list size must be " << kNumNewPoints;
			return -EINVAL;
		}

		std::optional<std::vector<uint16_t>> ratios =
			obj1["ratios"].getList<uint16_t>();
		if (ratios && ratios->size() != kNumRatios) {
			LOG(NxpNeoAlgoHdrDecomp, Error)
				<< "input1 ratios list size must be " << kNumRatios;
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
	 * When no user configuration is present in the configuration file we
	 * fallback to a default linear bypass configuration of the block.
	 * There is an hardware peculiarity in the ISP hardware revision V2
	 * related to PIPECONF.LPALIGN0 setting with 12-bit sensor pixel format:
	 * - Rescaling is done to 16-bit instead of 20-bit for other sensor
	 *   formats
	 * - Rescaling is applied even though LPALIGN=0
	 * This leads to 2 exceptions using ISP revision V2 with 12-bit input0
	 * pixel format:
	 * 1) In non HDR-merge mode, we rely on PIPECONF.LPALIGN0/1 to rescale
	 *    the pixels to 20-bits internal format. In that case an additional
	 *    (16) gain needs to be applied in HDR Decompression for the
	 *    remaining 16-bit to 20-bit conversion.
	 * 2) In HDR-merge mode there is the opposite issue where we want to
	 *    keep the native sensor format up to the HDR-merge block. For that
	 *    purpose LPALIGN0=0 is set to avoid PIPECONF rescaling. But it
	 *    does not apply to that specific case so a (1/16) fractional gain
	 *    needs to be set to revert the pixel format from 16-bit to 12-bit.
	 * During HDR merge operation where we want to keep the native sensor
	 * bitdepth up to the HDR merge block, there is a constraint coming from
	 * the OBWB block, whose saturation (obpp) is configurable only from
	 * 12-bit onwards. Thus, for a lower pixel format (10-bit) the necessary
	 * gain is applied in HDR Decomp block to rescale the input to 12-bit
	 * format in order to meet the OBWB0/1 constraints.
	 */
	IPAModeType &mode = context.configuration.pipelineMode;
	unsigned int &hwRevision = context.hw.hwRevision;
	std::array<uint32_t, 2> &bpps = context.configuration.sensor.bpps;

	/* Special cases: update ratio[4] to amend the unitary gain (u7.5). */
	if (!input0_.userConfig && bpps[0] == 12 && hwRevision == NEOISP_HW_V2) {
		if (mode != IPAModeTypeHdrMerge)
			input0_.ratios[4] = (1 << 5) * 16;
		else
			input0_.ratios[4] = (1 << 5) / 16;
	}

	if (!input0_.userConfig && bpps[0] == 10) {
		if (mode != IPAModeTypeHdrMerge)
			input0_.ratios[4] = (1 << 5);
		else
			input0_.ratios[4] = (1 << 5) * 4;
	}

	if (!input1_.userConfig && bpps[1] == 10) {
		if (mode != IPAModeTypeHdrMerge)
			input1_.ratios[4] = (1 << 5);
		else
			input1_.ratios[4] = (1 << 5) * 4;
	}

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
		<< "input0/1 user config "
		<< input0_.userConfig << "/" << input1_.userConfig;

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

REGISTER_IPA_ALGORITHM(HdrDecomp, "HdrDecomp")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
