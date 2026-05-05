/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO Black Level Correction
 *
 * Based on RkISP1 Black Level Correction control algorithm
 *     src/ipa/rkisp1/algorithms/blc.cpp
 * Copyright (C) 2021-2022, Ideas On Board
 */

#include "blc.h"

#include <algorithm>
#include <limits.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/value_node.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class BlackLevelCorrection
 * \brief NXP NEO Black Level Correction control
 *
 * Camera sensors do not output a zero value for the color channels of the black
 * pixels. Black Level Correction applies an offset in the ISP to each color
 * channel in order to shift each black pixel color channel to a zero value.
 * Libcamera convention is to represent the BLC offsets as signed values,
 * relevant to a 16-bit bit depth.
 * On NEO ISP, offsetting is done in the OBWB blocks of the ISP pipeline, either
 * in OBWB0/1 instances prior to the HDR-merge block, or in the OBWB2 instance
 * post HDR-merge.
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
 * At the output of the HDR Decomp blocks, the camera native bit depth is
 * expected to be:
 * - Rescaled to 20-bit (input0) and 16-bit (input1) when HDR-merge block is not
 *   used
 * - Native camera bit depth (no rescaling) when HDR-merge block is used,
 *   unless native bit depth is 10-bit where it would have been rescaled to
 *   12-bit to cope with OBWB saturation that requires at least 12-bit.
 *
 * Thus, those input formats are the ones relevant to OBWB0/1 instances.
 * Conversely, at the output of HDR-merge block, bit depth is expected to be
 * unconditionally 20-bit which is relevant to the OBWB2 instance input.
 *
 * When HDR-merge block is used to aggregate multiple captures, BLC is to be
 * applied before the merge as further gain will be applied by this block.
 * In other cases, either OBWB0/1 or OBWB2 may be used for BLC. Default OBWB
 * instances selected by the algorithm for BLC are the OBWB0/1. That can be
 * changed to select the OBWB2 via calibration file.
 *
 * The OBWB offset register values are defined as an unsigned 16-bit value,
 * that represents the offset directly applied to the block input bit depth.
 * BLC may share usage of the OBWB blocks with AWB, BLC configuring the
 * offsets and AWB configuring the gains. Thus, BLC also configures default
 * unitary gains in the OBWB blocks if they were not configured beforehand by
 * the AWB algorithm.
 *
 * Some sensors expose the same BLC digital value, for instance 64, when
 * operated from different driver modes having different bit-depth. In such
 * case, a calibration entry specifies the reference sensor bit depth
 * corresponding to the calibration value. That digital value will then be
 * applied to all pixel formats.
 *
 * Relevant keys in the BLC section of the calibration file:
 * R: offset for R channel (signed, 16-bit bit depth)
 * Gr: offset for Gr channel (signed, 16-bit bit depth)
 * Gb: offset for Gb channel (signed, 16-bit bit depth)
 * B: offset for B channel (signed, 16-bit bit depth)
 * reference-bitdepth: the camera mode bit-depth relevant to the offsets
 *  provided. If defined, the BLC will rescale the offsets applied according to
 *  bit-depth of the camera-mode selected for the stream.
 * obwb-blocks: the OBWB blocks where BLC offsets should apply - optional
 *              valid values: { "obwb0/1", "obwb2"}
 *              default value: "obwb0/1"
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoBlc)

const std::string BlackLevelCorrection::kDefaultObwb{ "obwb0/1" };

const std::map<const std::string, std::vector<uint8_t>> BlackLevelCorrection::kObwbMap = {
	{ "obwb0/1", { NEO_OBWB_LINE_PATH0, NEO_OBWB_LINE_PATH1 } },
	{ "obwb2", { NEO_OBWB_MERGE_PATH } },
};

BlackLevelCorrection::BlackLevelCorrection()
	: enabled_(false), obwbs_(kObwbMap.at(kDefaultObwb))
{
	setIrOps(IrOpPrepare);
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int BlackLevelCorrection::init(IPAContext &context, const ValueNode &tuningData)
{
	std::optional<int16_t> r = tuningData["R"].get<int16_t>();
	std::optional<int16_t> gR = tuningData["Gr"].get<int16_t>();
	std::optional<int16_t> gB = tuningData["Gb"].get<int16_t>();
	std::optional<int16_t> b = tuningData["B"].get<int16_t>();
	bool tuningHasLevels = r.has_value() && gR.has_value() &&
			       gB.has_value() && b.has_value();
	std::optional<uint32_t> referenceBitDepth =
		tuningData["reference-bitdepth"].get<uint32_t>();

	/* Valid raw format bit-depth is expected to be in the [8, 20] range */
	if (referenceBitDepth.has_value()) {
		uint32_t bitDepth = referenceBitDepth.value();
		referenceBitDepth_ = std::min(std::max(bitDepth, 8U), 20U);
		if (bitDepth != referenceBitDepth_.value())
			LOG(NxpNeoAlgoBlc, Warning)
				<< "Reference bit-depth was adjusted from "
				<< bitDepth << " to " << referenceBitDepth_.value();
	}

	/* Get the OBWB block(s) name from tuning file. */
	const std::string &obwb_name =
		tuningData["obwb-blocks"].get<std::string>().value_or(kDefaultObwb);

	/* Get the OBWB blocks where BLC offsets should apply. */
	auto it = kObwbMap.find(obwb_name);
	if (it != kObwbMap.end()) {
		obwbs_ = it->second;
		LOG(NxpNeoAlgoBlc, Debug) << "BLC offsets apply in " << it->first;
	} else {
		LOG(NxpNeoAlgoBlc, Warning)
			<< "BLC offsets are not applied! Invalid \"" << obwb_name
			<< "\" name from tuning file, should be \"obwb0/1\" or \"obwb2\"";
		enabled_ = false;
		return 0;
	}

	/*
	 * Give precedence to calibration file values if present.
	 * Fall back to using the values from the camHelper if present.
	 * When no offset is available, zero offset values will be applied.
	 */
	if (tuningHasLevels) {
		calibrationOffsets_[0] = static_cast<uint16_t>(r.value());
		calibrationOffsets_[1] = static_cast<uint16_t>(gR.value());
		calibrationOffsets_[2] = static_cast<uint16_t>(gB.value());
		calibrationOffsets_[3] = static_cast<uint16_t>(b.value());
	} else if (context.camHelper->blackLevel().has_value()) {
		uint16_t offset =
			static_cast<uint16_t>(context.camHelper->blackLevel().value_or(0));
		calibrationOffsets_[0] = static_cast<uint16_t>(offset);
		calibrationOffsets_[1] = static_cast<uint16_t>(offset);
		calibrationOffsets_[2] = static_cast<uint16_t>(offset);
		calibrationOffsets_[3] = static_cast<uint16_t>(offset);
	}

	enabled_ = true;

	LOG(NxpNeoAlgoBlc, Debug)
		<< "Calibration BLC offsets R " << calibrationOffsets_[0]
		<< " gR " << calibrationOffsets_[1]
		<< " gB " << calibrationOffsets_[2] << " B " << calibrationOffsets_[3]
		<< " Reference bit-depth " << referenceBitDepth_.value_or(0);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int BlackLevelCorrection::configure(IPAContext &context,
				    [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	if (!enabled_)
		return 0;

	std::array<uint32_t, kInputsCount> &bpps =
		context.configuration.sensor.bpps;
	auto &blcConfig = context.configuration.blc;

	/*
	 * Compute the BLC offset at sensor level for each ISP input. For each
	 * input, adjust the calibration offset that is defined for a 16-bit
	 * bit depth, to the actual bit depth in use by the current driver mode.
	 * Also, consider here the condition of the sensors that use the same
	 * BLC digital value for all pixel formats.
	 */
	std::array<ChannelArray<uint16_t>, kInputsCount> sensorOffsets;
	for (unsigned int input = 0; input < kInputsCount; input++) {
		unsigned int &bpp = bpps[input];
		int leftShift = bpp - 16;
		if (referenceBitDepth_)
			leftShift += referenceBitDepth_.value() - bpp;

		ChannelArray<uint16_t> &offsets = sensorOffsets[input];
		for (const auto &[channel, calibrationOffset] : utils::enumerate(calibrationOffsets_)) {
			if (leftShift >= 0)
				offsets[channel] = calibrationOffset << leftShift;
			else
				offsets[channel] = calibrationOffset >> (-leftShift);
		}
	}

	/*
	 * Compute the BLC offsets applicable to all the different OBWB blocks.
	 * At OBWB level, the offset equals to the sensor offset multiplied by
	 * the cumulated gain of the ISP upstream blocks. For OBWB0/1, upstream
	 * gains come from PIPECONF (LPALIGN) and the HDR Decomp blocks. For
	 * OBWB2, additional gain may come from HDR Merge block when enabled.
	 *
	 * The bit depth varies depending on the OBWB block instance and the
	 * pipeline operating mode:
	 * - OBWB0/1
	 *     - Without HDR merge: Fixed at 20-bit (input0) / 16-bit (input1)
	 *     - With HDR merge enabled: Uses native sensor bit depth
	 *       - Exception: 10-bit sensor data is expanded to 12-bit
	 * - OBWB2
	 *     - 20-bit unconditionally
	 */
	ObwbArray<unsigned int> gainLeftShift;
	IPAPipelineMode &mode = context.configuration.pipelineMode;
	if (mode != IPAPipelineMode::HdrMerge) {
		gainLeftShift[0] = 20 - bpps[0];
		gainLeftShift[1] = 16 - bpps[1];
	} else {
		gainLeftShift[0] = bpps[0] == 10 ? (12 - 10) : 0;
		gainLeftShift[1] = bpps[1] == 10 ? (12 - 10) : 0;
	}
	gainLeftShift[2] = 20 - bpps[0];

	auto applyGain = [](ChannelArray<uint16_t> &sensor,
			    ChannelArray<uint16_t> &obwb,
			    unsigned int leftShift) {
		uint16_t maxOffset = std::numeric_limits<uint16_t>::max() >> leftShift;
		for (unsigned channel = 0; channel < kObwbChannelsCount; channel++) {
			uint16_t offset = sensor[channel];
			if (offset > maxOffset) {
				LOG(NxpNeoAlgoBlc, Debug)
					<< "Offset too large " << offset
					<< " for shift " << leftShift
					<< " clamped at " << maxOffset;
				obwb[channel] = maxOffset << leftShift;
			} else {
				obwb[channel] = offset << leftShift;
			}
		}
	};

	for (unsigned int obwb = 0; obwb < kObwbInstancesCount; obwb++) {
		/*
		 * OBWB0/1 uses sensor offset from the respective sensor input
		 * paths. OBWB2 uses sensor offset from sensor input path0 as
		 * it is relevant to non-HDR merge cases.
		 */
		ChannelArray<uint16_t> &sensorOffset =
			obwb != 1 ? sensorOffsets[0] : sensorOffsets[1];
		ChannelArray<uint16_t> &obwbOffset = blcConfig.obwbOffsets[obwb];
		applyGain(sensorOffset, obwbOffset, gainLeftShift[obwb]);

		LOG(NxpNeoAlgoBlc, Debug)
			<< "Sensor mode BLC offsets OBWB" << +obwb
			<< " R " << obwbOffset[0] << " gR " << obwbOffset[1]
			<< " gB " << obwbOffset[2] << " B " << obwbOffset[3];
	}

	/*
	 * Store the BLC offsets to be reported in metadata.
	 * Metadata expects a 16-bit bit depth for the offset,
	 * so the sensor offsets values are rescaled accordingly.
	 */
	int leftShift = 16 - bpps[0];
	for (unsigned channel = 0; channel < kObwbChannelsCount; channel++) {
		uint16_t &sensorOffset = sensorOffsets[0][channel];
		if (leftShift >= 0)
			blcConfig.mdOffsets[channel] =
				static_cast<int32_t>(sensorOffset << leftShift);
		else
			blcConfig.mdOffsets[channel] =
				static_cast<int32_t>(sensorOffset >> (-leftShift));
	}

	/*
	 * Cache the OBWB obpp configuration that will be used at runtime.
	 * Assumption is that 20-bit (input0) and 16-bit (input1) bit depth
	 * is used in the ISP pipeline after HDR Decomp block. That is unless
	 * HDR merge block is enabled and sensor bit depth is used until merge.
	 */
	auto obpp = [](unsigned int ibpp) -> unsigned int {
		if (ibpp <= 12)
			return NEO_OBWB_OBPP_12BPP;
		else if (ibpp <= 14)
			return NEO_OBWB_OBPP_14BPP;
		else if (ibpp <= 16)
			return NEO_OBWB_OBPP_16BPP;
		else
			return NEO_OBWB_OBPP_20BPP;
	};

	if (mode != IPAPipelineMode::HdrMerge) {
		blcConfig.obwbObpp[0] = obpp(20);
		blcConfig.obwbObpp[1] = obpp(16);
	} else {
		blcConfig.obwbObpp[0] = obpp(bpps[0]);
		blcConfig.obwbObpp[1] = obpp(bpps[1]);
	}
	blcConfig.obwbObpp[2] = obpp(20);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void BlackLevelCorrection::prepare(IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   IPAFrameContext &frameContext,
				   NxpNeoParams *params)
{
	/*
	 * Although the BLC offsets are statically set, the params need to be
	 * updated for each frame since the AWB may share the same OBWB block
	 * to update the WB gains for each frame (dynamically).
	 */
	if (!enabled_)
		return;

	const auto &blcConfig = context.configuration.blc;

	for (const uint8_t &obwb : obwbs_) {
		struct neoisp_obwb_cfg_s *config;

		if (obwb == 0) {
			auto obwb0Config = params->block<BlockParamsType::Obwb0>();
			obwb0Config.setEnabled(true);
			obwb0Config->ctrl_obpp = blcConfig.obwbObpp[0];

			config = reinterpret_cast<neoisp_obwb_cfg_s *>(obwb0Config.params());
		} else if (obwb == 1) {
			auto obwb1Config = params->block<BlockParamsType::Obwb1>();
			obwb1Config.setEnabled(true);
			obwb1Config->ctrl_obpp = blcConfig.obwbObpp[1];

			config = reinterpret_cast<neoisp_obwb_cfg_s *>(obwb1Config.params());
		} else if (obwb == 2) {
			auto obwb2Config = params->block<BlockParamsType::Obwb2>();
			obwb2Config.setEnabled(true);
			obwb2Config->ctrl_obpp = blcConfig.obwbObpp[2];

			config = reinterpret_cast<neoisp_obwb_cfg_s *>(obwb2Config.params());
		} else {
			LOG(NxpNeoAlgoBlc, Warning) << "Invalid OBWB" << +obwb << " block,";
			continue;
		}

		const ChannelArray<uint16_t> &offsets = blcConfig.obwbOffsets[obwb];

		config->r_ctrl_offset = offsets[0];
		config->gr_ctrl_offset = offsets[1];
		config->gb_ctrl_offset = offsets[2];
		config->b_ctrl_offset = offsets[3];

		frameContext.blc.colorOffsetsSet[obwb] = true;

		if (!frameContext.awb.colorGainsSet[obwb]) {
			/* OBWB gain in u8.8 format */
			uint16_t gain = (1 << 8);
			config->r_ctrl_gain = gain;
			config->gr_ctrl_gain = gain;
			config->gb_ctrl_gain = gain;
			config->b_ctrl_gain = gain;
		}
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void BlackLevelCorrection::process(IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   [[maybe_unused]] IPAFrameContext &frameContext,
				   [[maybe_unused]] const NxpNeoStats *stats,
				   ControlList &metadata)
{
	if (!enabled_)
		return;
	const auto &blcConfig = context.configuration.blc;

	/* Report the offsets in 16-bit pixel format. */
	metadata.set(controls::SensorBlackLevels,
		     { blcConfig.mdOffsets[0], blcConfig.mdOffsets[1],
		       blcConfig.mdOffsets[2], blcConfig.mdOffsets[3] });
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
