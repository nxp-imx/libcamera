/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * blc.cpp - NXP NEO Black Level Correction
 * Copyright 2025 NXP
 */

#include "blc.h"

#include <algorithm>
#include <limits.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

#include "nxp-neoisp-enums.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class BlackLevelCorrection
 * \brief NXP NEO Black Level Correction control
 *
 * Camera sensors do no output a zero value for the color channels of the black
 * pixels. Black Level Correction applies an offset in the ISP to each color
 * channel in order to shift each black pixel color channel to a zero value.
 * Libcamera convention is to represent the BLC offsets as signed values,
 * relevant to a 16-bit pixel format.
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
 * At the output of the HDR Decomp blocks, the camera pixel native format is
 * expected to be:
 * - Rescaled to 20-bit (input0) and 16-bit (input1) when HDR-merge block is not
 *   used
 * - Native camera pixel format (no rescaling) when HDR-merge block is used.
 * Thus, those input formats are the ones relevant to OBWB0/1 instances.
 * Conversely, at the output of HDR-merge block, pixel format is expected to be
 * unconditionally 20-bit which is relevant to the OBWB2 instance input.
 *
 * When HDR-merge block is used to aggregate multiple captures, BLC is to be
 * applied before the merge as further gain will be applied by this block.
 * In other cases, either OBWB0/1 or OBWB2 may be used for BLC. Default OBWB
 * instances selected by the algorithm for BLC are the OBWB0/1. That can be
 * changed to select the OBWB2 via calibration file.
 *
 * The OBWB offset register values are defined as an unsigned 16-bit value,
 * that represents the offset directly applied to the block input pixel format.
 * BLC may share usage of the OBWB blocks with AWB, BLC configuring the
 * offsets and AWB configuring the gains. Thus, BLC also configures default
 * unitary gains in the OBWB blocks if they were not configured beforehand by
 * the AWB algorithm.
 *
 * Some sensors expose the same BLC digital value, for instance 64, when
 * operated from different driver modes having different bit-depth. In such
 * case, a calibration entry specifies the reference sensor pixel format
 * corresponding to the calibration value. That digital value will then be
 * applied to all pixel formats.
 *
 * Relevant keys in the BLC section of the calibration file:
 * R: offset for R channel (signed, 16-bit pixel format)
 * Gr: offset for Gr channel (signed, 16-bit pixel format)
 * Gb: offset for Gb channel (signed, 16-bit pixel format)
 * B: offset for B channel (signed, 16-bit pixel format)
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
	{ "obwb0/1", { 0, 1 } },
	{ "obwb2", { 2 } },
};

BlackLevelCorrection::BlackLevelCorrection()
	: enabled_(false), obwbs_(kObwbMap.at(kDefaultObwb))
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int BlackLevelCorrection::init(IPAContext &context, const YamlObject &tuningData)
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
		<< " Reference bit-depth " << referenceBitDepth.value_or(0);

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

	/*
	 * Compute the BLC offset at sensor level for each ISP input. For each
	 * input, adjust the calibration offset that is defined for a 16-bit
	 * format, to the actual pixel format in use by the current driver mode.
	 * Also, consider here the condition of the sensors that use the same
	 * BLC digital value for all pixel formats.
	 */
	std::array<ChannelOffsets<uint16_t>, kInputsCount> sensorOffsets;
	for (unsigned int input = 0; input < kInputsCount; input++) {
		unsigned int &bpp = bpps[input];
		int leftShift = bpp - 16;
		if (referenceBitDepth_)
			leftShift += referenceBitDepth_.value() - bpp;

		ChannelOffsets<uint16_t> &offsets = sensorOffsets[input];
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
	 * Assumption is that the internal pixel format at the input of OBWB
	 * blocks is:
	 * - OBWB0/1
	 *     - 20/16-bit (input0/input1) for operation without HDR merge
	 *     - The native sensor format (input0/input1) when HDR merge enabled
	 * - OBwB2
	 *     - 20-bit unconditionally
	 */
	std::array<unsigned int, kObwbCount> gainLeftShift;
	IPAModeType &mode = context.configuration.pipelineMode;
	if (mode != IPAModeTypeHdrMerge) {
		gainLeftShift[0] = 20 - bpps[0];
		gainLeftShift[1] = 16 - bpps[1];
	} else {
		gainLeftShift[0] = 0;
		gainLeftShift[1] = 0;
	}
	gainLeftShift[2] = 20 - bpps[0];

	auto applyGain = [](ChannelOffsets<uint16_t> &sensor,
			    ChannelOffsets<uint16_t> &obwb,
			    unsigned int leftShift) {
		uint16_t maxOffset = std::numeric_limits<uint16_t>::max() >> leftShift;
		for (unsigned channel = 0; channel < kChannelsCount; channel++) {
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

	for (unsigned int obwb = 0; obwb < kObwbCount; obwb++) {
		/*
		 * OBWB0/1 uses sensor offset from the respective sensor input
		 * paths. OBWB2 uses sensor offset from sensor input path0 as
		 * it is relevant to non-HDR merge cases.
		 */
		ChannelOffsets<uint16_t> &sensorOffset =
			obwb != 1 ? sensorOffsets[0] : sensorOffsets[1];
		ChannelOffsets<uint16_t> &obwbOffset = obwbOffsets_[obwb];
		applyGain(sensorOffset, obwbOffset, gainLeftShift[obwb]);
	}

	/*
	 * Store the BLC offsets to be reported in metadata. Metadata expects a
	 * 16-bit pixel format for the offset, so the sensor offsets values are
	 * rescaled accordingly.
	 */
	int leftShift = 16 - bpps[0];
	for (unsigned channel = 0; channel < kChannelsCount; channel++) {
		uint16_t &sensorOffset = sensorOffsets[0][channel];
		if (leftShift >= 0)
			mdOffsets_[channel] =
				static_cast<int32_t>(sensorOffset << leftShift);
		else
			mdOffsets_[channel] =
				static_cast<int32_t>(sensorOffset >> (-leftShift));
	}

	/*
	 * Cache the OBWB obpp configuration that will be used at runtime.
	 * Assumption is that 20-bit (input0) and 16-bit (input1) pixel format
	 * is used in the ISP pipeline after HDR Decomp block. That is unless
	 * HDR merge block is enabled and sensor pixel format is used until
	 * merge.
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

	if (mode != IPAModeTypeHdrMerge) {
		obwbObpp_[0] = obpp(20);
		obwbObpp_[1] = obpp(16);
	} else {
		obwbObpp_[0] = obpp(bpps[0]);
		obwbObpp_[1] = obpp(bpps[1]);
	}
	obwbObpp_[2] = obpp(20);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void BlackLevelCorrection::prepare([[maybe_unused]] IPAContext &context,
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

	auto obwb0Config = params->block<BlockParamsType::Obwb0>();
	auto obwb1Config = params->block<BlockParamsType::Obwb1>();
	auto obwb2Config = params->block<BlockParamsType::Obwb2>();

	const std::array<neoisp_obwb_cfg_s *, 3> obwbBlocks = {
		reinterpret_cast<neoisp_obwb_cfg_s *>(obwb0Config.data().data()),
		reinterpret_cast<neoisp_obwb_cfg_s *>(obwb1Config.data().data()),
		reinterpret_cast<neoisp_obwb_cfg_s *>(obwb2Config.data().data()),
	};

	for (const uint8_t &obwb : obwbs_) {
		if (obwb == 0) {
			obwb0Config.setUpdate(true);
			obwb0Config->ctrl_obpp = obwbObpp_[0];
		} else if (obwb == 1) {
			obwb1Config.setUpdate(true);
			obwb1Config->ctrl_obpp = obwbObpp_[1];
		} else if (obwb == 2) {
			obwb2Config.setUpdate(true);
			obwb2Config->ctrl_obpp = obwbObpp_[2];
		} else {
			LOG(NxpNeoAlgoBlc, Warning) << "Invalid OBWB" << +obwb << " block,";
			continue;
		}

		neoisp_obwb_cfg_s *config = obwbBlocks[obwb];
		const ChannelOffsets<uint16_t> &offsets = obwbOffsets_[obwb];

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

		if (frame == 0)
			LOG(NxpNeoAlgoBlc, Debug)
				<< "Sensor mode BLC offsets OBWB" << +obwb << " R " << offsets[0]
				<< " gR " << offsets[1]
				<< " gB " << offsets[2] << " B " << offsets[3];
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void BlackLevelCorrection::process([[maybe_unused]] IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   [[maybe_unused]] IPAFrameContext &frameContext,
				   [[maybe_unused]] const NxpNeoStats *stats,
				   ControlList &metadata)
{
	if (!enabled_)
		return;

	/* Report the offsets in 16-bit pixel format. */
	metadata.set(controls::SensorBlackLevels,
		     { mdOffsets_[0], mdOffsets_[1], mdOffsets_[2], mdOffsets_[3] });
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
