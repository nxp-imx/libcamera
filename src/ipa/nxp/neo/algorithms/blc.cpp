/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * blc.cpp - NXP NEO Black Level Correction
 * Copyright 2025 NXP
 */

#include "blc.h"

#include <algorithm>

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
 * On NEO ISP, offsetting is done in the OBWB blocks of the ISP pipeline.
 * At that stage, the pixels can have a:
 * - a 20-bit format for line path0 and merge path (OBWB0 and OBWB2)
 * - or 16-bit format for line path1 (OBWB1)
 * regardless of the actual bayer pixel format output by the sensor.
 * Also, the OBWB offset definitions are defined with an unsigned 16-bit value,
 * that represent the offset directly applied to the pixel format.
 * BLC correction is typically applied in the OBWB0/1 blocks which is the
 * default value. However, there is the option to specify in the calibration file
 * that BLC correction should be moved to OBWB2 instead.
 * BLC may share usage of the OBWB blocks with AWB, BLC configuring the
 * offsets and AWB configuring the gains. Thus, BLC also configures default unitary
 * gains in the OBWB blocks if they were not configured beforehand by AWB.
 *
 * When the camera driver supports multiple modes with different bit-depth, the
 * black offset is impacted by the rescaling so the offset correction has to be
 * adjusted accordingly. For that purpose, an optional definition of the
 * reference camera mode bit-depth is doable to specify the actual bit-depth
 * relevant to the offset listed. When that information is present, the BLC
 * algorithm can infer the black offset correction applicable to the other
 * bit-depths.
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

	/* Get the OBWB block name from tuning file. */
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
		refOffsets_.format20b.red = offsetToObwb(r.value(), 20);
		refOffsets_.format20b.greenR = offsetToObwb(gR.value(), 20);
		refOffsets_.format20b.greenB = offsetToObwb(gB.value(), 20);
		refOffsets_.format20b.blue = offsetToObwb(b.value(), 20);
		refOffsets_.format16b.red = offsetToObwb(r.value(), 16);
		refOffsets_.format16b.greenR = offsetToObwb(gR.value(), 16);
		refOffsets_.format16b.greenB = offsetToObwb(gB.value(), 16);
		refOffsets_.format16b.blue = offsetToObwb(b.value(), 16);
	} else if (context.camHelper->blackLevel().has_value()) {
		int16_t offset20b = offsetToObwb(
			context.camHelper->blackLevel().value(), 20);
		refOffsets_.format20b.red = offset20b;
		refOffsets_.format20b.greenR = offset20b;
		refOffsets_.format20b.greenB = offset20b;
		refOffsets_.format20b.blue = offset20b;
		int16_t offset16b = offsetToObwb(
			context.camHelper->blackLevel().value(), 16);
		refOffsets_.format16b.red = offset16b;
		refOffsets_.format16b.greenR = offset16b;
		refOffsets_.format16b.greenB = offset16b;
		refOffsets_.format16b.blue = offset16b;
	}

	enabled_ = true;

	LOG(NxpNeoAlgoBlc, Debug)
		<< "Reference BLC offsets 20b format R " << refOffsets_.format20b.red
		<< " gR " << refOffsets_.format20b.greenR
		<< " gB " << refOffsets_.format20b.greenB << " B " << refOffsets_.format20b.blue
		<< " Reference bit-depth " << referenceBitDepth.value_or(0);
	LOG(NxpNeoAlgoBlc, Debug)
		<< "Reference BLC offsets 16b format R " << refOffsets_.format16b.red
		<< " gR " << refOffsets_.format16b.greenR
		<< " gB " << refOffsets_.format16b.greenB << " B " << refOffsets_.format16b.blue
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

	IPASessionConfiguration &config = context.configuration;
	uint32_t bpp = config.sensor.bpp;

	modeOffsets_.format20b.red = adjustOffsetToBpp(refOffsets_.format20b.red, bpp);
	modeOffsets_.format20b.greenR = adjustOffsetToBpp(refOffsets_.format20b.greenR, bpp);
	modeOffsets_.format20b.greenB = adjustOffsetToBpp(refOffsets_.format20b.greenB, bpp);
	modeOffsets_.format20b.blue = adjustOffsetToBpp(refOffsets_.format20b.blue, bpp);
	modeOffsets_.format16b.red = adjustOffsetToBpp(refOffsets_.format16b.red, bpp);
	modeOffsets_.format16b.greenR = adjustOffsetToBpp(refOffsets_.format16b.greenR, bpp);
	modeOffsets_.format16b.greenB = adjustOffsetToBpp(refOffsets_.format16b.greenB, bpp);
	modeOffsets_.format16b.blue = adjustOffsetToBpp(refOffsets_.format16b.blue, bpp);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void BlackLevelCorrection::prepare([[maybe_unused]] IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   IPAFrameContext &frameContext,
				   neoisp_meta_params_s *params)
{
	/*
	 * Although the BLC offsets are statically set, the params need to be
	 * updated for each frame since the AWB may share the same OBWB block
	 * to update the WB gains for each frame (dynamically).
	 */
	if (!enabled_)
		return;

	for (const uint8_t &obwb : obwbs_) {
		int obpp;
		const Offsets &offsets_ = offsets(obwb);
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
			LOG(NxpNeoAlgoBlc, Warning) << "Invalid OBWB" << +obwb << " block,";
			continue;
		}

		params->regs.obwb[obwb].ctrl_obpp = obpp;
		params->regs.obwb[obwb].r_ctrl_offset = offsets_.red;
		params->regs.obwb[obwb].gr_ctrl_offset = offsets_.greenR;
		params->regs.obwb[obwb].gb_ctrl_offset = offsets_.greenB;
		params->regs.obwb[obwb].b_ctrl_offset = offsets_.blue;

		frameContext.blc.colorOffsetsSet[obwb] = true;

		if (!frameContext.awb.colorGainsSet[obwb]) {
			uint16_t gain = (1 << 8);
			params->regs.obwb[obwb].r_ctrl_gain = gain;
			params->regs.obwb[obwb].gr_ctrl_gain = gain;
			params->regs.obwb[obwb].gb_ctrl_gain = gain;
			params->regs.obwb[obwb].b_ctrl_gain = gain;
		}

		if (frame == 0)
			LOG(NxpNeoAlgoBlc, Debug)
				<< "Sensor mode BLC offsets OBWB" << +obwb << " R " << offsets_.red
				<< " gR " << offsets_.greenR
				<< " gB " << offsets_.greenB << " B " << offsets_.blue;
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void BlackLevelCorrection::process([[maybe_unused]] IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   [[maybe_unused]] IPAFrameContext &frameContext,
				   [[maybe_unused]] const neoisp_meta_stats_s *stats,
				   ControlList &metadata)
{
	if (!enabled_)
		return;

	/* Report the offsets in 16-bit pixel format. */
	metadata.set(controls::SensorBlackLevels,
		     { static_cast<int32_t>(modeOffsets_.format16b.red),
		       static_cast<int32_t>(modeOffsets_.format16b.greenR),
		       static_cast<int32_t>(modeOffsets_.format16b.greenB),
		       static_cast<int32_t>(modeOffsets_.format16b.blue) });
}

/**
 * \brief Convert the BLC offset to the ISP OBWB format
 * \param[in] offset The offset value in libcamera standard signed 16-bit format
 * \param[in] bpp The pixel bpp at the input of the OBWB block
 *
 * Libcamera standard format for the BLC offset is a signed 16-bit value,
 * relevant to a 16-bit pixel format. Depending on the OBWB input pixel
 * format, the BLC offset is rescaled appropriately.
 * Moreover the offset applied in the OBWB blocks has to be positive so
 * the negative values read from the calibration parameters are filtered out.
 *
 * \return The BLC offset rescaled according to the ISP OBWB format, clamped to a
 * 16-bit value
 */
uint16_t BlackLevelCorrection::offsetToObwb(int16_t offset, uint16_t bpp) const
{
	uint32_t u32Offset = 0;

	/* Input format should be at least 16-bit format */
	if (bpp < 16)
		return u32Offset;

	/* OBWB only accepts positive values */
	if (offset >= 0)
		u32Offset = static_cast<uint32_t>(offset);

	/* Convert from 16-bit to input OBWB pixel format, limited to 16-bit range */
	unsigned int shift = bpp - 16;
	u32Offset <<= shift;
	if (u32Offset > std::numeric_limits<uint16_t>::max()) {
		LOG(NxpNeoAlgoBlc, Warning)
			<< "BLC offset " << u32Offset
			<< " has been clamped to 16-bit value";
		u32Offset = std::numeric_limits<uint16_t>::max();
	}

	return static_cast<uint16_t>(u32Offset);
}

/**
 * \brief Get the offsets to apply for a specific OBWB block
 * \param[in] obwb The id of the OBWB block
 *
 * The pixel format at the input of OBWB0 and OBWB2 blocks is 20-bit.
 * The pixel format at the input of OBWB1 block is 16-bit.
 *
 * \return The offsets converted in the input format of the OBWB block
 */
const BlackLevelCorrection::Offsets &BlackLevelCorrection::offsets(uint16_t obwb) const
{
	if (obwb == 1)
		return modeOffsets_.format16b;
	else
		return modeOffsets_.format20b;
}

/**
 * \brief Adjust the OBWB offset to the camera mode pixel bit-depth
 * \param[in] offset The offset value in OBWB format
 * \param[in] bpp The target camera bit-depth
 *
 * The BLC offset specified in the calibration file or in camHelper is relevant
 * to a given camera pixel format bit-depth. If this camera driver exposes other
 * modes with different bit-depth, the absolute offset to be applied by the
 * ISP after the rescaling to internal 20-bit format is to be adjusted
 * accordingly. When multiple bit-depth are used by the camera driver modes, a
 * reference bit-depth is used when provided to rescale the offsets depending on
 * the actual mode.
 *
 * \return The BLC offset in 20-bit unsigned ISP OBWB format, clamped to a
 * 16-bit value
 */
uint16_t BlackLevelCorrection::adjustOffsetToBpp(uint16_t offset, uint32_t bpp) const
{
	if (!referenceBitDepth_.has_value())
		return offset;

	int32_t shift = static_cast<int32_t>(bpp - referenceBitDepth_.value());
	if (std::abs(shift) > 32) {
		LOG(NxpNeoAlgoBlc, Warning) << "Invalid bit shift " << shift;
		shift = 0;
	}

	uint32_t u32Offset = offset;
	if (shift > 0)
		u32Offset >>= shift;
	else
		u32Offset <<= -shift;

	if (u32Offset > std::numeric_limits<uint16_t>::max()) {
		/* \todo Move log to Warning when BLC is relocated to OBWB0/1 */
		LOG(NxpNeoAlgoBlc, Debug)
			<< "BLC offset " << u32Offset
			<< " has been clamped to 16-bit value";
		u32Offset = std::numeric_limits<uint16_t>::max();
	}

	offset = static_cast<uint16_t>(u32Offset);
	return offset;
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
