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
 * On NEO ISP, offsetting is done in the OBWB2 block of the ISP pipeline. At
 * that stage, the pixels have a 20-bit formats, regardless of the actual bayer
 * pixel format output by the sensor. Also, the OBWB2 offset definitions are
 * defined with an unsigned 16-bit value, that represent the offset directly
 * applied to the 20-bit pixels.
 * BLC shares usage of the OBWB2 block with AWB, BLC configuring the offsets and
 * AWB configuring the gains. Thus, BLC also configures default unitary gains in
 * the OWB2 block if they were not configured beforehand by AWB.
 *
 * The BLC correction is applied after rescaling the pixels to a 20-bit format.
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
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoBlc)

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

	/*
	 * Give precedence to calibration file values if present.
	 * Fall back to using the values from the camHelper if present.
	 * When no offset is available, zero offset values will be applied.
	 */
	if (tuningHasLevels) {
		offsetRed_ = offsetToObwb(r.value());
		offsetGreenR_ = offsetToObwb(gR.value());
		offsetGreenB_ = offsetToObwb(gB.value());
		offsetBlue_ = offsetToObwb(b.value());
	} else if (context.camHelper->blackLevel().has_value()) {
		int16_t offset = offsetToObwb(
			context.camHelper->blackLevel().value());
		offsetRed_ = offset;
		offsetGreenR_ = offset;
		offsetGreenB_ = offset;
		offsetBlue_ = offset;
	}

	LOG(NxpNeoAlgoBlc, Debug)
		<< "BLC offsets R " << offsetRed_ << " gR " << offsetGreenR_
		<< " gB " << offsetGreenB_ << " B " << offsetBlue_ << " Reference bit-depth "
		<< referenceBitDepth.value_or(0);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int BlackLevelCorrection::configure(IPAContext &context,
				    [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	IPASessionConfiguration &config = context.configuration;
	uint32_t bpp = config.sensor.bpp;

	config.blc.offsetRed_ = adjustOffsetToBpp(offsetRed_, bpp);
	config.blc.offsetGreenR_ = adjustOffsetToBpp(offsetGreenR_, bpp);
	config.blc.offsetGreenB_ = adjustOffsetToBpp(offsetGreenB_, bpp);
	config.blc.offsetBlue_ = adjustOffsetToBpp(offsetBlue_, bpp);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void BlackLevelCorrection::prepare(IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   IPAFrameContext &frameContext,
				   neoisp_meta_params_s *params)
{
	IPASessionConfiguration &config = context.configuration;

	/* Update channels offset configuration */
	params->features_cfg.obwb2_cfg = 1;
	params->regs.obwb[NEO_OBWB_MERGE_PATH].ctrl_obpp = NEO_OBWB_OBPP_20BPP;

	params->regs.obwb[NEO_OBWB_MERGE_PATH].r_ctrl_offset =
		config.blc.offsetRed_;
	params->regs.obwb[NEO_OBWB_MERGE_PATH].gr_ctrl_offset =
		config.blc.offsetGreenR_;
	params->regs.obwb[NEO_OBWB_MERGE_PATH].gb_ctrl_offset =
		config.blc.offsetGreenB_;
	params->regs.obwb[NEO_OBWB_MERGE_PATH].b_ctrl_offset =
		config.blc.offsetBlue_;
	frameContext.blc.colorOffsetsSet = true;

	/*
	 * When OBWB2 has not been configured by AWB, set some default gains.
	 * Unitary gain in U8.8 format is configured as default (no AWB).
	 */
	if (!frameContext.awb.colorGainsSet[NEO_OBWB_MERGE_PATH]) {
		uint16_t gain = (1 << 8);
		params->regs.obwb[NEO_OBWB_MERGE_PATH].r_ctrl_gain = gain;
		params->regs.obwb[NEO_OBWB_MERGE_PATH].gr_ctrl_gain = gain;
		params->regs.obwb[NEO_OBWB_MERGE_PATH].gb_ctrl_gain = gain;
		params->regs.obwb[NEO_OBWB_MERGE_PATH].b_ctrl_gain = gain;
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
	/* Report 20-bit offsets in 16-bit pixel format */
	unsigned int shift = 20 - 16;
	IPASessionConfiguration &config = context.configuration;
	metadata.set(controls::SensorBlackLevels,
		     { static_cast<int32_t>(config.blc.offsetRed_ >> shift),
		       static_cast<int32_t>(config.blc.offsetGreenR_ >> shift),
		       static_cast<int32_t>(config.blc.offsetGreenB_ >> shift),
		       static_cast<int32_t>(config.blc.offsetBlue_ >> shift) });
}

/**
 * \brief Convert the BLC offset to the ISP OBWB format
 * \param[in] offset The offset value in libcamera standard signed 16-bit format
 *
 * Libcamera standard format for the BLC offset is a signed 16-bit value,
 * relevant to a 16-bit pixel format. ISP internal pipeline being 20-bit, the
 * input pixels with their BLC offset are rescaled appropriately. Moreover the
 * offset applied in the OBWB blocks has to be positive so the negative values
 * read from the calibration parameters are filtered out.
 *
 * \return The BLC offset in 20-bit unsigned ISP OBWB format, clamped to a
 * 16-bit value
 */
uint16_t BlackLevelCorrection::offsetToObwb(int16_t offset)
{
	/* OBWB only accepts positive values */
	uint32_t u32Offset = 0;
	if (offset >= 0)
		u32Offset = static_cast<uint32_t>(offset);

	/* Convert from 16-bit to 20-bit pixel format, limited to 16-bit range */
	unsigned int shift = 20 - 16;
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
uint16_t BlackLevelCorrection::adjustOffsetToBpp(uint16_t offset, uint32_t bpp)
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
