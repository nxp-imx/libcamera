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
 * Relevant keys in the BLC section of the calibration file:
 * Red: offset for R channel (signed, 16-bit pixel format)
 * GreenR: offset for Gr channel (signed, 16-bit pixel format)
 * GreenB: offset for Gb channel (signed, 16-bit pixel format)
 * Blue: offset for B channel (signed, 16-bit pixel format)
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

	/*
	 * Give precedence to calibration file values if present.
	 * Fall back to using the values from the camHelper if present.
	 * When no offset is available, zero offset values will be applied.
	 */
	if (tuningHasLevels) {
		offsetRed_ = offsetConvertFormat(r.value());
		offsetGreenR_ = offsetConvertFormat(gR.value());
		offsetGreenB_ = offsetConvertFormat(gB.value());
		offsetBlue_ = offsetConvertFormat(b.value());
	} else if (context.camHelper->blackLevel().has_value()) {
		int16_t offset =
			offsetConvertFormat(context.camHelper->blackLevel().value());
		offsetRed_ = offset;
		offsetGreenR_ = offset;
		offsetGreenB_ = offset;
		offsetBlue_ = offset;
	}

	LOG(NxpNeoAlgoBlc, Debug)
		<< "BLC offsets R " << offsetRed_ << " gR " << offsetGreenR_
		<< " gB " << offsetGreenB_ << " B " << offsetBlue_;

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
	/* Update channels offset configuration */
	params->features_cfg.obwb2_cfg = 1;
	params->regs.obwb[NEO_OBWB_MERGE_PATH].ctrl_obpp = NEO_OBWB_OBPP_20BPP;

	params->regs.obwb[NEO_OBWB_MERGE_PATH].r_ctrl_offset = offsetRed_;
	params->regs.obwb[NEO_OBWB_MERGE_PATH].gr_ctrl_offset = offsetGreenR_;
	params->regs.obwb[NEO_OBWB_MERGE_PATH].gb_ctrl_offset = offsetGreenB_;
	params->regs.obwb[NEO_OBWB_MERGE_PATH].b_ctrl_offset = offsetBlue_;
	frameContext.blc.colorOffsetsSet = true;

	/*
	 * When OBWB2 has not been configured by AWB, set some default gains.
	 * Unitary gain in U8.8 format is configured as default (no AWB).
	 */
	if (!frameContext.awb.colorGainsSet) {
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
	metadata.set(controls::SensorBlackLevels,
		     { static_cast<int32_t>(offsetRed_ >> shift),
		       static_cast<int32_t>(offsetGreenR_ >> shift),
		       static_cast<int32_t>(offsetGreenB_ >> shift),
		       static_cast<int32_t>(offsetBlue_ >> shift) });
}

/**
 * \brief Convert the BLC offset to the ISP OBWB format
 * \param[in] offset The offset value in standard signed 16-bit format
 *
 * \return The BLC offset in 20-bit unsigned ISP OBWB format
 */
uint16_t BlackLevelCorrection::offsetConvertFormat(int16_t offset)
{
	/* OBWB only accepts positive values */
	uint32_t u32Offset = 0;
	if (offset >= 0)
		u32Offset = static_cast<uint32_t>(offset);

	/* Convert from 16-bit to 20-bit pixel format, limited to 16-bit range */
	unsigned int shift = 20 - 16;
	u32Offset = u32Offset << shift;
	if (u32Offset > std::numeric_limits<uint16_t>::max())
		u32Offset = std::numeric_limits<uint16_t>::max();

	return static_cast<uint16_t>(u32Offset);
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
