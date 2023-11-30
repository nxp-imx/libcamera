/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * drc.cpp - NXP NEO DRC configuration
 * Copyright 2024 NXP
 */

#include "drc.h"

#include <algorithm>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file drc.cpp
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class Drc
 * \brief DRC configuration
 *
 * This Algorithm configures the DRC unit.
 * It provides the static configuration for the global DRC operation.
 *
 * The DRC unit compresses the bit depth from 20 bit (used in the
 * ISP pipeline) to 16 bits.
 * The DRC operation controls the brightness of the output image.
 *
 * The DRC Global LUT (Tonemap) can be configured in yaml file using
 * the "gbl-lut" attribute defining a list [1-416] of u16 values with 8.8 format.
 * If no configuration is set, default values from the driver are used.
 *
 * The DRC Global gain is applied at the output of the global tonemapping step
 * where all entries of the Global LUT are multiplied by this gain.
 * It is used to compensate for the dark scene of an HDR sensor.
 * The global gain can be configured in yaml file using the "gbl-gain" attribute
 * defining an u16 value with 8.8 format.
 * If no configuration is set, the default value set by kGblGain is used.
 *
 * The following DRC operation is applied on each YUV component.
 * For the Y component: Y_GDRC = Y_IN * LUT[Linear2Bin(Y_IN)] * DRC_GBL_GAIN >> 4
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoDrc)

Drc::Drc()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Drc::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	/* Global DRC lut parsing */
	const YamlObject &lut = tuningData["gbl-lut"];
	if (lut.size()) {
		gblLutEnabled_ = true;

		gblLut_ = lut.getList<uint16_t>()
				  .value_or(std::vector<uint16_t>{});
		if (gblLut_.size() != NEO_DRC_GLOBAL_TONEMAP_SIZE) {
			LOG(NxpNeoAlgoDrc, Error) << "global lut list size must be "
					      << NEO_DRC_GLOBAL_TONEMAP_SIZE;
			return -EINVAL;
		}
	} else
		LOG(NxpNeoAlgoDrc, Debug) << "global DRC lut not configured";

	/* Global DRC gain parsing */
	gblGain_ = tuningData["gbl-gain"].get<uint16_t>(kGblGain);

	LOG(NxpNeoAlgoDrc, Debug) << "global GAIN=" << gblGain_;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Drc::prepare([[maybe_unused]] IPAContext &context, const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  neoisp_meta_params_s *params)
{
	if (frame > 0)
		return;

	LOG(NxpNeoAlgoDrc, Debug) << "global lut configuration enabled " << gblLutEnabled_;
	if (gblLutEnabled_) {
		/* Set global lut */
		params->features_cfg.drc_global_tonemap_cfg = 1;

		memcpy(params->mems.gtm.drc_global_tonemap,
		       gblLut_.data(),
		       sizeof(params->mems.gtm.drc_global_tonemap));
	}

	/* Set global gain */
	params->features_cfg.dr_comp_cfg = 1;
	params->regs.drc.lcl_stretch_stretch = kLocalStretchvalue;
	params->regs.drc.alpha_alpha = kAlphaValue;
	params->regs.drc.gbl_gain_gain = gblGain_;
}

REGISTER_IPA_ALGORITHM(Drc, "Drc")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
