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
 * It provides the static configuration for the global DRC look up table.
 */

LOG_DEFINE_CATEGORY(NxpNeoDrc)

Drc::Drc()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Drc::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	/*
	 * Global DRC lut parsing
	 */

	const YamlObject &obj = tuningData["gbl-lut"];
	if (!obj.size()) {
		LOG(NxpNeoDrc, Debug) << "global DRC lut not configured";
		return 0;
	}

	gblLut_ = obj.getList<uint16_t>()
			  .value_or(std::vector<uint16_t>{});
	if (gblLut_.size() != NEO_DRC_GLOBAL_TONEMAP_SIZE) {
		LOG(NxpNeoDrc, Error)
			<< "global lut list size must be " << NEO_DRC_GLOBAL_TONEMAP_SIZE;
		return -EINVAL;
	}

	gblLutEnabled_ = true;

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

	LOG(NxpNeoDrc, Debug) << "global lut configuration enabled " << gblLutEnabled_;

	if (gblLutEnabled_) {
		params->features_cfg.drc_global_tonemap_cfg = 1;

		memcpy(params->mems.gtm.drc_global_tonemap,
		       gblLut_.data(),
		       sizeof(params->mems.gtm.drc_global_tonemap));
	}
}

REGISTER_IPA_ALGORITHM(Drc, "Drc")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
