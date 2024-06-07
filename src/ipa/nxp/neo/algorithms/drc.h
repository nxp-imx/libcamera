/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * drc.h - NXP NEO DRC configuration
 * Copyright 2024 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class Drc : public Algorithm
{
public:
	Drc();
	~Drc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     neoisp_meta_params_s *params) override;

private:
	/* global DRC configuration */
	bool gblLutEnabled_ = false;
	std::vector<uint16_t> gblLut_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
