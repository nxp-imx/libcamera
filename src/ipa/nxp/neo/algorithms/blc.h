/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * blc.h - NXP NEO Black Level Correction
 * Copyright 2025 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class BlackLevelCorrection : public Algorithm
{
public:
	BlackLevelCorrection(){};
	~BlackLevelCorrection() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     neoisp_meta_params_s *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const neoisp_meta_stats_s *stats,
		     ControlList &metadata) override;

private:
	uint16_t offsetConvertFormat(int16_t offset);

	/* Offset values for 20-bit pixel format, 16-bit range */
	uint16_t offsetRed_ = 0;
	uint16_t offsetGreenR_ = 0;
	uint16_t offsetGreenB_ = 0;
	uint16_t offsetBlue_ = 0;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
