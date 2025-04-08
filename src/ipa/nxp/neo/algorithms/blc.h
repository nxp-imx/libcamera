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
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     neoisp_meta_params_s *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const neoisp_meta_stats_s *stats,
		     ControlList &metadata) override;

private:
	uint16_t offsetToObwb(int16_t offset);
	uint16_t adjustOffsetToBpp(uint16_t offset, uint32_t bpp);

	/* Offset values in 16-bit format for a reference bit-depth */
	uint16_t offsetRed_ = 0;
	uint16_t offsetGreenR_ = 0;
	uint16_t offsetGreenB_ = 0;
	uint16_t offsetBlue_ = 0;

	/* Offset reference bit-depth */
	std::optional<uint32_t> referenceBitDepth_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
