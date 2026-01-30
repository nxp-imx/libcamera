/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO Black Level Correction
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include "algorithm.h"
#include "neoisp-definitions.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class BlackLevelCorrection : public Algorithm
{
public:
	BlackLevelCorrection();
	~BlackLevelCorrection() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     NxpNeoParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const NxpNeoStats *stats,
		     ControlList &metadata) override;

private:
	static const std::string kDefaultObwb;
	static const std::map<const std::string, std::vector<uint8_t>> kObwbMap;

	/* ISP inputs: Input0, Input1 */
	static constexpr unsigned int kInputsCount = 2;

	bool enabled_;
	std::vector<uint8_t> obwbs_;

	/* BLC offset values from calibration (16-bit bit depth) */
	ChannelArray<uint16_t> calibrationOffsets_;

	/* Offset reference bit-depth */
	std::optional<uint32_t> referenceBitDepth_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
