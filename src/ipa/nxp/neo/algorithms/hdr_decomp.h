/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * hdr_decomp.h - NXP NEO HDR Decompression configuration
 * Copyright 2024 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class HdrDecomp : public Algorithm
{
public:
	HdrDecomp();
	~HdrDecomp() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     neoisp_meta_params_s *params) override;

private:
	static constexpr size_t kNumPoints = 4;
	static constexpr size_t kNumOffsets = 5;
	static constexpr size_t kNumNewPoints = 5;
	static constexpr size_t kNumRatios = 5;

	struct {
		bool enabled = false;
		std::vector<uint16_t> points;
		std::vector<uint16_t> offsets;
		std::vector<uint32_t> newpoints;
		std::vector<uint16_t> ratios;
	} input0_;

	struct {
		bool enabled = false;
		std::vector<uint16_t> points;
		std::vector<uint16_t> offsets;
		std::vector<uint16_t> newpoints;
		std::vector<uint16_t> ratios;
	} input1_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
