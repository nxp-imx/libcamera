/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * rgbir.h - NXP NEO RGBIR to RGB,IR block configuration
 * Copyright 2024 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class RgbIr : public Algorithm
{
public:
	RgbIr() = default;
	~RgbIr() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     neoisp_meta_params_s *params) override;

private:
	struct IrCompression {
		std::vector<uint32_t> points;
		std::vector<uint32_t> offsets;
		std::vector<uint16_t> newpoints;
		std::vector<uint16_t> ratios;
	};

	static constexpr size_t kNumColorChannels = 3;
	std::vector<uint32_t> ccm_;
	std::vector<uint32_t> crossTalkThreshold_;

	static constexpr size_t kNumPoints = 4;
	static constexpr size_t kNumOffsets = 5;
	static constexpr size_t kNumNewPoints = 5;
	static constexpr size_t kNumRatios = 5;

	struct IrCompression irComp8bits_;
	struct IrCompression irComp16bits_;

	int parseIrCompression(const YamlObject &tuningData, const char *key,
			       struct IrCompression &irComp);
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
