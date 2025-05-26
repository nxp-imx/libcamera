/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Lens Shading Correction control algorithm
 *     src/ipa/rkisp1/algorithms/lsc.h
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * lsc.h NXP NEO Lens Shading Correction control
 * Copyright 2025 NXP
 */

#pragma once

#include <map>

#include "libipa/interpolator.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class LensShadingCorrection : public Algorithm
{
public:
	LensShadingCorrection();
	~LensShadingCorrection() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     neoisp_meta_params_s *params) override;

	enum Status {
		ENABLED = 0,
		CONFIGURED = 1,
		NOT_CONFIGURED = 2
	};

	struct Components {
		uint32_t ct;
		std::vector<uint16_t> r;
		std::vector<uint16_t> g;
		std::vector<uint16_t> b;
	};
	using BlockCount = std::pair<uint16_t, uint16_t>;
	using BlockCountMap = std::map<Size, std::optional<BlockCount>>;
	using SetMap = std::map<Size, ipa::Interpolator<Components>>;

private:
	void copyTable(neoisp_vignetting_table_mem_params_s &vt,
		       const Components &set);
	const std::optional<BlockCount> blockCount(Size resolution) const;
	const ipa::Interpolator<Components> sets(Size resolution) const;

	static constexpr uint32_t kScalingFractionalSize = (1 << 15);

	Status status_;
	ipa::Interpolator<Components> sets_;
	unsigned int lastAppliedCt_;
	unsigned int lastAppliedQuantizedCt_;

	BlockCountMap blockCntMap_;
	SetMap setsMap_;
	/* Horizontal block count */
	uint16_t blockCountX_;
	/* Vertical block count */
	uint16_t blockCountY_;
	/* Number of pixels per block */
	uint16_t blockWidth_;
	/* Number of rows per block */
	uint16_t blockHeight_;
	/* Horizontal scaling factor for each pixel within the block (u1.15) */
	uint16_t blockStepX_;
	/* Vertical scaling factor for each line within the block (u1.15) */
	uint16_t blockStepY_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
