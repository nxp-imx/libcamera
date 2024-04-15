/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 AGC/AEC mean-based control algorithm
 *     src/ipa/rkisp1/algorithms/awb.h
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * awb.h - AWB control algorithm
 * Copyright 2024 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class Awb : public Algorithm
{
public:
	Awb();
	~Awb() = default;

	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     neoisp_meta_params_s *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const neoisp_meta_stats_s *stats,
		     ControlList &metadata) override;

private:
	/* \todo Make these structs available to all the ISPs ?
		this is reused from IPU3 algorithm */
	struct RGB {
		RGB(double _R = 0, double _G = 0, double _B = 0)
			: R(_R), G(_G), B(_B)
		{
		}
		double R, G, B;
		RGB &operator+=(RGB const &other)
		{
			R += other.R, G += other.G, B += other.B;
			return *this;
		}
	};

private:
	void generateBlocks(const neoisp_meta_stats_s *stats);
	void awbGreyWorld(IPAActiveState &activeState, IPAFrameContext &frameContext);
	uint32_t estimateCCT(double red, double green, double blue);
	static constexpr uint16_t gainDouble2Param(double gain);

	std::vector<RGB> blocks_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
