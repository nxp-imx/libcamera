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
	uint32_t estimateCCT(double red, double green, double blue);

	bool rgbMode_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
