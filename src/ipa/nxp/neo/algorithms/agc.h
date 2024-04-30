/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 AGC/AEC mean-based control algorithm
 *     src/ipa/rkisp1/algorithms/agc.h
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * agc.h - NXP NEO AGC/AEC mean-based control algorithm
 * Copyright 2024 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "libipa/agc_mean_luminance.h"
#include "libipa/histogram.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class Agc : public Algorithm, public AgcMeanLuminance
{
public:
	Agc();
	~Agc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context,
			  const uint32_t frame,
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
	double estimateLuminance(double gain) const override;
	void fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
			  ControlList &metadata);
	Histogram parseStatistics(const neoisp_meta_stats_s *stats);

	double rGain_;
	double gGain_;
	double bGain_;
	std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> rgbTriples_;

	static constexpr size_t kNumHist = 4;
	std::vector<uint32_t> histScale_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
