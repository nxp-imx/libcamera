/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 AGC/AEC mean-based control algorithm
 *     src/ipa/rkisp1/algorithms/awb.h
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * awb.h - AWB control algorithm
 * Copyright 2024-2025 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include "libcamera/internal/vector.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class Awb : public Algorithm
{
public:
	Awb();
	~Awb() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     NxpNeoParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const NxpNeoStats *stats,
		     ControlList &metadata) override;

private:
	void generateBlocks(const NxpNeoStats *stats);
	void awbGreyWorld(IPAActiveState &activeState, IPAFrameContext &frameContext,
			  const uint32_t frame);
	static constexpr uint16_t gainDouble2Param(double gain);

	/*
	 * Number of frames for which to run the algorithm at full speed,
	 * before slowing down to prevent flickering effect.
	 */
	static constexpr uint32_t kNumStartupFrames = 10;

	/* OBWB instances: OBWB0, OBWB1 and OBWB2 */
	static constexpr unsigned int kObwbCount = 3;
	/* ISP inputs: Input0, Input1 */
	static constexpr unsigned int kInputsCount = 2;

	static const std::string kDefaultObwb;
	static const std::map<const std::string, std::vector<uint8_t>> kObwbMap;

	bool enabled_;
	std::optional<std::string> obwbUserConfig_;
	std::vector<RGB<double>> blocks_;
	std::vector<uint8_t> obwbs_;
	std::array<unsigned int, kObwbCount> obwbObpp_;
};

const std::string Awb::kDefaultObwb("obwb2");
const std::map<const std::string, std::vector<uint8_t>> Awb::kObwbMap = {
	{ "obwb0/1", { 0, 1 } },
	{ "obwb2", { 2 } },
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
