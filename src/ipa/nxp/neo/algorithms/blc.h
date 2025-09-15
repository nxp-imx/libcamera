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
	BlackLevelCorrection();
	~BlackLevelCorrection() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
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

	/* Color channels: R, Gr, Gb, B */
	static constexpr unsigned int kChannelsCount = 4;
	/* ISP inputs: Input0, Input1 */
	static constexpr unsigned int kInputsCount = 2;
	/* OBWB instances: OBWB0, OBWB1 and OBWB2 */
	static constexpr unsigned int kObwbCount = 3;

	bool enabled_;
	std::vector<uint8_t> obwbs_;

	/* Color channels offsets: R, Gr, Gb, B */
	template<class T>
	using ChannelOffsets = std::array<T, kChannelsCount>;

	/* BLC offset values from calibration (16-bit pixel format) */
	ChannelOffsets<uint16_t> calibrationOffsets_;

	/* OBWB instances BLC offsets and obpp values */
	std::array<ChannelOffsets<uint16_t>, kObwbCount> obwbOffsets_;
	std::array<unsigned int, kObwbCount> obwbObpp_;

	/* BLC offset reported in metadata format */
	ChannelOffsets<int32_t> mdOffsets_;

	/* Offset reference bit-depth */
	std::optional<uint32_t> referenceBitDepth_;
};

const std::string BlackLevelCorrection::kDefaultObwb{ "obwb0/1" };
const std::map<const std::string, std::vector<uint8_t>> BlackLevelCorrection::kObwbMap = {
	{ "obwb0/1", { 0, 1 } },
	{ "obwb2", { 2 } },
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
