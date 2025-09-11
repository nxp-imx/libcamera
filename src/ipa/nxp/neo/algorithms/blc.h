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

	struct Offsets {
		uint16_t red;
		uint16_t greenR;
		uint16_t greenB;
		uint16_t blue;
	};
	/* Offset values in supported obwb format: 16-bit and 20-bit formats */
	struct OffsetsObwbFormat {
		Offsets format16b;
		Offsets format20b;
	};
	const Offsets &offsets(uint16_t obwb) const;

private:
	uint16_t offsetToObwb(int16_t offset, uint16_t bpp) const;
	uint16_t adjustOffsetToBpp(uint16_t offset, uint32_t bpp) const;

	static const std::string kDefaultObwb;
	static const std::map<const std::string, std::vector<uint8_t>> kObwbMap;

	bool enabled_;
	/* Offset values associated to the reference bit-depth */
	OffsetsObwbFormat refOffsets_;
	/* Offsets values applicable to the current driver mode */
	OffsetsObwbFormat modeOffsets_;
	std::vector<uint8_t> obwbs_;

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
