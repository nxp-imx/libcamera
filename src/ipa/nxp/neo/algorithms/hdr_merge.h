/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * hdr_merge.h - NXP NEO HDR Merge configuration
 * Copyright 2025 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class HdrMerge : public Algorithm
{
public:
	HdrMerge() = default;
	~HdrMerge() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     NxpNeoParams *params) override;

private:
	static constexpr size_t kNumImages = 2;

	/* Default block registers configuration values */
	static constexpr uint8_t kDefaultObppValue = 3;
	static constexpr uint8_t kDefaultMotionFixEn = 0;
	static constexpr uint8_t kDefaultBlend3x3 = 0;
	static constexpr uint8_t kDefaultGainBpp0 = 3;
	static constexpr uint8_t kDefaultGainBpp1 = 3;

	static constexpr uint16_t kDefaultGainOffset0 = 0;
	static constexpr uint16_t kDefaultGainOffset1 = 0;
	static constexpr uint16_t kDefaultGainScale0 = 0x0008;
	static constexpr uint16_t kDefaultGainScale1 = 0x1000;
	static constexpr uint8_t kDefaultGainShift0 = 4;
	static constexpr uint8_t kDefaultGainShift1 = 12;

	static constexpr uint16_t kDefaultLumaTh0 = 0x0004;
	static constexpr uint16_t kDefaultLumaScale = 0x0100;
	static constexpr uint8_t kDefaultLumaScaleShift = 8;
	static constexpr uint8_t kDefaultLumaScaleThShift = 8;

	static constexpr uint8_t kDefaultDownscale0 = 8;
	static constexpr uint8_t kDefaultDownscale1 = 0;
	static constexpr uint8_t kDefaultUpscale0 = 0;
	static constexpr uint8_t kDefaultUpscale1 = 8;
	static constexpr uint8_t kDefaultPostscale = 0;

	uint8_t obpp_;
	uint8_t motionfixEn_;
	uint8_t blend3x3_;
	std::vector<uint8_t> gainBpp_;

	std::vector<uint16_t> gainOffset_;
	std::vector<uint16_t> gainScale_;
	std::vector<uint8_t> gainShift_;

	uint16_t lumaTh0_;
	uint16_t lumaScale_;
	uint8_t lumaScaleShift_;
	uint8_t lumaScaleThShift_;

	std::vector<uint8_t> downscale_;
	std::vector<uint8_t> upscale_;
	uint8_t postscale_;

	bool enabled_ = false;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
