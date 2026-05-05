/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO HDR Merge configuration
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
	HdrMerge();
	~HdrMerge() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     NxpNeoParams *params) override;

private:
	int parseCommonParams(const ValueNode &tuningData);
	int parseManualParams(const ValueNode &tuningData);
	int parseAutoParams(const ValueNode &tuningData);
	void computeParams(IPAContext &context);
	bool isPowerOf2(int n) const
	{
		return n > 0 && (n & (n - 1)) == 0;
	}

	static constexpr size_t kNumImages = 2;
	static constexpr size_t kNumThresholds = 2;
	static constexpr bool kAutoEnabled = true;

	/* Default block registers configuration values */
	static constexpr uint8_t kDefaultObppValue = 3;
	static constexpr uint8_t kDefaultMotionFixEn = 1;
	static constexpr uint8_t kDefaultBlend3x3 = 1;

	static constexpr uint8_t kDefaultLumaScaleThShift = 0;

	static constexpr uint8_t kDefaultDownscale0 = 0;
	static constexpr uint8_t kDefaultDownscale1 = 0;
	static constexpr uint8_t kDefaultUpscale0 = 0;
	static constexpr uint8_t kDefaultUpscale1 = 0;
	static constexpr uint8_t kDefaultPostscale = 0;

	/* Default parameters used for the computation of the configuration. */
	static constexpr uint16_t kBlendingWindowLow = 65;
	static constexpr uint16_t kBlendingWindowHigh = 95;
	static constexpr uint16_t kBlendingFactorMax = 256;

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

	/* Ratio between the long and the short captures. */
	uint16_t ratioL2S_;

	/* Blending window of image0 (needed for the auto computation). */
	std::vector<uint16_t> blendingWindow_;

	bool enabled_;
	bool autoEnabled_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
