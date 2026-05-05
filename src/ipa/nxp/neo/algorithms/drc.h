/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * NXP NEO DRC configuration
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class Drc : public Algorithm
{
public:
	Drc();
	~Drc() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
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
	enum class GlobalMode {
		Passthrough = 0, /* Data passes through unchanged */
		Static = 1, /* Use pre-configured LUT from tuning file */
		Dynamic = 2, /* Dynamically compute LUT from histogram */
	};

	struct ControlContext {
		struct Range {
			uint32_t min;
			uint32_t max;
		};

		struct History {
			static constexpr unsigned int kMaxSize = 10;
			std::array<uint32_t, kMaxSize> values;
			std::array<uint32_t, kMaxSize> bins;
			uint8_t pointer;
			uint32_t maxValue;
			uint32_t maxBin;
		};

		Range value;
		Range bin;
		History history;

		uint16_t globalDrcAlpha;
		uint16_t extraGainOut;
		uint16_t gamma;
		uint16_t gammaFixed;
	};

	struct LutVariables {
		std::array<float, NEO_DRC_GLOBAL_TONEMAP_SIZE> ratio;
		std::array<float, NEO_DRC_GLOBAL_TONEMAP_SIZE> hist;
		float nextRange;
		float effGamma;
		float maxRatio;
		float histEqSum;

		LutVariables()
			: nextRange(0.0f), effGamma(0.0f), maxRatio(0.0f),
			  histEqSum(0.0f)
		{
			ratio.fill(1.0f);
			hist.fill(0.0f);
		}
	};

	void configureGlobalContext();
	uint32_t binToLinear(uint32_t bin) const;
	void fixedModeLut();
	void getMinMax(const std::vector<uint32_t> &inputHistogram,
		       const uint32_t frame);
	void getMin(const std::vector<uint32_t> &inputHistogram);
	void getMax(const std::vector<uint32_t> &inputHistogram);
	void getHistoryMax();
	void controlDynamicMode(const std::vector<uint32_t> &inputHistogram);
	void dynamicModeSum(const std::vector<uint32_t> &inputHistogram,
			    LutVariables *lutVars) const;
	void effectiveGamma(LutVariables *lutVars) const;
	void lutFirstRun(LutVariables *lutVars);
	void lutSecondRun(LutVariables *lutVars);

	/* Control constants default values. May be overriden by config yaml */
	static constexpr uint16_t kGlobalMode = 0;

	/*
	 * Pixel count can be adjusted to steer the contrast:
	 *     - min adjustement avoids dark input regions
	 *     - max adjustement avoids bright input regions
	 */
	static constexpr uint32_t kMinPixelCount = 100;
	static constexpr uint32_t kMaxPixelCount = 100;

	static constexpr uint16_t kLocalStretchvalue = 256;
	static constexpr uint16_t kAlphaValue = 256;
	static constexpr uint16_t kGdrcAlphaValue = 128;
	static constexpr uint16_t kGlobalGain = 256;

	static constexpr uint16_t kGdrcGammaValue = 140;

	static constexpr uint32_t kHEThreshold = 2000;
	static constexpr float kHESaturation = 0.5f;

	/* Global DRC configuration */
	std::array<uint16_t, NEO_DRC_GLOBAL_TONEMAP_SIZE> globalLut_;
	std::array<uint16_t, NEO_DRC_GLOBAL_TONEMAP_SIZE> globalFixedLut_;

	uint16_t globalGain_;
	/* init global DRC mode */
	GlobalMode globalInitMode_;
	std::string restrictMode_;

	ControlContext globalContext_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
