/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * drc.h - NXP NEO DRC configuration
 * Copyright 2024-2025 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

struct DrcControlContext {
	static constexpr unsigned int kDrcMaxHistory = 10;
	uint32_t maxValue;
	uint32_t minValue;
	uint32_t maxBin;
	uint32_t minBin;
	uint32_t minPixelCount;
	uint32_t maxPixelCount;
	uint32_t maxHistory[kDrcMaxHistory];
	uint32_t maxBinHistory[kDrcMaxHistory];
	uint8_t historyPointer;
	uint32_t historyMax;
	uint32_t historyMaxBin;
	uint16_t globalDrcAlpha;
	uint16_t extraGainOut;
	uint16_t gamma;
	uint16_t gammaOld;
	uint16_t gammaFixed;
	uint32_t histEqThreshold;
	double histEqSaturation;
};

struct DrcLut {
	float ratio[NEO_DRC_GLOBAL_TONEMAP_SIZE];
	double hist[NEO_DRC_GLOBAL_TONEMAP_SIZE];
	float nextRange;
	float effGamma;
	float maxRatio;
	double histEqSum;
};

class Drc : public Algorithm
{
public:
	Drc();
	~Drc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     NxpNeoParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const NxpNeoStats *stats,
		     ControlList &metadata) override;

private:
	void initializeGblDrcContext();
	void configureGblDrcContext();
	uint32_t binToLinear(uint32_t aBin) const;
	void fixedModeLut();
	void getMinMax(const std::vector<uint32_t> &inputHistogram, const uint32_t frame);
	void getMin(const std::vector<uint32_t> &inputHistogram);
	void getMax(const std::vector<uint32_t> &inputHistogram);
	void getHistoryMax();
	float applyNewPreGain();
	void controlDynamicMode(const std::vector<uint32_t> &inputHistogram,
				std::vector<uint16_t> &lut,
				uint16_t *extraGainOut);
	void dynamicModeSum(const std::vector<uint32_t> &inputHistogram, DrcLut *lutVars) const;
	void effectiveGamma(DrcLut *lutVars) const;
	uint16_t lutFirstRun(DrcLut *lutVars);
	void lutSecondRun(std::vector<uint16_t> &lut, DrcLut *lutVars);

	/* Initial values of control constants. May be overriden by config yaml */
	static constexpr uint16_t kGblMode = 0;

	static constexpr uint16_t kLocalStretchvalue = 256;
	static constexpr uint16_t kAlphaValue = 256;
	static constexpr uint16_t kGdrcAlphaValue = 256;
	static constexpr uint16_t kGblGain = 256;

	static constexpr uint16_t kGdrcGammaValue = 256;

	static constexpr uint16_t kHEThreshold = 2000;
	static constexpr float kHESaturation = 0.5;

	/* Global DRC configuration */
	std::vector<uint16_t> gblLut_;

	uint16_t gblGain_;
	uint16_t gblMode_;

	DrcControlContext gblDrcContext_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
