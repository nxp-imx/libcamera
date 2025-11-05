/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * drc.cpp - NXP NEO DRC configuration
 * Copyright 2024-2025 NXP
 */

#include "drc.h"

#include <algorithm>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file drc.cpp
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class Drc
 * \brief DRC configuration
 *
 * This Algorithm configures the DRC unit.
 * It provides the configuration for the global DRC operation.
 *
 * The DRC unit compresses the bit depth from 20 bit (used in the
 * ISP pipeline) to 16 bits.
 * The DRC operation controls the brightness of the output image.
 *
 * The following DRC operation is applied on each YUV component.
 * For the Y component: Y_GDRC = Y_IN * LUT[Linear2Bin(Y_IN)] * DRC_GBL_GAIN >> 4
 * Through the configuration of the LUT, the DRC unit can amplify or attenuate
 * the input Y value. To this end, the input value is assigned to a histogram bin,
 * and the LUT entry valid for the particular bin is applied to the input value.
 * This algorithm calculates the LUT entries.
 *
 * Three modes of operation are supported to determine the global DRC lookup table,
 * as specified by the "gbl-mode" attribute:
 * "gbl-mode: 0" is the Copy mode. In this case, the generated lookup table only
 * contains values corresponding to unit gain, which means no effective dynamic range
 * compression is done and the data passes through as is. This mode is especially
 * useful for sensors with pixel value bit depth up to 16.
 * No tunable values are required in the calibration file for this mode
 *
 * "gbl-mode: 1" is the Pre-configured mode. In this case the lookup table needs
 * to be configured in the tuning file, in the "gbl-lut" attribute. Also the "gbl-gain"
 * may be used in this case.
 * Tunable values:
 * - gbl-lut: list [1-416] of u16 values with 8.8 format.
 * If no configuration is set, default values from the driver are used.
 * - gbl-gain: global gain applied at the output of the global tonemapping step
 * where all entries of the Global LUT are multiplied by this gain. u16 value with 8.8 format.
 * If no configuration is set, the default value set by kGblGain is used.
 *
 * "gbl-mode: 2" is the Dynamic mode. In this case, the lookup table is generated based
 * on the image histogram measured by the ISP.
 * Tunable values:
 * - gdrc-alpha: Selects between histogram equalization (=0) and histogram stretching (=256)
 * If not configured, the default kGdrcAlphaValue is used.
 * - gdrc-gamma: Controls the strength of dynamic range compression with histogram stretching
 * Value is in u8.8 format. If not configured, the default kGdrcGammaValue is used.
 * - fixed-gamma: Controls the default compression curve which GDRC works with
 * Value is in u8.8 format. If not configured, the default kGdrcGammaValue is used.
 *
 * The dynamic mode is based on two principles, namely histogram equalization and
 * non-linear histogram stretching:
 * Histogram equalization relies on the linearization of the cumulative distribution
 * function evaluated from the input histogram - the gain for each bin is calculated
 * as the ratio between the relative cumulative frequency for a linear histogram
 * and for the empiric histogram (the real-time luminance histogram measured by the ISP).
 * Histogram stretching evaluates the empiric dynamic range of the input data,
 * scales the input histogram according to its real range, and applies a power
 * function whose exponent represents the strength of the dynamic range compression.
 */

/**
 * \struct DrcControlContext
 * \brief Properties of the dynamic range compression algorithm
 *
 * \var DrcControlContext::maxValue
 * \brief Maximum pixel value evaluated from the histogram
 *
 * \var DrcControlContext::minValue
 * \brief Minimum pixel value evaluated from the histogram
 *
 * \var DrcControlContext::maxHistogramBin
 * \brief Histogram bin index corresponding to the maximum pixel value
 *
 * \var DrcControlContext::minHistogramBin
 * \brief Histogram bin index corresponding to the minimum pixel value
 *
 * \var DrcControlContext::maxHistory
 * \brief History of maximum values. The histogram maximum is tracked for the
 * last n frames and the second largest maximum is then taken as the empirical
 * histogram upper limit. Taking second maximum mitigates power supply flicker.
 *
 * \var DrcControlContext::maxBinHistory
 * \brief History of maximum histogram bins
 *
 * \var DrcControlContext::historyPointer
 * \brief Pointer to the next maximum value to be written in the history
 *
 * \var DrcControlContext::historyMax
 * \brief Maximum value found in the history
 *
 * \var DrcControlContext::historyMaxBin
 * \brief Histogram bin index corresponding the maximum value found in the history
 *
 * \var DrcControlContext::globalDrcAlpha
 * \brief Alpha blending value for stretching: 256 = stretch, 0 = histogram equalization [1.8]
 * Alpha blending is the weighted sum of two inputs:
 * output = globalDrcAlpha/256 * stretch + (1 - (globalDrcAlpha/256)) * hist_eq
 *
 * \var DrcControlContext::extraGainOut
 * \brief Extra multiplication factor required after applying the global DRC LUT [8.8]
 *
 * \var DrcControlContext::gamma
 * \brief Dynamic gamma, selected according to occupied dynamic range
 *
 * \var DrcControlContext::gammaFixed
 * \brief Fixed gamma applied for histogram stretching on top of the dynamic gamma
 * GammaFixed is not depending on the dynamic range of the input histogram.
 *
 */

/**
 * \struct DrcLut
 * \brief Lookup table related properties for the DRC algorithm
 *
 * \var ratio
 * \brief The output gain lookup table, expressed for each histogram bin
 *
 * \var hist
 * \brief Preprocessed input histogram
 *
 * \var nextRange
 * \brief Dynamic range of the histogram
 *
 * \var effGamma
 * \brief Effective gamma, taking into account the fixed and the dynamic gamma
 *
 * \var maxRatio
 * \brief Maximum gain in the current lookup table
 *
 * \var histEqSum
 * \brief Sum of the histogram frequencies for histogram equalization
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoDrc)

#define DRC_INPUT_BPP 20
#define DRC_MINCLZ 8 /* Min count leading zero (clz) histogram category */
#define DRC_BITCLZ 5 /* 2<<n bins per histogram clz category */
#define DRC_BINCLZ (1 << DRC_BITCLZ)
#define MASK_LSB (DRC_BINCLZ - 1)
#define DRC_INPUT_UMAX ((1 << DRC_INPUT_BPP) - 1)

/* In the fixed point 8.8 representation, one is given as 0x100 */
static constexpr uint16_t kQ8One = 0x100;

Drc::Drc()
{
}

/**
 * \brief Configure the global context for a new streaming session
 */
void Drc::configureGblDrcContext()
{
	/* Init arrays for history of maximum values */
	for (uint32_t index = 0; index < gblDrcContext_.kDrcMaxHistory; index++) {
		gblDrcContext_.maxHistory[index] = 0U;
		gblDrcContext_.maxBinHistory[index] = 0U;
	}

	/* Min and max computed from Histogram */
	gblDrcContext_.maxValue = ((1 << (DRC_INPUT_BPP)) - 1);
	gblDrcContext_.minValue = 0;
	gblDrcContext_.maxBin = NEO_DRC_GLOBAL_TONEMAP_SIZE - 1;
	gblDrcContext_.minBin = 0;

	/* Set first items in arrays */
	gblDrcContext_.maxHistory[0] = ((1 << DRC_INPUT_BPP) - 1);
	gblDrcContext_.maxBinHistory[0] = NEO_DRC_GLOBAL_TONEMAP_SIZE - 1;

	gblDrcContext_.historyPointer = 0; /* Pointer to next maximum value to be written */
	gblDrcContext_.historyMax = ((1 << DRC_INPUT_BPP) - 1); /* Maximum value in History */
	gblDrcContext_.historyMaxBin = NEO_DRC_GLOBAL_TONEMAP_SIZE - 1; /* Maximum value in History */

	gblDrcContext_.extraGainOut = kQ8One;
	gblDrcContext_.gamma = kGdrcGammaValue;
}

/**
 * \brief Convert bin index to pixel value
 * \param[in] bin The bin index
 *
 * The DRC block is using 416-bin nonlinear histograms:
 * The value range is split into 13 logarithmic levels, each of these levels
 * is further split into 32 linear bins.
 *
 * When converting the pixel value to the corresponding bin index:
 * - the first one in the binary representation of the 20-bit pixel value is
 * found. The position of the first one gives the logarithmic level.
 * First one at bit 19 corresponds to level 12, first one at bit 18
 * to level 11, etc. Level 0 is selected when there are no ones at
 * bit 19 .. bit 8.
 * - the linear bin index at the given level is given by the five bits
 * immediately following the first one found in the binary number. For level 0,
 * bin index corresponds to the value of bits 7 .. 3
 * - finally, the bin is calculated as level * 32 + index.
 *
 * The function binToLinear provides the mapping from the bin index to
 * the corresponding linear pixel value, returning the value of the starting
 * pixel level for the given histogram bin.
 *
 * \return Pixel value corresponding to the bin index
 */
uint32_t Drc::binToLinear(uint32_t bin) const
{
	int lFf1 = bin >> DRC_BITCLZ;
	bin &= MASK_LSB;

	if (lFf1 == 0)
		bin <<= DRC_MINCLZ - DRC_BITCLZ;
	else {
		bin |= DRC_BINCLZ;
		bin <<= (lFf1 + DRC_MINCLZ - DRC_BITCLZ - 1);
	}
	return bin;
}

/**
 * \brief Generate lookup table from gamma only
 *
 * In the dynamic mode, for the first frame, no input histogram is taken
 * into account. The LUT for compression of the dynamic range is generated
 * by only following the configured gamma.
 * This function would typically only be called once at init time.
 */
void Drc::fixedModeLut()
{
	constexpr float inputUnsignedMaximum = ((1 << DRC_INPUT_BPP) - 1);
	float gamma = gblDrcContext_.gamma / 256.0;

	LOG(NxpNeoAlgoDrc, Debug) << "Fixed-mode global DRC Gamma: " << gblDrcContext_.gamma;

	for (uint index = 1; index < gblLut_.size(); index++) {
		float in, outRatio, ratio;

		in = binToLinear(index) / inputUnsignedMaximum;

		outRatio = pow(in, gamma);
		outRatio = std::min(outRatio, 1.0f);

		ratio = outRatio / in * kQ8One;
		gblLut_[index] = static_cast<uint16_t>(std::min<int>(ratio, UINT16_MAX));
	}

	gblLut_[0] = gblLut_[1];
	gblDrcContext_.extraGainOut = kQ8One;
}

/**
 * \brief Find the histogram minimum nonempty bin index and corresponding pixel value
 * \param[in] inputHistogram Vector containing the input histogram for analysis
 *
 * Find the constrained minimum and minimum bin index from the input histogram.
 * Constraint is the minimum pixel count kMinPixelCount.
 * Bins which do not cumulatively reach the minimum pixel count are ignored, which
 * improves the stability of minimum detection in a series of frames.
 * The result is stored in the global context gblDrcContext_.minBin
 * and gblDrcContext_.minValue.
 */
void Drc::getMin(const std::vector<uint32_t> &inputHistogram)
{
	uint32_t sum = 0U;

	gblDrcContext_.minBin = 0U;
	gblDrcContext_.minValue = 0U;

	for (uint index = 0; index < inputHistogram.size(); index++) {
		sum += inputHistogram[index];

		if (sum > kMinPixelCount)
			break;
		gblDrcContext_.minBin = index;
		gblDrcContext_.minValue = binToLinear(index);
	}
}

/**
 * \brief Find the histogram maximum nonempty bin index and corresponding pixel value
 * \param[in] inputHistogram Vector containing the input histogram for analysis
 *
 * Find the constrained maximum and maximum bin index from the input histogram.
 * Constraint is the maximum pixel count kMaxPixelCount.
 * Bins which do not cumulatively reach the maximum pixel count are ignored, which
 * improves the stability of maximum detection in a series of frames.
 * The result is stored in the global context gblDrcContext_.maxBin
 * and gblDrcContext_.maxValue.
 */
void Drc::getMax(const std::vector<uint32_t> &inputHistogram)
{
	uint32_t sum = 0U;

	gblDrcContext_.maxBin = inputHistogram.size() - 1;
	gblDrcContext_.maxValue = DRC_INPUT_UMAX;

	for (uint index = inputHistogram.size() - 1; index > 0; index--) {
		sum += inputHistogram[index];

		if (sum > kMaxPixelCount)
			break;
		gblDrcContext_.maxBin = index;
		gblDrcContext_.maxValue = binToLinear(index);
	}
}

/**
 * \brief Find the maximum bin and corresponding pixel value in history of maxima
 *
 * Search through the stored history of n frame histogram maxima and find the
 * second largest bin index and corresponding pixel level. The second maximum
 * is more stable than the first and avoids light source flicker.
 */
void Drc::getHistoryMax()
{
	gblDrcContext_.historyMax = 0;
	gblDrcContext_.historyMaxBin = 0;
	uint32_t maxIndex = 0;
	uint32_t maxValue = 0;

	/* Find maximum */
	for (unsigned int index = 0; index < gblDrcContext_.kDrcMaxHistory; index++) {
		if (maxValue < gblDrcContext_.maxHistory[index]) {
			maxIndex = index;
			maxValue = gblDrcContext_.maxHistory[index];
		}
		/* Use current frame as default HistoryMax */
	}

	/* Find 2nd maximum */
	gblDrcContext_.historyMax = 0;
	for (unsigned int index = 0; index < gblDrcContext_.kDrcMaxHistory; index++) {
		if (((uint32_t)index) != maxIndex) {
			if (gblDrcContext_.historyMax < gblDrcContext_.maxHistory[index]) {
				gblDrcContext_.historyMax = gblDrcContext_.maxHistory[index];
				gblDrcContext_.historyMaxBin = gblDrcContext_.maxBinHistory[index];
			}
		}
	}
}

/**
 * \brief Find the histogram min/max nonempty bin index and corresponding pixel value
 * \param[in] inputHistogram Vector containing the input histogram for analysis
 * \param[in] frame Frame number to decide for initialization at stream start
 *
 * Analyse the extrema of the histogram and store the min and max values in the
 * global context.
 */
void Drc::getMinMax(const std::vector<uint32_t> &inputHistogram, const uint32_t frame)
{
	getMin(inputHistogram);
	getMax(inputHistogram);
	/*
	 * add histogram maximum - write first frame result twice,
	 * as we search for the second biggest value
	 */
	if (frame == 0) {
		gblDrcContext_.historyPointer = 0;
		gblDrcContext_.maxHistory[gblDrcContext_.historyPointer + 1] = gblDrcContext_.maxValue;
		gblDrcContext_.maxBinHistory[gblDrcContext_.historyPointer + 1] = gblDrcContext_.maxBin;
	}

	/* Store Max to History */
	gblDrcContext_.maxHistory[gblDrcContext_.historyPointer] = gblDrcContext_.maxValue;
	gblDrcContext_.maxBinHistory[gblDrcContext_.historyPointer] = gblDrcContext_.maxBin;

	/* Increment History pointer and wrap if required */
	gblDrcContext_.historyPointer++;

	if (gblDrcContext_.historyPointer >= gblDrcContext_.kDrcMaxHistory)
		gblDrcContext_.historyPointer = 0;

	getHistoryMax();

	if (gblDrcContext_.historyMax < gblDrcContext_.minValue)
		LOG(NxpNeoAlgoDrc, Warning) << "Warning: DRC control found Min > Max!";
}

/**
 * \brief Preprocess the input histogram and calculate sum
 * \param[in] inputHistogram The measured histogram of the image
 * \param[in, out] lutVars Structure containing the input histogram, the lut and other needed parameters
 *
 * Each of the histogram bin frequencies is preprocessed by applying an upper bound
 * (kHEThreshold) and power(kHEThreshold) to each frequency value, respectively.
 * The preprocessed histogram and its sum are stored in the lutVars structure.
 */
void Drc::dynamicModeSum(const std::vector<uint32_t> &inputHistogram, DrcLut *lutVars) const
{
	for (uint index = 0; index < inputHistogram.size(); index++) {
		uint32_t merged = inputHistogram[index];

		/* Hard bin value limiter */
		if (merged > kHEThreshold)
			merged = kHEThreshold;

		/* Smooth bin value limiter */
		lutVars->hist[index] = pow(merged, kHESaturation);
		lutVars->histEqSum += lutVars->hist[index];
	}
}

/**
 * \brief Calculate the effective gamma from the histogram range
 * \param[in, out] lutVars Structure containing the input histogram, the lut and other needed parameters
 */
void Drc::effectiveGamma(DrcLut *lutVars) const
{
	lutVars->nextRange = (float)(gblDrcContext_.historyMax - gblDrcContext_.minValue);
	float gamma = gblDrcContext_.gamma * 1.0 / kQ8One;
	float dynGamma = 1.0;
	if (lutVars->nextRange == 1)
		LOG(NxpNeoAlgoDrc, Warning) << "Warning: The normalized histogram range should be larger than 0";
	else
		dynGamma = log2f(pow(DRC_INPUT_UMAX, gamma)) / /* Target bit range */
			   log2f(lutVars->nextRange); /* Input bit range */

	if (dynGamma > 1.0)
		dynGamma = 1.0;

	lutVars->effGamma = dynGamma * gblDrcContext_.gammaFixed * 1.0 / kQ8One;
}

/**
 * \brief First run to generate the DRC lookup table from measured histogram
 * \param[in, out] lutVars Structure containing the input histogram, the lut and other needed parameters
 *
 * Generate the lookup table from the input histogram with the use of two principles:
 * 1) Histogram equalization, calculating the gain for each histogram bin from the cumulative
 * distribution function of the measured histogram;
 * 2) non-linear gamma-based stretching  * using a fixed gamma and a dynamic gamma
 * calculated from histogram value range;
 * The output of the two is blended (taken as a weighted sum) depending on the
 * gblDrcContext_.globalDrcAlpha parameter.
 *
 * \return Extra gain to be applied on top of LUT
 */
uint16_t Drc::lutFirstRun(DrcLut *lutVars)
{
	float alpha = gblDrcContext_.globalDrcAlpha * 1.0 / kQ8One;
	float nextOffset = gblDrcContext_.minValue * 1.0;
	float alphaQ = 1.0 - alpha;
	double histEqAcc = 0.0;

	for (int index = 1; index < NEO_DRC_GLOBAL_TONEMAP_SIZE; index++) {
		float inVal, outRatio, inStretch, outStretch; /* Stretching of Gamma based */
		inVal = (float)binToLinear(index);
		inStretch = inVal - nextOffset;

		/* Low end clipping... high end clipping is done after scaling */
		if (inStretch < 0.0)
			inStretch = 0.0;

		/* Stretch it to next dynamic range */
		if (lutVars->nextRange == 0)
			LOG(NxpNeoAlgoDrc, Warning) << "Warning: value range calculated from current histogram is 0";
		else
			inStretch /= lutVars->nextRange;

		/* High end clipping */
		if (inStretch > 1.0)
			inStretch = 1.0;

		outStretch = pow(inStretch, lutVars->effGamma);

		if (outStretch > 1.0) {
			LOG(NxpNeoAlgoDrc, Warning) << "Warning: gamma output in first run of LUT calculation should not be > 1.0 ";
			outStretch = 1.0;
		}

		/* Do histogram equalization */
		histEqAcc += lutVars->hist[index];
		float histEq = 0.0;
		if (lutVars->histEqSum == 0)
			LOG(NxpNeoAlgoDrc, Warning) << "Warning: sum for histogram equalization should not be 0";
		else
			histEq = histEqAcc / lutVars->histEqSum;

		/* Do Blending between stretching and HE */
		outRatio = (histEq * alphaQ) + (outStretch * alpha);

		/* Convert output value to ratio */
		if (inVal == 0) {
			LOG(NxpNeoAlgoDrc, Warning) << "Warning: pixel level corresponding to bin " << index << " should not be 0";
			lutVars->ratio[index] = 1.0f;
		} else {
			inVal /= DRC_INPUT_UMAX;
			lutVars->ratio[index] = outRatio / inVal;
		}

		/* Check for maximum ratio to get the required extra factor */
		if (((uint32_t)index >= gblDrcContext_.minBin) && /* Limit the bin range at Minimum */
		    ((uint32_t)index <= gblDrcContext_.historyMaxBin) &&
		    (lutVars->maxRatio < lutVars->ratio[index]))
			lutVars->maxRatio = lutVars->ratio[index];
	}

	lutVars->ratio[0] = lutVars->ratio[1];

	/* Normalize ratio and make it fixed point; compute extra digital gain before LUT */
	lutVars->maxRatio = std::clamp<float>(lutVars->maxRatio, UINT8_MAX * 1.0, UINT16_MAX * 1.0);

	/* Make limit extra gain to 16 bit */
	gblDrcContext_.extraGainOut = static_cast<int>(lutVars->maxRatio);

	/* Store ExtraGain in MaxRatio variable */
	lutVars->maxRatio = gblDrcContext_.extraGainOut * 1.0 / kQ8One;

	return gblDrcContext_.extraGainOut;
}

/**
 * \brief Postprocess the generated LUT
 * \param[in] lutVars Structure with LUT-related parameters
 */
void Drc::lutSecondRun(DrcLut *lutVars)
{
	for (unsigned int index = 1; index < NEO_DRC_GLOBAL_TONEMAP_SIZE; index++) {
		float ratio = 1.0;
		if (lutVars->maxRatio == 0)
			LOG(NxpNeoAlgoDrc, Warning) << "Warning: maximum LUT ratio should not be 0";
		else
			ratio = lutVars->ratio[index] / lutVars->maxRatio;
		ratio *= 256.0; /* 8.8 */

		if (ratio > UINT16_MAX * 1.0)
			ratio = UINT16_MAX * 1.0;

		gblLut_[index] = static_cast<int>(ratio);
	}
}

/**
 * \brief Generate the dynamic DRC lookup table from measured image histogram
 * \param[in] inputHistogram The measured non-linear histogram from the ISP
 * \param[out] extraGainOut Global gain to be applied on top of the LUT values
 */
void Drc::controlDynamicMode(const std::vector<uint32_t> &inputHistogram,
			     uint16_t *extraGainOut)
{
	DrcLut lLutVars;

	/* Init variables */
	lLutVars.nextRange = 0.0;
	lLutVars.effGamma = 0.0;
	lLutVars.maxRatio = 0.0;
	lLutVars.histEqSum = 0.0;

	/* Smooth histogram - HE pre-processing */
	dynamicModeSum(inputHistogram, &lLutVars);

	/* Effective gamma - stretching pre-processing */
	effectiveGamma(&lLutVars);

	/* LUT first run */
	*(extraGainOut) = lutFirstRun(&lLutVars);

	/* LUT second run */
	lutSecondRun(&lLutVars);
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Drc::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	/* Parsing GDRC mode */
	gblMode_ = tuningData["gbl-mode"].get<uint16_t>(kGblMode);

	switch (gblMode_) {
	case 0: {
		LOG(NxpNeoAlgoDrc, Debug) << "Global DRC Mode = 0: Passthrough (no compression)";
		gblLut_.resize(NEO_DRC_GLOBAL_TONEMAP_SIZE, kQ8One);
		break;
	}
	case 1: {
		LOG(NxpNeoAlgoDrc, Debug) << "Global DRC Mode = 1: Using pre-configured LUT.";
		/* Global DRC lut parsing */
		const YamlObject &lut = tuningData["gbl-lut"];
		if (lut.size()) {
			gblLut_ = lut.getList<uint16_t>()
					  .value_or(std::vector<uint16_t>{});
			if (gblLut_.size() != NEO_DRC_GLOBAL_TONEMAP_SIZE) {
				LOG(NxpNeoAlgoDrc, Error) << "global lut list size must be "
							  << NEO_DRC_GLOBAL_TONEMAP_SIZE;
				return -EINVAL;
			}
		}
		break;
	}
	case 2: {
		LOG(NxpNeoAlgoDrc, Debug) << "Global DRC Mode = 2: Dynamic LUT computed from histogram.";
		gblLut_ = std::vector<uint16_t>(NEO_DRC_GLOBAL_TONEMAP_SIZE, 0);
		break;
	}
	default: {
		LOG(NxpNeoAlgoDrc, Error) << "gbl-mode must be 0 / 1 / 2";
		return -EINVAL;
	}
	}

	/* Global DRC gain parsing - for GDRC modes. Will be overriden by dynamic control */
	gblGain_ = tuningData["gbl-gain"].get<uint16_t>(kGblGain);

	/* Global DRC gamma and alpha parsing - for dynamic control */
	gblDrcContext_.gammaFixed = tuningData["fixed-gamma"].get<uint16_t>(kGdrcGammaValue);
	gblDrcContext_.gamma = tuningData["gdrc-gamma"].get<uint16_t>(kGdrcGammaValue);
	gblDrcContext_.globalDrcAlpha = tuningData["gdrc-alpha"].get<uint16_t>(kGdrcAlphaValue);

	LOG(NxpNeoAlgoDrc, Debug) << "global GAIN=" << gblGain_
				  << " GDRC gammaFixed/gamma/alpha: "
				  << gblDrcContext_.gammaFixed << "/"
				  << gblDrcContext_.gamma << "/"
				  << gblDrcContext_.globalDrcAlpha;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Drc::configure(IPAContext &context, const IPACameraSensorInfo &configInfo)
{
	context.configuration.drc.roi.xpos = 0;
	context.configuration.drc.roi.ypos = 0;
	context.configuration.drc.roi.width = configInfo.outputSize.width;
	context.configuration.drc.roi.height = configInfo.outputSize.height;

	if (gblMode_ == 2) {
		configureGblDrcContext();
		fixedModeLut();
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Drc::prepare([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  NxpNeoParams *params)
{
	bool update = gblMode_ == 2 || frame == 0;
	if (update) {
		auto drcGlobalTonemapConfig = params->block<BlockParamsType::DrcGlobalTonemap>();
		/* Set global lut */
		drcGlobalTonemapConfig.setUpdate(true);

		memcpy(drcGlobalTonemapConfig->drc_global_tonemap,
		       gblLut_.data(),
		       sizeof(struct neoisp_drc_global_tonemap_mem_params_s));
	}

	auto drcConfig = params->block<BlockParamsType::DrComp>();
	drcConfig.setUpdate(true);

	/* Set global gain */
	drcConfig->lcl_stretch_stretch = kLocalStretchvalue;
	drcConfig->alpha_alpha = kAlphaValue;
	drcConfig->gbl_gain_gain = gblGain_;

	/* Set ROI */
	/* Make ROI0 (foreground) empty, ROI1 covers the whole image */
	drcConfig->roi0.xpos = UINT16_MAX;
	drcConfig->roi0.ypos = UINT16_MAX;
	drcConfig->roi0.height = 0;
	drcConfig->roi0.width = 0;

	drcConfig->roi1.xpos = context.configuration.drc.roi.xpos;
	drcConfig->roi1.ypos = context.configuration.drc.roi.ypos;
	drcConfig->roi1.width = context.configuration.drc.roi.width;
	drcConfig->roi1.height = context.configuration.drc.roi.height;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Drc::process([[maybe_unused]] IPAContext &context,
		  const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  const NxpNeoStats *stats,
		  [[maybe_unused]] ControlList &metadata)
{
	if (gblMode_ == 2) {
		auto drcMemStats = stats->block<BlockStatsType::MDrc>();
		const unsigned int *statsHistogram = drcMemStats->drc_global_hist_roi1;
		std::vector<uint32_t> inputHistogram(statsHistogram,
						     statsHistogram + NEO_DRC_GLOBAL_TONEMAP_SIZE);

		getMinMax(inputHistogram, frame);

		uint16_t extraGainOut = 0;
		controlDynamicMode(inputHistogram,
				   &extraGainOut);
		gblGain_ = extraGainOut;

		LOG(NxpNeoAlgoDrc, Debug) << "Extra gain out: " << extraGainOut;
	}
}

REGISTER_IPA_ALGORITHM(Drc, "Drc")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
