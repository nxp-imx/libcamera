/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * NXP NEO DRC configuration
 */

#include "drc.h"

#include <algorithm>
#include <limits>
#include <stdint.h>

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
 * \brief Dynamic Range Compression (DRC) configuration algorithm
 *
 * This algorithm configures the DRC unit, providing configuration for the
 * global DRC operation.
 *
 * The DRC unit compresses the bit depth from 20 bits (used in the ISP pipeline)
 * to 16 bits. The DRC operation controls the brightness of the output image.
 *
 * The following DRC operation is applied on each YUV component.
 * For the Y component:
 *   Y_GDRC = Y_IN * LUT[Linear2Bin(Y_IN)] * DRC_GBL_GAIN >> 4
 *
 * Through the configuration of the LUT, the DRC unit can amplify or attenuate
 * the input Y value. The input value is assigned to a histogram bin, and the
 * LUT entry valid for the particular bin is applied to the input value.
 * This algorithm calculates the LUT entries.
 *
 * It is possible to restrict the DRC operation to run only if the pipeline is
 * in HDR merge mode. Such restriction can be set with the following key:
 * "restrict-mode": Restrict the DRC to run only in HDR merge mode.
 *                  If set to "hdr-merge" and the pipeline is not in such mode,
 *                  the DRC operation is disabled (the DRC mode is forced to
 *                                                 gbl-mode=0, Copy mode)
 *                  Valid values: { "hdr-merge", "none" }
 *                  Default value is none.
 *                  Other values are ignored.
 *
 * Three modes of operation are supported to determine the global DRC lookup
 * table, as specified by the "gbl-mode" attribute:
 * "gbl-mode: 0": Copy/Passthrough mode.
 *                The generated lookup table contains only unit gain values,
 *                meaning no effective dynamic range compression is performed.
 *                Data passes through as-is.
 *                Useful for sensors with pixel value bit depth up to 16.
 *                No tunable values required in the calibration file.
 * "gbl-mode: 1": Pre-configured mode.
 *                The lookup table must be configured in the tuning file via
 *                "gbl-lut". The "gbl-gain" may also be used.
 *
 *                The lookup table contains 416 entries corresponding to the
 *                416 non-linear histogram bins. Each LUT entry is a gain value
 *                applied to pixels whose values fall within the corresponding
 *                bin's range. The mapping from pixel values to bin indices is
 *                described in the binToLinear() function, but mainly follows a
 *                structure of 13 levels each containing 32 bins.
 *
 *                Tunable values:
 *                - gbl-lut: List [1-416] of u16 values with 8.8 format.
 *                  If not set, driver default values are used.
 *                - gbl-gain: Global gain applied at the output of the global
 *                  tonemapping step, being a u16 value with 8.8 format.
 *                  All Global LUT entries are multiplied by this gain.
 *                  If not set, the default value kGlobalGain is used.
 * "gbl-mode: 2": Dynamic mode.
 *                The lookup table is generated based on the image histogram
 *                measured by the ISP.
 *
 *                Tunable values:
 *                - gdrc-alpha: Selects between histogram equalization (=0)
 *                  and histogram stretching (=256).
 *                  If not set, the default value kGdrcAlphaValue is used.
 *                - gdrc-gamma: Controls the strength of dynamic range
 *                  compression with histogram stretching. Value in u8.8 format.
 *                  If not set, the default value kGdrcGammaValue is used.
 *                - fixed-gamma: Controls the default compression curve which
 *                  GDRC works with. Value in u8.8 format
 *                  If not set, the default value kGdrcGammaValue is used.
 *
 * The dynamic mode is based on two principles:
 * - Histogram equalization: Relies on linearization of the cumulative
 *   distribution function evaluated from the input histogram. The gain for
 *   each bin is calculated as the ratio between the relative cumulative
 *   frequency for a linear histogram and for the empiric histogram (real-time
 *   luminance histogram measured by the ISP).
 *
 * - Histogram stretching: Evaluates the empiric dynamic range of the
 *   input data, scales the input histogram according to its real range, and
 *   applies a power function whose exponent represents the strength of the
 *   dynamic range compression.
 */

/**
 * \struct Drc::ControlContext
 * \brief Properties of the dynamic range compression algorithm
 *
 * \struct Drc::ControlContext::Range
 * \brief A simple min/max pair representing a bounded interval
 *
 * Used to store minimum and maximum bounds for pixel values and histogram
 * bin indices.
 *
 * \var Drc::ControlContext::Range::min
 * \brief Minimum value of the range
 *
 * \var Drc::ControlContext::Range::max
 * \brief Maximum value of the range
 *
 * \var Drc::ControlContext::value
 * \brief Minimum and maximum pixel values evaluated from the histogram
 *
 * \var Drc::ControlContext::bin
 * \brief Histogram bin index for minimal and maximal pixel values
 *
 * \struct Drc::ControlContext::History
 * \brief Circular buffer tracking histogram maxima across multiple frames
 *
 * Maintains a history of the last kMaxSize frame histogram maximum values
 * and their corresponding bin indices. This temporal tracking allows the
 * algorithm to use the second-largest maximum instead of the absolute
 * maximum, providing stability against transient brightness spikes such
 * as power supply flicker or brief specular highlights.
 *
 * \var Drc::ControlContext::History::values
 * \brief History of maximum values
 *
 * The histogram maximum is tracked for the last n frames and the second
 * largest maximum is then taken as the empirical histogram upper limit.
 * Taking second maximum mitigates power supply flicker.
 *
 * \var Drc::ControlContext::History::bins
 * \brief History of maximum histogram bins
 *
 * \var Drc::ControlContext::History::pointer
 * \brief Pointer to the next maximum value to be written in the history
 *
 * \var Drc::ControlContext::History::maxValue
 * \brief Maximum value found in the history
 *
 * \var Drc::ControlContext::History::maxBin
 * \brief Histogram bin index
 *
 * Corresponds to the maximum value found in the history
 *
 * \var Drc::ControlContext::history
 * \brief History buffer for temporal maximum value tracking
 *
 * \var Drc::ControlContext::globalDrcAlpha
 * \brief Alpha blending value for stretching [u1.8 format]
 *
 * - 256 = stretch
 * - 0 = histogram equalization
 *
 * Alpha blending is the weighted sum of two inputs:
 *   output = globalDrcAlpha/256 * stretch +
 *            (1 - (globalDrcAlpha/256)) * hist_eq
 *
 * \var Drc::ControlContext::extraGainOut
 * \brief Extra multiplication factor required after applying the global DRC LUT
 *
 * Value in u8.8 fixed-point format.
 *
 * \var Drc::ControlContext::gamma
 * \brief Dynamic gamma, selected according to occupied dynamic range
 *
 * \var Drc::ControlContext::gammaFixed
 * \brief Fixed gamma applied for histogram stretching on top of dynamic gamma
 *
 * This gamma is independent of the dynamic range of the input histogram.
 *
 */

/**
 * \struct Drc::LutVariables
 * \brief Lookup table related properties for the DRC algorithm
 *
 * \var Drc::LutVariables::ratio
 * \brief The output gain lookup table, expressed for each histogram bin
 *
 * \var Drc::LutVariables::hist
 * \brief Preprocessed input histogram table
 *
 * \var Drc::LutVariables::nextRange
 * \brief Dynamic range of the histogram
 *
 * \var Drc::LutVariables::effGamma
 * \brief Effective gamma, taking into account the fixed and the dynamic gamma
 *
 * \var Drc::LutVariables::maxRatio
 * \brief Maximum gain in the current lookup table
 *
 * \var Drc::LutVariables::histEqSum
 * \brief Sum of the histogram frequencies for histogram equalization
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoDrc)

static constexpr uint32_t kInputBpp = 20;
static constexpr uint32_t kInputMax = (1 << kInputBpp) - 1;
static constexpr uint16_t kQ8Unit = 0x100; /* Unit gain in u8.8 format */
/* Histogram structure constants */
static constexpr uint32_t kLevel0Size = 8; /* Min count leading zero */
static constexpr uint32_t kBinIndexSize = 5; /* 2^n bins per histogram level */
static constexpr uint32_t kBinsPerLevel = 1 << kBinIndexSize;
static constexpr uint32_t kBinIndexMask = kBinsPerLevel - 1;

const std::map<const std::string, Drc::RestrictMode> Drc::kRestrictModeMap = {
	{ "none", RestrictMode::None },
	{ "hdr-merge", RestrictMode::HdrMerge },
};

Drc::Drc()
{
}

/**
 * \brief Reset the global mode
 */
void Drc::resetGlobalMode()
{
	switch (globalMode_) {
	case GlobalMode::Passthrough:
		LOG(NxpNeoAlgoDrc, Debug)
			<< "Reset global mode to 0: Passthrough (no compression)";
		std::fill(globalLut_.begin(), globalLut_.end(), kQ8Unit);
		break;

	case GlobalMode::Static:
		LOG(NxpNeoAlgoDrc, Debug)
			<< "Reset global mode to 1: Using pre-configured LUT.";
		std::copy(globalFixedLut_.begin(),
			  globalFixedLut_.end(),
			  globalLut_.begin());
		break;

	case GlobalMode::Dynamic:
		LOG(NxpNeoAlgoDrc, Debug)
			<< "Reset global mode to 2: "
			<< "Dynamic LUT computed from histogram.";
		std::fill(globalLut_.begin(), globalLut_.end(), 0);
		configureGlobalContext();
		fixedModeLut();
		break;
	}
	resetGlobalMode_ = true;
}

/**
 * \brief Configure the global context for a new streaming session
 *
 * Initializes the DRC control context with default values, including the
 * history arrays for tracking maximum values across frames.
 */
void Drc::configureGlobalContext()
{
	/* Initialize history arrays for maximum values tracking */
	globalContext_.history.values[0] = kInputMax;
	globalContext_.history.bins[0] = NEO_DRC_GLOBAL_TONEMAP_SIZE - 1;
	for (uint32_t idx = 1; idx < globalContext_.history.kMaxSize; idx++) {
		globalContext_.history.values[idx] = 0U;
		globalContext_.history.bins[idx] = 0U;
	}

	/* Set min/max values computed from Histogram */
	globalContext_.value = {
		.min = 0,
		.max = kInputMax
	};
	globalContext_.bin = {
		.min = 0,
		.max = NEO_DRC_GLOBAL_TONEMAP_SIZE - 1
	};

	/* Reset history state */
	globalContext_.history.pointer = 0;
	globalContext_.history.maxValue = kInputMax;
	globalContext_.history.maxBin = NEO_DRC_GLOBAL_TONEMAP_SIZE - 1;

	globalContext_.extraGainOut = kQ8Unit;
	globalContext_.gamma = kGdrcGammaValue;
}

/**
 * \brief Convert bin index to pixel value
 * \param[in] binIndex The bin index to convert
 *
 * The DRC block uses 416-bin nonlinear histograms. The 20-bit pixel value
 * range is split into 13 logarithmic levels, each further divided into 32
 * linear bins.
 *
 * Conversion from pixel value to bin index:
 * - Find the MSB first '1' bit in the 20-bit pixel value binary representation.
 *   The position gives the logarithmic level:
 *   - Bit 19 = level 12
 *   - Bit 18 = level 11
 *   - ...
 *   - Level 0 is selected when no '1' exists at bits 19..8
 * - The linear bin index within the level is given by the five bits
 *   immediately following the first '1'. For level 0, the bin index
 *   corresponds to bits 7..3.
 * - Finally, bin = level * 32 + index
 *
 * This function provides the mapping from the bin index to the corresponding
 * linear pixel value, returning the value of the starting pixel level for the
 * given histogram bin.
 *
 * \return Pixel value corresponding to the bin index
 */
uint32_t Drc::binToLinear(uint32_t binIndex) const
{
	unsigned int level = binIndex >> kBinIndexSize;
	uint32_t pixelValue;

	binIndex &= kBinIndexMask;
	if (level == 0)
		pixelValue = binIndex << (kLevel0Size - kBinIndexSize);
	else {
		pixelValue = binIndex | kBinsPerLevel;
		pixelValue <<= (level + kLevel0Size - kBinIndexSize - 1);
	}
	return pixelValue;
}

/**
 * \brief Generate lookup table from gamma only
 *
 * In dynamic mode, the first frame has no input histogram available. The LUT
 * for dynamic range compression is generated using only the configured gamma.
 * This function is typically called once at initialization time.
 */
void Drc::fixedModeLut()
{
	float gamma = globalContext_.gamma / 256.0f;

	LOG(NxpNeoAlgoDrc, Debug)
		<< "Fixed-mode global DRC Gamma: " << globalContext_.gamma;

	for (unsigned int idx = 1; idx < globalLut_.size(); idx++) {
		float in, outRatio, ratio;

		in = binToLinear(idx) / static_cast<float>(kInputMax);

		outRatio = pow(in, gamma);
		outRatio = std::min(outRatio, 1.0f);

		ratio = outRatio / in * kQ8Unit;
		globalLut_[idx] = static_cast<uint16_t>(
			std::min<int>(ratio,
				      std::numeric_limits<uint16_t>::max()));
	}

	globalLut_[0] = globalLut_[1];
	globalContext_.extraGainOut = kQ8Unit;
}

/**
 * \brief Find the histogram minimum non-empty bin index and pixel value
 * \param[in] inputHistogram Vector containing the input histogram for analysis
 *
 * Finds the constrained minimum bin index and corresponding pixel value from
 * the input histogram. Constraint is the minimum pixel count kMinPixelCount.
 *
 * Bins which do not cumulatively reach the minimum pixel count are ignored,
 * which improves minimum detection stability across frames.
 *
 * Results are stored in globalContext_.bin.min and globalContext_.value.min.
 */
void Drc::getMin(const std::vector<uint32_t> &inputHistogram)
{
	uint32_t sum = 0U;

	globalContext_.bin.min = 0U;
	globalContext_.value.min = 0U;

	for (unsigned int idx = 0; idx < inputHistogram.size(); idx++) {
		sum += inputHistogram[idx];

		if (sum > kMinPixelCount)
			break;
		globalContext_.bin.min = idx;
		globalContext_.value.min = binToLinear(idx);
	}
}

/**
 * \brief Find the histogram maximum non-empty bin index and pixel value
 * \param[in] inputHistogram Vector containing the input histogram for analysis
 *
 * Finds the constrained maximum bin index and corresponding pixel value from
 * the input histogram. Constraint is the maximum pixel count kMaxPixelCount.
 *
 * Bins that do not cumulatively reach the maximum pixel count are ignored,
 * which improves maximum detection stability across frames.
 *
 * Results are stored in globalContext_.bin.max and globalContext_.value.max.
 */
void Drc::getMax(const std::vector<uint32_t> &inputHistogram)
{
	uint32_t sum = 0U;

	globalContext_.bin.max = inputHistogram.size() - 1;
	globalContext_.value.max = kInputMax;

	for (unsigned int idx = inputHistogram.size() - 1; idx > 0; idx--) {
		sum += inputHistogram[idx];

		if (sum > kMaxPixelCount)
			break;
		globalContext_.bin.max = idx;
		globalContext_.value.max = binToLinear(idx);
	}
}

/**
 * \brief Find the maximum bin and pixel value in history of maxima
 *
 * Searches through the stored history of n frame histogram maxima and finds
 * the second largest bin index and corresponding pixel level.
 *
 * The second maximum is more stable than the first one and helps avoid light
 * source flickering.
 */
void Drc::getHistoryMax()
{
	unsigned int maxSize = globalContext_.history.kMaxSize;
	globalContext_.history.maxValue = 0;
	globalContext_.history.maxBin = 0;
	uint32_t maxIndex = 0;
	uint32_t maxValue = 0;

	/* Find the maximum value in history */
	for (unsigned int idx = 0; idx < maxSize; idx++) {
		if (maxValue < globalContext_.history.values[idx]) {
			maxIndex = idx;
			maxValue = globalContext_.history.values[idx];
		}
		/* Use current frame as default history.maxValue */
	}

	/* Find the second maximum value in history */
	for (unsigned int idx = 0; idx < maxSize; idx++) {
		if (static_cast<uint32_t>(idx) != maxIndex) {
			if (globalContext_.history.maxValue <
			    globalContext_.history.values[idx]) {
				globalContext_.history.maxValue =
					globalContext_.history.values[idx];
				globalContext_.history.maxBin =
					globalContext_.history.bins[idx];
			}
		}
	}

	if (globalContext_.history.maxValue < globalContext_.value.min)
		LOG(NxpNeoAlgoDrc, Warning) << "DRC control found Min > Max!";
}

/**
 * \brief Find histogram min/max non-empty bin index and pixel value
 * \param[in] inputHistogram Vector containing the input histogram for analysis
 *
 * Analyzes the extrema of the histogram and stores the min and max values
 * in the global context. Also maintains the history of maximum values for
 * temporal stability.
 */
void Drc::getMinMax(const std::vector<uint32_t> &inputHistogram)
{
	getMin(inputHistogram);
	getMax(inputHistogram);

	/* Store current maximum to history */
	globalContext_.history.values[globalContext_.history.pointer] =
		globalContext_.value.max;
	globalContext_.history.bins[globalContext_.history.pointer] =
		globalContext_.bin.max;

	/*
	 * Add histogram maximum to history.
	 * Since the second largest maximum is used, write the histogram
	 * maximum twice in case of global mode reset.
	 */
	if (resetGlobalMode_) {
		unsigned int nextIdx = (globalContext_.history.pointer + 1) %
				       globalContext_.history.kMaxSize;
		globalContext_.history.values[nextIdx] =
			globalContext_.value.max;
		globalContext_.history.bins[nextIdx] =
			globalContext_.bin.max;
		resetGlobalMode_ = false;
	}

	/* Increment history pointer and wrap if required */
	globalContext_.history.pointer = (globalContext_.history.pointer + 1) %
					 globalContext_.history.kMaxSize;

	getHistoryMax();
}

/**
 * \brief Pre-process the input histogram and calculate sum
 * \param[in] inputHistogram The measured histogram of the image
 * \param[in,out] lutVars Structure containing histogram, LUT, and parameters
 *
 * Each histogram bin frequency is preprocessed by applying:
 * - An upper bound (kHEThreshold)
 * - A power function (kHESaturation) to each frequency value
 *
 * The preprocessed histogram and its sum are stored in the lutVars structure.
 */
void Drc::dynamicModeSum(const std::vector<uint32_t> &inputHistogram,
			 LutVariables *lutVars) const
{
	for (unsigned int idx = 0; idx < inputHistogram.size(); idx++) {
		uint32_t merged = std::min(inputHistogram[idx], kHEThreshold);

		/* Apply smooth bin value limiter */
		lutVars->hist[idx] = pow(merged, kHESaturation);
		lutVars->histEqSum += lutVars->hist[idx];
	}
}

/**
 * \brief Calculate the effective gamma from the histogram range
 * \param[in,out] lutVars Structure containing histogram, LUT, and parameters
 *
 * Computes the effective gamma by combining the fixed gamma with a dynamic
 * gamma that adapts to the current histogram's dynamic range.
 */
void Drc::effectiveGamma(LutVariables *lutVars) const
{
	lutVars->nextRange = static_cast<float>(
		globalContext_.history.maxValue - globalContext_.value.min);
	float gamma = globalContext_.gamma * 1.0f / kQ8Unit;
	float dynGamma = 1.0f;

	if (lutVars->nextRange > 0.0f && lutVars->nextRange != 1.0f)
		/* Target bit range / Input bit range */
		dynGamma = log2f(pow(kInputMax, gamma)) /
			   log2f(lutVars->nextRange);
	else
		LOG(NxpNeoAlgoDrc, Warning)
			<< "The normalized histogram range should be > 0";
	dynGamma = std::min(dynGamma, 1.0f);

	lutVars->effGamma =
		dynGamma * globalContext_.gammaFixed * 1.0f / kQ8Unit;
}

/**
 * \brief First run to generate the DRC lookup table from measured histogram
 * \param[in,out] lutVars Structure containing histogram, LUT, and parameters
 *
 * Generates the lookup table from the input histogram using two principles:
 *
 * 1) Histogram equalization: Calculates the gain for each histogram bin
 *    from the cumulative distribution function of the measured histogram.
 *
 * 2) Non-linear gamma-based stretching: Uses a fixed gamma and a dynamic
 *    gamma calculated from the histogram value range.
 *
 * The outputs are blended (weighted sum) depending on the
 * globalContext_.globalDrcAlpha parameter.
 */
void Drc::lutFirstRun(LutVariables *lutVars)
{
	float alpha = globalContext_.globalDrcAlpha * 1.0f / kQ8Unit;
	float nextOffset = globalContext_.value.min * 1.0f;
	float alphaQ = 1.0f - alpha;
	float histEqAcc = 0.0f;

	for (int idx = 1; idx < NEO_DRC_GLOBAL_TONEMAP_SIZE; idx++) {
		float inVal, outRatio, inStretch, outStretch;
		inVal = static_cast<float>(binToLinear(idx));
		inStretch = inVal - nextOffset;

		/* Stretch to next dynamic range */
		if (lutVars->nextRange <= 0.0f)
			LOG(NxpNeoAlgoDrc, Warning)
				<< "Histogram value range is <= 0";
		else
			inStretch /= lutVars->nextRange;

		inStretch = std::clamp(inStretch, 0.0f, 1.0f);

		outStretch = pow(inStretch, lutVars->effGamma);
		if (outStretch > 1.0f) {
			LOG(NxpNeoAlgoDrc, Warning)
				<< "Gamma output should not be > 1.0 ";
			outStretch = 1.0f;
		}

		/* Do histogram equalization */
		histEqAcc += lutVars->hist[idx];
		float histEq = 0.0f;
		if (lutVars->histEqSum == 0.0f)
			LOG(NxpNeoAlgoDrc, Warning)
				<< "Histogram equalization sum should not be 0";
		else
			histEq = histEqAcc / lutVars->histEqSum;

		/* Do Blending between stretching and HE */
		outRatio = (histEq * alphaQ) + (outStretch * alpha);

		/* Convert output value to ratio */
		if (inVal == 0.0f) {
			LOG(NxpNeoAlgoDrc, Warning)
				<< "Pixel level corresponding to bin "
				<< idx << " should not be 0";
			lutVars->ratio[idx] = 1.0f;
		} else {
			inVal /= kInputMax;
			lutVars->ratio[idx] = outRatio / inVal;
		}

		/* Track maximum ratio within valid bin range */
		uint32_t binIdx = static_cast<uint32_t>(idx);
		if ((binIdx >= globalContext_.bin.min) &&
		    (binIdx <= globalContext_.history.maxBin) &&
		    (lutVars->maxRatio < lutVars->ratio[idx]))
			lutVars->maxRatio = lutVars->ratio[idx];
	}

	lutVars->ratio[0] = lutVars->ratio[1];
	lutVars->hist[0] = lutVars->hist[1];

	/* Normalize ratio and compute extra digital gain before LUT */
	lutVars->maxRatio = std::clamp<float>(
		lutVars->maxRatio,
		std::numeric_limits<uint8_t>::max() * 1.0f,
		std::numeric_limits<uint16_t>::max() * 1.0f);

	/* Make limit extra gain to 16 bit */
	globalContext_.extraGainOut = static_cast<int>(lutVars->maxRatio);

	/* Store ExtraGain in MaxRatio variable */
	lutVars->maxRatio = globalContext_.extraGainOut * 1.0f / kQ8Unit;
}

/**
 * \brief Second run to post-process the generated LUT
 * \param[in] lutVars Structure with LUT-related parameters
 *
 * Normalizes the ratio values computed in the first run and converts them
 * to the final u8.8 fixed-point format for the hardware LUT.
 */
void Drc::lutSecondRun(LutVariables *lutVars)
{
	for (unsigned int idx = 1; idx < NEO_DRC_GLOBAL_TONEMAP_SIZE; idx++) {
		float ratio = 1.0f;
		if (lutVars->maxRatio == 0.0f)
			LOG(NxpNeoAlgoDrc, Warning)
				<< "Maximum LUT ratio should not be 0";
		else
			ratio = lutVars->ratio[idx] / lutVars->maxRatio;

		/* Convert to u8.8 fixed-point format */
		ratio *= 256.0f;

		if (ratio > std::numeric_limits<uint16_t>::max() * 1.0f)
			ratio = std::numeric_limits<uint16_t>::max() * 1.0f;

		globalLut_[idx] = static_cast<uint16_t>(ratio);
	}
}

/**
 * \brief Generate the dynamic DRC lookup table from measured image histogram
 * \param[in] inputHistogram The measured non-linear histogram from the ISP
 *
 * Coordinates the histogram preprocessing, gamma calculation, and two-pass LUT
 * generation.
 */
void Drc::controlDynamicMode(const std::vector<uint32_t> &inputHistogram)
{
	LutVariables lutVars;

	/* Smooth histogram - HE pre-processing */
	dynamicModeSum(inputHistogram, &lutVars);

	/* Effective gamma - stretching pre-processing */
	effectiveGamma(&lutVars);

	/* LUT first run */
	lutFirstRun(&lutVars);

	/* LUT second run */
	lutSecondRun(&lutVars);
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Drc::init([[maybe_unused]] IPAContext &context,
	      const ValueNode &tuningData)
{
	/* Parsing restrict mode. */
	restrictMode_ = RestrictMode::None;
	auto restrictMode = tuningData["restrict-mode"].get<std::string>("none");
	auto it = kRestrictModeMap.find(restrictMode);
	if (it != kRestrictModeMap.end())
		restrictMode_ = it->second;
	LOG(NxpNeoAlgoDrc, Debug) << "restrict mode is " << restrictMode;

	/* Parsing GDRC mode */
	uint16_t mode = tuningData["gbl-mode"].get<uint16_t>(kGlobalMode);
	if (mode > static_cast<uint16_t>(GlobalMode::Dynamic)) {
		LOG(NxpNeoAlgoDrc, Error) << "Invalid gbl-mode: " << mode;
		return -EINVAL;
	}
	globalInitMode_ = static_cast<GlobalMode>(mode);

	/*
	 * Global fixed LUT (needed for global mode 1).
	 * Initialize LUT with unit gain if nothing provided
	 * by calibration file.
	 */
	std::fill(globalFixedLut_.begin(), globalFixedLut_.end(), kQ8Unit);
	auto lut = tuningData["gbl-lut"].get<std::vector<uint16_t>>().value_or(
		std::vector<uint16_t>{});
	if (!lut.empty()) {
		if (lut.size() != NEO_DRC_GLOBAL_TONEMAP_SIZE) {
			LOG(NxpNeoAlgoDrc, Error)
				<< "global LUT list size must be "
				<< NEO_DRC_GLOBAL_TONEMAP_SIZE;
			return -EINVAL;
		}
		std::copy(lut.begin(), lut.end(), globalFixedLut_.begin());
	}

	/* Parse global DRC gain (overridden by dynamic control) */
	globalGain_ = tuningData["gbl-gain"].get<uint16_t>(kGlobalGain);

	/* Parse global DRC gamma and alpha parsing for dynamic control */
	globalContext_.gammaFixed =
		tuningData["fixed-gamma"].get<uint16_t>(kGdrcGammaValue);
	globalContext_.gamma =
		tuningData["gdrc-gamma"].get<uint16_t>(kGdrcGammaValue);
	globalContext_.globalDrcAlpha =
		tuningData["gdrc-alpha"].get<uint16_t>(kGdrcAlphaValue);

	LOG(NxpNeoAlgoDrc, Debug) << "global GAIN=" << globalGain_
				  << " GDRC gammaFixed/gamma/alpha: "
				  << globalContext_.gammaFixed << "/"
				  << globalContext_.gamma << "/"
				  << globalContext_.globalDrcAlpha;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Drc::configure(IPAContext &context, const IPACameraSensorInfo &configInfo)
{
	IPAPipelineMode &pipelineMode = context.configuration.pipelineMode;
	globalMode_ = globalInitMode_;

	if (restrictMode_ == RestrictMode::HdrMerge &&
	    pipelineMode != IPAPipelineMode::HdrMerge) {
		/*
		 * Disable dynamic DRC (set to Copy mode) if restrict mode is
		 * "hdr-merge" but pipeline is not in HDR merge mode.
		 */
		globalMode_ = GlobalMode::Passthrough;
		LOG(NxpNeoAlgoDrc, Debug)
			<< "DRC is disabled - "
			<< "pipeline mode doesn't match the restrict mode.";
	}

	resetGlobalMode();

	context.configuration.drc.roi.xpos = 0;
	context.configuration.drc.roi.ypos = 0;
	context.configuration.drc.roi.width = configInfo.outputSize.width;
	context.configuration.drc.roi.height = configInfo.outputSize.height;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Drc::prepare(IPAContext &context,
		  const uint32_t frame,
		  IPAFrameContext &frameContext,
		  NxpNeoParams *params)
{
	bool updateHdr = frameContext.hdr.update &&
			 restrictMode_ == RestrictMode::HdrMerge;
	if (updateHdr) {
		/* Disable DRC if HDR is disabled. */
		globalMode_ = frameContext.hdr.enabled ?
			globalInitMode_ :
			GlobalMode::Passthrough;
		resetGlobalMode();
	}

	bool updateDrc = globalMode_ == GlobalMode::Dynamic || frame == 0 ||
			 updateHdr;
	if (updateDrc) {
		auto globalTonemap =
			params->block<BlockParamsType::DrcGlobalTonemap>();
		/* Set global lut */
		globalTonemap.setEnabled(true);

		std::copy(globalLut_.begin(),
			  globalLut_.end(),
			  globalTonemap->drc_global_tonemap);
	}

	auto config = params->block<BlockParamsType::DrComp>();
	config.setEnabled(true);

	/* Set global gain */
	config->lcl_stretch_stretch = kLocalStretchvalue;
	config->alpha_alpha = kAlphaValue;
	config->gbl_gain_gain = globalGain_;

	/* Disable ROI0 (foreground) and set ROI1 to cover the full image */
	config->roi0.xpos = std::numeric_limits<uint16_t>::max();
	config->roi0.ypos = std::numeric_limits<uint16_t>::max();
	config->roi0.height = 0;
	config->roi0.width = 0;

	config->roi1.xpos = context.configuration.drc.roi.xpos;
	config->roi1.ypos = context.configuration.drc.roi.ypos;
	config->roi1.width = context.configuration.drc.roi.width;
	config->roi1.height = context.configuration.drc.roi.height;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Drc::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  const NxpNeoStats *stats,
		  [[maybe_unused]] ControlList &metadata)
{
	if (globalMode_ != GlobalMode::Dynamic)
		return;

	auto memStats = stats->block<BlockStatsType::MDrc>();
	const unsigned int *statsHistogram = memStats->drc_global_hist_roi1;
	std::vector<uint32_t> inputHistogram(
		statsHistogram, statsHistogram + NEO_DRC_GLOBAL_TONEMAP_SIZE);

	getMinMax(inputHistogram);

	controlDynamicMode(inputHistogram);
	globalGain_ = globalContext_.extraGainOut;

	LOG(NxpNeoAlgoDrc, Debug) << "Extra gain out: " << globalGain_;
}

REGISTER_IPA_ALGORITHM(Drc, "Drc")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
