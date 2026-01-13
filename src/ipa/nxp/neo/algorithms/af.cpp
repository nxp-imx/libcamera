/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025 NXP
 *
 * Autofocus control algorithm
 */

#include "af.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "af_base.h"
#include "af_impl.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

using namespace libcamera::controls;

LOG_DEFINE_CATEGORY(NxpNeoAlgoAf)

Af::Af()
{
}

/**
 * \class Af
 * \brief AutoFocus algorithm
 *
 * This class implements an IPA AutoFocus algorithm relying on a platform-
 * agnostic implementation of the AfBase interface. As such, class implements
 * the usual IPA algorithm callbacks, and the IPA-specific operations: user
 * controls, metadata controls, ISP CDAF statistics configuration and
 * collection, PDAF handling etc.
 *
 * Auto Focus unit:
 * The AF block accumulates the output of convolution filters for each pixel
 * within a ROI. The two configurable filters have a 3x3 dimension. For each
 * relevant pixel, the absolute value of each filter is summed individually
 * after right-shift. The resulting sum is truncated to a 32-bit value during
 * accumulation.
 * sum = min(0xffffffff, sum + abs(filtered) >> shift).
 * with `filtered` being the result of the 3x3 convolution filter.
 *
 * Some parameters of the calibration file can be used to amend the default AF
 * block configuration:
 *   - filter0: the 9x coefficients of the 3x3 filter0 (s8).
 *              default: Sobel horizontal filter
 *   - filter1: the 9x coefficients of the 3x3 filter1 (s8).
 *              default: Sobel vertical filter
 *   - shift0: right-shift applied to filter0 accumulation (u5).
 *              default: 8
 *   - shift1: right-shift applied to filter1 accumulation (u5).
 *              default: 8
 */

namespace {

/* Optional IPA controls */
const ControlInfoMap::Map afControls{
	{ &controls::AfMode, ControlInfo(controls::AfModeValues) },
	{ &controls::AfRange, ControlInfo(controls::AfRangeValues) },
	{ &controls::AfSpeed, ControlInfo(controls::AfSpeedValues) },
	{ &controls::AfMetering, ControlInfo(controls::AfMeteringValues) },
	{ &controls::AfWindows, ControlInfo(Rectangle{}, Rectangle(65535, 65535, 65535, 65535), Rectangle{}) },
	{ &controls::AfTrigger, ControlInfo(controls::AfTriggerValues) },
	{ &controls::AfPause, ControlInfo(controls::AfPauseValues) },
};

} /* namespace */

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Af::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	algo_ = std::make_unique<AfImpl>();
	int ret = algo_->doInit(tuningData);
	if (ret)
		return ret;

	/* Parse calibration file ISP filters coefficients and shifts. */
	static_assert(kFilterTapsCount == NEO_AF_FILTERS_CNT,
		      "Unexpected numbers of AF filters taps");
	auto parseFilter = [](const YamlObject &node,
			      const std::string name,
			      const std::array<int8_t, kFilterTapsCount> &defaultFilter,
			      std::array<int8_t, kFilterTapsCount> &out) {
		std::copy(defaultFilter.begin(), defaultFilter.end(), out.begin());
		std::optional<std::vector<int8_t>> filter = node[name].getList<int8_t>();
		if (filter) {
			if (filter->size() != kFilterTapsCount)
				LOG(NxpNeoAlgoAf, Warning) << "Invalid filter size for " << name;
			else
				std::copy(filter->begin(), filter->end(), out.begin());
		}
	};

	parseFilter(tuningData, "filter0", kFilter0Default, filters_[0]);
	parseFilter(tuningData, "filter1", kFilter1Default, filters_[1]);

	auto parseShift = [](const YamlObject &node,
			     const std::string name,
			     const uint8_t defaultShift,
			     uint8_t &out) {
		std::optional<uint8_t> shift = node[name].get<uint8_t>();
		out = defaultShift;
		if (shift) {
			uint8_t value = shift.value();
			if (value > kShitMax)
				LOG(NxpNeoAlgoAf, Warning) << "Invalid shit value for " << name;
			else
				out = value;
		}
	};

	parseShift(tuningData, "shift0", kShiftDefault, shifts_[0]);
	parseShift(tuningData, "shift1", kShiftDefault, shifts_[1]);

	std::stringstream ss;
	ss << "filter0: ";
	for (auto coeff : filters_[0])
		ss << +coeff << " ";
	ss << "filter1: ";
	for (auto coeff : filters_[1])
		ss << +coeff << " ";
	ss << "shift0: " << +shifts_[0] << " shift1: " << +shifts_[1];
	LOG(NxpNeoAlgoAf, Debug) << ss.str();

	/* Create user controls. */
	context.ctrlMap.insert(afControls.begin(), afControls.end());

	double min, max;
	algo_->getLensLimits(min, max);
	float def = static_cast<float>(algo_->getDefaultLensPosition());
	context.ctrlMap[&controls::LensPosition] =
		ControlInfo(static_cast<float>(min), static_cast<float>(max), def);

	return ret;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Af::configure([[maybe_unused]] IPAContext &context, const IPACameraSensorInfo &configInfo)
{
	/* Camera mode related values. */
	auto &afConfig = context.configuration.af;
	const Rectangle &analogCrop = configInfo.analogCrop;
	afConfig.cropX = analogCrop.x;
	afConfig.cropY = analogCrop.y;
	const Size &outputSize = configInfo.outputSize;
	uint16_t width = outputSize.width;
	uint16_t height = outputSize.height;
	afConfig.scaleX = analogCrop.width * 1.0 / width;
	afConfig.scaleY = analogCrop.height * 1.0 / height;

	/* Setup the 9 ROIs as a 3x3 grid */
	static_assert(NEO_AF_REG_STATS_ROIS_CNT == 9);
	const Size &size = configInfo.outputSize;
	width = size.width / NEO_AF_BLOCK_NB_X;
	height = size.height / NEO_AF_BLOCK_NB_Y;
	for (size_t row = 0; row < NEO_AF_BLOCK_NB_Y; row++) {
		for (size_t col = 0; col < NEO_AF_BLOCK_NB_X; col++) {
			neoisp_roi_cfg_s &roi = afConfig.rois[row * NEO_AF_BLOCK_NB_X + col];
			roi.xpos = col * width;
			roi.ypos = row * height;
			roi.width = col < NEO_AF_BLOCK_NB_X - 1
					    ? width
					    : (size.width - roi.xpos);
			roi.height = row < NEO_AF_BLOCK_NB_Y - 1
					     ? height
					     : (size.height - roi.ypos);
		}
	}

	/*
	 * The metric used to estimate the contrast for a ROI is to compute the
	 * L2 norm of the vector [filter0 sum, filter1 sum]. This metric is
	 * normalized in the [0, 1] range after division by the gain factor of
	 * the metric. Gain factor is computed at configure() time to later used
	 * at process() time.
	 * AF block is post DRC so pixel bitdepth is 12 bits.
	 * Gain =
	 *   {L2([filter0_gain, filter1_gain]) * max_pixel_value * pixel_count}
	 * with:
	 *   filterN_gain = L1([filterN coefficients]) / (2 ^ shiftN)
	 *   max_pixel_value = 2^12 - 1 = 4095
	 *   pixel_count = ROI width * height
	 */
	auto computeL1Norm = [](const std::array<int8_t, kFilterTapsCount> &filter) {
		double l1Norm = 0.0;
		for (int8_t coeff : filter)
			l1Norm += std::abs(coeff);
		return l1Norm;
	};
	double filter0Gain = computeL1Norm(filters_[0]) / (1 << shifts_[0]);
	double filter1Gain = computeL1Norm(filters_[1]) / (1 << shifts_[1]);
	double filtersL2Norm =
		std::sqrt(filter0Gain * filter0Gain + filter1Gain * filter1Gain);
	double pixelGain = filtersL2Norm * ((1 << 12) - 1);

	for (auto const &[i, roi] : utils::enumerate(afConfig.rois)) {
		unsigned int pixelCount = roi.width * roi.height;
		afConfig.normalGains[i] = pixelGain * pixelCount;
	}

	/* Initialize the active state. */
	auto &afState = context.activeState.af;
	afState.mode = AfModeManual;
	algo_->setMode(AfModeManual);
	int32_t hwPosition;
	double def = algo_->getDefaultLensPosition();
	algo_->setLensPosition(def, &hwPosition);

	int ret = algo_->doConfigure(configInfo);
	if (ret)
		return ret;

	/*
	 * When in metering auto mode, the focus area selected by the algorithm
	 * is too broad because of the limited number of ROIs supported by the
	 * AF block. Thus, start in metering windows mode, and configure the
	 * center cell of the ISP 9x9 AF grid as the default window for focus.
	 */
	algo_->setMetering(AfMeteringWindows);
	neoisp_roi_cfg_s &center = afConfig.rois[NEO_AF_ROIS_CNT / 2];
	Rectangle rect{ center.xpos, center.ypos, center.width, center.height };
	Span<Rectangle> windows{ &rect, 1 };
	windowsToNative(windows, afConfig.cropX, afConfig.cropY,
			afConfig.scaleX, afConfig.scaleY);
	algo_->setWindows({ windows.data(), windows.size() });

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Af::queueRequest([[maybe_unused]] IPAContext &context,
		      [[maybe_unused]] const uint32_t frame,
		      [[maybe_unused]] IPAFrameContext &frameContext,
		      const ControlList &controls)
{
	auto &afFContext = frameContext.af;

	const auto &afMode = controls.get(controls::AfMode);
	if (afMode) {
		if (std::find(AfModeValues.begin(), AfModeValues.end(),
			      *afMode) != AfModeValues.end())
			afFContext.mode = static_cast<AfModeEnum>(*afMode);
		else
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfMode: " << *afMode;
	}

	const auto &afRange = controls.get(controls::AfRange);
	if (afRange) {
		if (std::find(AfRangeValues.begin(), AfRangeValues.end(),
			      *afRange) != AfRangeValues.end())
			afFContext.range = static_cast<AfRangeEnum>(*afRange);
		else
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfRange: " << *afRange;
	}

	const auto &afSpeed = controls.get(controls::AfSpeed);
	if (afSpeed) {
		if (std::find(AfSpeedValues.begin(), AfSpeedValues.end(),
			      *afSpeed) != AfSpeedValues.end())
			afFContext.speed = static_cast<AfSpeedEnum>(*afSpeed);
		else
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfSpeed: " << *afSpeed;
	}

	const auto &afMetering = controls.get(controls::AfMetering);
	if (afMetering) {
		if (std::find(AfMeteringValues.begin(), AfMeteringValues.end(),
			      *afMetering) != AfMeteringValues.end())
			afFContext.metering = static_cast<AfMeteringEnum>(*afMetering);
		else
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfMetering: " << *afMetering;
	}

	const auto &afWindows = controls.get(controls::AfWindows);
	auto &afConfig = context.configuration.af;
	if (afWindows) {
		afFContext.windows = std::vector<Rectangle>{};
		for (const auto &window : afWindows.value())
			afFContext.windows->push_back(window);
		Span<Rectangle> windows{ afFContext.windows->data(),
					 afFContext.windows->size() };
		windowsToNative(windows, afConfig.cropX, afConfig.cropY,
				afConfig.scaleX, afConfig.scaleY);
	}

	const auto &afPause = controls.get(controls::AfPause);
	if (afPause) {
		if (std::find(AfPauseValues.begin(), AfPauseValues.end(),
			      *afPause) != AfPauseValues.end())
			afFContext.pause = static_cast<AfPauseEnum>(*afPause);
		else
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfPause: " << *afPause;
	}

	const auto &afTrigger = controls.get(controls::AfTrigger);
	if (afTrigger) {
		if (algo_->getMode() != AfModeAuto)
			LOG(NxpNeoAlgoAf, Warning) << "AfTrigger is restricted to Auto mode";
		else if (std::find(AfTriggerValues.begin(), AfTriggerValues.end(),
				   *afTrigger) != AfTriggerValues.end())
			afFContext.trigger = static_cast<AfTriggerEnum>(*afTrigger);
		else
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfTrigger: " << *afTrigger;
	}

	const auto &afPosition = controls.get(controls::LensPosition);
	if (afPosition)
		afFContext.lensPosition = afPosition.value();
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Af::prepare([[maybe_unused]] IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 [[maybe_unused]] NxpNeoParams *params)
{
	/* User controls handling */
	auto &afFContext = frameContext.af;
	auto &afState = context.activeState.af;

	if (afFContext.mode) {
		AfModeEnum mode = afFContext.mode.value();
		afState.mode = mode;
		algo_->setMode(mode);
	}
	if (afFContext.range) {
		AfRangeEnum range = afFContext.range.value();
		algo_->setRange(range);
	}
	if (afFContext.speed) {
		AfSpeedEnum speed = afFContext.speed.value();
		algo_->setSpeed(speed);
	}
	if (afFContext.metering) {
		AfMeteringEnum metering = afFContext.metering.value();
		algo_->setMetering(metering != AfMeteringAuto);
	}
	if (afFContext.windows) {
		const std::vector<Rectangle> &vec = afFContext.windows.value();
		Span<const Rectangle> afWindows(vec.data(), vec.size());
		algo_->setWindows(afWindows);
	}
	if (afFContext.pause) {
		AfPauseEnum pauseValue = afFContext.pause.value();
		algo_->pause(pauseValue);
	}
	if (afFContext.trigger) {
		AfTriggerEnum trigger = afFContext.trigger.value();
		if (trigger == AfTriggerStart)
			algo_->triggerScan();
		else
			algo_->cancelScan();
	}

	if (afFContext.lensPosition) {
		int32_t hwPosition;
		float position = afFContext.lensPosition.value();
		algo_->setLensPosition(position, &hwPosition);
	}

	/* \todo populate PDAF data. */
	PdafRegions pdafRegions;
	AfStatus afStatus;
	algo_->doPrepare(pdafRegions, afStatus);

	afFContext.state = afStatus.state;
	afFContext.pauseState = afStatus.pauseState;

	afState.hwPositionUpdate = false;
	if (afStatus.lensSetting) {
		int32_t hwPosition = afStatus.lensSetting.value();
		if (!afState.hwPosition) {
			/* Lens position unknown - update unconditionally. */
			afState.hwPosition = hwPosition;
			afState.hwPositionUpdate = true;
		} else {
			/* Lens position known - update only if changed. */
			if (afState.hwPosition.value() != hwPosition) {
				afState.hwPosition = hwPosition;
				afState.hwPositionUpdate = true;
			}
		}
	}

	/* ISP configuration */
	if (frame > 0)
		return;

	auto config = params->block<BlockParamsType::Af>();
	config.setUpdate(true);

	auto &afConfig = context.configuration.af;
	for (const auto &[i, roi] : utils::enumerate(afConfig.rois))
		config->af_roi[i] = roi;
	for (const auto &[i, coeff] : utils::enumerate(filters_[0]))
		config->fil0_coeffs[i] = coeff;
	config->fil0_shift_shift = shifts_[0];
	for (const auto &[i, coeff] : utils::enumerate(filters_[1]))
		config->fil1_coeffs[i] = coeff;
	config->fil1_shift_shift = shifts_[1];
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Af::process([[maybe_unused]] IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 [[maybe_unused]] const NxpNeoStats *stats,
		 ControlList &metadata)
{
	auto afStats = stats->block<BlockStatsType::RAf>();
	auto afConfig = context.configuration.af;

	/* Populate CDAF statistics from the ISP AF grid. */
	FocusRegions focusRegions;
	focusRegions.init({ NEO_AF_BLOCK_NB_X, NEO_AF_BLOCK_NB_Y });
	for (int row = 0; row < NEO_AF_BLOCK_NB_Y; row++) {
		for (int col = 0; col < NEO_AF_BLOCK_NB_X; col++) {
			size_t i = row * NEO_AF_BLOCK_NB_X + col;
			double c0 = static_cast<double>(afStats->rois[i].sum0);
			double c1 = static_cast<double>(afStats->rois[i].sum1);
			double c = std::sqrt(c0 * c0 + c1 * c1);
			double cNorm = c / afConfig.normalGains[i];
			/*
			 * [0, 1] contrast is scaled to an arbitrary value.
			 * RPi AF algorithm implementation uses 1.0e9 as the
			 * maximum cumulated and weighted value over all cells.
			 */
			double cScale = 1.0e6;
			uint64_t val = static_cast<uint64_t>(cNorm * cScale);
			RPiController::RegionStats<uint64_t>::Region
				region{ val, 0, 0 };
			focusRegions.set({ col, row }, region);
		}
	}

	/* Populate AWB statistics from the CTEMP grid. */
	const auto &blockSums = context.activeState.awb.blockSums;
	RgbyRegions awbRegions;
	awbRegions.init({ NEO_CTEMP_BLOCK_NB_X, NEO_CTEMP_BLOCK_NB_Y });
	for (int row = 0; row < NEO_CTEMP_BLOCK_NB_Y; row++) {
		for (int col = 0; col < NEO_CTEMP_BLOCK_NB_X; col++) {
			const RGB<uint64_t> &sum = blockSums[row][col];
			/* \todo compute ySum - unused by algorithm as of now.*/
			RPiController::RegionStats<RgbySums>::Region
				region{ { sum.r(), sum.g(), sum.b() }, 0, 0 };
			awbRegions.set({ col, row }, region);
		}
	}

	algo_->doProcess(focusRegions, awbRegions);

	/* Populate metadata */
	auto &afFContext = frameContext.af;
	metadata.set(controls::AfState, afFContext.state);
	metadata.set(controls::AfPauseState, afFContext.pauseState);
	std::optional<float> position = algo_->getLensPosition();
	if (position)
		metadata.set(controls::LensPosition, position.value());
}

void Af::windowsToNative(Span<Rectangle> windows,
			 uint16_t cropX, uint16_t cropY,
			 double scaleX, double scaleY)
{
	/*
	 * AutoFocus algorithm expects windows in native sensor coordinates.
	 * Thus, adjust the coordinates based on the sensor cropping and binning
	 * applicable to the current mode.
	 */
	for (Rectangle &window : windows) {
		window.x = window.x * scaleX + cropX;
		window.y = window.y * scaleY + cropY;
		window.width *= scaleX;
		window.height *= scaleY;
	}
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
