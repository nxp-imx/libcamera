/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * Auto focus algorithm interface
 *
 * Copyright 2025 NXP
 * Adapted from the file src/ipa/rpi/controller/rpi/af_algorithm.h
 * to be used as an interface for standard libcamera IPA autofocus algorithm
 */
#pragma once

#include <optional>

#include <libcamera/base/span.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/yaml_parser.h"

#include "region_stats.h"

namespace libcamera {

namespace ipa {

/*
 * RgbySums, RgbyRegions and FocusRegions definitions are imported from header
 * src/ipa/rpi/controller/statistics.h
 */
struct RgbySums {
	RgbySums(uint64_t _rSum = 0, uint64_t _gSum = 0, uint64_t _bSum = 0, uint64_t _ySum = 0)
		: rSum(_rSum), gSum(_gSum), bSum(_bSum), ySum(_ySum)
	{
	}
	uint64_t rSum;
	uint64_t gSum;
	uint64_t bSum;
	uint64_t ySum;
};

using RgbyRegions = RPiController::RegionStats<RgbySums>;
using FocusRegions = RPiController::RegionStats<uint64_t>;

/*
 * PdafDat and PdafData definitions are imported from header
 * src/ipa/rpi/controller/pdaf_data.h
 */
struct PdafData {
	/* Confidence, in arbitrary units */
	uint16_t conf;
	/* Phase error, in s16 Q4 format (S.11.4) */
	int16_t phase;
};

using PdafRegions = RPiController::RegionStats<PdafData>;

/*
 * AfStatus definition is imported from header
 * src/ipa/rpi/controller/af_status.h
 */
struct AfStatus {
	/* state for reporting */
	libcamera::controls::AfStateEnum state;
	libcamera::controls::AfPauseStateEnum pauseState;
	/* lensSetting should be sent to the lens driver, when valid */
	std::optional<int> lensSetting;
};

class AfBase
{
public:
	AfBase() {}
	virtual ~AfBase() = default;

	/*
	 * An autofocus algorithm should provide the following calls.
	 *
	 * Where a ControlList combines a change of AfMode with other AF
	 * controls, setMode() should be called first, to ensure the
	 * algorithm will be in the correct state to handle controls.
	 *
	 * setLensPosition() returns true if the mode was AfModeManual and
	 * the lens position has changed, otherwise returns false. When it
	 * returns true, hwpos should be sent immediately to the lens driver.
	 *
	 * getMode() is provided mainly for validating controls.
	 * getLensPosition() is provided for populating DeviceStatus.
	 *
	 * getDefaultlensPosition() and getLensLimits() were added for
	 * populating ControlInfoMap. They return the static API limits
	 * which should be independent of the current range or mode.
	 */

	virtual void setRange([[maybe_unused]] controls::AfRangeEnum range)
	{
	}
	virtual void setSpeed([[maybe_unused]] controls::AfSpeedEnum speed)
	{
	}
	virtual void setMetering([[maybe_unused]] bool use_windows)
	{
	}
	virtual void setWindows([[maybe_unused]] Span<Rectangle const> const &wins)
	{
	}
	virtual void setMode(controls::AfModeEnum mode) = 0;
	virtual controls::AfModeEnum getMode() const = 0;
	virtual double getDefaultLensPosition() const = 0;
	virtual void getLensLimits(double &min, double &max) const = 0;
	virtual bool setLensPosition(double dioptres, int32_t *hwpos, bool force = false) = 0;
	virtual std::optional<double> getLensPosition() const = 0;
	virtual void triggerScan() = 0;
	virtual void cancelScan() = 0;
	virtual void pause(controls::AfPauseEnum pause) = 0;

	/*
	 * Triggers to activate the algorithm to be invoked from their
	 * respective IPA calls.
	 */
	virtual int doInit(const YamlObject &tuningData) = 0;
	virtual int doConfigure(const IPACameraSensorInfo &sensorInfo) = 0;
	virtual void doPrepare(const PdafRegions &regions, AfStatus &status) = 0;
	virtual void doProcess(const FocusRegions &focusRegions,
			       const RgbyRegions &awbRegions) = 0;
};

} /* namespace ipa */

} /* namespace libcamera */
