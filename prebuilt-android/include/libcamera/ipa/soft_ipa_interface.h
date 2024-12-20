/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm interface for soft
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once


#include <string>


#include <libcamera/base/flags.h>
#include <libcamera/base/signal.h>

#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>

#include <libcamera/ipa/core_ipa_interface.h>
#include <libcamera/ipa/ipa_interface.h>

namespace libcamera {

namespace ipa {

namespace soft {




enum class _SoftCmd {
	Exit = 0,
	Init = 1,
	Start = 2,
	Stop = 3,
	Configure = 4,
	ProcessStats = 5,
};

enum class _SoftEventCmd {
	SetSensorControls = 1,
	SetIspParams = 2,
};


class IPASoftInterface : public IPAInterface
{
public:

	virtual int32_t init(
		const IPASettings &settings,
		const SharedFD &fdStats,
		const SharedFD &fdParams,
		const ControlInfoMap &sensorCtrlInfoMap) = 0;

	virtual int32_t start() = 0;

	virtual void stop() = 0;

	virtual int32_t configure(
		const ControlInfoMap &sensorCtrlInfoMap) = 0;

	virtual void processStats(
		const ControlList &sensorControls) = 0;

	Signal<const ControlList &> setSensorControls;

	Signal<> setIspParams;
};

} /* namespace soft */

} /* namespace ipa */

} /* namespace libcamera */