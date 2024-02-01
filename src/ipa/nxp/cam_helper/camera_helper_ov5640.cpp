/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Helper class that performs sensor-specific parameter computations
 *     src/ipa/libipa/camera_sensor_helper.c
 * Copyright (C) 2021, Google Inc.
 *
 * camera_helper_ov5640.c
 * Helper class that performs sensor-specific parameter computations
 * for ov5640 sensor
 * Copyright 2024 NXP
 */
#include "camera_helper.h"

#include <cmath>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpCameraHelper)

namespace nxp {

class CameraHelperOv5640 : public CameraHelper
{
public:
	CameraHelperOv5640()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 1, 0, 0, 16 };
	}

};

REGISTER_CAMERA_HELPER("ov5640", CameraHelperOv5640)

} /* namespace nxp */

} /* namespace libcamera */
