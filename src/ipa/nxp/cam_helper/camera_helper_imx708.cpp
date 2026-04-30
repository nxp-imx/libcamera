/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * Helper class that performs sensor-specific parameter computations
 * for Sony imx708 sensor
 */

#include "camera_helper.h"

namespace libcamera {

namespace nxp {

class CameraHelperImx708 : public CameraHelper
{
public:
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
};

/* Gain conversions come from RPi cam_helper_imx708.cpp implementation */
uint32_t CameraHelperImx708::gainCode(double gain) const
{
	return static_cast<uint32_t>(1024 - 1024 / gain);
}

double CameraHelperImx708::gain(uint32_t gainCode) const
{
	return 1024.0 / (1024 - gainCode);
}

REGISTER_CAMERA_HELPER("imx708", CameraHelperImx708)

} /* namespace nxp */

} /* namespace libcamera */
