/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * camera_helper_os08a20.c
 * Helper class that performs sensor-specific parameter computations
 * for Omnivision OS08A20 sensor
 * Copyright 2024 NXP
 */

#include <cmath>

#include <libcamera/base/log.h>

#include "camera_helper.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpCameraHelper)

namespace nxp {

class CameraHelperOs08a20 : public CameraHelper
{
public:
	CameraHelperOs08a20()
	{
		/* gainType_ / gainConstants_ are unused */
	}

	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
};

uint32_t CameraHelperOs08a20::gainCode(double gain) const
{
	uint32_t code;

	/*
	 * Real gain
	 * code           real gain
	 * 0x0080-0x00FF  INT(code/8)/16
	 * 0x0100-0x01FF  INT(code/16)/8
	 * 0x0200-0x03FF  INT(code/32)/4
	 * 0x0400-0x07FF  INT(code/64)/2
	 */
	if (gain >= 15.5)
		gain = 15.5;
	else if (gain < 1.0)
		gain = 1.0;

	if (gain >= 8.0)
		code = (static_cast<int>(std::ceil(gain * 2.0 * 64.0)));
	else if (gain >= 4.0)
		code = (static_cast<int>(std::ceil(gain * 4.0 * 32.0)));
	else if (gain >= 2.0)
		code = (static_cast<int>(std::ceil(gain * 8.0 * 16.0)));
	else
		code = (static_cast<int>(std::ceil(gain * 16.0 * 8.0)));

	return code;
}

double CameraHelperOs08a20::gain(uint32_t gainCode) const
{
	double gain;

	/*
	 * See gainCode() for format (inverse transform)
	 */
	gainCode &= 0x7FFU;
	if (gainCode >= 0x400U)
		gain = std::floor(gainCode / 64) / 2.0;
	else if (gainCode >= 0x200)
		gain = std::floor(gainCode / 32) / 4.0;
	else if (gainCode >= 0x100)
		gain = std::floor(gainCode / 16) / 8.0;
	else
		gain = std::floor(gainCode / 8) / 16.0;

	return gain;
}

REGISTER_CAMERA_HELPER("os08a20", CameraHelperOs08a20)

} /* namespace nxp */

} /* namespace libcamera */
