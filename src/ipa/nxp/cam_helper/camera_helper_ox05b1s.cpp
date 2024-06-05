/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * camera_helper_ox05b1s.c
 * Helper class that performs sensor-specific parameter computations
 * for Omnivision ox05b1s sensor
 * Copyright 2024 NXP
 */

#include <cmath>

#include <libcamera/base/log.h>

#include "camera_helper.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpCameraHelper)

namespace nxp {

class CameraHelperOx05b1s : public CameraHelper
{
public:
	CameraHelperOx05b1s()
	{
		attributes_.rgbIr = true;

		/* gainType_ / gainConstants_ are unused */
	}

	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
};

uint32_t CameraHelperOx05b1s::gainCode(double gain) const
{
	/* Real Gain is (gain registers)/16 */
	if (gain >= 15.5)
		gain = 15.5;
	else if (gain < 1.0)
		gain = 1.0;

	return static_cast<uint32_t>(gain * 16);
}

double CameraHelperOx05b1s::gain(uint32_t gainCode) const
{
	/* Real Gain is (gain registers)/16 */
	if (gainCode >= 255)
		gainCode = 255;
	else if (gainCode < 16)
		gainCode = 16;

	return (gainCode / 16.0);
}

REGISTER_CAMERA_HELPER("ox05b1s", CameraHelperOx05b1s)

} /* namespace nxp */

} /* namespace libcamera */
