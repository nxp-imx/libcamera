/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * Helper class that performs sensor-specific parameter computations
 * for Onsemi AR0235 sensor
 */

#include <linux/v4l2-controls.h>

#include "camera_helper.h"

namespace libcamera {

namespace nxp {

class CameraHelperAr0235 : public CameraHelper
{
public:
	CameraHelperAr0235()
	{
		/* gainType_ / gainConstants_ are unused */

		/* Adapt the default delayedControls for the ar0235 custom controls */
		attributes_.delayedControlParams = {
			{ V4L2_CID_ANALOGUE_GAIN, { 3, false } },
			{ V4L2_CID_EXPOSURE, { 3, false } },
		};
	}

	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	static constexpr double kGainMin = 1.1875;
	static constexpr double kGainMax = 8.0;

	/* \todo Check gainCode to ensure it is included between min and max */
	static constexpr uint32_t kCodeMin = 3;
	static constexpr uint32_t kCodeMax = 46;
};

uint32_t CameraHelperAr0235::gainCode(double gain) const
{
	gain = std::clamp(gain, kGainMin, kGainMax);

	if (gain < 2.0)
		return (gain - 1.0) / 0.0625;
	else if (gain < 4.0)
		return 16 + (gain - 2.0) / 0.125;
	else
		return 32 + (gain - 4.0) / 0.25;
}

double CameraHelperAr0235::gain(uint32_t gainCode) const
{
	gainCode = std::clamp(gainCode, kCodeMin, kCodeMax);

	if (gainCode < 16)
		return 1.0 + gainCode * 0.0625;
	else if (gainCode < 32)
		return 2.0 + (gainCode - 16) * 0.125;
	else
		return 4.0 + (gainCode - 32) * 0.25;
}

REGISTER_CAMERA_HELPER("ar0235", CameraHelperAr0235)

} /* namespace nxp */

} /* namespace libcamera */
