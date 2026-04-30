/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2026 NXP
 *
 * Helper class that performs sensor-specific parameter computations
 * for a VIVID test camera
 */

#include "camera_helper.h"

namespace libcamera {

namespace nxp {

class CameraHelperVivid : public CameraHelper
{
public:
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
};

uint32_t CameraHelperVivid::gainCode([[maybe_unused]] double gain) const
{
	return static_cast<uint32_t>(1);
}

double CameraHelperVivid::gain([[maybe_unused]] uint32_t gainCode) const
{
	return 1.0;
}

REGISTER_CAMERA_HELPER("vivid", CameraHelperVivid)

} /* namespace nxp */

} /* namespace libcamera */
