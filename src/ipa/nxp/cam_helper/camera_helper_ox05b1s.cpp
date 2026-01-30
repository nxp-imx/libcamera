/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Helper class that performs sensor-specific parameter computations
 * for Omnivision ox05b1s sensor
 */

#include <cmath>

#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

#include "camera_helper.h"

using namespace std::literals::chrono_literals;

namespace libcamera {

#ifndef V4L2_CID_EXPOSURE_MULTI
#define V4L2_CID_EXPOSURE_MULTI (V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 10)
#define V4L2_CID_AGAIN_MULTI (V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 11)
#define V4L2_CID_DGAIN_MULTI (V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 12)
#endif

LOG_DECLARE_CATEGORY(NxpCameraHelper)

namespace nxp {

class CameraHelperOx05b1s : public CameraHelper
{
public:
	CameraHelperOx05b1s()
	{
		/* gainType_ / gainConstants_ are unused */

		/* Adapt the default delayedControls for the ox05b1s custom controls */
		attributes_.delayedControlParams = {
			{ V4L2_CID_ANALOGUE_GAIN, { 1, false } },
			{ V4L2_CID_EXPOSURE, { 2, false } },
			{ V4L2_CID_AGAIN_MULTI, { 1, false } },
			{ V4L2_CID_EXPOSURE_MULTI, { 2, false } },
		};
	}

	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

	void controlListSetAGC(
		ControlList *ctrls,
		Span<const Duration> exposures, Span<const double> gains) override;
	int sensorControlsToMetaData(
		const ControlList *sensorCtrls, ControlList *mdCtrls) const override;

private:
	static constexpr double kMinAnalogGain = 1.0;
	static constexpr uint32_t kMinexposureLines = 6;
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

void CameraHelperOx05b1s::controlListSetAGC(
	ControlList *ctrls,
	Span<const Duration> exposures, Span<const double> gains)
{
	/* In non Dual Context mode, the standard single-capture controls are used. */
	if (mode_.streamMode != SensorStreamMode::DualContext)
		return CameraHelper::controlListSetAGC(ctrls, exposures, gains);

	/* In Dual Context mode, the multi-capture controls are used. */
	if (exposures.empty() || gains.empty())
		return;

	if (exposures.size() != 2 || gains.size() != 2) {
		LOG(NxpCameraHelper, Error) << "Exposure and gain values should be 2.";
		return;
	}

	/*
	 * The multi-capture controls will be enabled in RGBIr dual mode when
	 * the sensor driver will have proper context switch operation.
	 */
	std::array<uint32_t, 2> exposureLines;
	std::array<uint32_t, 2> gainCodes;
	Duration lineLength = hblankToLineLength(mode_.hblank);
	for (size_t i = 0; i < 2; i++) {
		exposureLines[i] = CameraHelper::exposureLines(exposures[i],
							       lineLength);
		gainCodes[i] = gainCode(gains[i]);
	}

	ctrls->set(V4L2_CID_AGAIN_MULTI, Span<uint32_t>(gainCodes));
	ctrls->set(V4L2_CID_EXPOSURE_MULTI, Span<uint32_t>(exposureLines));
}

int CameraHelperOx05b1s::sensorControlsToMetaData(const ControlList *sensorCtrls,
						  ControlList *mdCtrls) const
{
	/* In non Dual Context mode, the standard single-capture controls are used. */
	if (mode_.streamMode != SensorStreamMode::DualContext)
		return CameraHelper::sensorControlsToMetaData(sensorCtrls,
							      mdCtrls);

	/* In Dual Context mode, the multi-capture controls are used. */
	int ret = 0;
	const ControlValue &aGainCtrl = sensorCtrls->get(V4L2_CID_AGAIN_MULTI);
	std::array<float, 2> aGainsArray = { 1.0f, 1.0f };
	if (!aGainCtrl.isNone()) {
		Span<const uint32_t> aGainCodes = aGainCtrl.get<Span<const uint32_t>>();
		ASSERT(aGainCodes.size() == 2);
		aGainsArray[0] = gain(aGainCodes[0]);
		aGainsArray[1] = gain(aGainCodes[1]);
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid analog gain control";
		ret = -EINVAL;
	}
	mdCtrls->set(md::AnalogueGain, Span<float>(aGainsArray));

	/* Unitary gain for digital gain */
	std::array<float, 2> dGainsArray = { 1.0f, 1.0f };
	mdCtrls->set(md::DigitalGain, Span<float>(dGainsArray));

	const ControlValue &exposureCtrl = sensorCtrls->get(V4L2_CID_EXPOSURE_MULTI);
	std::array<float, 2> exposureArray = { 0.0f, 0.0f };
	if (!exposureCtrl.isNone()) {
		Span<const uint32_t> exposures = exposureCtrl.get<Span<const uint32_t>>();
		ASSERT(exposures.size() == 2);
		Duration lineLength = hblankToLineLength(mode_.hblank);
		exposureArray[0] = exposure(exposures[0], lineLength) / 1.0s;
		exposureArray[1] = exposure(exposures[1], lineLength) / 1.0s;
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid exposure control";
		ret = -EINVAL;
	}
	mdCtrls->set(md::Exposure, Span<float>(exposureArray));

	/* Unitary gains for white balance */
	std::array<float, 4> wbGains = { 1.0f, 1.0f, 1.0f, 1.0f };
	mdCtrls->set(md::WhiteBalanceGain, Span<float>(wbGains));

	/* Arbitrary temperature value */
	mdCtrls->set(md::Temperature, kDefaultTemperatureCelsius);

	return ret;
}

REGISTER_CAMERA_HELPER("ox05b1s", CameraHelperOx05b1s)

} /* namespace nxp */

} /* namespace libcamera */
