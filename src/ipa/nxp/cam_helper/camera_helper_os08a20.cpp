/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Helper class that performs sensor-specific parameter computations
 * for Omnivision OS08A20 sensor
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

class CameraHelperOs08a20 : public CameraHelper
{
public:
	CameraHelperOs08a20()
	{
		/* gainType_ / gainConstants_ are unused */

		/* Adapt the default delayedControls for the os08a20 custom controls */
		attributes_.delayedControlParams = {
			{ V4L2_CID_ANALOGUE_GAIN, { 2, false } },
			{ V4L2_CID_EXPOSURE, { 2, false } },
			{ V4L2_CID_AGAIN_MULTI, { 2, false } },
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
	void controlInfoMapGetExposureRange(
		const ControlInfoMap *ctrls, std::vector<Duration> *minExposure,
		std::vector<Duration> *maxExposure, std::vector<Duration> *defExposure) const override;
	void controlInfoMapGetAnalogGainRange(
		const ControlInfoMap *ctrls, std::vector<double> *minGain,
		std::vector<double> *maxGain, std::vector<double> *defGain) const override;

private:
	uint32_t maxExposureLines() const;

	/* min/max analog real gain value */
	static constexpr double kMinAnalogGain = 1.0;
	static constexpr double kMaxAnalogGain = 15.5;
	static constexpr double kDefAnalogGain = 5.0;
	/* min/max exposures */
	/*
	 * Set min short exposure lines to 1 (versus 0) to prevent division by 0
	 * when distributing total exposure between long and short captures.
	 */
	static constexpr uint32_t kMinShortExposureLines = 1;
	static constexpr uint32_t kMaxShortExposureLines = 32;
	static constexpr uint32_t kMinLongExposureLines = 8;

	static constexpr uint32_t kRatioL2S = 16;
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

void CameraHelperOs08a20::controlListSetAGC(
	ControlList *ctrls,
	Span<const Duration> exposures, Span<const double> gains)
{
	/* In non-HDR mode, the standard single-capture controls are used. */
	if (mode_.streamMode != SensorStreamMode::Hdr)
		return CameraHelper::controlListSetAGC(ctrls, exposures, gains);

	/* In HDR mode, the multi-capture controls are used. */

	if (exposures.empty() || gains.empty())
		return;

	/* Exposure time and gain provided by AGC apply to long capture. */
	uint32_t expRowsLong = exposureLines(exposures[0], hblankToLineLength(mode_.hblank));
	double aGainLong = gains[0];
	uint32_t expRowsShort = std::clamp(maxExposureLines() - expRowsLong,
					   kMinShortExposureLines,
					   kMaxShortExposureLines);
	double aGainShort;
	double expTotalLong = expRowsLong * aGainLong;
	/* Total short exposure is total long exposure divided by ratio. */
	double expTotalShort = expTotalLong / kRatioL2S;

	aGainShort = expTotalShort / expRowsShort;
	if (aGainShort < kMinAnalogGain) {
		/*
		 * If short gain is below minimum, set short gain to minimum and
		 * reduce short exposure to obtain same short total exposure.
		 */
		aGainShort = kMinAnalogGain;
		expRowsShort = std::max(static_cast<uint32_t>(std::round(expTotalShort / aGainShort)),
					kMinShortExposureLines);
	} else if (aGainShort > kMaxAnalogGain) {
		/*
		 * If short gain is above maximum, set short gain to maximum and
		 * reduce long gain to keep ratio between long and short total exposure.
		 */
		aGainShort = kMaxAnalogGain;
		aGainLong = (aGainShort * kRatioL2S) /
			    (static_cast<double>(expRowsLong) / expRowsShort);
		if (aGainLong < kMinAnalogGain) {
			/* Increase long gain to min. */
			aGainLong = kMinAnalogGain;
			/* Decrease long exposure based on ratio. */
			expRowsLong = static_cast<uint32_t>(
				std::round((aGainShort * expRowsShort * kRatioL2S) / aGainLong));
		}
	}

	std::array<uint32_t, 2> multiExposures = { expRowsLong, expRowsShort };
	ctrls->set(V4L2_CID_EXPOSURE_MULTI, Span<uint32_t>(multiExposures));

	std::array<uint32_t, 2> multiGains = { gainCode(aGainLong),
					       gainCode(aGainShort) };
	ctrls->set(V4L2_CID_AGAIN_MULTI, Span<uint32_t>(multiGains));
}

void CameraHelperOs08a20::controlInfoMapGetExposureRange(
	const ControlInfoMap *ctrls, std::vector<Duration> *minExposure,
	std::vector<Duration> *maxExposure, std::vector<Duration> *defExposure) const
{
	(void)ctrls;

	/*
	 * In non-HDR mode, the exposure range comes from
	 * the standard single-capture controls.
	 */
	if (mode_.streamMode != SensorStreamMode::Hdr)
		return CameraHelper::controlInfoMapGetExposureRange(ctrls, minExposure,
								    maxExposure, defExposure);

	/*
	 * In HDR mode, the exposure range is provided from the long exposure
	 * which is the one computed by the AGC. This is hard coded setting
	 * depending on the sensor mode.
	 */
	/*
	 * When maximum exposure is required, we should ensure to reach
	 * as high value as possible for both long and short exposures.
	 * Hence the long exposure should be clamped to the maximum supported
	 * according to both constraints:
	 * - T_long + T_short <= maxExposureLines
	 * - T_short is set to maximum for short capture
	 */
	uint32_t maxLongExposureLines = maxExposureLines() - kMaxShortExposureLines;

	Duration lineLength = hblankToLineLength(mode_.hblank);
	minExposure->clear();
	minExposure->push_back(exposure(kMinLongExposureLines, lineLength));

	maxExposure->clear();
	maxExposure->push_back(exposure(maxLongExposureLines, lineLength));

	defExposure->clear();
	defExposure->push_back(exposure((kMinLongExposureLines + maxLongExposureLines) / 2, lineLength));
}

void CameraHelperOs08a20::controlInfoMapGetAnalogGainRange(
	const ControlInfoMap *ctrls, std::vector<double> *minGain,
	std::vector<double> *maxGain, std::vector<double> *defGain) const
{
	(void)ctrls;

	/*
	 * In non-HDR mode, the analog gain range comes from
	 * the standard single-capture controls.
	 */
	if (mode_.streamMode != SensorStreamMode::Hdr)
		return CameraHelper::controlInfoMapGetAnalogGainRange(ctrls, minGain,
								      maxGain, defGain);

	/* In HDR mode, the analog gain range is provided from hard coded setting.
	 */
	minGain->clear();
	minGain->push_back(kMinAnalogGain);

	maxGain->clear();
	maxGain->push_back(kMaxAnalogGain);

	defGain->clear();
	defGain->push_back(kDefAnalogGain);
}

int CameraHelperOs08a20::sensorControlsToMetaData(const ControlList *sensorCtrls,
						  ControlList *mdCtrls) const
{
	int ret = 0;

	/* In non-HDR mode, the standard single-capture controls are used. */
	if (mode_.streamMode != SensorStreamMode::Hdr)
		return CameraHelper::sensorControlsToMetaData(sensorCtrls, mdCtrls);

	/* In HDR mode, the multi-capture controls are used. */
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
	std::array<float, 1> dGainsArray = { 1.0f };
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

/**
 * \brief Returns the maximum total exposure combining short and long captures
 *        Datasheet mentions T_long + T_short <= frame_length(VTS) - 4
 *        However image is improper if reaching (frame_length(VTS) - 4) - 1.
 *        Images are correct if maximum total exposure is clamped with (frame_length(VTS) - 8)
 *
 * \return Maximum total exposure combining short and long captures
 */
uint32_t CameraHelperOs08a20::maxExposureLines() const
{
	uint32_t vts = mode_.height + mode_.vblank;
	return vts - 8;
}

REGISTER_CAMERA_HELPER("os08a20", CameraHelperOs08a20)

} /* namespace nxp */

} /* namespace libcamera */
