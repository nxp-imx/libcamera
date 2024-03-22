/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * camera_helper_mx95mbcam.c
 * Helper class that performs sensor-specific parameter computations
 * for MX95MBCAM module (OX03C10 camera and a Maxim MAX96717 GMSL2 serializer)
 * Copyright 2024 NXP
 */

#include <cmath>

#include <linux/ox03c10.h>

#include <libcamera/base/log.h>

#include "camera_helper.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpCameraHelper)

namespace nxp {

class CameraHelperMx95mbcam : public CameraHelper
{
public:
	CameraHelperMx95mbcam()
	{
		/* gainType / gainConstants_ are unused */
	}

	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

	uint32_t controlListGetExposure(const ControlList *ctrls) const override;
	uint32_t controlListGetGain(const ControlList *ctrls) const override;

	void controlListSetExposure(
		ControlList *ctrls, uint32_t exposure) const override;
	void controlListSetGain(
		ControlList *ctrls, uint32_t gainCode) const override;

	void controlInfoMapGetExposureRange(
		const ControlInfoMap *ctrls, uint32_t *minExposure,
		uint32_t *maxExposure, uint32_t *defExposure = nullptr) const override;

	void controlInfoMapGetGainRange(
		const ControlInfoMap *ctrls, uint32_t *minGainCode,
		uint32_t *maxGainCode, uint32_t *defGainCode = nullptr) const override;

	std::map<int32_t, std::pair<uint32_t, bool>> delayedControlParams() const override;

private:
	/* total number of raws per frame from driver init sequence (unit double-raw) */
	static constexpr unsigned int kVts = 0x2AEU;
};

uint32_t CameraHelperMx95mbcam::gainCode(double gain) const
{
	/* Analog gain is Q4.4 with variable fractional resolution */
	if (gain >= 15.5)
		gain = 15.5;
	else if (gain < 1.0)
		gain = 1.0;

	uint32_t code;
	if (gain >= 8.0)
		code = (static_cast<int>(std::ceil(gain * 2.0)) << 3);
	else if (gain >= 4.0)
		code = (static_cast<int>(std::ceil(gain * 4.0)) << 2);
	else if (gain >= 2.0)
		code = (static_cast<int>(std::ceil(gain * 8.0)) << 1);
	else
		code = static_cast<int>(std::ceil(gain * 16.0));

	return code;
}

double CameraHelperMx95mbcam::gain(uint32_t gainCode) const
{
	/* Analog gain is Q4.4 */
	return static_cast<double>(gainCode) * 0.0625;
}

uint32_t
CameraHelperMx95mbcam::controlListGetExposure(const ControlList *ctrls) const
{
	uint32_t exposure = 0;
	if (!ctrls->contains(V4L2_CID_OX03C10_EXPOSURE)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_OX03C10_EXPOSURE cannot be got";
		return exposure;
	}

	/*
	 * \todo Spurious frame loss at stream start() induces invalid
	 * control values reported by pipeline's DelayedControl object.
	 * Occurence is logged by pipeline, workaround the issue to avoid
	 * later assert condition failure.
	 */

	const ControlValue &value = ctrls->get(V4L2_CID_OX03C10_EXPOSURE);
	if (value.type() != ControlTypeNone) {
		Span<const uint8_t> data = value.data();
		ASSERT(data.size() == sizeof(ox03c10_exposure));
		const struct ox03c10_exposure *payload;
		payload = reinterpret_cast<const struct ox03c10_exposure *>(data.data());

		/* Exposure is reported in rows, sensor uses double-row unit */
		exposure = payload->dcg * 2;
	} else {
		/* Keep debug level for now as root cause occurence is logged */
		LOG(NxpCameraHelper, Warning) << "Invalid exposure control";
	}

	return exposure;
}

uint32_t CameraHelperMx95mbcam::controlListGetGain(const ControlList *ctrls) const
{
	uint32_t gain = 0;
	if (!ctrls->contains(V4L2_CID_OX03C10_ANALOGUE_GAIN)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_OX03C10_ANALOGUE_GAIN cannot be got";
		return gain;
	}

	/*
	 * \todo Spurious frame loss at stream start() induces invalid
	 * control values reported by pipeline's DelayedControl object.
	 * Occurence is logged by pipeline, workaround the issue to avoid
	 * later assert condition failure.
	 */
	const ControlValue &value = ctrls->get(V4L2_CID_OX03C10_ANALOGUE_GAIN);

	if (value.type() != ControlTypeNone) {
		Span<const uint8_t> data = value.data();
		ASSERT(data.size() == sizeof(ox03c10_analog_gain));
		const struct ox03c10_analog_gain *payload;
		payload = reinterpret_cast<const struct ox03c10_analog_gain *>(data.data());

		gain = payload->hcg;
	} else {
		/* Keep debug level for now as root cause occurence is logged */
		LOG(NxpCameraHelper, Debug) << "Invalid gain control";
	}

	return gain;
}

void CameraHelperMx95mbcam::controlListSetExposure(
	ControlList *ctrls, uint32_t exposure) const
{
	if (!controlListHasId(ctrls, V4L2_CID_OX03C10_EXPOSURE)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_OX03C10_EXPOSURE cannot be set";
		return;
	}

	/*
	 * unit is double row
	 * HDR3: dcg_exp + vs_exp < vts - 12
	 *       vs_exp <= 5 (empirical - documented value is 34)
	 *       dcg_exp >= 2, vs_exp >= 2, spd_exp >= 1
	 */
	uint16_t dcg = exposure / 2;
	uint16_t vs = 4;
	if (dcg <= 2)
		dcg = 2;
	if ((dcg + vs) >= (kVts - 12))
		dcg = kVts - 12 - vs - 1;
	uint16_t spd = dcg + vs;

	struct ox03c10_exposure exposures;
	exposures.dcg = dcg;
	exposures.spd = spd;
	exposures.vs = vs;

	Span<uint8_t> data(reinterpret_cast<uint8_t *>(&exposures),
			   sizeof(ox03c10_exposure));

	ctrls->set(V4L2_CID_OX03C10_EXPOSURE, data);
}

void CameraHelperMx95mbcam::controlListSetGain(
	ControlList *ctrls, uint32_t gainCode) const
{
	if (!controlListHasId(ctrls, V4L2_CID_OX03C10_ANALOGUE_GAIN)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_OX03C10_ANALOGUE_GAIN cannot be set";
		return;
	}

	struct ox03c10_analog_gain again;
	uint8_t gainCode8 = static_cast<uint8_t>(gainCode);
	again.hcg = gainCode8;
	again.lcg = gainCode8;
	again.spd = gainCode8;
	again.vs = gainCode8;

	Span<uint8_t> adata(reinterpret_cast<uint8_t *>(&again),
			    sizeof(ox03c10_analog_gain));

	ctrls->set(V4L2_CID_OX03C10_ANALOGUE_GAIN, adata);

	/* Set unitary digital gain for now (Q4.10) */
	struct ox03c10_digital_gain dgain;
	dgain.hcg = 0x400U;
	dgain.lcg = 0x400U;
	dgain.spd = 0x400U;
	dgain.vs = 0x400U;

	Span<uint8_t> ddata(reinterpret_cast<uint8_t *>(&dgain),
			    sizeof(ox03c10_digital_gain));

	ctrls->set(V4L2_CID_OX03C10_DIGITAL_GAIN, ddata);
}

void CameraHelperMx95mbcam::controlInfoMapGetExposureRange(
	const ControlInfoMap *ctrls, uint32_t *minExposure,
	uint32_t *maxExposure, uint32_t *defExposure) const
{
	(void)ctrls;

	/*
	 * unit is double row
	 * HDR3: dcg_exp + vs_exp < vts - 12
	 *       vs_exp <= 5 (empirical - documented value is 34)
	 *       dcg_exp >= 2, vs_exp >= 2, spd_exp >= 1
	 */
	if (minExposure) {
		*minExposure = 2;
		*minExposure *= 2;
	}
	if (maxExposure) {
		*maxExposure = kVts - 12 - 5 - 1;
		*maxExposure *= 2;
	}
	if (defExposure) {
		/* Default dcg init value from driver */
		*defExposure = 0x200U;
		*defExposure *= 2;
	}
}

void CameraHelperMx95mbcam::controlInfoMapGetGainRange(
	const ControlInfoMap *ctrls, uint32_t *minGainCode,
	uint32_t *maxGainCode, uint32_t *defGainCode) const
{
	(void)ctrls;

	/* Analog gain is Q4.4 */
	if (minGainCode)
		*minGainCode = 0x10U; // 1.0
	if (maxGainCode)
		*maxGainCode = 0xF8U; // 15.5
	if (defGainCode)
		/* Default hcg init value from driver */
		*defGainCode = 0x50U; // 5.0
}

std::map<int32_t, std::pair<uint32_t, bool>>
CameraHelperMx95mbcam::delayedControlParams() const
{
	static const std::map<int32_t, std::pair<uint32_t, bool>> params = {
		{ V4L2_CID_OX03C10_ANALOGUE_GAIN, { 2, false } },
		{ V4L2_CID_OX03C10_DIGITAL_GAIN, { 2, false } },
		{ V4L2_CID_OX03C10_EXPOSURE, { 2, false } },
	};

	return params;
}

REGISTER_CAMERA_HELPER("mx95mbcam", CameraHelperMx95mbcam)

} /* namespace nxp */

} /* namespace libcamera */
