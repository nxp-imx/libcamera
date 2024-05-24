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
#include "md_parser_ox.h"

#define ENABLE_EMBEDDED_DATA 1

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpCameraHelper)

namespace nxp {

/*
 * Registers included in embedded data
 * List comes from Linux driver init, defined by access control configuration
 * for group 4 hold (register address 0x3804).
 */
constexpr uint32_t AecHcgCtrl01Reg = 0x3501;
constexpr uint32_t AecHcgCtrl02Reg = 0x3502;
constexpr uint32_t AecHcgCtrl08Reg = 0x3508;
constexpr uint32_t AecHcgCtrl09Reg = 0x3509;
constexpr uint32_t AecHcgCtrl0aReg = 0x350a;
constexpr uint32_t AecHcgCtrl0bReg = 0x350b;
constexpr uint32_t AecHcgCtrl0cReg = 0x350c;
constexpr uint32_t AecSpdCtrl01Reg = 0x3541;
constexpr uint32_t AecSpdCtrl02Reg = 0x3542;
constexpr uint32_t AecSpdCtrl08Reg = 0x3548;
constexpr uint32_t AecSpdCtrl09Reg = 0x3549;
constexpr uint32_t AecSpdCtrl0aReg = 0x354a;
constexpr uint32_t AecSpdCtrl0bReg = 0x354b;
constexpr uint32_t AecSpdCtrl0cReg = 0x354c;
constexpr uint32_t AecLcgCtrl08Reg = 0x3588;
constexpr uint32_t AecLcgCtrl09Reg = 0x3589;
constexpr uint32_t AecLcgCtrl0aReg = 0x358a;
constexpr uint32_t AecLcgCtrl0bReg = 0x358b;
constexpr uint32_t AecLcgCtrl0cReg = 0x358c;
constexpr uint32_t AecVsCtrl01Reg = 0x35c1;
constexpr uint32_t AecVsCtrl02Reg = 0x35c2;
constexpr uint32_t AecVsCtrl08Reg = 0x35c8;
constexpr uint32_t AecVsCtrl09Reg = 0x35c9;
constexpr uint32_t AecVsCtrl0aReg = 0x35ca;
constexpr uint32_t AecVsCtrl0bReg = 0x35cb;
constexpr uint32_t AecVsCtrl0cReg = 0x35cc;
constexpr uint32_t MipiCtrl3eReg = 0x483e;
constexpr uint32_t MipiCtrl3fReg = 0x483f;
constexpr uint32_t TmpReg26Reg = 0x4d2a;
constexpr uint32_t TmpReg27Reg = 0x4d2b;
constexpr uint32_t AwbGainHcg0Reg = 0x5280;
constexpr uint32_t AwbGainHcg1Reg = 0x5281;
constexpr uint32_t AwbGainHcg2Reg = 0x5282;
constexpr uint32_t AwbGainHcg3Reg = 0x5283;
constexpr uint32_t AwbGainHcg4Reg = 0x5284;
constexpr uint32_t AwbGainHcg5Reg = 0x5285;
constexpr uint32_t AwbGainHcg6Reg = 0x5286;
constexpr uint32_t AwbGainHcg7Reg = 0x5287;
constexpr uint32_t AwbGainLcg0Reg = 0x5480;
constexpr uint32_t AwbGainLcg1Reg = 0x5481;
constexpr uint32_t AwbGainLcg2Reg = 0x5482;
constexpr uint32_t AwbGainLcg3Reg = 0x5483;
constexpr uint32_t AwbGainLcg4Reg = 0x5484;
constexpr uint32_t AwbGainLcg5Reg = 0x5485;
constexpr uint32_t AwbGainLcg6Reg = 0x5486;
constexpr uint32_t AwbGainLcg7Reg = 0x5487;
constexpr uint32_t AwbGainSpd0Reg = 0x5680;
constexpr uint32_t AwbGainSpd1Reg = 0x5681;
constexpr uint32_t AwbGainSpd2Reg = 0x5682;
constexpr uint32_t AwbGainSpd3Reg = 0x5683;
constexpr uint32_t AwbGainSpd4Reg = 0x5684;
constexpr uint32_t AwbGainSpd5Reg = 0x5685;
constexpr uint32_t AwbGainSpd6Reg = 0x5686;
constexpr uint32_t AwbGainSpd7Reg = 0x5687;
constexpr uint32_t AwbGainVs0Reg = 0x5880;
constexpr uint32_t AwbGainVs1Reg = 0x5881;
constexpr uint32_t AwbGainVs2Reg = 0x5882;
constexpr uint32_t AwbGainVs3Reg = 0x5883;
constexpr uint32_t AwbGainVs4Reg = 0x5884;
constexpr uint32_t AwbGainVs5Reg = 0x5885;
constexpr uint32_t AwbGainVs6Reg = 0x5886;
constexpr uint32_t AwbGainVs7Reg = 0x5887;

constexpr std::initializer_list<uint32_t> registerList [[maybe_unused]] = {
	AecHcgCtrl01Reg,
	AecHcgCtrl02Reg,
	AecHcgCtrl08Reg,
	AecHcgCtrl09Reg,
	AecHcgCtrl0aReg,
	AecHcgCtrl0bReg,
	AecHcgCtrl0cReg,
	AecSpdCtrl01Reg,
	AecSpdCtrl02Reg,
	AecSpdCtrl08Reg,
	AecSpdCtrl09Reg,
	AecSpdCtrl0aReg,
	AecSpdCtrl0bReg,
	AecSpdCtrl0cReg,
	AecLcgCtrl08Reg,
	AecLcgCtrl09Reg,
	AecLcgCtrl0aReg,
	AecLcgCtrl0bReg,
	AecLcgCtrl0cReg,
	AecVsCtrl01Reg,
	AecVsCtrl02Reg,
	AecVsCtrl08Reg,
	AecVsCtrl09Reg,
	AecVsCtrl0aReg,
	AecVsCtrl0bReg,
	AecVsCtrl0cReg,
	MipiCtrl3eReg,
	MipiCtrl3fReg,
	TmpReg26Reg,
	TmpReg27Reg,
	AwbGainHcg0Reg,
	AwbGainHcg1Reg,
	AwbGainHcg2Reg,
	AwbGainHcg3Reg,
	AwbGainHcg4Reg,
	AwbGainHcg5Reg,
	AwbGainHcg6Reg,
	AwbGainHcg7Reg,
	AwbGainLcg0Reg,
	AwbGainLcg1Reg,
	AwbGainLcg2Reg,
	AwbGainLcg3Reg,
	AwbGainLcg4Reg,
	AwbGainLcg5Reg,
	AwbGainLcg6Reg,
	AwbGainLcg7Reg,
	AwbGainSpd0Reg,
	AwbGainSpd1Reg,
	AwbGainSpd2Reg,
	AwbGainSpd3Reg,
	AwbGainSpd4Reg,
	AwbGainSpd5Reg,
	AwbGainSpd6Reg,
	AwbGainSpd7Reg,
	AwbGainVs0Reg,
	AwbGainVs1Reg,
	AwbGainVs2Reg,
	AwbGainVs3Reg,
	AwbGainVs4Reg,
	AwbGainVs5Reg,
	AwbGainVs6Reg,
	AwbGainVs7Reg,
};

class CameraHelperMx95mbcam : public CameraHelper
{
public:
	CameraHelperMx95mbcam();

	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

	uint32_t controlListGetExposure(const ControlList *ctrls) const override;
	uint32_t controlListGetGain(const ControlList *ctrls) const override;

	void controlListSetAGC(
		ControlList *ctrls, uint32_t exposure, double gain) const override;

	void controlInfoMapGetExposureRange(
		const ControlInfoMap *ctrls, uint32_t *minExposure,
		uint32_t *maxExposure, uint32_t *defExposure = nullptr) const override;

	void controlInfoMapGetGainRange(
		const ControlInfoMap *ctrls, uint32_t *minGainCode,
		uint32_t *maxGainCode, uint32_t *defGainCode = nullptr) const override;

	std::map<int32_t, std::pair<uint32_t, bool>> delayedControlParams() const override;
	void parseEmbedded(Span<const uint8_t> buffer, ControlList *mdControls) override;

private:
	/* total number of raws per frame from driver init sequence (unit double-raw) */
	static constexpr unsigned int kVts = 0x2AEU;
	/* gain conversion ratio of HCG/LCG \todo should get from OTP sensor data */
	static constexpr double kConvGain = 7.32;
	/* total exposure ratio between Long (HCG) and Short (DCG) captures */
	static constexpr uint16_t kRatioL2S = 32;
	/* total exposure ratio between Long (HCG) and Very Short (VS) captures */
	static constexpr uint16_t kRatioL2VS = 1024;
	/* min/max exposure values in double row time */
	static constexpr uint16_t kMinExposureDCG = 2;
	static constexpr uint16_t kMinExposureVS = 1;
	/*
	 * \todo Documented value set to 34 gives improper image
	 * use max exposure for VS set to 4 as a workaround
	 */
	static constexpr uint16_t kMaxExposureVS = 4;
	/* min/max analog real gain value */
	static constexpr double kMinAnalogGain = 1.0;
	static constexpr double kMaxAnalogGain = 15.5;
	static constexpr double kDefAnalogGain = 5.0;

	std::unique_ptr<MdParser> parser_;
};

CameraHelperMx95mbcam::CameraHelperMx95mbcam()
{
	/*
	 * Setup embedded data params
	 * \todo setup actual min/max values
	 */
#if ENABLE_EMBEDDED_DATA
	mdParams_.topLines = 2;
#endif
	int32_t minInt32 = std::numeric_limits<int32_t>::min();
	int32_t maxInt32 = std::numeric_limits<int32_t>::max();

	ControlInfoMap::Map ctrlMap = {};
	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&md::AnalogueGain),
			std::forward_as_tuple(minInt32, maxInt32));
	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&md::DigitalGain),
			std::forward_as_tuple(minInt32, maxInt32));
	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&md::Exposure),
			std::forward_as_tuple(minInt32, maxInt32));

	mdParams_.controls =
		ControlInfoMap(std::move(ctrlMap), md::controlIdMap);

	parser_ = std::make_unique<MdParserOmniOx>(registerList);

	/* Note: gainType / gainConstants_ are unused */
}

uint32_t CameraHelperMx95mbcam::gainCode(double gain) const
{
	/* Analog gain is Q4.4 with variable fractional resolution */
	if (gain >= kMaxAnalogGain)
		gain = kMaxAnalogGain;
	else if (gain < kMinAnalogGain)
		gain = kMinAnalogGain;

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

		/* DCG exposure is used for the AGC algorithm */
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

		/* HCG gain is used for the AGC algorithm */
		gain = payload->hcg;
	} else {
		/* Keep debug level for now as root cause occurence is logged */
		LOG(NxpCameraHelper, Debug) << "Invalid gain control";
	}

	return gain;
}

void CameraHelperMx95mbcam::controlListSetAGC(
	ControlList *ctrls, uint32_t exposure, double gain) const
{
	if (!controlListHasId(ctrls, V4L2_CID_OX03C10_EXPOSURE)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_OX03C10_EXPOSURE cannot be set";
		return;
	}
	if (!controlListHasId(ctrls, V4L2_CID_OX03C10_ANALOGUE_GAIN)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_OX03C10_ANALOGUE_GAIN cannot be set";
		return;
	}
	if (!controlListHasId(ctrls, V4L2_CID_OX03C10_DIGITAL_GAIN)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_OX03C10_DIGITAL_GAIN cannot be set";
		return;
	}

	/* converted ratio with conversion gain */
	double convertedRatioL2S = kRatioL2S / kConvGain;
	double convertedRatioL2VS = kRatioL2VS / kConvGain;
	/* min analog gain for HCG capture */
	double minAgainHCG = kMinAnalogGain * convertedRatioL2S;
	/* max exposure for DCG capture - HDR3: dcg_exp + vs_exp < vts - 12 */
	static constexpr uint16_t maxExposureDCG = kVts - 12 - kMaxExposureVS - 1;

	/*
	 * Set DCG exposure and HCG gain
	 */

	/* set DCG exposure to the AGC exposure decision */
	/* exposure unit is double row */
	uint16_t exposureDCG = std::clamp<uint16_t>(exposure / 2,
						    kMinExposureDCG,
						    maxExposureDCG);

	/* set HCG gain to the AGC gain decision */
	double againHCG = gain;
	if (againHCG < minAgainHCG) {
		/* decrease exposure by againHCG/minAgain */
		exposureDCG = std::round((exposureDCG * againHCG) / minAgainHCG);
		/* set HCG gain  */
		againHCG = minAgainHCG;
		/* clamp exposure value to the minimum value */
		exposureDCG = std::max<uint16_t>(kMinExposureDCG, exposureDCG);
	}

	/*
	 * Set LCG analog gain
	 */
	double againLCG = againHCG / convertedRatioL2S;

	/*
	 * Set VS exposure to maximum value
	 */
	uint16_t exposureVS = kMaxExposureVS;

	/*
	 * Set VS analog gain
	 */
	/* ratioL2VS: ratio between total exposure of Long capture with VS capture */
	double againVS = (exposureDCG * againHCG) / (exposureVS * convertedRatioL2VS);
	if (againVS < kMinAnalogGain) {
		againVS = kMinAnalogGain;
		/* decrease exposure according to minimal gain */
		exposureVS = (exposureDCG * againHCG) / (againVS * convertedRatioL2VS);
		/* clamp exposure value to the minimum value */
		exposureVS = std::max<uint16_t>(kMinExposureVS, exposureVS);
	}

	LOG(NxpCameraHelper, Debug) << "exposure[DCG,VS]="
				    << exposureDCG << ", " << exposureVS;
	LOG(NxpCameraHelper, Debug) << "gain[HCG,LCG,VS]=" << againHCG
				    << ", " << againLCG << ", " << againVS;

	/* Set ctrls for exposure */
	struct ox03c10_exposure exposures;
	exposures.dcg = exposureDCG;
	exposures.spd = exposureDCG + exposureVS;
	exposures.vs = exposureVS;
	Span<uint8_t> data(reinterpret_cast<uint8_t *>(&exposures),
			   sizeof(ox03c10_exposure));

	ctrls->set(V4L2_CID_OX03C10_EXPOSURE, data);

	/* Set ctrls for analog gain - use default value for SPD */
	struct ox03c10_analog_gain again;
	again.hcg = static_cast<uint8_t>(gainCode(againHCG));
	again.lcg = static_cast<uint8_t>(gainCode(againLCG));
	again.spd = 0x10U;
	again.vs = static_cast<uint8_t>(gainCode(againVS));

	Span<uint8_t> adata(reinterpret_cast<uint8_t *>(&again),
			    sizeof(ox03c10_analog_gain));

	ctrls->set(V4L2_CID_OX03C10_ANALOGUE_GAIN, adata);

	/* Set ctrls for unitary digital gain for now (Q4.10) */
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
		*minGainCode = gainCode(kMinAnalogGain);
	if (maxGainCode)
		*maxGainCode = gainCode(kMaxAnalogGain);
	if (defGainCode)
		/* Default hcg init value from driver */
		*defGainCode = gainCode(kDefAnalogGain);
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

void CameraHelperMx95mbcam::parseEmbedded(Span<const uint8_t> buffer,
					  ControlList *mdControls)
{
	(void)mdControls;

	if (!buffer.size())
		return;

	MdParser::Status status;
	MdParser::RegisterMap registers;
	status = parser_->parse(buffer, registers);
	if (status != MdParser::Status::OK)
		return;

	/*
	 * Analog gain is currently configured identical for hcg, lcg, vs and
	 * spd in controlListSetGain().
	 * Registers mapping is derived from V4L2_CID_OX03C10_ANALOG_GAIN
	 * implementation.
	 */
	uint32_t hcgAnalogGainCode;
	hcgAnalogGainCode = ((registers[AecHcgCtrl08Reg] & 0xf) << 4) |
			    ((registers[AecHcgCtrl09Reg] & 0xf0) >> 4);
	mdControls->set(md::AnalogueGain, hcgAnalogGainCode);

	/*
	 * Digital gain is currently configured identical for hcg, lcg, vs and
	 * spd in controlListSetGain().
	 * Registers mapping is derived from V4L2_CID_OX03C10_DIGITAL_GAIN
	 * implementation.
	 */
	uint32_t hcgDigitalGainCode;
	hcgDigitalGainCode = ((registers[AecHcgCtrl0aReg] & 0xf) << 10) |
			     ((registers[AecHcgCtrl0bReg] & 0xff) << 2) |
			     ((registers[AecHcgCtrl0cReg] & 0xc0) >> 6);
	mdControls->set(md::DigitalGain, hcgDigitalGainCode);

	/*
	 * Exposure is currently configured identical for dcg, vs and spd in
	 * controlListSetExposure().
	 * V4L2_CID_OX03C10_EXPOSURE to registers mapping comes from the
	 * Linux driver implementation.
	 * Sensor uses double row as exposure unit whereas standard
	 * V4L2_CID_EXPOSURE control uses single row, hence the factor 2 here.
	 */
	uint32_t hcgExposure;
	hcgExposure = ((registers[AecHcgCtrl01Reg] & 0xff) << 8) |
		      ((registers[AecHcgCtrl02Reg] & 0xff));
	hcgExposure *= 2;
	mdControls->set(md::Exposure, hcgExposure);
}

REGISTER_CAMERA_HELPER("mx95mbcam", CameraHelperMx95mbcam)

} /* namespace nxp */

} /* namespace libcamera */
