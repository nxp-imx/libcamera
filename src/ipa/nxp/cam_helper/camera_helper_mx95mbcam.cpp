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

#define Q8_1 (0x100U)
#define Q10_1 (0x400U)
#define Q16_1 (0x10000U)

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

	void controlListSetAGC(
		ControlList *ctrls, double exposure, double gain) const override;

	virtual void controlInfoMapGetExposureRange(
		const ControlInfoMap *ctrls, std::vector<double> *minExposure,
		std::vector<double> *maxExposure, std::vector<double> *defExposure) const;

	virtual void controlInfoMapGetAnalogGainRange(
		const ControlInfoMap *ctrls, std::vector<double> *minGain,
		std::vector<double> *maxGain, std::vector<double> *defGain) const;

	void controlListSetAWB(
		ControlList *ctrls, const Span<const double, 4> gains) const override;

	int parseEmbedded(Span<const uint8_t> buffer, ControlList *mdControls) override;

	int sensorControlsToMetaData(
		const ControlList *sensorCtrls, ControlList *mdCtrls) const override;

private:
	uint32_t calcConvRatio(uint32_t ratio) const;
	uint32_t us2ns(uint32_t us) const;
	uint32_t convertExposureTime2Rows(uint32_t exposureUs) const;
	uint32_t convertRows2ExposureTime(uint32_t rows) const;
	uint32_t calcAdditionalGain(
		uint32_t exposureUs, uint32_t exposureRows) const;
	uint32_t distributeAnalogGain(
		uint32_t gain, uint32_t minGain,
		uint32_t maxGain, uint32_t *digitalGain) const;
	uint32_t distributeDigitalGain(
		uint32_t gain, uint32_t minGain, uint32_t maxGain) const;

	Span<float> analogGains(
		Span<const uint32_t> gainCodes, Span<float> gains) const;
	Span<float> digitalGains(
		Span<const uint32_t> gainCodes, Span<float> gains) const;
	Span<float> whiteBalanceGains(
		Span<const uint32_t> gainCodes, Span<float> gains) const;

	/* min/max analog real gain value */
	static constexpr double kMinAnalogGain = 1.0;
	static constexpr double kMaxAnalogGain = 15.5;
	static constexpr double kDefAnalogGain = 5.0;

	static constexpr float kMinAnalogGainLong = 1.0f;
	static constexpr float kMaxAnalogGainLong = 15.5f;
	static constexpr float kMinAnalogGainShort = 1.0f;
	static constexpr float kMaxAnalogGainShort = 15.5f;
	static constexpr float kMinAnalogGainVs = 1.0f;
	static constexpr float kMaxAnalogGainVs = 15.5f;

	static constexpr float kMinDigitalGainLong = 1.0f;
	static constexpr float kMaxDigitalGainLong = 15.999f;
	static constexpr float kMinDigitalGainShort = 1.0f;
	static constexpr float kMaxDigitalGainShort = 15.999f;
	static constexpr float kMinDigitalGainVs = 1.0f;
	static constexpr float kMaxDigitalGainVs = 15.999f;

	static constexpr uint32_t kMinAnalogGainLongQ16 = kMinAnalogGainLong * Q16_1;
	static constexpr uint32_t kMaxAnalogGainLongQ16 = kMaxAnalogGainLong * Q16_1;
	static constexpr uint32_t kMinAnalogGainShortQ16 = kMinAnalogGainShort * Q16_1;
	static constexpr uint32_t kMaxAnalogGainShortQ16 = kMaxAnalogGainShort * Q16_1;
	static constexpr uint32_t kMinAnalogGainVsQ16 = kMinAnalogGainVs * Q16_1;
	static constexpr uint32_t kMaxAnalogGainVsQ16 = kMaxAnalogGainVs * Q16_1;

	static constexpr uint32_t kMinDigitalGainLongQ16 = kMinDigitalGainLong * Q16_1;
	static constexpr uint32_t kMaxDigitalGainLongQ16 = kMaxDigitalGainLong * Q16_1;
	static constexpr uint32_t kMinDigitalGainShortQ16 = kMinDigitalGainShort * Q16_1;
	static constexpr uint32_t kMaxDigitalGainShortQ16 = kMaxDigitalGainShort * Q16_1;
	static constexpr uint32_t kMinDigitalGainVsQ16 = kMinDigitalGainVs * Q16_1;
	static constexpr uint32_t kMaxDigitalGainVsQ16 = kMaxDigitalGainVs * Q16_1;

	/* full fraction range */
	static constexpr uint32_t kAnalogGainMaskRange1 = 0xFF000U;
	/* 1 bit loss in fraction */
	static constexpr uint32_t kAnalogGainMaskRange2 = 0xFE000U;
	/* 2 bits loss in fraction */
	static constexpr uint32_t kAnalogGainMaskRange3 = 0xFC000U;
	/* 3 bits loss in fraction */
	static constexpr uint32_t kAnalogGainMaskRange4 = 0xF8000U;

	/* as maximum for register mapping */
	static constexpr uint32_t kDigitalGainMask = 0xFFFC0U;

	/* 1.0 gain is minimum for register mapping */
	static constexpr uint32_t kAnalogGainRange1Min = 0x10000U;
	static constexpr uint32_t kAnalogGainRange1Max = 0x20000U;
	static constexpr uint32_t kAnalogGainRange2Min = kAnalogGainRange1Max;
	static constexpr uint32_t kAnalogGainRange2Max = 0x40000U;
	static constexpr uint32_t kAnalogGainRange3Min = kAnalogGainRange2Max;
	static constexpr uint32_t kAnalogGainRange3Max = 0x80000U;

	static constexpr uint32_t kPrecMult = 1000U;
	static constexpr uint32_t kMult256 = 256U;

	/* 62MHz */
	static constexpr float kSclk = 62.0f;

	static constexpr uint32_t kHtsDcg = 0x5E2U;
	static constexpr uint32_t kHtsSpd = 0x2F1U;
	static constexpr uint32_t kHtsVs = 0x2F1U;
	static constexpr uint32_t kHts = kHtsDcg + kHtsVs;
	static constexpr uint32_t kVts = 0x2AEU;

	static constexpr uint32_t kMinVsExposureLines = 0U;
	static constexpr uint32_t kMaxVsExposureLines = 4U;
	static constexpr uint32_t kMinExposureLines = 4U;
	/* 23281us */
	static constexpr uint32_t kMaxExposureLines = kVts - kMaxVsExposureLines - 12U - 1U;

	static constexpr uint32_t kRowTimeNs = (kHts * 1000U) / kSclk;

	/* gain conversion ratio of HCG/LCG \todo should get from OTP sensor data */
	static constexpr uint32_t kConvGainQ16 = 7.32f * Q16_1;

#ifdef USE_OFFSET_M
	static constexpr float kOffsetM = 0.232621227534758f;
#else
	static constexpr float kOffsetM = 0.0f;
#endif
	static constexpr float kOffsetVs = 0.729738894540522f;

	static constexpr uint32_t kRatioL2SQ16 = 32U * Q16_1;
	static constexpr uint32_t kRatioL2VsQ16 = 1024U * Q16_1;

	std::unique_ptr<MdParser> parser_;
	/* Embedded data from previous frame */
	struct embeddedData {
		std::array<float, 3> analogGain;
		std::array<float, 3> digitalGain;
		std::array<float, 3> exposure;
		std::array<float, 4> whiteBalanceGain;
		float temperature;
	};
	embeddedData embeddedPreviousFrame_;
	bool embeddedInitialized_;
};

CameraHelperMx95mbcam::CameraHelperMx95mbcam()
{
	/* Adapt the default delayedControls for the ox03c10 custom controls */
	attributes_.delayedControlParams = {
		{ V4L2_CID_OX03C10_ANALOGUE_GAIN, { 3, false } },
		{ V4L2_CID_OX03C10_DIGITAL_GAIN, { 3, false } },
		{ V4L2_CID_OX03C10_EXPOSURE, { 3, false } },
		{ V4L2_CID_OX03C10_WB_GAIN, { 2, false } },
	};

	/*
	 * Setup embedded data params
	 * \todo setup actual min/max values
	 */
#if ENABLE_EMBEDDED_DATA
	attributes_.mdParams.topLines = 2;
#endif

	parser_ = std::make_unique<MdParserOmniOx>(registerList);
	embeddedPreviousFrame_ = {};
	embeddedInitialized_ = false;

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

/**
 * \brief Calculates the conversion ratio from the conversion gain accounting for the minimal gains
 *
 * \param[in] gain The conversion gain in Q16
 *
 * \return The conversion ratio in Q16 with the minimal gains taken into account
 */
uint32_t CameraHelperMx95mbcam::calcConvRatio(uint32_t gain) const
{
	return gain * kMinAnalogGainLong * kMinDigitalGainLong;
}

uint32_t CameraHelperMx95mbcam::us2ns(uint32_t us) const
{
	return us * 1000U;
}

uint32_t CameraHelperMx95mbcam::convertExposureTime2Rows(uint32_t exposureUs) const
{
	return (us2ns(exposureUs) + kRowTimeNs / 2U) / kRowTimeNs;
}

uint32_t CameraHelperMx95mbcam::convertRows2ExposureTime(uint32_t rows) const
{
	return (rows * kRowTimeNs) / kPrecMult;
}

/**
 * \brief Recovers lost exposure time as gain
 *
 * \details Attempts to recover the lost, due to conversion to rows, exposure time as gain
 *
 * \param[in] exposureUs the exposure time before converting it to rows
 * \param[in] exposureRows the resulting rows after converting exposureUs to rows
 *
 * \return The lost exposure time as gain in Q8
 */
uint32_t CameraHelperMx95mbcam::calcAdditionalGain(
	uint32_t exposureUs, uint32_t exposureRows) const
{
	const uint64_t exposureNs = us2ns(exposureUs);
	const uint64_t exposureRowsNs = exposureRows * kRowTimeNs;

	return (exposureNs * Q8_1 + exposureRowsNs / 2) / exposureRowsNs;
}

uint32_t CameraHelperMx95mbcam::distributeAnalogGain(
	uint32_t gain, uint32_t minGain,
	uint32_t maxGain, uint32_t *digitalGain) const
{
	uint64_t tmpGain;
	uint64_t currentDigitalGain = *digitalGain;
	uint32_t resGain = gain;

	if (maxGain < resGain) {
		tmpGain = resGain;
		resGain = maxGain;
		/* Carry overflow gain into digitalGain */
		currentDigitalGain = (tmpGain * currentDigitalGain) / resGain;
	} else {
		/* Should not enter here, fractional gain is bad, but just for completeness sake */
		if (minGain > resGain)
			resGain = minGain;
	}

	tmpGain = resGain;

	/* Select appropriate mask for Analog gain */
	if (resGain <= kAnalogGainRange1Max)
		resGain = resGain & kAnalogGainMaskRange1;
	else if (resGain <= kAnalogGainRange2Max)
		resGain = resGain & kAnalogGainMaskRange2;
	else if (resGain <= kAnalogGainRange3Max)
		resGain = resGain & kAnalogGainMaskRange3;
	else
		resGain = resGain & kAnalogGainMaskRange4;

	/* Attempt to carry masked gain into digital */
	*digitalGain = (uint32_t)((currentDigitalGain * tmpGain + resGain / 2U) / resGain);

	return resGain;
}

uint32_t CameraHelperMx95mbcam::distributeDigitalGain(
	uint32_t gain, uint32_t minGain, uint32_t maxGain) const
{
	/* Clip to valid range */
	if (maxGain < gain)
		gain = maxGain;

	if (minGain > gain)
		gain = minGain;

	/* Mask gain to valid settings */
	gain = gain & kDigitalGainMask;

	return gain;
}

void CameraHelperMx95mbcam::controlListSetAGC(
	ControlList *ctrls, double exposure, double gain) const
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

	const uint32_t sensorConversionRatio = calcConvRatio(kConvGainQ16);

	uint64_t lAgainL, lAgainS, lAgainVS;
	uint32_t lDgainL, lDgainS, lDgainVS;
	uint64_t lAddGain;
	uint32_t lExpIn = static_cast<uint32_t>(exposure * 1.0e6); // to usec
	uint32_t lExpLinRows;
	uint32_t lExpVSinRows;
	uint64_t lExpTotalInRows;
	uint64_t lExpTotalL;
	uint64_t lRatioL2S, lRatioL2VS;
	uint64_t lMinGainL;
	float lTmpF;

	/* Minimum total gain for the Short frame */
	uint64_t lMinGainS =
		(static_cast<uint64_t>(kMinAnalogGainShortQ16) * kMinDigitalGainShortQ16) / Q16_1;
	/* Minimum total gain for the VS frame */
	uint64_t lMinGainVS =
		(static_cast<uint64_t>(kMinAnalogGainVsQ16) * kMinDigitalGainVsQ16) / Q16_1;

	/* make exposure a multiple of row time as driver needed it */
	lExpLinRows = convertExposureTime2Rows(lExpIn); /* in rows */
	/* distribution should provide valid values, here an extra check */
	if (lExpLinRows < kMinExposureLines) {
		lExpLinRows = kMinExposureLines;
		LOG(NxpCameraHelper, Warning)
			<< "Long exposure distribution < kMinExposureLines. Value adjusted.";
	} else {
		if (lExpLinRows > kMaxExposureLines) {
			lExpLinRows = kMaxExposureLines;
			LOG(NxpCameraHelper, Warning)
				<< "Long exposure distribution > kMaxExposureLines. Value adjusted.";
		}
	}

	lRatioL2S = kRatioL2SQ16;
	lRatioL2VS = kRatioL2VsQ16;

	if (lRatioL2S < sensorConversionRatio) {
		/* minGain for L, recalculated by conv_ratio */
		lMinGainL = lMinGainS * sensorConversionRatio / Q16_1;
	} else {
		/* minGain for S, recalculated by ratio */
		lMinGainL = lMinGainS * lRatioL2S / Q16_1;
	}

	/* L gain set to AEC total gain decision */
	lAgainL = static_cast<uint64_t>(gain * Q16_1); // to UQ.16

	/* check lAgainL for minimum converted level, and re-calc for MIN */
	if (lAgainL < lMinGainL) {
		lExpTotalL = (uint64_t)lExpIn * kPrecMult * lAgainL; /* in e[us]*1000*[g*65536] */
		lAgainL = lMinGainL;
		lExpIn = (uint32_t)(lExpTotalL / (kPrecMult * lAgainL)); /* LFM restriction may not strict */
		/* make exposure a multiple of row time as driver needed it */
		lExpLinRows = convertExposureTime2Rows(lExpIn); /* in rows */
		/* distribution should provide valid values, here an extra check */
		if (lExpLinRows < kMinExposureLines)
			lExpLinRows = kMinExposureLines;
	}

	lAddGain = calcAdditionalGain(lExpIn, lExpLinRows);
	lAgainL = (lAgainL * lAddGain + kMult256 / 2U) / kMult256; /* correct gain */

	/* as result of lExpLinRows limitation causing fractional lAddGain */
	if (lAgainL < lMinGainL)
		lAgainL = lMinGainL;

	/* get lAgainS in ratio to lAgainL */
	lAgainS = lAgainL * Q16_1 / lRatioL2S;
	lExpVSinRows = kMaxVsExposureLines; /* default VS exposure to max */

	/* calculations of acAgain with restrictions for VS max exposure and min VS gain */
	lTmpF = kOffsetM * (float)Q10_1;
	lExpTotalInRows = lAgainL * ((uint64_t)lExpLinRows * Q10_1 + (uint64_t)lTmpF); /* total exposure (with rows...) */
	lTmpF = kOffsetVs * (float)Q10_1;
	lAgainVS = lExpTotalInRows * Q16_1 / (lRatioL2VS * ((uint64_t)lExpVSinRows * Q10_1 + (uint64_t)lTmpF));
	/* but if min gain restriction fails */
	if (lAgainVS < lMinGainVS) {
		lAgainVS = lMinGainVS; /* set mandatory min gain and decrease exposure */
		lExpVSinRows = (uint32_t)(lExpTotalInRows * Q16_1 / (lRatioL2VS * lAgainVS * Q10_1));
		/**
		 * \note <= is used instead of < because if kMinVsExposureLines is 0
		 * then the expression would be always false and the compiler
		 * might issue a warning than will be treated as error due to compiler flags
		 */
		if (lExpVSinRows <= kMinVsExposureLines) {
			lExpVSinRows = kMinVsExposureLines;
		}
		lTmpF = kOffsetVs * (float)Q10_1;
		lAgainVS = lExpTotalInRows * Q16_1 / (lRatioL2VS * ((uint64_t)lExpVSinRows * Q10_1 + (uint32_t)lTmpF));
		if (lAgainVS < lMinGainVS) {
			lExpVSinRows = (uint32_t)(lExpVSinRows * lAgainVS / Q16_1);
			if (lExpVSinRows <= kMinVsExposureLines) {
				lExpVSinRows = kMinVsExposureLines;
			}
			lAgainVS = lExpTotalInRows * Q16_1 / (lRatioL2VS * ((uint64_t)lExpVSinRows * Q10_1 + (uint32_t)lTmpF));
		}
	}

	ox03c10_exposure v4l2Exposures;
	v4l2Exposures.dcg = lExpLinRows; // DCG_exp_row
	v4l2Exposures.spd = 0x04; /* keep the default */
	v4l2Exposures.vs = lExpVSinRows; // VS_exp_row

	Span<uint8_t> expData(reinterpret_cast<uint8_t *>(&v4l2Exposures), sizeof(v4l2Exposures));
	ctrls->set(V4L2_CID_OX03C10_EXPOSURE, expData);

	lDgainL = kMinDigitalGainLongQ16;
	lDgainS = kMinDigitalGainShortQ16;
	lDgainVS = kMinDigitalGainVsQ16;

	lAgainL = (lAgainL * Q16_1) / (((uint64_t)lDgainL * sensorConversionRatio) / Q16_1);
	lAgainS = (lAgainS * Q16_1) / lDgainS;
	lAgainVS = (lAgainVS * Q16_1) / lDgainVS;

	ox03c10_analog_gain v4l2AnalogGains;
	uint32_t lX3cMinGain = kMinAnalogGainLongQ16;
	uint32_t lX3cMaxGain = kMaxAnalogGainLongQ16;
	/* all gains are calculated as absolute gains. To get reg values divide by conversion gain and sensitivity */
	/* lDgainL corrected to get fraction not covered by againL */
	const uint32_t hcgAnalogGain = distributeAnalogGain(
		(uint32_t)(lAgainL),
		lX3cMinGain,
		lX3cMaxGain,
		&lDgainL);
	/* 1.0 = 65536 to 1.0 = 16 conversion */
	v4l2AnalogGains.hcg = hcgAnalogGain >> 12;

	lX3cMinGain = kMinAnalogGainShortQ16;
	lX3cMaxGain = kMaxAnalogGainShortQ16;
	/* lDgainS corrected to get fraction not covered by lAgainS */
	const uint32_t lcgAnalogGain = distributeAnalogGain(
		(uint32_t)(lAgainS),
		lX3cMinGain,
		lX3cMaxGain,
		&lDgainS);
	/* 1.0 = 65536 to 1.0 = 16 conversion */
	v4l2AnalogGains.lcg = lcgAnalogGain >> 12;

	v4l2AnalogGains.spd = 0x10; /* keep the default */

	lX3cMinGain = kMinAnalogGainVsQ16;
	lX3cMaxGain = kMaxAnalogGainVsQ16;
	/* lDgainVS corrected to get fraction not covered by lAgainVS */
	const uint32_t vsAnalogGain = distributeAnalogGain(
		(uint32_t)(lAgainVS),
		lX3cMinGain,
		lX3cMaxGain,
		&lDgainVS);
	/* 1.0 = 65536 to 1.0 = 16 conversion */
	v4l2AnalogGains.vs = vsAnalogGain >> 12;

	Span<uint8_t> analogGainsData(reinterpret_cast<uint8_t *>(&v4l2AnalogGains),
				      sizeof(v4l2AnalogGains));
	ctrls->set(V4L2_CID_OX03C10_ANALOGUE_GAIN, analogGainsData);

	ox03c10_digital_gain v4l2DigitalGains;
	lX3cMinGain = kMinDigitalGainLongQ16;
	lX3cMaxGain = kMaxDigitalGainLongQ16;
	const uint32_t hcgDigitalGain =
		distributeDigitalGain(lDgainL, lX3cMinGain, lX3cMaxGain);
	/* 1.0 = 65536 to 1.0 = 1024 conversion */
	v4l2DigitalGains.hcg = hcgDigitalGain >> 6;

	lX3cMinGain = kMinDigitalGainShortQ16;
	lX3cMaxGain = kMaxDigitalGainShortQ16;
	const uint32_t lcgDigitalGain =
		distributeDigitalGain(lDgainS, lX3cMinGain, lX3cMaxGain);
	/* 1.0 = 65536 to 1.0 = 1024 conversion */
	v4l2DigitalGains.lcg = lcgDigitalGain >> 6;

	/* Keep the default */
	v4l2DigitalGains.spd = 0x400;

	lX3cMinGain = kMinDigitalGainVsQ16;
	lX3cMaxGain = kMaxDigitalGainVsQ16;
	const uint32_t vsDigitalGain =
		distributeDigitalGain(lDgainVS, lX3cMinGain, lX3cMaxGain);
	/* 1.0 = 65536 to 1.0 = 1024 conversion */
	v4l2DigitalGains.vs = vsDigitalGain >> 6;

	Span<uint8_t> digitalGainsData(
		reinterpret_cast<uint8_t *>(&v4l2DigitalGains),
		sizeof(v4l2DigitalGains));
	ctrls->set(V4L2_CID_OX03C10_DIGITAL_GAIN, digitalGainsData);
}

void CameraHelperMx95mbcam::controlInfoMapGetExposureRange(
	const ControlInfoMap *ctrls, std::vector<double> *minExposure,
	std::vector<double> *maxExposure, std::vector<double> *defExposure) const
{
	(void)ctrls;

	/* \todo Append short and very short exposure values */
	double line = kRowTimeNs * 1.0e-9;
	minExposure->clear();
	minExposure->push_back(kMinExposureLines * line);

	maxExposure->clear();
	maxExposure->push_back(kMaxExposureLines * line);

	defExposure->clear();
	defExposure->push_back((kMinExposureLines + kMaxExposureLines) / 2 * line);
}

void CameraHelperMx95mbcam::controlInfoMapGetAnalogGainRange(
	const ControlInfoMap *ctrls, std::vector<double> *minGain,
	std::vector<double> *maxGain, std::vector<double> *defGain) const
{
	(void)ctrls;

	/* Fixup HCG gain with conversion ratio */
	const uint32_t sensorConversionRatioQ16 = calcConvRatio(kConvGainQ16);
	double convGain = static_cast<double>(sensorConversionRatioQ16) / Q16_1;

	/* \todo Append short and very short analog gain values */
	minGain->clear();
	minGain->push_back(kMinAnalogGain * convGain);

	maxGain->clear();
	maxGain->push_back(kMaxAnalogGain * convGain);

	defGain->clear();
	defGain->push_back(kDefAnalogGain * convGain);
}

int CameraHelperMx95mbcam::parseEmbedded(Span<const uint8_t> buffer,
					  ControlList *mdControls)
{
	if (!buffer.size())
		return -1;

	MdParser::Status status;
	MdParser::RegisterMap registers;
	status = parser_->parse(buffer, registers);
	if (status != MdParser::Status::OK)
		return -1;

	/* Analog gain */
	uint32_t hcgAnalogGainCode =
		((registers[AecHcgCtrl08Reg] & 0x0fU) << 4U) |
		((registers[AecHcgCtrl09Reg] & 0xf0U) >> 4U);
	uint32_t lcgAnalogGainCode =
		((registers[AecLcgCtrl08Reg] & 0x0fU) << 4U) |
		((registers[AecLcgCtrl09Reg] & 0xf0U) >> 4U);
	uint32_t vsAnalogGainCode =
		((registers[AecVsCtrl08Reg] & 0x0fU) << 4U) |
		((registers[AecVsCtrl09Reg] & 0xf0U) >> 4U);

	std::array<uint32_t, 3> aGainCodes = { hcgAnalogGainCode,
					       lcgAnalogGainCode,
					       vsAnalogGainCode };
	std::array<float, 3> aGainsArray;
	analogGains(Span<uint32_t>(aGainCodes), Span<float>(aGainsArray));

	/* Fixup HCG gain with conversion ratio */
	const uint32_t sensorConversionRatioQ16 = calcConvRatio(kConvGainQ16);
	aGainsArray[0] *= (static_cast<float>(sensorConversionRatioQ16) / Q16_1);

	/* workaround: add one frame delay in reported metadata */
	Span<float> aGains;
	if (embeddedInitialized_)
		aGains = Span<float>(embeddedPreviousFrame_.analogGain);
	else
		aGains = Span<float>(aGainsArray);
	mdControls->set(md::AnalogueGain, aGains);
	/* store current embedded data for next frame */
	ASSERT(aGainsArray.size() == embeddedPreviousFrame_.analogGain.size());
	std::copy(aGainsArray.begin(), aGainsArray.end(),
		  embeddedPreviousFrame_.analogGain.begin());

	/* Digital gain */
	uint32_t hcgDigitalGainCode =
		((registers[AecHcgCtrl0aReg] & 0x0fU) << 10U) |
		(registers[AecHcgCtrl0bReg] << 2U) |
		((registers[AecHcgCtrl0cReg] & 0xc0U) >> 6U);
	uint32_t lcgDigitalGainCode =
		((registers[AecLcgCtrl0aReg] & 0x0fU) << 10U) |
		(registers[AecLcgCtrl0bReg] << 2U) |
		((registers[AecLcgCtrl0cReg] & 0xc0U) >> 6U);
	uint32_t vsDigitalGainCode =
		((registers[AecVsCtrl0aReg] & 0x0fU) << 10U) |
		(registers[AecVsCtrl0bReg] << 2U) |
		((registers[AecVsCtrl0cReg] & 0xc0U) >> 6U);

	std::array<uint32_t, 3> dGainCodes = { hcgDigitalGainCode,
					       lcgDigitalGainCode,
					       vsDigitalGainCode };
	std::array<float, 3> dGainsArray;
	digitalGains(Span<uint32_t>(dGainCodes), Span<float>(dGainsArray));

	/* workaround: add one frame delay in reported metadata */
	Span<float> dGains;
	if (embeddedInitialized_)
		dGains = Span<float>(embeddedPreviousFrame_.digitalGain);
	else
		dGains = Span<float>(dGainsArray);
	mdControls->set(md::DigitalGain, dGains);
	/* store current embedded data for next frame */
	ASSERT(dGainsArray.size() == embeddedPreviousFrame_.digitalGain.size());
	std::copy(dGainsArray.begin(), dGainsArray.end(),
		  embeddedPreviousFrame_.digitalGain.begin());

	uint32_t dcgExposure =
		(registers[AecHcgCtrl01Reg] << 8U) | registers[AecHcgCtrl02Reg];
	uint32_t vsExposure =
		(registers[AecVsCtrl01Reg] << 8U) | registers[AecVsCtrl02Reg];

	float dcgExposureS =
		dcgExposure * kRowTimeNs / 1.0e9f;
	/* Fixup VS exposure with OffsetVS (double) raw */
	float vsExposureS =
		(1.0 + kOffsetVs) * vsExposure * kRowTimeNs / 1.0e9f;

	std::array<float, 3> exposuresArray = { dcgExposureS,
						dcgExposureS,
						vsExposureS };

	/* workaround: add one frame delay in reported metadata */
	Span<float> exposures;
	if (embeddedInitialized_)
		exposures = Span<float>(embeddedPreviousFrame_.exposure);
	else
		exposures = Span<float>(exposuresArray);
	mdControls->set(md::Exposure, exposures);
	/* store current embedded data for next frame */
	ASSERT(exposuresArray.size() == embeddedPreviousFrame_.exposure.size());
	std::copy(exposuresArray.begin(), exposuresArray.end(),
		  embeddedPreviousFrame_.exposure.begin());

	/* White balance */
	uint32_t blueGain =
		((registers[AwbGainHcg0Reg] & 0x7fU) << 8U) |
		registers[AwbGainHcg1Reg];
	uint32_t greenBGain =
		((registers[AwbGainHcg2Reg] & 0x7fU) << 8U) |
		registers[AwbGainHcg3Reg];
	uint32_t greenRGain =
		((registers[AwbGainHcg4Reg] & 0x7fU) << 8U) |
		registers[AwbGainHcg5Reg];
	uint32_t redGain =
		((registers[AwbGainHcg6Reg] & 0x7fU) << 8U) |
		registers[AwbGainHcg7Reg];

	std::array<uint32_t, 4> wbGainCodes = { redGain, greenRGain, greenBGain, blueGain };
	std::array<float, 4> wbGainsArray;

	whiteBalanceGains(Span<uint32_t>(wbGainCodes), Span<float>(wbGainsArray));

	/* workaround: add one frame delay in reported metadata */
	Span<float> wbGains;
	if (embeddedInitialized_)
		wbGains = Span<float>(embeddedPreviousFrame_.whiteBalanceGain);
	else
		wbGains = Span<float>(wbGainsArray);
	mdControls->set(md::WhiteBalanceGain, wbGains);
	/* store current embedded data for next frame */
	ASSERT(wbGainsArray.size() == embeddedPreviousFrame_.whiteBalanceGain.size());
	std::copy(wbGainsArray.begin(), wbGainsArray.end(),
		  embeddedPreviousFrame_.whiteBalanceGain.begin());

	/* Sensor temperature is UQ8.8 and negative above 0xc000 */
	static constexpr uint32_t kTemperatureMax = 0xc000U;
	const uint32_t uTemperature =
		(registers[TmpReg26Reg] << 8U) | registers[TmpReg27Reg];
	float temperature;
	if (uTemperature <= kTemperatureMax)
		temperature = uTemperature * 1.0f / Q8_1;
	else
		temperature = -((uTemperature - kTemperatureMax) * 1.0f / Q8_1);

	/* workaround: add one frame delay in reported metadata */
	float temp;
	if (embeddedInitialized_)
		temp = embeddedPreviousFrame_.temperature;
	else
		temp = temperature;
	mdControls->set(md::Temperature, temp);
	/* store current embedded data for next frame */
	embeddedPreviousFrame_.temperature = temperature;

	embeddedInitialized_ = true;

	return 0;
}

void CameraHelperMx95mbcam::controlListSetAWB(
	ControlList *ctrls, Span<const double, 4> gains) const
{
	if (!controlListHasId(ctrls, V4L2_CID_OX03C10_WB_GAIN)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_OX03C10_WB_GAIN cannot be set";
		return;
	}

	/* Sensor white balance gain format is UQ5.10 */
	std::array<uint16_t, 4> uGains;
	for (size_t i = 0; i < gains.size(); i++)
		uGains[i] = static_cast<uint16_t>(gains[i] * Q10_1);

	ox03c10_wb_capture_gain wbGains[4];
	for (size_t i = 0; i < 4; i++) {
		wbGains[i].r = uGains[0];
		wbGains[i].gr = uGains[1];
		wbGains[i].gb = uGains[2];
		wbGains[i].b = uGains[3];
	}

	/* SPD is not used - set it to 1.0 */
	wbGains[2].r = Q10_1;
	wbGains[2].gr = Q10_1;
	wbGains[2].gb = Q10_1;
	wbGains[2].b = Q10_1;

	Span<uint8_t> data(reinterpret_cast<uint8_t *>(wbGains), sizeof(wbGains));
	ctrls->set(V4L2_CID_OX03C10_WB_GAIN, data);
}

int CameraHelperMx95mbcam::sensorControlsToMetaData(const ControlList *sensorCtrls,
						    ControlList *mdCtrls) const
{
	int ret = 0;

	ASSERT(controlListHasId(mdCtrls, md::AnalogueGain.id()));
	const ControlValue &aGainCtrl = sensorCtrls->get(V4L2_CID_OX03C10_ANALOGUE_GAIN);
	std::array<float, 3> aGainsArray = { 1.0f, 1.0f, 1.0f };
	if (!aGainCtrl.isNone()) {
		Span<const uint8_t> data = aGainCtrl.data();
		ASSERT(data.size() == sizeof(ox03c10_analog_gain));
		/* Sensor control is UQ4.4, metadata is UQ16.16 */
		const struct ox03c10_analog_gain *aGainSensor =
			reinterpret_cast<const struct ox03c10_analog_gain *>(data.data());

		std::array<uint32_t, 3> aGainCodes = { aGainSensor->hcg,
						       aGainSensor->lcg,
						       aGainSensor->vs };
		analogGains(Span<uint32_t>(aGainCodes), Span<float>(aGainsArray));

		/* Fixup HCG gain with conversion ratio */
		const uint32_t sensorConversionRatioQ16 = calcConvRatio(kConvGainQ16);
		aGainsArray[0] *= (static_cast<float>(sensorConversionRatioQ16) / Q16_1);
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid analog gain control";
		ret = -1;
	}
	mdCtrls->set(md::AnalogueGain, Span<float>(aGainsArray));

	ASSERT(controlListHasId(mdCtrls, md::DigitalGain.id()));
	const ControlValue &dGainCtrl = sensorCtrls->get(V4L2_CID_OX03C10_DIGITAL_GAIN);
	std::array<float, 3> dGainsArray = { 1.0f, 1.0f, 1.0f };
	if (!dGainCtrl.isNone()) {
		Span<const uint8_t> data = dGainCtrl.data();
		ASSERT(data.size() == sizeof(ox03c10_digital_gain));
		/* Sensor control is UQ4.10, metadata is UQ16.16 */
		const struct ox03c10_digital_gain *dGainSensor =
			reinterpret_cast<const struct ox03c10_digital_gain *>(data.data());

		std::array<uint32_t, 3> dGainCodes = { dGainSensor->hcg,
						       dGainSensor->lcg,
						       dGainSensor->vs };
		digitalGains(Span<uint32_t>(dGainCodes), Span<float>(dGainsArray));
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid digital gain control";
		ret = -1;
	}
	mdCtrls->set(md::DigitalGain, Span<float>(dGainsArray));

	ASSERT(controlListHasId(mdCtrls, md::Exposure.id()));
	const ControlValue &exposureCtrl = sensorCtrls->get(V4L2_CID_OX03C10_EXPOSURE);
	std::array<float, 3> exposureArray = { 0.0f, 0.0f, 0.0f };
	if (!exposureCtrl.isNone()) {
		Span<const uint8_t> data = exposureCtrl.data();
		ASSERT(data.size() == sizeof(ox03c10_exposure));
		const struct ox03c10_exposure *exposure =
			reinterpret_cast<const struct ox03c10_exposure *>(data.data());

		float dcgExposureS = exposure->dcg * kRowTimeNs / 1.0e9f;
		/* Fixup VS exposure with OffsetVS (double) raw */
		float vsExposureS = (1.0 + kOffsetVs) * exposure->vs * kRowTimeNs / 1.0e9f;

		exposureArray[0] = dcgExposureS;
		exposureArray[1] = dcgExposureS;
		exposureArray[2] = vsExposureS;
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid exposure control";
		ret = -1;
	}
	mdCtrls->set(md::Exposure, Span<float>(exposureArray));

	ASSERT(controlListHasId(mdCtrls, md::WhiteBalanceGain.id()));
	const ControlValue &wbCtrl = sensorCtrls->get(V4L2_CID_OX03C10_WB_GAIN);
	std::array<float, 4> wbGainsArray = { 1.0f, 1.0f, 1.0f, 1.0f };
	if (!wbCtrl.isNone()) {
		Span<const uint8_t> data = wbCtrl.data();
		ASSERT(data.size() == sizeof(ox03c10_wb_capture_gain[4]));
		/* Sensor control is UQ5.10, metadata is UQ16.16 */
		const struct ox03c10_wb_capture_gain *wb =
			reinterpret_cast<const struct ox03c10_wb_capture_gain *>(data.data());

		std::array<uint32_t, 4> wbGainCodes = { wb->r, wb->gr, wb->gb, wb->b };
		whiteBalanceGains(Span<uint32_t>(wbGainCodes), Span<float>(wbGainsArray));
	} else {
		/* Do nothing - optional control */
	}
	mdCtrls->set(md::WhiteBalanceGain, Span<float>(wbGainsArray));

	/* No temperature information, report arbitrary value */
	ASSERT(controlListHasId(mdCtrls, md::TEMPERATURE));
	mdCtrls->set(md::Temperature, 25.0);

	return ret;
}

Span<float> CameraHelperMx95mbcam::analogGains(
	Span<const uint32_t> gainCodes, Span<float> gains) const
{
	/* Analog gain for hcg, lcg, vs is UQ4.4 */
	ASSERT(gainCodes.size() == gains.size());
	for (size_t i = 0; i < gainCodes.size(); i++)
		gains[i] = gainCodes[i] * 1.0f / (1 << 4);
	return gains;
}

Span<float> CameraHelperMx95mbcam::digitalGains(
	Span<const uint32_t> gainCodes, Span<float> gains) const
{
	/* Digital gain for hcg, lcg, vs is UQ4.10 */
	ASSERT(gainCodes.size() == gains.size());
	for (size_t i = 0; i < gainCodes.size(); i++)
		gains[i] = gainCodes[i] * 1.0f / (1 << 10);
	return gains;
}

Span<float> CameraHelperMx95mbcam::whiteBalanceGains(
	Span<const uint32_t> gainCodes, Span<float> gains) const
{
	/* White Balance gain for hcg, lcg, vs is UQ5.10 */
	ASSERT(gainCodes.size() == gains.size());
	for (size_t i = 0; i < gainCodes.size(); i++)
		gains[i] = gainCodes[i] * 1.0f / (1 << 10);
	return gains;
}

REGISTER_CAMERA_HELPER("mx95mbcam", CameraHelperMx95mbcam)

/*
 * This definition is to support the other variant of the MX95MBCAM module,
 * using TI DS90UB953/DS90UB960 serializer/deserializer instead of the Maxim
 * ones. A separate Linux kernel driver is used for this variant that reports a
 * different sensor model name. Because the same underlying sensor driver is
 * actually used, the Maxim CameraHelper is subclassed to avoid duplication.
 */
class CameraHelperOx03c10Drv : public CameraHelperMx95mbcam
{
};
REGISTER_CAMERA_HELPER("ox03c10_drv", CameraHelperOx03c10Drv)

} /* namespace nxp */

} /* namespace libcamera */
