/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * camera_helper_mx95mbcam.c
 * Helper class that performs sensor-specific parameter computations
 * for MX95MBCAM module (OX03C10 camera and a Maxim MAX96717 GMSL2 serializer)
 * Copyright 2024-2025 NXP
 */

#include <cmath>

#include <linux/ox03c10.h>

#include <libcamera/base/log.h>

#include "camera_helper.h"
#include "md_parser_ox.h"

#define ENABLE_EMBEDDED_DATA_TOP_LINES 1
#define USE_CUSTOM_CONTROLS 1

#define Q8_1 (0x100U)
#define Q10_1 (0x400U)
#define Q16_1 (0x10000U)

using namespace std::literals::chrono_literals;

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpCameraHelper)

namespace nxp {

/*
 * Registers included in embedded data
 * List comes from Linux driver init, defined by access control configuration
 * for group 4 hold (register address 0x3208).
 */
constexpr uint32_t AecHcgCtrl0eReg = 0x350e;
constexpr uint32_t AecHcgCtrl0fReg = 0x350f;
constexpr uint32_t AecHcgCtrl14Reg = 0x3514;
constexpr uint32_t AecHcgCtrl15Reg = 0x3515;
constexpr uint32_t AecHcgCtrl18Reg = 0x3518;
constexpr uint32_t AecHcgCtrl19Reg = 0x3519;
constexpr uint32_t AecHcgCtrl1aReg = 0x351a;
constexpr uint32_t AecSpdCtrl0eReg = 0x354e;
constexpr uint32_t AecSpdCtrl0fReg = 0x354f;
constexpr uint32_t AecSpdCtrl14Reg = 0x3554;
constexpr uint32_t AecSpdCtrl15Reg = 0x3555;
constexpr uint32_t AecSpdCtrl18Reg = 0x3558;
constexpr uint32_t AecSpdCtrl19Reg = 0x3559;
constexpr uint32_t AecSpdCtrl1aReg = 0x355a;
constexpr uint32_t AecLcgCtrl14Reg = 0x3594;
constexpr uint32_t AecLcgCtrl15Reg = 0x3595;
constexpr uint32_t AecLcgCtrl18Reg = 0x3598;
constexpr uint32_t AecLcgCtrl19Reg = 0x3599;
constexpr uint32_t AecLcgCtrl1aReg = 0x359a;
constexpr uint32_t AecVsCtrl0eReg = 0x35ce;
constexpr uint32_t AecVsCtrl0fReg = 0x35cf;
constexpr uint32_t AecVsCtrl14Reg = 0x35d4;
constexpr uint32_t AecVsCtrl15Reg = 0x35d5;
constexpr uint32_t AecVsCtrl18Reg = 0x35d8;
constexpr uint32_t AecVsCtrl19Reg = 0x35d9;
constexpr uint32_t AecVsCtrl1aReg = 0x35da;
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
constexpr uint32_t AwbGainVs8Reg = 0x5888;
constexpr uint32_t AwbGainVs9Reg = 0x5889;

constexpr std::initializer_list<uint32_t> registerList [[maybe_unused]] = {
	AecHcgCtrl0eReg,
	AecHcgCtrl0fReg,
	AecHcgCtrl14Reg,
	AecHcgCtrl15Reg,
	AecHcgCtrl18Reg,
	AecHcgCtrl19Reg,
	AecHcgCtrl1aReg,
	AecSpdCtrl0eReg,
	AecSpdCtrl0fReg,
	AecSpdCtrl14Reg,
	AecSpdCtrl15Reg,
	AecSpdCtrl18Reg,
	AecSpdCtrl19Reg,
	AecSpdCtrl1aReg,
	AecLcgCtrl14Reg,
	AecLcgCtrl15Reg,
	AecLcgCtrl18Reg,
	AecLcgCtrl19Reg,
	AecLcgCtrl1aReg,
	AecVsCtrl0eReg,
	AecVsCtrl0fReg,
	AecVsCtrl14Reg,
	AecVsCtrl15Reg,
	AecVsCtrl18Reg,
	AecVsCtrl19Reg,
	AecVsCtrl1aReg,
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
	AwbGainVs8Reg,
	AwbGainVs9Reg,
};

class CameraHelperMx95mbcam : public CameraHelper
{
public:
	CameraHelperMx95mbcam();

	void setCameraMode(const CameraMode &mode);
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

#if USE_CUSTOM_CONTROLS
	void setControls(const ControlList *sensorCtrls) override;
	void controlListSetAGC(
		ControlList *ctrls, Duration exposure, double gain) const override;
#endif

	virtual void controlInfoMapGetExposureRange(
		const ControlInfoMap *ctrls, std::vector<Duration> *minExposure,
		std::vector<Duration> *maxExposure, std::vector<Duration> *defExposure) const override;

	virtual void controlInfoMapGetAnalogGainRange(
		const ControlInfoMap *ctrls, std::vector<double> *minGain,
		std::vector<double> *maxGain, std::vector<double> *defGain) const override;

#if USE_CUSTOM_CONTROLS
	void controlListSetAWB(
		ControlList *ctrls, const Span<const double, 4> gains) const override;
#endif

	int parseEmbedded(Span<const uint8_t> buffer, ControlList *mdControls) override;

#if USE_CUSTOM_CONTROLS
	int sensorControlsToMetaData(
		const ControlList *sensorCtrls, ControlList *mdCtrls) const override;
#endif

private:
	uint32_t calcConvRatio(uint32_t ratio) const;
	uint32_t us2ns(uint32_t us) const;
	uint32_t convertExposureTime2DoubleRows(uint32_t exposureUs) const;
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
	uint32_t maxExposureDoubleLines() const;

	/* min/max analog real gain value */
	static constexpr double kMinAnalogGain = 1.0;
	static constexpr double kMaxAnalogGain = 15.0;
	static constexpr double kDefAnalogGain = 5.0;

	static constexpr float kMinAnalogGainLong = 1.0f;
	static constexpr float kMaxAnalogGainLong = 15.0f;
	static constexpr float kMinAnalogGainShort = 1.0f;
	static constexpr float kMaxAnalogGainShort = 15.0f;
	static constexpr float kMinAnalogGainSpd = 1.0f;
	static constexpr float kMaxAnalogGainSpd = 15.0f;
	static constexpr float kMinAnalogGainVs = 1.0f;
	static constexpr float kMaxAnalogGainVs = 15.0f;

	static constexpr float kMinDigitalGainLong = 1.0f;
	static constexpr float kMaxDigitalGainLong = 15.0f;
	static constexpr float kMinDigitalGainShort = 1.0f;
	static constexpr float kMaxDigitalGainShort = 15.0f;
	static constexpr float kMinDigitalGainSpd = 1.0f;
	static constexpr float kMaxDigitalGainSpd = 15.0f;
	static constexpr float kMinDigitalGainVs = 1.0f;
	static constexpr float kMaxDigitalGainVs = 15.0f;

	static constexpr uint32_t kMinAnalogGainLongQ16 = kMinAnalogGainLong * Q16_1;
	static constexpr uint32_t kMaxAnalogGainLongQ16 = kMaxAnalogGainLong * Q16_1;
	static constexpr uint32_t kMinAnalogGainShortQ16 = kMinAnalogGainShort * Q16_1;
	static constexpr uint32_t kMaxAnalogGainShortQ16 = kMaxAnalogGainShort * Q16_1;
	static constexpr uint32_t kMinAnalogGainSpdQ16 = kMinAnalogGainSpd * Q16_1;
	static constexpr uint32_t kMaxAnalogGainSpdQ16 = kMaxAnalogGainSpd * Q16_1;
	static constexpr uint32_t kMinAnalogGainVsQ16 = kMinAnalogGainVs * Q16_1;
	static constexpr uint32_t kMaxAnalogGainVsQ16 = kMaxAnalogGainVs * Q16_1;

	static constexpr uint32_t kMinDigitalGainLongQ16 = kMinDigitalGainLong * Q16_1;
	static constexpr uint32_t kMaxDigitalGainLongQ16 = kMaxDigitalGainLong * Q16_1;
	static constexpr uint32_t kMinDigitalGainShortQ16 = kMinDigitalGainShort * Q16_1;
	static constexpr uint32_t kMaxDigitalGainShortQ16 = kMaxDigitalGainShort * Q16_1;
	static constexpr uint32_t kMinDigitalGainSpdQ16 = kMinDigitalGainSpd * Q16_1;
	static constexpr uint32_t kMaxDigitalGainSpdQ16 = kMaxDigitalGainSpd * Q16_1;
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

	/* min/max exposure in double lines */
	static constexpr uint32_t kMinVsExposureDoubleLines = 0U;
	static constexpr uint32_t kMaxVsExposureDoubleLines = 31U;
	static constexpr uint32_t kMinExposureDoubleLines = 2U;
	uint32_t maxExposureDoubleLines_;

#ifdef USE_OFFSET_M
	static constexpr float kOffsetM = 0.232621227534758f;
#else
	static constexpr float kOffsetM = 0.0f;
#endif
	static constexpr float kOffsetVs = 0.6;

	static constexpr uint32_t kRatioL2SQ16 = 16U * Q16_1;
	static constexpr uint32_t kRatioL2VsQ16 = 128U * Q16_1;
	static constexpr uint32_t kRatioL2SpdQ16 = 1024U * Q16_1;

	std::unique_ptr<MdParser> parser_;
	/* gain conversion ratio of HCG/LCG  */
	uint32_t convGainQ16_ = 7.32f * Q16_1;
	/* LPD/SPD sensitivity ratio */
	uint32_t lpdSpdSensRatioQ16_ = 108.5f * Q16_1;
	/* Single line duration */
	Duration lineDuration_;
};

CameraHelperMx95mbcam::CameraHelperMx95mbcam()
{
	/* Adapt the default delayedControls for the ox03c10 custom controls */
	attributes_.delayedControlParams = {
		{ V4L2_CID_ANALOGUE_GAIN, { 3, false } },
		{ V4L2_CID_DIGITAL_GAIN, { 3, false } },
		{ V4L2_CID_EXPOSURE, { 3, false } },
		{ V4L2_CID_OX03C10_ANALOGUE_GAIN, { 3, false } },
		{ V4L2_CID_OX03C10_DIGITAL_GAIN, { 3, false } },
		{ V4L2_CID_OX03C10_EXPOSURE, { 3, false } },
		{ V4L2_CID_OX03C10_WB_GAIN, { 2, false } },
	};

	/*
	 * Setup embedded data params
	 */
#if ENABLE_EMBEDDED_DATA_TOP_LINES
	attributes_.mdParams.topLines = 2;
#endif

	/* Use CRC-32C polynomial for embedded data validation */
	MdParserOmniOx::CrcParams crcParams = {
		MdParserOmniOx::CrcType::Crc32Le,
		/* Polynomial and check values come from ox03c10 datasheet. */
		0x1edc6f41,
		0x48674bc7,
	};
	parser_ = std::make_unique<MdParserOmniOx>(registerList, crcParams);

	/* Embedded data are 16-bit words when transmitted as image top lines */
	int bpp = attributes_.mdParams.topLines ? 16 : 8;
	if (attributes_.mdParams.topLines)
		parser_->setBitsPerPixel(bpp);

	/* Note: gainType / gainConstants_ are unused */
}

void CameraHelperMx95mbcam::setCameraMode(const CameraMode &mode)
{
	CameraHelper::setCameraMode(mode);

	/* Calculate the line duration based on the sensor hblank */
	lineDuration_ = hblankToLineLength(mode.hblank);

	/* Calculate the maximum exposure in double lines */
	maxExposureDoubleLines_ = maxExposureDoubleLines();
	LOG(NxpCameraHelper, Debug) << "maxExposureDoubleLines_=" << maxExposureDoubleLines_;
}

uint32_t CameraHelperMx95mbcam::gainCode(double gain) const
{
	/* V4L2_CID_ANALOGUE_GAIN code is Q16.16 */
	return static_cast<uint32_t>(gain * (1 << 16));
}

double CameraHelperMx95mbcam::gain(uint32_t gainCode) const
{
	/* V4L2_CID_ANALOGUE_GAIN code is Q16.16 */
	return (gainCode * 1.0 /  (1 << 16));
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

/**
 * \brief Convert from exposure time to double rows
 *
 * \param[in] exposureUs the exposure time in us
 *
 * \return Exposure in double rows
 */
uint32_t CameraHelperMx95mbcam::convertExposureTime2DoubleRows(uint32_t exposureUs) const
{
	const uint32_t doubleLineDurationNs = 2 * lineDuration_ / 1.0ns;
	return (us2ns(exposureUs) + doubleLineDurationNs / 2U) / doubleLineDurationNs;
}

/**
 * \brief Recovers lost exposure time as gain
 *
 * \details Attempts to recover the lost, due to conversion to rows, exposure time as gain
 *
 * \param[in] exposureUs the exposure time before converting it to rows
 * \param[in] exposureRows the resulting double rows after converting exposureUs to double rows
 *
 * \return The lost exposure time as gain in Q8
 */
uint32_t CameraHelperMx95mbcam::calcAdditionalGain(
	uint32_t exposureUs, uint32_t exposureRows) const
{
	const uint64_t exposureNsBeforeConv = us2ns(exposureUs);
	const uint32_t doubleLineDurationNs = 2 * lineDuration_ / 1.0ns;
	const uint64_t exposureNsAfterConv = exposureRows * doubleLineDurationNs;

	return (exposureNsBeforeConv * Q8_1 + exposureNsAfterConv / 2) / exposureNsAfterConv;
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

#if USE_CUSTOM_CONTROLS
void CameraHelperMx95mbcam::setControls(const ControlList *sensorCtrls)
{
	const ControlValue &val = sensorCtrls->get(V4L2_CID_OX03C10_OTP_CORRECTION);
	if (val.type() == ControlTypeNone) {
		LOG(NxpCameraHelper, Error) << "Invalid OX03C10 OTP control";
		return;
	}

	Span<const uint8_t> data = val.data();
	ASSERT(data.size() == sizeof(ox03c10_otp_correction));

	const struct ox03c10_otp_correction *payload;
	payload = reinterpret_cast<const struct ox03c10_otp_correction *>(data.data());
	if (!payload->val1 || !payload->val2) {
		LOG(NxpCameraHelper, Warning) << "Invalid Gain conversion ratio: ["
					      << payload->val1 << ", "
					      << payload->val2 << "] - Use default: "
					      << convGainQ16_;
	} else {
		double convGainOTP = payload->val1 / static_cast<double>(payload->val2);
		convGainQ16_ = convGainOTP * Q16_1;
		LOG(NxpCameraHelper, Debug) << "Gain conversion ratio HCG/LCG: "
					    << convGainOTP;
	}

	if (!payload->val2 || !payload->val3) {
		LOG(NxpCameraHelper, Warning) << "Invalid LPD/SPD sensitivity ratio: ["
					      << payload->val2 << ", "
					      << payload->val3 << "]  - Use default: "
					      << lpdSpdSensRatioQ16_;
	} else {
		double lpdSpdRatioOTP = payload->val2 / static_cast<double>(payload->val3);
		lpdSpdSensRatioQ16_ = lpdSpdRatioOTP * Q16_1;
		LOG(NxpCameraHelper, Debug) << "LPD/SPD sensitivity ratio: "
					    << lpdSpdRatioOTP;
	}
}

void CameraHelperMx95mbcam::controlListSetAGC(
	ControlList *ctrls, Duration exposure, double gain) const
{
	const uint32_t sensorConversionRatio = calcConvRatio(convGainQ16_);

	uint64_t lAgainL, lAgainS, lAgainSPD, lAgainVS;
	uint32_t lDgainL, lDgainS, lDgainSPD, lDgainVS;
	uint64_t lAddGain;
	uint32_t lExpIn = static_cast<uint32_t>(exposure / 1.0us); // to usec
	uint32_t lExpLdoubleRows, lExpSPDdoubleRows, lExpVSdoubleRows;
	uint64_t lExpTotalDoubleRows;
	uint64_t lExpTotalL;
	uint64_t lRatioL2S, lRatioL2SPD, lRatioL2VS;
	uint64_t lMinGainL;
	float lTmpF;

	/* Minimum total gain for the Short frame */
	uint64_t lMinGainS =
		(static_cast<uint64_t>(kMinAnalogGainShortQ16) * kMinDigitalGainShortQ16) / Q16_1;
	/* Minimum total gain for the SPD frame */
	uint64_t lMinGainSPD =
		(static_cast<uint64_t>(kMinAnalogGainSpdQ16) * kMinDigitalGainSpdQ16) / Q16_1;
	/* Minimum total gain for the VS frame */
	uint64_t lMinGainVS =
		(static_cast<uint64_t>(kMinAnalogGainVsQ16) * kMinDigitalGainVsQ16) / Q16_1;

	/* make exposure a multiple of row time as driver needed it */
	lExpLdoubleRows = convertExposureTime2DoubleRows(lExpIn); /* in double rows */
	/* distribution should provide valid values, here an extra check */
	if (lExpLdoubleRows < kMinExposureDoubleLines) {
		lExpLdoubleRows = kMinExposureDoubleLines;
		LOG(NxpCameraHelper, Warning)
			<< "Long exposure distribution < kMinExposureDoubleLines. Value adjusted.";
	} else {
		if (lExpLdoubleRows > maxExposureDoubleLines_) {
			lExpLdoubleRows = maxExposureDoubleLines_;
			LOG(NxpCameraHelper, Warning)
				<< "Long exposure distribution > maxExposureDoubleLines_. Value adjusted.";
		}
	}

	/* Fixed ratios are used by default */
	lRatioL2S = kRatioL2SQ16;
	lRatioL2SPD = kRatioL2SpdQ16;
	lRatioL2VS = kRatioL2VsQ16;

	if (lRatioL2S < sensorConversionRatio) {
		/* minGain for L, recalculated by conv_ratio */
		lMinGainL = lMinGainS * sensorConversionRatio / Q16_1;
	} else {
		/* minGain for L, recalculated by ratio */
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
		lExpLdoubleRows = convertExposureTime2DoubleRows(lExpIn); /* in double rows */
		/* distribution should provide valid values, here an extra check */
		if (lExpLdoubleRows < kMinExposureDoubleLines)
			lExpLdoubleRows = kMinExposureDoubleLines;
	}

	lAddGain = calcAdditionalGain(lExpIn, lExpLdoubleRows);
	lAgainL = (lAgainL * lAddGain + kMult256 / 2U) / kMult256; /* correct gain */

	/* as result of lExpLdoubleRows limitation causing fractional lAddGain */
	if (lAgainL < lMinGainL)
		lAgainL = lMinGainL;

	/* get lAgainS in ratio to lAgainL */
	lAgainS = lAgainL * Q16_1 / lRatioL2S;

	/* SPD distribution */
	lExpSPDdoubleRows = maxExposureDoubleLines_;
	/* calculations of lAgainSPD with restrictions for SPD max exposure and min SPD gain */
	lExpTotalDoubleRows = lAgainL * ((uint64_t)lExpLdoubleRows * lpdSpdSensRatioQ16_ / Q16_1); /* total exposure (with rows...) */
	lAgainSPD = lExpTotalDoubleRows * Q16_1 / (lRatioL2SPD * (uint64_t)lExpSPDdoubleRows);
	if (lAgainSPD < lMinGainSPD) /* but if min gain restriction fails */
	{
		lAgainSPD = lMinGainSPD; /* set mandatory min gain and decrease exposure */
		lExpSPDdoubleRows = (uint32_t)(lExpTotalDoubleRows * Q16_1 / (lRatioL2SPD * lAgainSPD));
		if (lExpSPDdoubleRows < kMinExposureDoubleLines) {
			lExpSPDdoubleRows = kMinExposureDoubleLines;
		}
		lAgainSPD = lExpTotalDoubleRows * Q16_1 / (lRatioL2SPD * (uint64_t)lExpSPDdoubleRows);
	}

	/* VS distribution */
	lExpVSdoubleRows = kMaxVsExposureDoubleLines; /* default VS exposure to max */

	/* calculations of acAgain with restrictions for VS max exposure and min VS gain */
	lTmpF = kOffsetM * (float)Q10_1;
	lExpTotalDoubleRows = lAgainL * ((uint64_t)lExpLdoubleRows * Q10_1 + (uint64_t)lTmpF); /* total exposure (with rows...) */
	lTmpF = kOffsetVs * (float)Q10_1;
	lAgainVS = lExpTotalDoubleRows * Q16_1 / (lRatioL2VS * ((uint64_t)lExpVSdoubleRows * Q10_1 + (uint64_t)lTmpF));
	/* but if min gain restriction fails */
	if (lAgainVS < lMinGainVS) {
		lAgainVS = lMinGainVS; /* set mandatory min gain and decrease exposure */
		lExpVSdoubleRows = (uint32_t)(lExpTotalDoubleRows * Q16_1 / (lRatioL2VS * lAgainVS * Q10_1));
		/**
		 * \note <= is used instead of < because if kMinVsExposureDoubleLines is 0
		 * then the expression would be always false and the compiler
		 * might issue a warning that will be treated as error due to compiler flags
		 */
		if (lExpVSdoubleRows <= kMinVsExposureDoubleLines) {
			lExpVSdoubleRows = kMinVsExposureDoubleLines;
		}
		lTmpF = kOffsetVs * (float)Q10_1;
		lAgainVS = lExpTotalDoubleRows * Q16_1 / (lRatioL2VS * ((uint64_t)lExpVSdoubleRows * Q10_1 + (uint32_t)lTmpF));
		if (lAgainVS < lMinGainVS) {
			lExpVSdoubleRows = (uint32_t)(lExpVSdoubleRows * lAgainVS / Q16_1);
			if (lExpVSdoubleRows <= kMinVsExposureDoubleLines) {
				lExpVSdoubleRows = kMinVsExposureDoubleLines;
			}
			lAgainVS = lExpTotalDoubleRows * Q16_1 / (lRatioL2VS * ((uint64_t)lExpVSdoubleRows * Q10_1 + (uint32_t)lTmpF));
		}
	}

	ox03c10_exposure v4l2Exposures;
	v4l2Exposures.dcg = lExpLdoubleRows; // DCG_exp_row
	v4l2Exposures.spd = lExpSPDdoubleRows; // SPD_exp_row
	v4l2Exposures.vs = lExpVSdoubleRows; // VS_exp_row

	Span<uint8_t> expData(reinterpret_cast<uint8_t *>(&v4l2Exposures), sizeof(v4l2Exposures));
	ctrls->set(V4L2_CID_OX03C10_EXPOSURE, expData);

	lDgainL = kMinDigitalGainLongQ16;
	lDgainS = kMinDigitalGainShortQ16;
	lDgainSPD = kMinDigitalGainVsQ16;
	lDgainVS = kMinDigitalGainVsQ16;

	lAgainL = (lAgainL * Q16_1) / (((uint64_t)lDgainL * sensorConversionRatio) / Q16_1);
	lAgainS = (lAgainS * Q16_1) / lDgainS;
	lAgainSPD = (lAgainSPD * Q16_1) / lDgainSPD;
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

	lX3cMinGain = kMinAnalogGainSpdQ16;
	lX3cMaxGain = kMaxAnalogGainSpdQ16;
	/* lDgainS corrected to get fraction not covered by lAgainS */
	const uint32_t spdAnalogGain = distributeAnalogGain(
		(uint32_t)(lAgainSPD),
		lX3cMinGain,
		lX3cMaxGain,
		&lDgainSPD);
	/* 1.0 = 65536 to 1.0 = 16 conversion */
	v4l2AnalogGains.spd = spdAnalogGain >> 12;

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

	lX3cMinGain = kMinDigitalGainSpdQ16;
	lX3cMaxGain = kMaxDigitalGainSpdQ16;
	const uint32_t spdDigitalGain =
		distributeDigitalGain(lDgainSPD, lX3cMinGain, lX3cMaxGain);
	/* 1.0 = 65536 to 1.0 = 1024 conversion */
	v4l2DigitalGains.spd = spdDigitalGain >> 6;

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
#endif

void CameraHelperMx95mbcam::controlInfoMapGetExposureRange(
	const ControlInfoMap *ctrls, std::vector<Duration> *minExposure,
	std::vector<Duration> *maxExposure, std::vector<Duration> *defExposure) const
{
	(void)ctrls;

	/* \todo Append short and very short exposure values */
	/* Min/max exposure lines are in double rows */
	minExposure->clear();
	minExposure->push_back(exposure(2 * kMinExposureDoubleLines, lineDuration_));

	maxExposure->clear();
	maxExposure->push_back(exposure(2 * maxExposureDoubleLines_, lineDuration_));

	defExposure->clear();
	defExposure->push_back(exposure(2 * (kMinExposureDoubleLines + maxExposureDoubleLines_) / 2,
					lineDuration_));
}

void CameraHelperMx95mbcam::controlInfoMapGetAnalogGainRange(
	const ControlInfoMap *ctrls, std::vector<double> *minGain,
	std::vector<double> *maxGain, std::vector<double> *defGain) const
{
	(void)ctrls;

	/* Fixup HCG gain with conversion ratio */
	const uint32_t sensorConversionRatioQ16 = calcConvRatio(convGainQ16_);
	double convGain = static_cast<double>(sensorConversionRatioQ16) / Q16_1;
	/*
	 * Ratio between HCG and LCG min values is the maximum between:
	 * - Gain conversion ratio HCG/LCG (read from OTP)
	 * - Ratio between total exposure of Long and Short captures
	 */
	const uint32_t ratioHcgLcgMin = std::max(convGainQ16_, kRatioL2SQ16);

	/* \todo Append short and very short analog gain values */
	minGain->clear();
	minGain->push_back(kMinAnalogGain * (ratioHcgLcgMin / Q16_1));

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
		((registers[AecHcgCtrl14Reg] & 0x1fU) << 7U) |
		(registers[AecHcgCtrl15Reg] & 0x7fU);
	uint32_t lcgAnalogGainCode =
		((registers[AecLcgCtrl14Reg] & 0x1fU) << 7U) |
		(registers[AecLcgCtrl15Reg] & 0x7fU);
	uint32_t spdAnalogGainCode =
		((registers[AecSpdCtrl14Reg] & 0x1fU) << 7U) |
		(registers[AecSpdCtrl15Reg] & 0x7fU);
	/* SPD analog gain is provided as the last (shortest) capture. */
	std::array<uint32_t, 3> aGainCodes = { hcgAnalogGainCode,
					       lcgAnalogGainCode,
					       spdAnalogGainCode };
	std::array<float, 3> aGainsArray;
	analogGains(Span<uint32_t>(aGainCodes), Span<float>(aGainsArray));

	/* Fixup HCG gain with conversion ratio */
	const uint32_t sensorConversionRatioQ16 = calcConvRatio(convGainQ16_);
	aGainsArray[0] *= (static_cast<float>(sensorConversionRatioQ16) / Q16_1);

	Span<float> aGains = Span<float>(aGainsArray);
	mdControls->set(md::AnalogueGain, aGains);

	/* Digital gain */
	uint32_t hcgDigitalGainCode =
		((registers[AecHcgCtrl18Reg] & 0x0fU) << 10U) |
		(registers[AecHcgCtrl19Reg] << 2U) |
		(registers[AecHcgCtrl1aReg] & 0x03U);
	uint32_t lcgDigitalGainCode =
		((registers[AecLcgCtrl18Reg] & 0x0fU) << 10U) |
		(registers[AecLcgCtrl19Reg] << 2U) |
		(registers[AecLcgCtrl1aReg] & 0x03U);
	uint32_t spdDigitalGainCode =
		((registers[AecSpdCtrl18Reg] & 0x0fU) << 10U) |
		(registers[AecSpdCtrl19Reg] << 2U) |
		(registers[AecSpdCtrl1aReg] & 0x03U);

	/* SPD digital gain is provided as the last (shortest) capture. */
	std::array<uint32_t, 3> dGainCodes = { hcgDigitalGainCode,
					       lcgDigitalGainCode,
					       spdDigitalGainCode };
	std::array<float, 3> dGainsArray;
	digitalGains(Span<uint32_t>(dGainCodes), Span<float>(dGainsArray));

	Span<float> dGains = Span<float>(dGainsArray);
	mdControls->set(md::DigitalGain, dGains);

	/* Exposure */
	uint32_t dcgExposure =
		(registers[AecHcgCtrl0eReg] << 8U) | registers[AecHcgCtrl0fReg];
	uint32_t spdExposure =
		(registers[AecSpdCtrl0eReg] << 8U) | registers[AecSpdCtrl0fReg];

	/* Exposure from sensor is in double rows */
	float dcgExposureS = exposure(2 * dcgExposure, lineDuration_) / 1.0s;
	/* SPD exposure time is provided as the last (shortest) capture. */
	float spdExposureS = exposure(2 * spdExposure, lineDuration_) / 1.0s;

	std::array<float, 3> exposuresArray = { dcgExposureS,
						dcgExposureS,
						spdExposureS };

	Span<float> exposures = Span<float>(exposuresArray);
	mdControls->set(md::Exposure, exposures);

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

	Span<float> wbGains = Span<float>(wbGainsArray);
	mdControls->set(md::WhiteBalanceGain, wbGains);

	/* Sensor temperature is UQ8.8 and negative above 0xc000 */
	static constexpr uint32_t kTemperatureMax = 0xc000U;
	const uint32_t uTemperature =
		(registers[TmpReg26Reg] << 8U) | registers[TmpReg27Reg];
	float temperature;
	if (uTemperature <= kTemperatureMax)
		temperature = uTemperature * 1.0f / Q8_1;
	else
		temperature = -((uTemperature - kTemperatureMax) * 1.0f / Q8_1);

	mdControls->set(md::Temperature, temperature);

	return 0;
}

#if USE_CUSTOM_CONTROLS
void CameraHelperMx95mbcam::controlListSetAWB(
	ControlList *ctrls, Span<const double, 4> gains) const
{
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

	Span<uint8_t> data(reinterpret_cast<uint8_t *>(wbGains), sizeof(wbGains));
	ctrls->set(V4L2_CID_OX03C10_WB_GAIN, data);
}

int CameraHelperMx95mbcam::sensorControlsToMetaData(const ControlList *sensorCtrls,
						    ControlList *mdCtrls) const
{
	int ret = 0;

	const ControlValue &aGainCtrl = sensorCtrls->get(V4L2_CID_OX03C10_ANALOGUE_GAIN);
	std::array<float, 3> aGainsArray = { 1.0f, 1.0f, 1.0f };
	if (!aGainCtrl.isNone()) {
		Span<const uint8_t> data = aGainCtrl.data();
		ASSERT(data.size() == sizeof(ox03c10_analog_gain));
		/* Sensor control is UQ4.4, metadata is UQ16.16 */
		const struct ox03c10_analog_gain *aGainSensor =
			reinterpret_cast<const struct ox03c10_analog_gain *>(data.data());

		/* SPD analog gain is provided as the last (shortest) capture. */
		std::array<uint32_t, 3> aGainCodes = { aGainSensor->hcg,
						       aGainSensor->lcg,
						       aGainSensor->spd };
		analogGains(Span<uint32_t>(aGainCodes), Span<float>(aGainsArray));

		/* Fixup HCG gain with conversion ratio */
		const uint32_t sensorConversionRatioQ16 = calcConvRatio(convGainQ16_);
		aGainsArray[0] *= (static_cast<float>(sensorConversionRatioQ16) / Q16_1);
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid analog gain control";
		ret = -1;
	}
	mdCtrls->set(md::AnalogueGain, Span<float>(aGainsArray));

	const ControlValue &dGainCtrl = sensorCtrls->get(V4L2_CID_OX03C10_DIGITAL_GAIN);
	std::array<float, 3> dGainsArray = { 1.0f, 1.0f, 1.0f };
	if (!dGainCtrl.isNone()) {
		Span<const uint8_t> data = dGainCtrl.data();
		ASSERT(data.size() == sizeof(ox03c10_digital_gain));
		/* Sensor control is UQ4.10, metadata is UQ16.16 */
		const struct ox03c10_digital_gain *dGainSensor =
			reinterpret_cast<const struct ox03c10_digital_gain *>(data.data());

		/* SPD digital gain is provided as the last (shortest) capture. */
		std::array<uint32_t, 3> dGainCodes = { dGainSensor->hcg,
						       dGainSensor->lcg,
						       dGainSensor->spd };
		digitalGains(Span<uint32_t>(dGainCodes), Span<float>(dGainsArray));
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid digital gain control";
		ret = -1;
	}
	mdCtrls->set(md::DigitalGain, Span<float>(dGainsArray));

	const ControlValue &exposureCtrl = sensorCtrls->get(V4L2_CID_OX03C10_EXPOSURE);
	std::array<float, 3> exposureArray = { 0.0f, 0.0f, 0.0f };
	if (!exposureCtrl.isNone()) {
		Span<const uint8_t> data = exposureCtrl.data();
		ASSERT(data.size() == sizeof(ox03c10_exposure));
		const struct ox03c10_exposure *exposure =
			reinterpret_cast<const struct ox03c10_exposure *>(data.data());

		/* Exposure from sensor is in double rows */
		float dcgExposureS = CameraHelper::exposure(2 * exposure->dcg, lineDuration_) / 1.0s;
		float spdExposureS = CameraHelper::exposure(2 * exposure->spd, lineDuration_) / 1.0s;

		/* SPD exposure time is provided as the last (shortest) capture. */
		exposureArray[0] = dcgExposureS;
		exposureArray[1] = dcgExposureS;
		exposureArray[2] = spdExposureS;
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid exposure control";
		ret = -1;
	}
	mdCtrls->set(md::Exposure, Span<float>(exposureArray));

	const ControlValue &wbCtrl = sensorCtrls->get(V4L2_CID_OX03C10_WB_GAIN);
	std::array<float, 4> wbGainsArray = { 1.0f, 1.0f, 1.0f, 1.0f };
	if (!wbCtrl.isNone()) {
		Span<const uint8_t> data = wbCtrl.data();
		ASSERT(data.size() == sizeof(ox03c10_wb_capture_gain[4]));
		/* Sensor control is UQ5.10, metadata is UQ16.16 */
		const struct ox03c10_wb_capture_gain *wb =
			reinterpret_cast<const struct ox03c10_wb_capture_gain *>(data.data());

		std::array<uint32_t, 4> wbGainCodes = { wb->r, wb->gr, wb->gb, wb->b };
		auto iter = std::find(wbGainCodes.begin(), wbGainCodes.end(), 0);
		/*
		 * In case WB gains are reported from sensor with value 0.0f,
		 * wbGain should be forced to default value 1.0f for
		 * valid processing.
		 * Hence avoid updating wbGainArray with wbGainCodes
		 */
		if (iter == wbGainCodes.end())
			whiteBalanceGains(Span<uint32_t>(wbGainCodes),
					  Span<float>(wbGainsArray));
	} else {
		/* Do nothing - optional control */
	}
	mdCtrls->set(md::WhiteBalanceGain, Span<float>(wbGainsArray));

	/* No temperature information, report arbitrary value */
	mdCtrls->set(md::Temperature, 25.0);

	return ret;
}
#endif

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

/**
 * \brief Calculate the maximum exposure based on VTS
 *        Datasheet specifies
 *        DCG_exp + VS_exp_max < VTS - 12
 *
 * \return Maximum exposure in double lines
 */
uint32_t CameraHelperMx95mbcam::maxExposureDoubleLines() const
{
	uint32_t vts = mode_.height + mode_.vblank;
	return (vts / 2) - kMaxVsExposureDoubleLines - 12 - 1;
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
