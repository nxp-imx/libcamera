
/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * goc.cpp NXP NEO Gamma out control
 * Copyright 2025 NXP
 */

#include "goc.h"

#include <linux/nxp_neoisp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

#include "libipa/fixedpoint.h"

/**
 * \file goc.cpp
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

namespace {

/*
 * The following Transfer Functions are defined from standards
 * as described in this link:
 * https://linuxtv.org/downloads/v4l-dvb-apis/userspace-api/v4l/colorspaces-details.html
 */
const std::map<const IPATransferFunction, GammaOutCorrection::XferFunc> kXferMap = {
	/* Linear transfer function */
	{ IPATransferFunctionLinear, { 0.0f, 0.0f, 1.0f, 0.0f, 1.0f } },
	/*
	 * sRGB transfer function:
	 * L' = 12.92L, for 0 <= L <= 0.0031308
	 * L' = 1.055L^(1/2.4) - 0.055, for L > 0.0031308
	 *    = 1.055 * (L^(1/2.4) - (0.055 / 1.055)), for L > 0.0031308
	 */
	{ IPATransferFunctionSrgb, { 12.92f, 0.0031308f, 1.055f, 0.0521327f, 0.416667f } },
	/*
	 * Rec. 709 transfer function:
	 * L' = 4.5L, for 0 <= L < 0.018
	 * L' = 1.099L^0.45 - 0.099, for L >= 0.018
	 *    = 1.099 * (L^0.45 - (0.099 / 1.099)), for L >= 0.018
	 */
	{ IPATransferFunctionRec709, { 4.5f, 0.018f, 1.099f, 0.0900819f, 0.45f } },
};

/*
 * The following YCbCr encodings are defined from standards
 * as described in this link:
 * https://linuxtv.org/downloads/v4l-dvb-apis/userspace-api/v4l/colorspaces-details.html
 */
const std::map<const IPAYcbcrEncoding, GammaOutCorrection::YCbCrEnc> kEncMap = {
	/*
	 * BT.601 full-range encoding - floating-point matrix:
	 *	[0.299, 0.5870, 0.1140
	 *	 -0.1687, -0.3313, 0.5
	 *	 0.5, -0.4187, -0.0813]
	 */
	{
		IPAYcbcrEncodingRec601,
		{ { Matrix<float, 3, 3>(
			  { 0.299f, 0.5870f, 0.1140f,
			    -0.1687f, -0.3313f, 0.5f,
			    0.5f, -0.4187f, -0.0813f }) },
		  { Matrix<uint32_t, 3, 1>({ 0, 128, 128 }) } },
	},
	/*
	 * BT.709 full-range encoding - floating-point matrix:
	 *	[0.2126, 0.7152, 0.0722
	 *	 -0.1146, -0.3854, 0.5
	 *	 0.5, -0.4542, -0.0458]
	 */
	{
		IPAYcbcrEncodingRec709,
		{ { Matrix<float, 3, 3>(
			  { 0.2126f, 0.7152f, 0.0722f,
			    -0.1146f, -0.3854f, 0.5f,
			    0.5f, -0.4542f, -0.0458f }) },
		  { Matrix<uint32_t, 3, 1>({ 0, 128, 128 }) } },
	},
	/* No encoding - used for RGB output formats. */
	{
		IPAYcbcrEncodingNone,
		{ { Matrix<float, 3, 3>(
			  { 1.0f, 0.0f, 0.0f,
			    0.0f, 1.0f, 0.0f,
			    0.0f, 0.0f, 1.0f }) },
		  { Matrix<uint32_t, 3, 1>({ 0, 0, 0 }) } },
	},
};
} /* namespace */

/**
 * \class GammaOutCorrection
 * \brief NXP NEO Gamma Output Correction control
 *
 * This algorithm configures the GCM block of the ISP.
 * This block performs a gamma correction on input pixels.
 *
 * The GCM input is in YUV format. Since the gamma correction applies to
 * linear RGB pixel formats, the first stage of the GCM is to convert the input
 * pixels into RGB format using the inverse RGB2YUV conversion of the ISP
 * RGB2YUV block. This RGB2YUV conversion is configured by the CCM algorithm and
 * is defined for sRGB YUV encoding: BT.601 full-range encoding.
 * Hence the only color spaces supported are for the sRGB gamut of chromaticities.
 * This YUV2RGB conversion is configured with the GCM input matrix.
 *
 * The second stage of the GCM is applying the gamma correction using
 * the transfer function relevant to the selected color space.
 * The supported standards for the transfer function are sRGB and Rec.709.
 * The ISP applies following operation for the gamma correction:
 * L' = LinearGain * L						(for R <= LinearThreshold)
 * L' = NonLinearGain * (L^(GammaInverse) - NonLinearOffset)	(for R > LinearThreshold)
 * The ISP gamma correction is not applying any non-linear gain for the transfer function.
 * Hence this is compensated by applying this gain in the third stage of the GCM.
 * There is the option to configure the gamma value of the transfer function using
 * the calibration file, overriding the default gamma value relevant to the
 * selected color space.
 *
 * The third and last stage of the GCM is converting the non linear R'G'B'
 * for the YUV pixel format into the Y'U'V' output format using the Y'CbCr
 * encoding function relevant to the selected color space.
 * The supported standards for the Y'CbCr encoding are BT.601 and BT.709.
 * At this stage, the ISP uses a configured output matrix set for the encoding.
 * Also this matrix is quantized according to the full or limited range
 * and is compensated with the transfer function gain.
 *
 * Relevant keys in the GammaOutCorrection section of the calibration file:
 * gamma: this value is the camera gamma used for the gamma correction.
 *              It applies in the transfer function as 1.0/gamma.
 *              Optional parameter
 *              Float type - range value is inverse of the gamma ISP s1.8 format
 *              Min: inverse(511/256) - Max: inverse(1/256)
 *              Default value: value set by the standards according to the user color space
 *
 * Useful links:
 * - https://linuxtv.org/downloads/v4l-dvb-apis/userspace-api/v4l/colorspaces-details.html
 * - https://en.wikipedia.org/wiki/YCbCr
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoGoc)

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int GammaOutCorrection::init([[maybe_unused]] IPAContext &context,
			     const YamlObject &tuningData)
{
	/* Get the gamma value from tuning file. */
	gamma_ = tuningData["gamma"].get<float>();

	if (gamma_.has_value())
		LOG(NxpNeoAlgoGoc, Debug) << "Configured gamma: " << gamma_.value();

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int GammaOutCorrection::configure([[maybe_unused]] IPAContext &context,
				  [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	IPAColorSpace colorSpace = context.configuration.colorSpace;
	LOG(NxpNeoAlgoGoc, Debug) << "ColorSpace Name: " << colorSpaceName(colorSpace);

	/* Get the Transfer function according to the color space. */
	auto itXferFunc = kXferMap.find(colorSpace.transferFunction);
	if (itXferFunc == kXferMap.end()) {
		LOG(NxpNeoAlgoGoc, Error) << "XferFunc not found - sRGB and Rec.709 are supported";
		return -EINVAL;
	}
	xferFunc_ = itXferFunc->second;
	if (gamma_.has_value())
		xferFunc_.gammaInverse = validateGamma(1.0f / gamma_.value());
	context.activeState.goc.gamma = 1.0f / xferFunc_.gammaInverse;

	LOG(NxpNeoAlgoGoc, Debug) << "[xferFunc] linearGain: " << xferFunc_.linearGain
				  << ", linearThreshold: " << xferFunc_.linearThreshold
				  << ", nonLinearGain: " << xferFunc_.nonLinearGain
				  << ", nonLinearOffset: " << xferFunc_.nonLinearOffset
				  << ", gammaInverse: " << xferFunc_.gammaInverse;

	/* Get the YCbCr Encoding according to the color space. */
	auto itEncoding = kEncMap.find(colorSpace.ycbcrEncoding);
	if (itEncoding == kEncMap.end()) {
		LOG(NxpNeoAlgoGoc, Error) << "YCbCr Encoding not found - BT.601 and BT.709 are supported";
		return -EINVAL;
	}
	ycbcrEnc_ = itEncoding->second;
	LOG(NxpNeoAlgoGoc, Debug) << "[ycbcrEnc] matrix: " << ycbcrEnc_.matrix
				  << ", offsets: " << ycbcrEnc_.offsets;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void GammaOutCorrection::queueRequest(IPAContext &context, const uint32_t frame,
				      IPAFrameContext &frameContext,
				      const ControlList &controls)
{
	if (frame == 0)
		frameContext.goc.update = true;

	const auto &gamma = controls.get(controls::Gamma);
	if (gamma) {
		frameContext.goc.update = true;
		xferFunc_.gammaInverse = validateGamma(1.0f / *gamma);
		context.activeState.goc.gamma = 1.0f / xferFunc_.gammaInverse;
		LOG(NxpNeoAlgoGoc, Debug) << "User gamma: " << *gamma
					  << " - Set gamma to: " << 1.0f / xferFunc_.gammaInverse;
	}

	frameContext.goc.gamma = context.activeState.goc.gamma;
}

/**
 * \brief Set the GCM ISP params for the YUV2RGB conversion
 * \param[in] gcm The ISP GCM params
 *
 * The YUV2RGB conversion is set with the GCM input matrix as the inverse
 * of the conversion set in the RGB2YUV block of the ISP. The RGB2YUV conversion
 * is configured by the CCM algorithm and is defined for sRGB YUV encoding:
 * BT.601 full-range encoding.
 */
void GammaOutCorrection::setYuv2RgbParams(neoisp_gcm_cfg_s &gcm) const
{
	const Matrix<float, 3, 3> imat = kEncMap.at(IPAYcbcrEncodingRec601).matrix.inverse();

	/* Input matrix */
	for (unsigned int i = 0; i < 3; i++) {
		gcm.ioffsets[i] = 0;
		for (unsigned int j = 0; j < 3; j++) {
			/* imat in s8.8 */
			gcm.imat_rxcy[i][j] =
				floatingToFixedPoint<8, 8, int16_t, float>(imat[i][j]);
		}
	}
}

/**
 * \brief Set the GCM ISP params for the transfer function
 * \param[in] gcm The ISP GCM params
 */
void GammaOutCorrection::setXferParams(neoisp_gcm_cfg_s &gcm) const
{
	/* gamma: u1.8 format */
	gcm.gamma0_gamma0 = floatingToFixedPoint<1, 8, uint16_t, float>(xferFunc_.gammaInverse);
	gcm.gamma1_gamma1 =
		floatingToFixedPoint<1, 8, uint16_t, float>(xferFunc_.gammaInverse);
	gcm.gamma2_gamma2 =
		floatingToFixedPoint<1, 8, uint16_t, float>(xferFunc_.gammaInverse);
	/* gamma offset: u0.12 format */
	gcm.gamma0_offset0 =
		floatingToFixedPoint<0, 12, uint16_t, float>(xferFunc_.nonLinearOffset);
	gcm.gamma1_offset1 =
		floatingToFixedPoint<0, 12, uint16_t, float>(xferFunc_.nonLinearOffset);
	gcm.gamma2_offset2 =
		floatingToFixedPoint<0, 12, uint16_t, float>(xferFunc_.nonLinearOffset);
	/* black level gain: u8.8 format */
	gcm.blklvl0_ctrl_gain0 =
		floatingToFixedPoint<8, 8, uint16_t, float>(xferFunc_.linearGain);
	gcm.blklvl1_ctrl_gain1 =
		floatingToFixedPoint<8, 8, uint16_t, float>(xferFunc_.linearGain);
	gcm.blklvl2_ctrl_gain2 =
		floatingToFixedPoint<8, 8, uint16_t, float>(xferFunc_.linearGain);
	/* black level offset set to 0 */
	gcm.blklvl0_ctrl_offset0 = 0;
	gcm.blklvl1_ctrl_offset1 = 0;
	gcm.blklvl2_ctrl_offset2 = 0;
	/* linear threshold: u0.16 format */
	gcm.lowth_ctrl01_threshold0 =
		floatingToFixedPoint<0, 16, uint16_t, float>(xferFunc_.linearThreshold);
	gcm.lowth_ctrl01_threshold1 =
		floatingToFixedPoint<0, 16, uint16_t, float>(xferFunc_.linearThreshold);
	gcm.lowth_ctrl2_threshold2 =
		floatingToFixedPoint<0, 16, uint16_t, float>(xferFunc_.linearThreshold);
}

/**
 * \brief Set the GCM ISP params for the YCbCr encoding
 * \param[in] gcm The ISP GCM params
 * \param[in] range Limited or full range
 *
 * The RGB formats and the JPEG color space uses full range as default quantization.
 */
void GammaOutCorrection::setEncodingParams(neoisp_gcm_cfg_s &gcm, const IPARange range) const
{
	/*
	 * The default offsets are set for 8-bit full-range.
	 * In limited range the offsets are defined by standard as: (16, 128, 128)
	 * for 8-bit range.
	 * However ISP offsets are defined for 12-bit range, hence the offsets
	 * defined by standard are converted for 12-bit range.
	 */
	uint32_t yOffset = (range == IPARangeLimited) ? 16 : ycbcrEnc_.offsets[0][0];
	gcm.ooffsets[0] = encOffsetsToParams(yOffset);
	gcm.ooffsets[1] = encOffsetsToParams(ycbcrEnc_.offsets[1][0]);
	gcm.ooffsets[2] = encOffsetsToParams(ycbcrEnc_.offsets[2][0]);

	/*
	 * The default matrices are set for full-range.
	 * In limited range, Y' ranges from 16 to 235, Cb and Cr range from 16 to 240.
	 * The same quantization factors are applied to Y'CbCr for BT.601 and BT.709:
	 * (219*Y, 224*Pb, 224*Pr).
	 */
	Matrix<float, 3, 1> qFactor = (range == IPARangeLimited)
				? Matrix<float, 3, 1>({ 219.0f / 256, 224.0f / 256, 224.0f / 256 })
				: Matrix<float, 3, 1>({ 256.0f / 256, 256.0f / 256, 256.0f / 256 });
	LOG(NxpNeoAlgoGoc, Debug) << "Quantized factor: " << qFactor;

	for (unsigned int i = 0; i < 3; i++) {
		LOG(NxpNeoAlgoGoc, Debug) << "ISP gcm.ooffsets[" << i << "]: " << gcm.ooffsets[i];
		for (unsigned int j = 0; j < 3; j++) {
			/* omat in s8.8 */
			gcm.omat_rxcy[i][j] =
				std::round(quantizedOmat(
					floatingToFixedPoint<8, 8, int16_t, float>(
						ycbcrEnc_.matrix[i][j]),
					qFactor[i][0]));
			LOG(NxpNeoAlgoGoc, Debug) << "ISP gcm.omat_rxcy[" << i << "][" << j << "]: "
						  << gcm.omat_rxcy[i][j];
		}
	}
	/* Set the sign configuration to unsigned format. */
	gcm.mat_confg_sign_confg = 1;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void GammaOutCorrection::prepare(IPAContext &context,
				 [[maybe_unused]] const uint32_t frame,
				 [[maybe_unused]] IPAFrameContext &frameContext,
				 NxpNeoParams *params)
{
	if (!frameContext.goc.update)
		return;

	IPAColorSpace colorSpace = context.configuration.colorSpace;

	/* Enable GCM block configuration. */
	auto config = params->block<BlockParamsType::Gcm>();
	config.setUpdate(true);

	/* Set GCM params. */
	setYuv2RgbParams(*config);
	setXferParams(*config);
	setEncodingParams(*config, colorSpace.range);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void GammaOutCorrection::process([[maybe_unused]] IPAContext &context,
				 [[maybe_unused]] const uint32_t frame,
				 IPAFrameContext &frameContext,
				 [[maybe_unused]] const NxpNeoStats *stats,
				 ControlList &metadata)
{
	metadata.set(controls::Gamma, frameContext.goc.gamma);
}

REGISTER_IPA_ALGORITHM(GammaOutCorrection, "GammaOutCorrection")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
