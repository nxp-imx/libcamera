/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 Color Correction Matrix control algorithm
 *     src/ipa/rkisp1/algorithms/ccm.cpp
 * Copyright (C) 2024, Ideas On Board
 *
 * ccm.cpp - Color Correction Matrix control algorithm
 * Copyright 2024 NXP
 */

#include "ccm.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tuple>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/yaml_parser.h"

#include "libipa/interpolator.h"

/**
 * \file ccm.h
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class Ccm
 * \brief A color correction matrix algorithm
 *
 * The CCM algorithm should run after the AWB algorithm
 * since the CCM has dependency with the AWB.
 * Indeed the CCM algorithm is using the Colour Temperature
 * computed by the AWB algorithm to select matching colour
 * correction matrices.
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoCcm)

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Ccm::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	int ret = ccm_.readYaml(tuningData["ccms"], "ct", "ccm");
	if (ret < 0) {
		LOG(NxpNeoAlgoCcm, Warning)
			<< "Failed to parse 'ccm' "
			<< "parameter from tuning file; falling back to unit matrix";
		ccm_.setData({ { 0, Matrix<float, 3, 3>::identity() } });
	}

	ret = offsets_.readYaml(tuningData["ccms"], "ct", "offsets");
	if (ret < 0) {
		LOG(NxpNeoAlgoCcm, Warning)
			<< "Failed to parse 'offsets' "
			<< "parameter from tuning file; falling back to zero offsets";
		offsets_.setData({ { 0, Matrix<int32_t, 3, 1>({ 0, 0, 0 }) } });
	}

	return 0;
}

void Ccm::setParameters(neoisp_meta_params_s *params,
			const Matrix<float, 3, 3> &matrix,
			const Matrix<int32_t, 3, 1> &offsets)
{
	/*
	 * This matrix is defined with the full range YUV conversion matrix.
	 * This is fixed regardless the desired color space.
	 * The sRGB YUV conversion is used to set this matrix.
	 * The definition can be found below:
	 * https://linuxtv.org/downloads/v4l-dvb-apis/userspace-api/v4l/colorspaces-details.html
	 */
	Matrix<float, 3, 3> RGB2YUV({ 0.299, 0.587, 0.114,
				      -0.169, -0.331, 0.5,
				      0.5, -0.419, -0.081 });
	/*
	 * The CSC matrix is the resulting merge (multiplication) of
	 * both the RGB to YUV conversion matrix and the color correction
	 * matrix interpolated according to the measured color temperature.
	 */
	Matrix<float, 3, 3> CSC = RGB2YUV * matrix;

	params->features_cfg.rgb2yuv_cfg = 1;
	/* NEO ISP gain format is u8.8 */
	params->regs.rgb2yuv.gain_ctrl_rgain = 256;
	params->regs.rgb2yuv.gain_ctrl_bgain = 256;

	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			/*
			 * NEO ISP matrix format is s8.8:
			 * 8 bit integer and 8 bit fractional,
			 * ranging from -128 to +127.99609375
			 * in fixed point: from -32768 (0x8000) to
			 * +32767 (0x7fff)
			 */
			params->regs.rgb2yuv.mat_rxcy[i][j] =
				std::clamp<int16_t>(std::round(256 * CSC[i][j]),
						    0x8000, 0x7fff);
	}

	for (unsigned int i = 0; i < 3; i++)
		/* NEO ISP offset format is s21 */
		params->regs.rgb2yuv.csc_offsets[i] = offsets[i][0] & 0x1fffff;

	LOG(NxpNeoAlgoCcm, Debug) << "Setting matrix " << matrix;
	LOG(NxpNeoAlgoCcm, Debug) << "Setting CSC " << CSC;
	LOG(NxpNeoAlgoCcm, Debug) << "Setting offsets " << offsets;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Ccm::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext,
		  neoisp_meta_params_s *params)
{
	uint32_t ct = context.activeState.awb.temperatureK;

	LOG(NxpNeoAlgoCcm, Debug) << "Colour temperature=" << ct;
	/*
	 * \todo The colour temperature will likely be noisy, add filtering to
	 * avoid updating the CCM matrix all the time.
	 */
	if (frame > 0 && ct == ct_) {
		frameContext.ccm.ccm = context.activeState.ccm.ccm;
		return;
	}

	ct_ = ct;
	Matrix<float, 3, 3> ccm = ccm_.getInterpolated(ct);
	Matrix<int32_t, 3, 1> offsets = offsets_.getInterpolated(ct);

	context.activeState.ccm.ccm = ccm;
	frameContext.ccm.ccm = ccm;

	setParameters(params, ccm, offsets);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Ccm::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const neoisp_meta_stats_s *stats,
		  ControlList &metadata)
{
	float m[9];
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			m[i * 3 + j] = frameContext.ccm.ccm[i][j];
	}
	metadata.set(controls::ColourCorrectionMatrix, m);
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
