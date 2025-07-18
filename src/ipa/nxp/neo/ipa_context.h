/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 IPA Context
 *     src/ipa/rkisp1/ipa_context.h
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * ipa_context.h - NXP NEO IPA Context
 * Copyright 2024-2025 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include <libcamera/ipa/core_ipa_interface.h>
#include <libcamera/ipa/nxpneo_ipa_interface.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include <libipa/fc_queue.h>

#include "nxp/cam_helper/camera_helper.h"

namespace libcamera {

namespace ipa::nxpneo {

struct IPASessionConfiguration {
	struct {
		/* ROI for statistics measurements */
		struct neoisp_roi_cfg_s roi;
	} agc;

	struct {
		/* ROI for statistics measurements */
		struct neoisp_roi_cfg_s roi;
	} awb;

	struct {
		utils::Duration minExposureTime;
		utils::Duration maxExposureTime;
		double minAnalogueGain;
		double maxAnalogueGain;

		int32_t defVBlank;
		utils::Duration lineDuration;
		Size size;
		uint32_t bpp;
	} sensor;

	struct {
		uint32_t revision;
	} hw;

	std::vector<IPAStream> streams;

	IPAColorSpace colorSpace;
};

struct IPAActiveState {
	struct {
		struct {
			uint32_t exposure;
			double gain;
		} manual;
		struct {
			uint32_t exposure;
			double gain;
		} automatic;

		uint32_t constraintMode;
		uint32_t exposureMode;
		bool autoEnabled;
	} agc;

	struct {
		struct {
			RGB<double> manual;
			RGB<double> automatic;
		} gains;

		unsigned int temperatureK;
		bool autoEnabled;
	} awb;

	struct {
		Matrix<float, 3, 3> ccm;
	} ccm;

	struct {
		float gamma;
	} goc;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double gain;
		bool autoEnabled;
	} agc;

	struct {
		RGB<double> gains;
		unsigned int temperatureK;
		bool autoEnabled;
		/* Set of WB enabled flags for the 3 OBWB blocks */
		std::array<bool, 3> colorGainsSet;
	} awb;

	struct {
		/* Set of BLC enabled flags for the 3 OBWB blocks */
		std::array<bool, 3> colorOffsetsSet;
	} blc;

	struct {
		uint32_t exposure;
		double gain;
		ControlList mdControls;
		bool metaDataValid;
	} sensor;

	struct {
		Matrix<float, 3, 3> ccm;
	} ccm;

	struct {
		float gamma;
		bool update;
	} goc;
};

struct IPAContext {
	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	ControlInfoMap::Map ctrlMap;

	/* Interface to the Camera Helper */
	std::unique_ptr<nxp::CameraHelper> camHelper;
};

} /* namespace ipa::nxpneo */

} /* namespace libcamera*/
