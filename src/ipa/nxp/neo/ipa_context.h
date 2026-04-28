/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * NXP NEO IPA Context
 *
 * Based on RkISP1 IPA Context
 *     src/ipa/rkisp1/ipa_context.h
 * Copyright (C) 2021-2022, Ideas On Board
 */

#pragma once

#include <array>
#include <map>
#include <optional>
#include <vector>

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include <libcamera/ipa/core_ipa_interface.h>
#include <libcamera/ipa/nxpneo_ipa_interface.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include <libipa/fc_queue.h>

#include "nxp/cam_helper/camera_helper.h"
#include "neoisp-definitions.h"

namespace libcamera {

namespace ipa::nxpneo {

struct IPAHwSettings {
	uint32_t hwRevision;
	uint32_t hwCapabilities;
	uint64_t supportedParamsBlocks;
};

struct IPASessionConfiguration {
	struct {
		std::array<neoisp_roi_cfg_s, NEO_AF_ROIS_CNT> rois;
		std::array<double, NEO_AF_ROIS_CNT> normalGains;

		/* Camera mode */
		uint16_t cropX;
		uint16_t cropY;
		double scaleX;
		double scaleY;
	} af;

	struct {
		/* ROI for statistics measurements */
		struct neoisp_roi_cfg_s roi;
	} agc;

	struct {
		/* ROI for statistics measurements */
		struct neoisp_roi_cfg_s roi;
		bool awbGainInSensor;
		ObwbArray<unsigned int> obwbObpp;
		ObwbArray<ChannelArray<float>> blcFactors;
	} awb;

	struct {
		/* OBWB instances BLC offsets */
		ObwbArray<ChannelArray<uint16_t>> obwbOffsets;
		/* OBWB instances obpp values */
		ObwbArray<unsigned int> obwbObpp;
		/* BLC offset reported in metadata format */
		ChannelArray<int32_t> mdOffsets;
	} blc;

	struct {
		uint16_t ratioLong2Short;
	} hdr;

	struct {
		utils::Duration minExposureTime;
		utils::Duration maxExposureTime;
		utils::Duration defExposureTime;
		double minAnalogueGain;
		double maxAnalogueGain;
		double defAnalogueGain;

		int32_t defVBlank;
		utils::Duration lineDuration;
		Size size;
		/* bpp per ISP input */
		std::array<uint32_t, 2> bpps;
	} sensor;

	struct {
		struct neoisp_roi_cfg_s roi;
		uint16_t gblMode;
	} drc;

	std::map<IPAStreamType, IPAStream> streams;
	std::vector<IPACameraContext> activeContexts;

	IPAColorSpace colorSpace;
	IPAPipelineMode pipelineMode;
};

struct IPAActiveState {
	struct {
		controls::AfModeEnum mode;
		std::optional<int32_t> hwPosition;
		bool hwPositionUpdate;
	} af;

	struct agc {
		struct {
			uint32_t exposure;
			double gain;
		} manual;
		struct {
			uint32_t exposure;
			double gain;
		} automatic;

		controls::AeConstraintModeEnum constraintMode;
		controls::AeExposureModeEnum exposureMode;
		bool autoEnabled;
	};
	std::map<IPACameraContext, agc> agcs;

	struct {
		struct {
			RGB<double> manual;
			RGB<double> automatic;
		} gains;

		unsigned int temperatureK;
		bool autoEnabled;
		std::array<std::array<RGB<uint64_t>, NEO_CTEMP_BLOCK_NB_X>, NEO_CTEMP_BLOCK_NB_Y>
			blockSums;
	} awb;

	struct {
		Matrix<float, 3, 3> ccm;
	} ccm;

	struct {
		bool enabled;
	} lsc;

	struct {
		float gamma;
	} goc;
};

struct IPAFrameContext : public FrameContext {
	struct {
		/* User control updates. */
		std::optional<controls::AfModeEnum> mode;
		std::optional<controls::AfRangeEnum> range;
		std::optional<controls::AfSpeedEnum> speed;
		std::optional<controls::AfMeteringEnum> metering;
		std::optional<std::vector<Rectangle>> windows;
		std::optional<controls::AfPauseEnum> pause;
		std::optional<controls::AfTriggerEnum> trigger;
		std::optional<float> lensPosition;

		/* AF states. */
		controls::AfStateEnum state;
		controls::AfPauseStateEnum pauseState;
	} af;

	struct agc {
		uint32_t exposure;
		double gain;
		bool autoEnabled;
	};
	std::map<IPACameraContext, agc> agcs;

	struct {
		RGB<double> gains;
		unsigned int temperatureK;
		bool autoEnabled;
		/* Set of WB enabled flags for the 3 OBWB blocks */
		std::array<bool, 3> colorGainsSet;
		std::vector<RGB<double>> blockAverages;
	} awb;

	struct {
		/* Set of BLC enabled flags for the 3 OBWB blocks */
		std::array<bool, 3> colorOffsetsSet;
	} blc;

	struct {
		struct agc {
			uint32_t exposure;
			double gain;
		};
		std::map<IPACameraContext, agc> agcs;
		RGB<double> wbGains;
		ControlList mdControls;
		bool metaDataValid;
	} sensor;

	struct {
		Matrix<float, 3, 3> ccm;
	} ccm;

	struct {
		bool enabled;
		bool update;
	} lsc;

	struct {
		float gamma;
		bool update;
	} goc;

	IPACameraContext cameraContext;
	std::map<IPACameraContext, bool> processed;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPAHwSettings hw;
	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	ControlInfoMap::Map ctrlMap;

	/* Interface to the Camera Helper */
	std::unique_ptr<nxp::CameraHelper> camHelper;
};

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
