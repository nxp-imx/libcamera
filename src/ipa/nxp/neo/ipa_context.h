/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 IPA Context
 *     src/ipa/rkisp1/ipa_context.h
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * ipa_context.h - NXP NEO IPA Context
 * Copyright 2024 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::nxpneo {

struct IPASessionConfiguration {
	struct {
		bool enabled;
	} agc;

	struct {
		/* ROI for statistics measurements */
		struct neoisp_roi_cfg_s roi;
	} awb;

	struct {
		utils::Duration minShutterSpeed;
		utils::Duration maxShutterSpeed;
		double minAnalogueGain;
		double maxAnalogueGain;

		int32_t defVBlank;
		utils::Duration lineDuration;
		Size size;
	} sensor;

	struct {
		uint32_t revision;
	} hw;
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

		bool autoEnabled;
	} agc;

	struct {
		struct {
			struct {
				double red;
				double green;
				double blue;
			} manual;
			struct {
				double red;
				double green;
				double blue;
			} automatic;
		} gains;

		unsigned int temperatureK;
		bool autoEnabled;
	} awb;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double gain;
		bool autoEnabled;
	} agc;

	struct {
		struct {
			double red;
			double green;
			double blue;
		} gains;

		unsigned int temperatureK;
		bool autoEnabled;
	} awb;

	struct {
		uint32_t exposure;
		double gain;
	} sensor;
};

struct IPAContext {
	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;
};

} /* namespace ipa::nxpneo */

} /* namespace libcamera*/
