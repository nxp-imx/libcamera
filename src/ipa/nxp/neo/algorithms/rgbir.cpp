/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * rgbir.cpp - NXP NEO RGBIR to RGB,IR block configuration
 * Copyright 2024 NXP
 */

#include "rgbir.h"

#include <algorithm>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file rgbir.cpp
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class RgbIr
 * \brief RGBIR to HC (Head Color), RGBB,IR block and IR Compression units
 * configuration
 *
 * Head Color unit:
 * This block reorders any Bayer or RGBIr pattern in horizontal and vertical
 * directions to become RGGB. Block is controlled by configuring the horizontal
 * (hoffset) and vertical (voffset) position of the R pixel in the sensor
 * pattern.
 * For regular Bayer pattern, ISP driver configures the HC blocks, inferring the
 * R pixel position from the video device input buffer format. For RGBIr
 * patterns, the HC block has to be configured explicitly by the IPA.
 * head-color: [ hoffset, voffset ]
 *
 * RGBIR unit:
 * Converts a RGBIR frame into separate RGGB bayer and IR components.
 * RGGB color channels constructed are further corrected by a color correction
 * weighted value of the IR.
 * ccm: CCM[0-2] (s4.12)
 *   Color correction parameter with which IR component is scaled and later
 *   added to color component.
 * crosstalk-threshold: CCM[0-2]_TH
 *  Pixel with higher values than the threshold are treated as saturated and
 *   crosstalk removal is not performed.
 *
 * IR COMPRESSION Unit:
 * Input is 20 bits and output is either 8 or 16 bits depending on the
 * user-selected stream format.
 * Using points[] evaluated in increasing order, conversion logic is:
 * if (pv < points[N])
 *   opv = (pv - offsets[N]) * ratios[N] + newpoints[N]
 * with:
 * - pv: input pixel value
 * - opv: output pixel value
 * - points: KNEE_POINT[1-4] (u20)
 * - offsets: KNEE_NPOINT[0-4] (u20)
 * - newpoints: KNEE_NPOINT[0-4] u16
 * - ratios: KNEE_RATIO[0-4] u1.15
 * Last entry in the offsets/newpoints/ratios arrays are used as the default
 * case, meaning that no points[] value matched the condition (pv < points[N]).
 * */

LOG_DEFINE_CATEGORY(NxpNeoAlgoRgbIr)

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int RgbIr::init([[maybe_unused]] IPAContext &context,
		const YamlObject &tuningData)
{
	int ret;

	headColor_ = tuningData["head-color"]
			     .getList<uint32_t>()
			     .value_or(std::vector<uint32_t>{});
	if (headColor_.size() != kNumHeadColorEntries) {
		LOG(NxpNeoAlgoRgbIr, Error)
			<< "Invalid head-color size: expected "
			<< kNumHeadColorEntries << " elements, got "
			<< headColor_.size();
		return -EINVAL;
	}

	ccm_ = tuningData["ccm"]
		       .getList<uint32_t>()
		       .value_or(std::vector<uint32_t>{});
	if (ccm_.size() != kNumColorChannels) {
		LOG(NxpNeoAlgoRgbIr, Error)
			<< "Invalid ccm size: expected "
			<< kNumColorChannels << " elements, got "
			<< ccm_.size();
		return -EINVAL;
	}

	crossTalkThreshold_ = tuningData["crosstalk-threshold"]
				      .getList<uint32_t>()
				      .value_or(std::vector<uint32_t>{});
	if (crossTalkThreshold_.size() != kNumColorChannels) {
		LOG(NxpNeoAlgoRgbIr, Error)
			<< "Invalid crosstalk-threshold size: expected "
			<< kNumColorChannels << " elements, got "
			<< crossTalkThreshold_.size();
		return -EINVAL;
	}

	ret = parseIrCompression(tuningData, "ir-compression-8b", irComp8bits_);
	if (ret)
		return ret;

	ret = parseIrCompression(tuningData, "ir-compression-16b", irComp16bits_);
	return ret;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void RgbIr::prepare([[maybe_unused]] IPAContext &context, const uint32_t frame,
		    [[maybe_unused]] IPAFrameContext &frameContext,
		    neoisp_meta_params_s *params)
{
	if (frame > 0)
		return;

	/* Head Color configuration */
	params->features_cfg.head_color_cfg = 1;

	neoisp_head_color_cfg_s *hc = &params->regs.head_color;
	hc->ctrl_hoffset = headColor_[0];
	hc->ctrl_voffset = headColor_[1];

	LOG(NxpNeoAlgoRgbIr, Debug)
		<< "Head Color hoffset " << static_cast<unsigned int>(hc->ctrl_hoffset)
		<< " voffset " << static_cast<unsigned int>(hc->ctrl_voffset);

	/* RGBIR configuration */
	params->features_cfg.rgbir_cfg = 1;

	neoisp_rgbir_cfg_s *rgbir = &params->regs.rgbir;
	*rgbir = {};

	rgbir->ctrl_enable = 1;
	rgbir->ccm0_ccm = ccm_[0];
	rgbir->ccm1_ccm = ccm_[1];
	rgbir->ccm2_ccm = ccm_[2];

	rgbir->ccm0_th_threshold = crossTalkThreshold_[0];
	rgbir->ccm1_th_threshold = crossTalkThreshold_[1];
	rgbir->ccm2_th_threshold = crossTalkThreshold_[2];

	LOG(NxpNeoAlgoRgbIr, Debug)
		<< "RGBIR ccm[0-2] "
		<< rgbir->ccm0_ccm << " "
		<< rgbir->ccm1_ccm << " "
		<< rgbir->ccm2_ccm << ", ct-threshold [0-2] "
		<< rgbir->ccm0_th_threshold << " "
		<< rgbir->ccm1_th_threshold << " "
		<< rgbir->ccm2_th_threshold;

	/* Look for 8 or 16 bits IR stream, and defaults to 8 bits */
	bool irStream16bits = false;
	std::vector<IPAStream> &streams = context.configuration.streams;
	auto iter = std::find_if(streams.begin(), streams.end(),
				 [](auto &stream) {
					 return stream.pixelFormat == formats::R16.fourcc();
				 });
	if (iter != streams.end())
		irStream16bits = true;

	/* IR Compression configuration */
	params->features_cfg.ir_compress_cfg = 1;
	neoisp_ir_compress_cfg_s *ircomp = &params->regs.ir_compress;
	RgbIr::IrCompression &comp =
		irStream16bits ? irComp16bits_ : irComp8bits_;

	ircomp->ctrl_enable = 1;
	ircomp->ctrl_obpp = irStream16bits ? 1 : 0;

	ircomp->knee_point1_kneepoint = comp.points[0];
	ircomp->knee_point2_kneepoint = comp.points[1];
	ircomp->knee_point3_kneepoint = comp.points[2];
	ircomp->knee_point4_kneepoint = comp.points[3];

	ircomp->knee_offset0_offset = comp.offsets[0];
	ircomp->knee_offset1_offset = comp.offsets[1];
	ircomp->knee_offset2_offset = comp.offsets[2];
	ircomp->knee_offset3_offset = comp.offsets[3];
	ircomp->knee_offset4_offset = comp.offsets[4];

	ircomp->knee_npoint0_kneepoint = comp.newpoints[0];
	ircomp->knee_npoint1_kneepoint = comp.newpoints[1];
	ircomp->knee_npoint2_kneepoint = comp.newpoints[2];
	ircomp->knee_npoint3_kneepoint = comp.newpoints[3];
	ircomp->knee_npoint4_kneepoint = comp.newpoints[4];

	ircomp->knee_ratio01_ratio0 = comp.ratios[0];
	ircomp->knee_ratio01_ratio1 = comp.ratios[1];
	ircomp->knee_ratio23_ratio2 = comp.ratios[2];
	ircomp->knee_ratio23_ratio3 = comp.ratios[3];
	ircomp->knee_ratio4_ratio4 = comp.ratios[4];

	LOG(NxpNeoAlgoRgbIr, Debug)
		<< "IR Compression obpp "
		<< static_cast<unsigned int>(ircomp->ctrl_obpp)
		<< " kneepoints "
		<< comp.points[0] << " " << comp.points[1] << " "
		<< comp.points[2] << " " << comp.points[3]
		<< " offsets "
		<< comp.offsets[0] << " " << comp.offsets[1] << " "
		<< comp.offsets[2] << " " << comp.offsets[3] << " "
		<< comp.offsets[4]
		<< " newpoints "
		<< comp.newpoints[0] << " " << comp.newpoints[1] << " "
		<< comp.newpoints[2] << " " << comp.newpoints[3] << " "
		<< comp.newpoints[4]
		<< " ratios "
		<< comp.ratios[0] << " " << comp.ratios[1] << " "
		<< comp.ratios[2] << " " << comp.ratios[3] << " "
		<< comp.ratios[4];
}

/**
 * \copydoc libcamera::ipa::Algorithm::parseIrCompression
 */
int RgbIr::parseIrCompression(const YamlObject &tuningData, const char *key,
			      IrCompression &irComp)
{
	const YamlObject &compObj = tuningData[key];
	if (!compObj.isDictionary() || (!compObj.size())) {
		LOG(NxpNeoAlgoRgbIr, Debug)
			<< "compression " << key << "not configured";
		return -EINVAL;
	}

	irComp.points = compObj["points"].getList<uint32_t>().value_or(std::vector<uint32_t>{});
	if (irComp.points.size() != kNumPoints) {
		LOG(NxpNeoAlgoRgbIr, Error)
			<< "compression points list size must be " << kNumPoints;
		return -EINVAL;
	}

	irComp.offsets = compObj["offsets"].getList<uint32_t>().value_or(std::vector<uint32_t>{});
	if (irComp.offsets.size() != kNumOffsets) {
		LOG(NxpNeoAlgoRgbIr, Error)
			<< "compression offsets list size must be " << kNumOffsets;
		return -EINVAL;
	}

	irComp.newpoints = compObj["newpoints"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (irComp.newpoints.size() != kNumNewPoints) {
		LOG(NxpNeoAlgoRgbIr, Error)
			<< "compression newpoints list size must be " << kNumNewPoints;
		return -EINVAL;
	}

	irComp.ratios = compObj["ratios"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (irComp.ratios.size() != kNumRatios) {
		LOG(NxpNeoAlgoRgbIr, Error)
			<< "compression ratios list size must be " << kNumRatios;
		return -EINVAL;
	}

	return 0;
}

REGISTER_IPA_ALGORITHM(RgbIr, "RgbIr")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
