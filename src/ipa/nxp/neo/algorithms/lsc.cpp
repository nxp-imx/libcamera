/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Lens Shading Correction control algorithm
 *     src/ipa/rkisp1/algorithms/lsc.cpp
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * lsc.cpp NXP NEO Lens Shading Correction control
 * Copyright 2025 NXP
 */

#include "lsc.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <linux/nxp_neoisp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

/**
 * \file lsc.cpp
 */

namespace libcamera {

namespace ipa {

constexpr int kColourTemperatureChangeThreshhold = 10;
/* The vignetting LUT combines factors for red, green and blue channels */
constexpr int kChannelLutSize = NEO_VIGNETTING_TABLE_SIZE / 3;

template<typename T>
void interpolateVector(const std::vector<T> &a, const std::vector<T> &b,
		       std::vector<T> &dest, double lambda)
{
	assert(a.size() == b.size());
	dest.resize(a.size());
	for (size_t i = 0; i < a.size(); i++) {
		dest[i] = a[i] * (1.0 - lambda) + b[i] * lambda;
	}
}

template<>
void Interpolator<nxpneo::algorithms::LensShadingCorrection::Components>::
	interpolate(const nxpneo::algorithms::LensShadingCorrection::Components &a,
		    const nxpneo::algorithms::LensShadingCorrection::Components &b,
		    nxpneo::algorithms::LensShadingCorrection::Components &dest,
		    double lambda)
{
	interpolateVector(a.r, b.r, dest.r, lambda);
	interpolateVector(a.g, b.g, dest.g, lambda);
	interpolateVector(a.b, b.b, dest.b, lambda);
}
} /* namespace ipa */

namespace ipa::nxpneo::algorithms {

/**
 * \class LensShadingCorrection
 * \brief NXP NEO Lens Shading Correction control
 *
 * Due to the optical characteristics of the lens, the light intensity received
 * by the sensor is not uniform.
 *
 * The Lens Shading Correction algorithm applies multipliers to all pixels
 * to compensate for the lens shading effect. The coefficients are
 * specified with a set of 3 LUTs [r, g, b] for each color channel in the YAML
 * tuning file.
 * Each coefficient is in 16 bits (u3.7) and each color channel LUT contains
 * 1024 coefficients (3072/3).
 *
 * Relevant parameters in the YAML calibration file are:
 * - "resolution": Sensor resolution associated with following parameters
 * - "block-count": Number of blocks used to partition the image
 * - "sets": Set of LUT entries composed of "r"/"g"/"b" channels associated
 *   with a specific "ct" color temperature.
 *
 * The ISP is partitionning the image in blocks.
 * Each LUT entry is mapped to each block of the image.
 * Before applying the LUT entry to the pixels of the block, the LUT coefficient
 * is converted into a factor following a 3-tap horizontal and vertical interpolation.
 * The horizontal interpolation applies for each row as below:
 * hFactor = 0.5 * (1 – alpha) * left_LUT + 0.5 * current_LUT + 0.5 * (alpha) * right_LUT
 *   where left_LUT: entry of the LUT of the left neighbor block,
 *         right_LUT: entry of the LUT of the right neighbor block,
 *         current_LUT: entry of the LUT of the block where the pixel being interpolated is located.
 *         alpha = (n * step / 32768),
 *           with n: the position of the pixel within the current block,
 *                step = 32768 / block_size (scaling factor defined in u1.15),
 *                block_size = image_size / block_count
 * The same vertical interpolation applies to the results of the block row interpolations.
 * At the edge of the image, the value of the missing neighbor is taken as equal to the
 * boundary valid value.
 * If the resolution is not a multiple of the block count, the block size is
 * rounded up to the nearest integer to ensure covering all the pixels of the image.
 * In this case, the last block in a row (or column) will only get interpolated up to
 * the last pixel position.
 *
 * The maximum number of blocks supported is 1024.
 * If the image is partionned with less than 1024 blocks, the latest remaining
 * LUT entries are not used.
 *
 * Each LUT is defined for a color temperature.
 * The LUT is interpolated according to the measured color temperature.
 * Hence the LSC algorithm depends on the AWB algorithm which is measuring
 * the color temperature. For this reason, the LSC algorithm should run after the
 * AWB algorithm.
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoLsc)

class LscTableLoader
{
public:
	int parseLscData(const YamlObject &yamlProfile,
			 std::map<unsigned int, LensShadingCorrection::Components> &lscData)
	{
		/* Get all defined sets to apply. */
		const YamlObject &yamlSets = yamlProfile["sets"];
		if (!yamlSets.isList()) {
			LOG(NxpNeoAlgoLsc, Warning)
				<< "'sets' parameter not found in tuning file";
			return -EINVAL;
		}

		const auto &sets = yamlSets.asList();

		for (const auto &yamlSet : sets) {
			uint32_t ct = yamlSet["ct"].get<uint32_t>(0);

			if (lscData.count(ct)) {
				LOG(NxpNeoAlgoLsc, Error)
					<< "Multiple sets found for color temperature "
					<< ct;
				return -EINVAL;
			}

			LensShadingCorrection::Components &set = lscData[ct];

			set.ct = ct;
			set.r = parseLut(yamlSet, "r");
			set.g = parseLut(yamlSet, "g");
			set.b = parseLut(yamlSet, "b");

			if (set.r.empty() || set.g.empty() || set.b.empty()) {
				LOG(NxpNeoAlgoLsc, Error)
					<< "Set for color temperature " << ct
					<< " is missing lut table";
				return -EINVAL;
			}
		}

		return 0;
	}
	const std::optional<LensShadingCorrection::BlockCount> parseBlockCnt(
		const YamlObject &yamlProfile) const
	{
		std::vector<uint16_t> blockCnt = yamlProfile["block-count"].getList<uint16_t>()
					.value_or(std::vector<uint16_t>{});
		if (blockCnt.size() != 2) {
			LOG(NxpNeoAlgoLsc, Error)
				<< "Invalid block count size which should be composed of "
				<< "{horizontal block count; vertical block count}.";
			return std::nullopt;
		}
		return std::make_pair(blockCnt[0], blockCnt[1]);
	}

private:
	std::vector<uint16_t> parseLut(const YamlObject &tuningData,
				       const char *prop)
	{
		std::vector<uint16_t> lut =
			tuningData[prop].getList<uint16_t>().value_or(std::vector<uint16_t>{});
		if (lut.size() != kChannelLutSize) {
			LOG(NxpNeoAlgoLsc, Error)
				<< "Invalid '" << prop << "' values: expected "
				<< NEO_VIGNETTING_TABLE_SIZE / 3
				<< " elements, got " << lut.size();
			return {};
		}

		return lut;
	}
};

LensShadingCorrection::LensShadingCorrection()
	: status_(NOT_CONFIGURED), lastAppliedCt_(0), lastAppliedQuantizedCt_(0)
{
	sets_.setQuantization(kColourTemperatureChangeThreshhold);
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int LensShadingCorrection::init([[maybe_unused]] IPAContext &context,
				const YamlObject &tuningData)
{
	auto loader = LscTableLoader();

	/* Get all defined profiles. */
	const YamlObject &yamlProfiles = tuningData["profiles"];
	if (!yamlProfiles.isList()) {
		LOG(NxpNeoAlgoLsc, Error)
			<< "'profiles' parameter not found in tuning file";
		return -EINVAL;
	}

	const auto &profiles = yamlProfiles.asList();
	for (const auto &yamlProfile : profiles) {
		std::optional<Size> resolution = yamlProfile["resolution"].get<Size>();
		if (!resolution.has_value())
			break;

		/* Parse all block count configurations. */
		blockCntMap_[resolution.value()] = loader.parseBlockCnt(yamlProfile);

		/* Parse all vignetting LUT configurations. */
		std::map<unsigned int, Components> lscData;
		if (loader.parseLscData(yamlProfile, lscData))
			LOG(NxpNeoAlgoLsc, Warning)
				<< "'sets' parameter not defined in tuning file for "
				<< resolution.value();
		else
			setsMap_[resolution.value()].setData(std::move(lscData));
	}
	if (blockCntMap_.empty()) {
		LOG(NxpNeoAlgoLsc, Error) << "Failed to load any block count";
		return -EINVAL;
	}
	if (setsMap_.empty()) {
		LOG(NxpNeoAlgoLsc, Error) << "Failed to load any sets";
		return -EINVAL;
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int LensShadingCorrection::configure([[maybe_unused]] IPAContext &context,
				     [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	/* clear lastAppliedCt_ and lastAppliedQuantizedCt_ */
	lastAppliedCt_ = 0;
	lastAppliedQuantizedCt_ = 0;
	status_ = NOT_CONFIGURED;

	/* Get the block count according to the sensor resolution */
	std::optional<BlockCount> bc = blockCount(configInfo.outputSize);
	if (!bc.has_value()) {
		/* Lsc is disabled for this resolution */
		LOG(NxpNeoAlgoLsc, Warning) << "LSC is disabled: block count value for "
					    << configInfo.outputSize
					    << " not found in tuning file.";
		return 0;
	}
	blockCountX_ = bc.value().first;
	blockCountY_ = bc.value().second;
	blockWidth_ = ceil(configInfo.outputSize.width /
			   static_cast<float>(blockCountX_));
	blockHeight_ = ceil(configInfo.outputSize.height /
			    static_cast<float>(blockCountY_));
	/* Scaling step factor (u1.15) */
	blockStepX_ = floor(kScalingFractionalSize / static_cast<float>(blockWidth_));
	blockStepY_ = floor(kScalingFractionalSize / static_cast<float>(blockHeight_));

	LOG(NxpNeoAlgoLsc, Debug) << "blockCount=[" << blockCountX_
				  << ", " << blockCountY_
				  << "], blockSize=[" << blockWidth_
				  << ", " << blockHeight_
				  << "], step=[" << blockStepX_
				  << ", " << blockStepY_ << "]";

	/* Get the LUT sets according to the sensor resolution */
	sets_ = sets(configInfo.outputSize);
	if (sets_.data().empty()) {
		/* Lsc is disabled for this resolution */
		LOG(NxpNeoAlgoLsc, Warning) << "LSC is disabled: Sets for "
					    << configInfo.outputSize
					    << " not found in tuning file";
		return 0;
	}

	status_ = CONFIGURED;

	return 0;
}

void LensShadingCorrection::copyTable(neoisp_vignetting_table_mem_params_s &vt,
				      const Components &set)
{
	/*
	 * The vignetting LUT combines factors starting for red channel,
	 * followed by green channel and then followed by blue channels.
	 */
	std::copy(set.r.begin(), set.r.end(), &vt.vignetting_table[0]);
	std::copy(set.g.begin(), set.g.end(), &vt.vignetting_table[kChannelLutSize]);
	std::copy(set.b.begin(), set.b.end(), &vt.vignetting_table[2 * kChannelLutSize]);
}

const std::optional<LensShadingCorrection::BlockCount>
LensShadingCorrection::blockCount(Size resolution) const
{
	auto iter = blockCntMap_.find(resolution);

	if (iter != blockCntMap_.end())
		return iter->second;

	LOG(NxpNeoAlgoLsc, Warning) << "No parsed block count for " << resolution;
	return std::nullopt;
}

const ipa::Interpolator<LensShadingCorrection::Components>
LensShadingCorrection::sets(Size resolution) const
{
	auto iter = setsMap_.find(resolution);

	if (iter != setsMap_.end())
		return iter->second;

	LOG(NxpNeoAlgoLsc, Warning) << "No parsed sets for " << resolution;
	return {};
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void LensShadingCorrection::prepare(IPAContext &context,
				    [[maybe_unused]] const uint32_t frame,
				    [[maybe_unused]] IPAFrameContext &frameContext,
				    neoisp_meta_params_s *params)
{
	if (status_ == NOT_CONFIGURED)
		/* No Lsc is configured for current context */
		return;

	uint32_t ct = context.activeState.awb.temperatureK;
	if (std::abs(static_cast<int>(ct) - static_cast<int>(lastAppliedCt_)) <
	    kColourTemperatureChangeThreshhold)
		return;

	unsigned int quantizedCt;
	const Components &set = sets_.getInterpolated(ct, &quantizedCt);
	LOG(NxpNeoAlgoLsc, Debug)
		<< "frame=" << frame << " ct=" << ct
		<< " lastAppliedQuantizedCt_=" << lastAppliedQuantizedCt_
		<< " quantizedCt=" << quantizedCt;

	if (lastAppliedQuantizedCt_ == quantizedCt)
		return;

	if (status_ != ENABLED) {
		params->features_cfg.vignetting_ctrl_cfg = 1;
		params->regs.vignetting_ctrl.ctrl_enable = 1;
		params->regs.vignetting_ctrl.blk_conf_cols = blockCountX_;
		params->regs.vignetting_ctrl.blk_conf_rows = blockCountY_;
		params->regs.vignetting_ctrl.blk_size_xsize = blockWidth_;
		params->regs.vignetting_ctrl.blk_size_ysize = blockHeight_;
		params->regs.vignetting_ctrl.blk_stepx_step = blockStepX_;
		params->regs.vignetting_ctrl.blk_stepy_step = blockStepY_;

		LOG(NxpNeoAlgoLsc, Debug) << "Lsc is enabled";
		status_ = ENABLED;
	}

	params->features_cfg.vignetting_table_cfg = 1;
	copyTable(params->mems.vt, set);

	lastAppliedCt_ = ct;
	lastAppliedQuantizedCt_ = quantizedCt;

	LOG(NxpNeoAlgoLsc, Debug)
		<< "ct is " << ct << ", quantized to "
		<< quantizedCt;
}

REGISTER_IPA_ALGORITHM(LensShadingCorrection, "LensShadingCorrection")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
