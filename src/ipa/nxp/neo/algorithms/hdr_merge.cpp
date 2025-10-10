/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * hdr_merge.cpp - NXP NEO HDR Merge configuration
 * Copyright 2025 NXP
 */

#include "hdr_merge.h"

#include <algorithm>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file hdr_merge.cpp
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class HdrMerge
 * \brief HDR Merge unit configuration
 *
 * This block when enabled combines the pixels of the two images of line path 0
 * and line path 1 into a single output.
 *
 *       input0              input1
 *     AXI IN0 DMA         AXI IN1 DMA
 *          │                   │
 *  ┌───────▼───────────────────▼───────┐
 *  │ PIPECONF                          │
 *  │    LPALIGN0             LPALIGN1  │
 *  │    INALIGN0             INALIGN1  │
 *  └───────┬───────────────────┬───────┘
 *  ┌───────▼───────┐   ┌───────▼───────┐
 *  │      HC0      │   │      HC1      │
 *  └───────┬───────┘   └───────┬───────┘
 *  ┌───────▼───────┐   ┌───────▼───────┐
 *  │  HDR Decomp0  │   │  HDR Decomp1  │
 *  └───────┬───────┘   └───────┬───────┘
 *  ┌───────▼───────┐   ┌───────▼───────┐
 *  │     OBWB0     │   │     OBWB1     │
 *  └───────┬───────┘   └───────┬───────┘
 *  ┌───────▼───────────────────▼───────┐
 *  │             HDR Merge             │
 *  └─────────────────┬─────────────────┘
 *  ┌─────────────────▼─────────────────┐
 *  │               RGBIR               │
 *  └───────┬───────────────────┬───────┘
 *  ┌───────▼───────┐           │
 *  │     OBWB2     │           │
 *  └───────┬───────┘           │
 *          ▼                   ▼
 *      to RGB Path        to IR path
 *
 * At first, image0 and image1 pixels (x,y) are scaled to the the same level by
 * the gain, offset and shift parameters:
 * gimageN[x,y] = ((imageN[x,y] - gain-offset[N]) * gain-scale[])
 * 							>> gain-shift[N]
 *     with N = <0|1> for image0 and image1
 * Relevant parameters in the calibration file are:
 * - gain-offset[]: offset substracted from the image, 16-bits values (2 entries)
 * - gain-scale[]: scaling factor, 16-bits values (2 entries)
 * - gain-shift[]: shift value, 5 bits values (2 entries)
 *
 * Per (x,y) pixel processing is then applied to those scaled images gimage0 and
 * gimage1:
 * - Approximated luminance is computed with a 3x3 binomial filter on gimage0
 * - This computed luminance is compared to a threshold to detect an
 *       overexposed pixel and to derive a configurable blending factor for
 *       gimage0 and gimage1 pixels
 * - gimage0 and gimage1 pixels are rescaled before blending
 * - Blended pixel value is interpolated from above scaled pixels, according to
 *       the blending factor
 * - Resulting blended pixel value is post scaled before storage
 *
 * The (x,y) pixel processing can be modelled by the steps below:
 * - luma = binomial3x3(gimage0[x,y]) >> luma-scale-th-shift
 * - if (luma < luma-th0)
 *       mluma = 0
 *   else if (luma > luma-th0)
 *       mluma = 1
 *   else
 *       mluma = ((luma - luma-th0) * luma-scale) >> luma-scale-shift
 *   endif
 * - spv0 = (gimage0[x,y] << upscale[0]) >> downscale[0]
 * - spv1 = (gimage1[x,y] << upscale[1]) >> downscale[1]
 * - opv = (mluma * spv1) + ((256 - mluma) * spv0)
 * - opvr = (opv + 128) >> 8
 * - opvs = opvr >> postcale
 *
 * The relevant parameters from the calibration file are:
 * - luma-scale-th-shift: right shift value applied to the computed luma value
 *       before comparison with the luma threshold, 5 bits value
 * - luma-th0: luma threshold used for comparing the computed luma value to
 *       derive a per-pixel blending factor, 16 bits value
 * - luma-scale: scaling factor applied to the computed luma after being offset
 *       by the luma threshold, 16 bits value
 * - luma-scale-shift: shift value applied to the computed luma after being
 *       offset by the luma threshold, 5 bits value
 * - downscale[]: downscale shift applied to gimageN pixel value before
 *       blending, 5 bits values (2 entries)
 * - upscale[]: upscale shift applied to gimageN pixel value before blending,
 *       5 bits values (2 entries)
 * - postscale: downscale shift applied to pixel value obtained from blending,
 *       5 bits value
 *
 * Other configurable values are:
 * - obpp: pixel fomat at the output of the merge block, that defines the
 *       saturation level to apply. Possible values are:
 *       0 (12 bpp), 1 (14 bpp), 2 (16 bpp) or 3 (20 bpp)
 * - motion-fix-en: motion correction, 1 to enable, 0 to disable
 * - gainbpp[]: size of the pixel components after applying the gain/leveling,
 *       defining the saturation level to apply. Possible values are:
 *       0 (12 bpp), 1 (14 bpp), 2 (16 bpp) or 3 (20 bpp)
 *       (2 entries)
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoHdrMerge)

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int HdrMerge::init([[maybe_unused]] IPAContext &context,
		   const YamlObject &tuningData)
{
	obpp_ = tuningData["obpp"].get<uint8_t>().value_or(kDefaultObppValue);
	motionfixEn_ = tuningData["motion-fix-en"].get<uint8_t>().value_or(kDefaultMotionFixEn);
	blend3x3_ = tuningData["blend-3x3"].get<uint8_t>().value_or(kDefaultBlend3x3);
	std::vector<uint8_t> gainBppDefault = { kDefaultGainBpp0, kDefaultGainBpp1 };
	gainBpp_ = tuningData["gain-bpp"].getList<uint8_t>().value_or(gainBppDefault);
	if (gainBpp_.size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error) << "Invalid number of gain-bpp entries";
		return -EINVAL;
	}

	std::vector<uint16_t> gainOffsetDefault = { kDefaultGainOffset0, kDefaultGainOffset1 };
	gainOffset_ = tuningData["gain-offset"].getList<uint16_t>().value_or(gainOffsetDefault);
	if (gainOffset_.size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error) << "Invalid number of gain-offset entries";
		return -EINVAL;
	}
	std::vector<uint16_t> gainScaleDefault = { kDefaultGainScale0, kDefaultGainScale1 };
	gainScale_ = tuningData["gain-scale"].getList<uint16_t>().value_or(gainScaleDefault);
	if (gainScale_.size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error) << "Invalid number of gain-scale entries";
		return -EINVAL;
	}
	std::vector<uint8_t> gainShiftDefault = { kDefaultGainShift0, kDefaultGainShift1 };
	gainShift_ = tuningData["gain-shift"].getList<uint8_t>().value_or(gainShiftDefault);
	if (gainShift_.size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error) << "Invalid number of gain-shift entries";
		return -EINVAL;
	}

	lumaTh0_ = tuningData["luma-th0"].get<uint16_t>().value_or(kDefaultLumaTh0);
	lumaScale_ = tuningData["luma-scale"].get<uint16_t>().value_or(kDefaultLumaScale);
	lumaScaleShift_ =
		tuningData["luma-scale-shift"].get<uint8_t>().value_or(kDefaultLumaScaleShift);
	lumaScaleThShift_ =
		tuningData["luma-scale-th-shift"].get<uint8_t>().value_or(kDefaultLumaScaleThShift);

	std::vector<uint8_t> downscaleDefault = { kDefaultDownscale0, kDefaultDownscale1 };
	downscale_ = tuningData["downscale"].getList<uint8_t>().value_or(downscaleDefault);
	if (downscale_.size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error) << "Invalid number of downscale entries";
		return -EINVAL;
	}
	std::vector<uint8_t> upscaleDefault = { kDefaultUpscale0, kDefaultUpscale1 };
	upscale_ = tuningData["upscale"].getList<uint8_t>().value_or(upscaleDefault);
	if (upscale_.size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error) << "Invalid number of upscale entries";
		return -EINVAL;
	}
	postscale_ = tuningData["postscale"].get<uint8_t>().value_or(kDefaultPostscale);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int HdrMerge::configure(IPAContext &context,
			[[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	IPAModeType &mode = context.configuration.pipelineMode;
	enabled_ = mode == IPAModeTypeHdrMerge;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void HdrMerge::prepare([[maybe_unused]] IPAContext &context, const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       NxpNeoParams *params)
{
	if (!enabled_ || frame > 0)
		return;

	/* HDR Merge block configuration */
	auto config = params->block<BlockParamsType::HdrMerge>();
	config.setUpdate(true);

	config->ctrl_enable = 1;
	config->ctrl_obpp = obpp_;
	config->ctrl_motion_fix_en = motionfixEn_;
	config->ctrl_blend_3x3 = blend3x3_;
	config->ctrl_gain0bpp = gainBpp_[0];
	config->ctrl_gain1bpp = gainBpp_[1];

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "obpp " << static_cast<int>(config->ctrl_obpp)
		<< " motion_fix_en " << static_cast<int>(config->ctrl_motion_fix_en)
		<< " blend_3x3 " << static_cast<int>(config->ctrl_blend_3x3)
		<< " gain bpp (0/1) " << static_cast<int>(config->ctrl_gain0bpp)
		<< "/" << static_cast<int>(config->ctrl_gain0bpp);

	config->gain_offset_offset0 = gainOffset_[0];
	config->gain_offset_offset1 = gainOffset_[1];

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "gain offset (0/1) " << utils::hex(config->gain_offset_offset0)
		<< "/" << utils::hex(config->gain_offset_offset1);

	config->gain_scale_scale0 = gainScale_[0];
	config->gain_scale_scale1 = gainScale_[1];

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "gain scale (0/1) " << utils::hex(config->gain_scale_scale0)
		<< "/" << utils::hex(config->gain_scale_scale1);

	config->gain_shift_shift0 = gainShift_[0];
	config->gain_shift_shift1 = gainShift_[1];

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "gain shift (0/1) " << static_cast<int>(config->gain_shift_shift0)
		<< "/" << static_cast<int>(config->gain_shift_shift1);

	config->luma_th_th0 = lumaTh0_;
	config->luma_scale_scale = lumaScale_;
	config->luma_scale_shift = lumaScaleShift_;
	config->luma_scale_thshift = lumaScaleThShift_;

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "luma th0 " << utils::hex(config->luma_th_th0)
		<< " luma scale " << utils::hex(config->luma_scale_scale)
		<< " luma shift " << static_cast<int>(config->luma_scale_shift)
		<< " luma th shift " << static_cast<int>(config->luma_scale_thshift);

	config->downscale_imgscale0 = downscale_[0];
	config->downscale_imgscale1 = downscale_[1];
	config->upscale_imgscale0 = upscale_[0];
	config->upscale_imgscale1 = upscale_[1];
	config->post_scale_scale = postscale_;
	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "downscale (0/1) " << static_cast<int>(config->downscale_imgscale0)
		<< "/" << static_cast<int>(config->downscale_imgscale1)
		<< " upscale (0/1) " << static_cast<int>(config->upscale_imgscale0)
		<< "/" << static_cast<int>(config->upscale_imgscale1)
		<< " postscale " << static_cast<int>(config->post_scale_scale);
}

REGISTER_IPA_ALGORITHM(HdrMerge, "HdrMerge")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
