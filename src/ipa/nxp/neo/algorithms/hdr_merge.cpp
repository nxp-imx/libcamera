/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO HDR Merge configuration
 */

#include "hdr_merge.h"

#include <algorithm>
#include <cmath>

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
 * The diagram showing the path from the two input images to the HDR-merge block
 * can be found in:
 * <src/ipa/nxp/neo/Documentation/source/neo_ipa_algorithms.rst>
 *
 * At first, image0 and image1 pixels (x,y) are scaled to the same level by
 * the gain, offset and shift parameters:
 * gimageN[x,y] = ((imageN[x,y] - gain-offset[N]) * gain-scale[N])
 * 							>> gain-shift[N]
 *     with N = <0|1> for image0 and image1
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
 *   else
 *       mluma = ((luma - luma-th0) * luma-scale) >> luma-scale-shift
 *       if (mluma > 256)
 *           mluma = 256
 *   endif
 * - spv0 = (gimage0[x,y] << upscale[0]) >> downscale[0]
 * - spv1 = (gimage1[x,y] << upscale[1]) >> downscale[1]
 * - opv = (mluma * spv1) + ((256 - mluma) * spv0)
 * - opvr = (opv + 128) >> 8
 * - opvs = opvr >> postcale
 *
 * This algorithm can operate in two modes:
 * - Auto mode: the HDR-merge parameters are computed automatically.
 * - Manual mode: the parameters are taken directly from the calibration file.
 *
 * In auto mode, the computation requires the following inputs:
 * - the HDR ratio between the two images (from the calibration file),
 * - the blending window size (from the calibration file),
 * - the bit depth of both images (available from IPA).
 * The computation is based on the algorithm considerations and hardware
 * constraints as described in:
 * <src/ipa/nxp/neo/Documentation/source/neo_ipa_algorithms.rst>.
 * The calculation of the scaling parameters (scale and shift) assumes that
 * the HDR ratio is an integer power of two. Arbitrary gain values can be used
 * but requires usage of the manual configuration mode.
 * If both low and high thresholds are equal, no blending applies and
 * image0 is output of the HDR merge block.
 * Auto mode is enabled by default, but it can be overridden in the
 * calibration file.
 *
 * Details about HDR-merge dependencies can be found in the:
 * <src/ipa/nxp/neo/Documentation/source/neo_ipa_algorithms.rst>
 *
 * The following parameters are read from the calibration file:
 *
 * - auto: flag indicating if auto mode is enabled (HDR merge parameters
 *       computed by the algorithm), otherwise the manual values are used
 *       instead.
 *       optional, default value: true (auto mode enabled)
 *
 * Parameters used for both manual and auto modes:
 * - ratio-long2short: ratio between the long capture and the short capture.
 *       This parameter is used in auto mode for the parameters computation,
 *       it is also used in both manual and auto modes for the AGC algorithm
 *       to compute the histogram scaling factor.
 *       mandatory
 * - motion-fix-en: motion correction, 1 to enable, 0 to disable
 *       optional, default value: 1
 * - blend-3x3: blending feature, 1 for 3x3 pixel array, 0 for single pixel
 *       optional, default value: 1
 *
 * Mandatory parameters used for the manual mode only (block register values):
 * - obpp: pixel bit depth at the output of the merge block, that defines the
 *       saturation level to apply.
 *       Possible values are: 0 (12 bpp), 1 (14 bpp), 2 (16 bpp) or 3 (20 bpp)
 * - gain-bpp[]: size of the pixel components after applying the gain/levelling,
 *       defining the saturation level to apply.
 *       Possible values are: 0 (12 bpp), 1 (14 bpp), 2 (16 bpp) or 3 (20 bpp)
 *       (2 entries)
 * - gain-offset[]: offset subtracted from the image, 16-bits values (2 entries)
 * - gain-scale[]: scaling factor, 16-bits values (2 entries)
 * - gain-shift[]: shift value used for a fractional gain factor,
 *       5 bits values (2 entries)
 * - luma-th0: luma threshold used for comparing the computed luma value to
 *       derive a per-pixel blending factor, 16 bits value
 * - luma-scale: scaling factor applied to the computed luma after being offset
 *       by the luma threshold, 16 bits value
 * - luma-scale-shift: shift value applied to the computed luma after being
 *       offset by the luma threshold, 5 bits value
 * - luma-scale-th-shift: right shift value applied to the computed luma value
 *       before comparison with the luma threshold, 5 bits value
 * - downscale[]: downscale shift applied to gimageN pixel value before
 *       blending, 5 bits values (2 entries)
 * - upscale[]: upscale shift applied to gimageN pixel value before blending,
 *       5 bits values (2 entries)
 * - postscale: downscale shift applied to pixel value obtained from blending,
 *       5 bits value
 *
 * Parameters used for the auto mode only:
 * - blending-window: low and high thresholds, relative to the maximum luma
 *       value of the long image, defining the range in which pixel blending
 *       between the long and short images is performed. The thresholds are
 *       expressed as a percentage of the total amplitude of image0,
 *       with values in the [0..100] range.
 *       (2 entries: low threshold, high threshold),
 *       optional, default values: 65, 95
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoHdrMerge)

HdrMerge::HdrMerge()
	: obpp_(kDefaultObppValue),
	  downscale_{ kDefaultDownscale0, kDefaultDownscale1 },
	  upscale_{ kDefaultUpscale0, kDefaultUpscale1 },
	  postscale_(kDefaultPostscale),
	  autoEnabled_(kAutoEnabled)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int HdrMerge::init([[maybe_unused]] IPAContext &context,
		   const ValueNode &tuningData)
{
	autoEnabled_ = tuningData["auto"].get<bool>().value_or(kAutoEnabled);

	LOG(NxpNeoAlgoHdrMerge, Debug) << "HDR merge auto mode "
				       << autoEnabled_;

	/* Parse tuning parameters common to both auto and manual modes. */
	int ret = parseCommonParams(tuningData);
	if (ret)
		return ret;

	context.ctrlMap[&controls::HdrMode] = ControlInfo(controls::HdrModeValues,
							  kDefaultHdrMode_);

	return autoEnabled_ ?
		parseAutoParams(tuningData) :
		parseManualParams(tuningData);
}

/**
 * \brief Parse the tuning data used for both auto and manual configurations
 * \param[in] tuningData The ValueNode representing the tuning data
 *
 * \return 0 on success or a negative error code
 */
int HdrMerge::parseCommonParams(const ValueNode &tuningData)
{
	/* Ratio is a mandatory parameter. */
	std::optional<uint16_t> ratio =
		tuningData["ratio-long2short"].get<uint16_t>();
	if (!ratio.has_value() || !isPowerOf2(ratio.value())) {
		/*
		 * If the HDR ratio is not provided in calibration file
		 * or if it is not a power‑of‑two integer, return an error.
		 */
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Missing HDR ratio or HDR ratio not a power of 2!";
		return -EINVAL;
	} else {
		ratioL2S_ = ratio.value();
	}

	const ValueNode &motionObj = tuningData["motion-fix-en"];
	motionfixEn_ = motionObj.get<uint8_t>().value_or(kDefaultMotionFixEn);

	const ValueNode &blend3x3Obj = tuningData["blend-3x3"];
	blend3x3_ = blend3x3Obj.get<uint8_t>().value_or(kDefaultBlend3x3);

	return 0;
}

/**
 * \brief Parse the tuning data used for the manual configuration
 *
 * All manual paramaters are mandatory. If one manual parameter is missing
 * an error is returned.
 *
 * \param[in] tuningData The ValueNode representing the tuning data
 *
 * \return 0 on success or a negative error code
 */
int HdrMerge::parseManualParams(const ValueNode &tuningData)
{
	/* All manual parameters are mandatory. */

	std::optional<uint8_t> obpp = tuningData["obpp"].get<uint8_t>();
	if (!obpp.has_value()) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Missing obpp parameter!";
		return -EINVAL;
	}
	obpp_ = obpp.value();

	std::optional<std::vector<uint8_t>> gainBpp =
		tuningData["gain-bpp"].get<std::vector<uint8_t>>();
	if (!gainBpp || gainBpp->size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Invalid number of gain-bpp entries";
		return -EINVAL;
	}
	gainBpp_ = std::move(gainBpp.value());

	std::optional<std::vector<uint16_t>> gainOffset =
		tuningData["gain-offset"].get<std::vector<uint16_t>>();
	if (!gainOffset || gainOffset->size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Invalid number of gain-offset entries";
		return -EINVAL;
	}
	gainOffset_ = std::move(gainOffset.value());

	std::optional<std::vector<uint16_t>> gainScale =
		tuningData["gain-scale"].get<std::vector<uint16_t>>();
	if (!gainScale || gainScale->size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Invalid number of gain-scale entries";
		return -EINVAL;
	}
	gainScale_ = std::move(gainScale.value());

	std::optional<std::vector<uint8_t>> gainShift =
		tuningData["gain-shift"].get<std::vector<uint8_t>>();
	if (!gainShift || gainShift->size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Invalid number of gain-shift entries";
		return -EINVAL;
	}
	gainShift_ = std::move(gainShift.value());

	std::optional<uint16_t> lumaTh0 =
		tuningData["luma-th0"].get<uint16_t>();
	if (!lumaTh0.has_value()) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Missing lumaTh0 parameter!";
		return -EINVAL;
	}
	lumaTh0_ = lumaTh0.value();

	std::optional<uint16_t> lumaScale =
		tuningData["luma-scale"].get<uint16_t>();
	if (!lumaScale.has_value()) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Missing lumaScale parameter!";
		return -EINVAL;
	}
	lumaScale_ = lumaScale.value();

	std::optional<uint8_t> lumaScaleShift =
		tuningData["luma-scale-shift"].get<uint8_t>();
	if (!lumaScaleShift.has_value()) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Missing lumaScaleShift parameter!";
		return -EINVAL;
	}
	lumaScaleShift_ = lumaScaleShift.value();

	std::optional<uint8_t> lumaScaleThShift =
		tuningData["luma-scale-th-shift"].get<uint8_t>();
	if (!lumaScaleThShift.has_value()) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Missing lumaScaleThShift parameter!";
		return -EINVAL;
	}
	lumaScaleThShift_ = lumaScaleThShift.value();

	std::optional<std::vector<uint8_t>> downscale =
		tuningData["downscale"].get<std::vector<uint8_t>>();
	if (!downscale || downscale->size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Invalid number of downscale entries";
		return -EINVAL;
	}
	downscale_ = std::move(downscale.value());

	std::optional<std::vector<uint8_t>> upscale =
		tuningData["upscale"].get<std::vector<uint8_t>>();
	if (!upscale || upscale->size() != kNumImages) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Invalid number of upscale entries";
		return -EINVAL;
	}
	upscale_ = std::move(upscale.value());

	std::optional<uint8_t> postscale =
		tuningData["postscale"].get<uint8_t>();
	if (!postscale.has_value()) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Missing postscale parameter!";
		return -EINVAL;
	}
	postscale_ = postscale.value();

	return 0;
}

/**
 * \brief Parse the tuning data used for the auto configuration
 * \param[in] tuningData The ValueNode representing the tuning data
 *
 * \return 0 on success or a negative error code
 */
int HdrMerge::parseAutoParams(const ValueNode &tuningData)
{
	static const std::vector<uint16_t> blendingWindowDefault =
		{ kBlendingWindowLow, kBlendingWindowHigh };
	const ValueNode &blendingWindowObj = tuningData["blending-window"];
	blendingWindow_ =
		blendingWindowObj.get<std::vector<uint16_t>>().value_or(blendingWindowDefault);
	if (blendingWindow_.size() != kNumThresholds) {
		LOG(NxpNeoAlgoHdrMerge, Error)
			<< "Invalid number of blending-window entries";
		return -EINVAL;
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int HdrMerge::configure(IPAContext &context,
			[[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	IPAPipelineMode &mode = context.configuration.pipelineMode;
	/*
	 * Set the IPA context HDR enabled flag with the HDR merge pipeline
	 * mode.
	 */
	context.configuration.hdr.enabled = mode == IPAPipelineMode::HdrMerge;
	/*
	 * Initialize the active state HDR enabled flag with the IPA context
	 * HDR enabled flag.
	 */
	context.activeState.hdr.enabled = context.configuration.hdr.enabled;

	context.configuration.hdr.ratioLong2Short = ratioL2S_;

	/* Store image0 gain scale/shift for HDR disabled (scale to 20-bit). */
	std::array<uint32_t, kNumImages> &bpps =
		context.configuration.sensor.bpps;

	int scaleShift = 20 - bpps[0];
	if (scaleShift >= 0) {
		gainScale20bitsImage0_ = 1 << scaleShift;
		gainShift20bitsImage0_ = 0;
	} else {
		gainScale20bitsImage0_ = 1;
		gainShift20bitsImage0_ = std::abs(scaleShift);
	}

	if (autoEnabled_)
		computeParams(context);

	return 0;
}

/**
 * \brief Compute the HDR merge parameters
 *
 * This function computes following parameters:
 * - the scaling domain parameters used to scale the input images values
 * - the threshold parameters used to decide if blending applies
 * - the blending parameters used to compute blended pixel values
 *
 * \param[in] context The IPA context
 */
void HdrMerge::computeParams(IPAContext &context)
{
	/*
	 * The scaling domain parameters are used to convert the images to
	 * the 20-bit HDR bit depth, accounting for the long to short ratio as
	 * well as respective SDR images bit depths.
	 * - Input0 should be scaled to the HDR ratio of 20-bit.
	 * - Input1 should be scaled to 20-bit.
	 * Scaling conversion is performed by ISP as follow
	 * gimageN[x,y] = ((imageN[x,y] - gain-offset[N]) * gain-scale[N])
	 * 							>> gain-shift[N]
	 *     with N = <0|1> for image0 and image1
	 */
	static const std::array<unsigned int, kNumImages> scaleBitDepths =
		{ 20 - static_cast<unsigned int>(std::log2(ratioL2S_)), 20 };
	std::array<uint32_t, kNumImages> &bpps =
		context.configuration.sensor.bpps;

	auto gainBpp = [](unsigned int bpp) -> unsigned int {
		if (bpp <= 12)
			return NEO_HDR_MERGE_BPP_12BPP;
		else if (bpp <= 14)
			return NEO_HDR_MERGE_BPP_14BPP;
		else if (bpp <= 16)
			return NEO_HDR_MERGE_BPP_16BPP;
		else
			return NEO_HDR_MERGE_BPP_20BPP;
	};

	for (unsigned int input = 0; input < kNumImages; input++) {
		/*
		 * Configure the saturation level to apply to each image before
		 * blending, according to their respective bit depth after
		 * rescaling.
		 */
		gainBpp_.push_back(gainBpp(scaleBitDepths[input]));

		int scaleShift = scaleBitDepths[input] - bpps[input];
		if (scaleShift >= 0) {
			/*
			 * The scaled factor is >= 1 and is an integer.
			 * Therefore, the gain shift is set to 0.
			 */
			gainScale_.push_back(1 << scaleShift);
			gainShift_.push_back(0);
		} else {
			/* The scaled factor is < 1 and is fractional. */
			gainScale_.push_back(1);
			gainShift_.push_back(std::abs(scaleShift));
		}

		/* Offset from the scaling operation is unused. */
		gainOffset_.push_back(0);
	}

	/*
	 * The ISP operation compares the pixel value against threshold values
	 * to determine whether:
	 * - the pixel value is taken from image0,
	 * - the pixel value is taken from image1, or
	 * - a blend of both images is used.
	 * This comparison is performed using values scaled in the image0 domain
	 * and shifted by the Luma Threshold shift, which is considered to be 0.
	 */

	/* Scaled factor in the image0 domain. */
	double scaledFactorImg0 = static_cast<double>(gainScale_[0]) /
				  (1 << gainShift_[0]);
	std::array<uint16_t, kNumThresholds> lumaThScaled;

	lumaScaleThShift_ = kDefaultLumaScaleThShift;
	for (unsigned int i = 0; i < kNumThresholds; i++) {
		double lumaThValue =
			static_cast<double>(blendingWindow_[i] << bpps[0]) /
			100.0;
		lumaThScaled[i] =
			static_cast<uint32_t>(std::round(
				lumaThValue * scaledFactorImg0)) >>
			lumaScaleThShift_;
	}
	lumaTh0_ = lumaThScaled[0];

	/*
	 * Blending factor mluma is calculated as below with register value
	 *       mluma = ((luma - luma-th0) * luma-scale) >> luma-scale-shift
	 * When luma reaches lumaTh1, the blending factor is 256 (mluma=256)
	 *     256 = ((luma-th1 - luma-th0) * luma-scale) >> luma-scale-shift
	 *  => luma_scale = 256 * (1 << luma-scale-shift)/(luma-th1 - luma-th0)
	 *        with luma-scale-shift a 5-bits value
	 *             luma_scale a 16-bits value
	 * If both low and high thresholds are equal, no blending applies and
	 * image0 is output of the HDR merge block.
	 */
	unsigned long lumaScale = 0;
	uint16_t lumaThDiff = lumaThScaled[1] - lumaThScaled[0];
	if (!lumaThDiff) {
		lumaScale_ = 0;
		lumaScaleShift_ = 0;
		return;
	}
	for (lumaScaleShift_ = 0; lumaScaleShift_ < 32; lumaScaleShift_++) {
		lumaScale = ((1 << lumaScaleShift_) / lumaThDiff) *
			    kBlendingFactorMax;
		if (lumaScale > std::numeric_limits<std::uint16_t>::max())
			break;
		lumaScale_ = static_cast<uint16_t>(lumaScale);
	}
	lumaScaleShift_--;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void HdrMerge::queueRequest(IPAContext &context,
			    const uint32_t frame,
			    IPAFrameContext &frameContext,
			    const ControlList &controls)
{
	if (!context.configuration.hdr.enabled)
		return;

	auto &hdr = context.activeState.hdr;
	const auto &hdrMode = controls.get(controls::HdrMode);

	/* Force the static configuration for the first frame. */
	if (!frame) {
		frameContext.hdr.update = true;
		hdr.userMode = kDefaultHdrMode_;
	}

	if (hdrMode) {
		if (*hdrMode != controls::HdrModeOff &&
		    *hdrMode != controls::HdrModeMultiExposure) {
			LOG(NxpNeoAlgoHdrMerge, Warning)
				<< " Values of controls::HdrMode " << *hdrMode
				<< " is not supported";
			return;
		} else if ((*hdrMode == controls::HdrModeOff && hdr.enabled) ||
			   (*hdrMode == controls::HdrModeMultiExposure && !hdr.enabled)) {
			hdr.enabled = *hdrMode != controls::HdrModeOff;

			LOG(NxpNeoAlgoHdrMerge, Debug)
				<< (hdr.enabled ? "Enabling" : "Disabling")
				<< " HDR";
			frameContext.hdr.update = true;
			hdr.userMode = *hdrMode;
		}
	}

	frameContext.hdr.enabled = hdr.enabled;
	frameContext.hdr.userMode = hdr.userMode;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void HdrMerge::prepare([[maybe_unused]] IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       NxpNeoParams *params)
{
	if (!frameContext.hdr.update)
		return;

	/* HDR Merge block configuration */
	auto config = params->block<BlockParamsType::HdrMerge>();
	config.setEnabled(true);

	/*
	 * If HDR merge is disabled by user controls, the HDR blending is
	 * disabled and only the image0 is provided as the output of the
	 * HDR merge block.
	 * For this, the image0 is scaled to 20-bit.
	 */
	config->ctrl_obpp = obpp_;
	config->ctrl_motion_fix_en = motionfixEn_;
	config->ctrl_blend_3x3 = blend3x3_;
	/* If HDR is disabled, saturate image0 bit depth to 20-bit. */
	config->ctrl_gain0bpp = frameContext.hdr.enabled ?
		gainBpp_[0] :
		static_cast<unsigned int>(NEO_HDR_MERGE_BPP_20BPP);
	config->ctrl_gain1bpp = gainBpp_[1];

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "obpp " << static_cast<int>(config->ctrl_obpp)
		<< " motion_fix_en "
		<< static_cast<int>(config->ctrl_motion_fix_en)
		<< " blend_3x3 " << static_cast<int>(config->ctrl_blend_3x3)
		<< " gain bpp (0/1) " << static_cast<int>(config->ctrl_gain0bpp)
		<< "/" << static_cast<int>(config->ctrl_gain1bpp);

	config->gain_offset_offset0 = gainOffset_[0];
	config->gain_offset_offset1 = gainOffset_[1];

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "gain offset (0/1) "
		<< utils::hex(config->gain_offset_offset0)
		<< "/" << utils::hex(config->gain_offset_offset1);

	/* If HDR is disabled, scale image0 to 20-bit. */
	config->gain_scale_scale0 =
		frameContext.hdr.enabled ? gainScale_[0] : gainScale20bitsImage0_;
	config->gain_scale_scale1 = gainScale_[1];

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "gain scale (0/1) " << utils::hex(config->gain_scale_scale0)
		<< "/" << utils::hex(config->gain_scale_scale1);

	config->gain_shift_shift0 =
		frameContext.hdr.enabled ? gainShift_[0] : gainShift20bitsImage0_;
	config->gain_shift_shift1 = gainShift_[1];

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "gain shift (0/1) "
		<< static_cast<int>(config->gain_shift_shift0)
		<< "/" << static_cast<int>(config->gain_shift_shift1);

	/*
	 * If HDR is disabled, disable blending and output image0 only.
	 * For this, the luma threshold is set to the highest value of the
	 * 20-bit range.
	 */
	config->luma_th_th0 =
		frameContext.hdr.enabled ? lumaTh0_ : (1 << 20) - 1;
	config->luma_scale_scale = lumaScale_;
	config->luma_scale_shift = lumaScaleShift_;
	config->luma_scale_thshift = lumaScaleThShift_;

	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "luma th0 " << utils::hex(config->luma_th_th0)
		<< " luma scale " << utils::hex(config->luma_scale_scale)
		<< " luma shift " << static_cast<int>(config->luma_scale_shift)
		<< " luma th shift "
		<< static_cast<int>(config->luma_scale_thshift);

	config->downscale_imgscale0 = downscale_[0];
	config->downscale_imgscale1 = downscale_[1];
	config->upscale_imgscale0 = upscale_[0];
	config->upscale_imgscale1 = upscale_[1];
	config->post_scale_scale = postscale_;
	LOG(NxpNeoAlgoHdrMerge, Debug)
		<< "downscale (0/1) "
		<< static_cast<int>(config->downscale_imgscale0)
		<< "/" << static_cast<int>(config->downscale_imgscale1)
		<< " upscale (0/1) "
		<< static_cast<int>(config->upscale_imgscale0)
		<< "/" << static_cast<int>(config->upscale_imgscale1)
		<< " postscale " << static_cast<int>(config->post_scale_scale);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void HdrMerge::process([[maybe_unused]] IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       [[maybe_unused]] const NxpNeoStats *stats,
		       ControlList &metadata)
{
	metadata.set(controls::HdrMode, frameContext.hdr.userMode);
}

REGISTER_IPA_ALGORITHM(HdrMerge, "HdrMerge")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
