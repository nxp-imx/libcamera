/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * goc.h NXP NEO Gamma out control
 * Copyright 2025 NXP
 */

#pragma once

#include <map>

#include <libcamera/color_space.h>

#include "libcamera/internal/matrix.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class GammaOutCorrection : public Algorithm
{
public:
	GammaOutCorrection() {}
	~GammaOutCorrection() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     NxpNeoParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const NxpNeoStats *stats,
		     ControlList &metadata) override;

	struct YCbCrEnc {
		/* YCbCr encoding matrix */
		Matrix<float, 3, 3> matrix;
		/* YCbCr encoding offsets in 8-bits format */
		Matrix<uint32_t, 3, 1> offsets;
	};

	/*
	 * Transfer function formula:
	 * L' = LinearGain * L						(for R <= LinearThreshold)
	 * L' = NonLinearGain * (L^(GammaInverse) - NonLinearOffset)	(for R > LinearThreshold)
	 */
	struct XferFunc {
		float linearGain;
		float linearThreshold;
		float nonLinearGain;
		float nonLinearOffset;
		float gammaInverse;
	};
private:
	void setYuv2RgbParams(neoisp_gcm_cfg_s &gcm) const;
	void setXferParams(neoisp_gcm_cfg_s &gcm) const;
	void setEncodingParams(neoisp_gcm_cfg_s &gcm, const IPARange range) const;
	uint16_t encOffsetsToParams(uint8_t offset) const { return offset << (12 - 8); }
	float validateGamma(float gamma) const
	{
		/*
		 * Clamp to gamma s1.8 format from ISP:
		 * min is 1 / 256
		 * max is (2^9-1) / 256
		*/
		return std::clamp(gamma, 1.0f / 256, 511.0f / 256);
	}
	float quantizedOmat(int16_t omat, float factor) const
	{
		/* Compensate quantized omat with the xfer non-linear gain. */
		return (omat * factor * xferFunc_.nonLinearGain);
	}
	std::string colorSpaceName(IPAColorSpace colorSpace) const
	{
		return ColorSpace(static_cast<ColorSpace::Primaries>(colorSpace.primaries),
				  static_cast<ColorSpace::TransferFunction>(colorSpace.transferFunction),
				  static_cast<ColorSpace::YcbcrEncoding>(colorSpace.ycbcrEncoding),
				  static_cast<ColorSpace::Range>(colorSpace.range))
			.toString();
	}
	std::optional<float> gamma_;
	XferFunc xferFunc_;
	YCbCrEnc ycbcrEnc_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
