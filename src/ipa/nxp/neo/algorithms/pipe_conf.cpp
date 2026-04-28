/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO PIPE_CONF configuration
 */

#include "pipe_conf.h"

#include <algorithm>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file pipe_conf.cpp
 */

namespace libcamera {

namespace ipa::nxpneo::algorithms {

/**
 * \class PipeConf
 * \brief PIPE_CONF configuration
 *
 * This Algorithm configures the PIPE_CONF unit.
 * A limited subset of the PIPE_CONF block is exposed to the user space, the
 * other bitfields remaining under sole control of the ISP driver.
 * The PIPE_CONF bitfields present in the uAPI are INALIGN0/1 and LPALIGN0/1
 * from the IMG_CONF_CAM0 register, relevant to configuration of the input0
 * and input1 paths of the ISP.
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
 * INALIGN0/1 configures, for the 10, 12, 14 and 20-bit bit depths, if the
 * significant bits should be fetched MSB or LSB-aligned from the 16-bit aligned
 * words in the DDR buffer. On i.MX95 SoC, the DDR buffers produced by the ISI
 * device have the significant data bits MSB-aligned because of hardware
 * limitation.
 *
 * LPALIGN0/1 configure for each input path how the N-bit pixel data fetched
 * from the DDR buffer will be stored into the ISP internal pipeline.
 * When using LPALIGN0/1=0, pixel data are stored internally the same as fetched.
 * Conversely, when LPALIGN0/1=1, pixel data are rescaled to be stored left-
 * shifted, with:
 *   - 20-bit MSB alignment for input0
 *   - 16-bit MSB alignment for input1
 * However there is a hardware peculiarity in the ISP (i.MX95 rev B0, i.MX952)
 * with 12-bit bit depth: rescaling for input0 and input1 is done to
 * 16-bit regardless of the PIPECONF.LPALIGN setting.
 *
 * Tables below recap the ISP internal pipeline pixel data alignment depending
 * on the input camera bit per pixel (ibpp) and the LPALIGN0/1 configuration.
 *
 * input0 (LPALIGN0)
 * +------+------------+------------+
 * | ibpp | LPALIGN0=0 | LPALIGN0=1 |
 * +------+------------+------------+
 * |  10  |     10     |     20     |
 * |  12  |     16     |     16     |
 * |  14  |     14     |     20     |
 * |  16  |     16     |     20     |
 * +------+------------+------------+
 *
 * input1 (LPALIGN1)
 * +------+------------+------------+
 * | ibpp | LPALIGN1=0 | LPALIGN1=1 |
 * +------+------------+------------+
 * |  10  |    10      |     16     |
 * |  12  |    16      |     16     |
 * |  14  |    14      |     16     |
 * +------+------------+------------+
 *
 * Relevant entries in the configuration file is a mapping of the following
 * keys:
 *   lpalign0: LPALIGN0 value (0/1)
 *   lpalign1: LPALIGN1 value (0/1)
 *   inalign0: INALIGN0 value (0/1)
 *   inalign1: INALIGN1 value (0/1)
 *
 * When LPALIGN0/1 is explicitly configured in the calibration file with above
 * entries, those are applied with priority. If not configured, the algorithm
 * falls back into automatic configuration mode using the following logic:
 * - For non HDR-merge mode of operation, configure LPALIGN0/1=1
 * - For HDR-merge mode of operation, configure LPALIGN0/1=0 to keep the native
 *   camera bit depth, as required for the HDR merge block.
 *
 * When INALIGN0/1 is explicitly configured in the calibration file with above
 * entries, those are applied with priority. If not configured, the algorithm
 * falls back into automatic configuration mode using the following logic:
 * - By default use INALIGN0/1=0 (pixel data LSB-aligned) which is V4L2 buffer
 *   standard
 * - For the platforms producing MSB-aligned pixel data because of hardware
 *   limitation, configure INALIGN0/1=1 instead
 *
 * Note: for non-linear pixel format decompression using HDR Decompression unit,
 * a bit depth lower or equal to 16-bit is required to be able to define the
 * relevant knee-points. In that case LPALIGN automatic configuration logic does
 * not apply, so LPALIGN0/1 values should be set to 0 in the calibration file.
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoPipeConf)

PipeConf::PipeConf()
	: inAlign0_(0), lpAlign0_(kLpAlignDefault),
	  inAlign1_(0), lpAlign1_(kLpAlignDefault)
{
	setIrOps(IrOpPrepare);
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int PipeConf::init([[maybe_unused]] IPAContext &context,
		   const YamlObject &tuningData)
{
	const YamlObject &lpAlign0Obj = tuningData["lpalign0"];
	lpAlign0_ = lpAlign0Obj.get<uint8_t>();
	const YamlObject &lpAlign1Obj = tuningData["lpalign1"];
	lpAlign1_ = lpAlign1Obj.get<uint8_t>();

	uint8_t inAlignAuto =
		context.hw.hwCapabilities & NEO_CAP_ALIGNMENT_MSB ? 1 : 0;
	const YamlObject &inAlign0Obj = tuningData["inalign0"];
	inAlign0_ = inAlign0Obj.get<uint8_t>().value_or(inAlignAuto);
	const YamlObject &inAlign1Obj = tuningData["inalign1"];
	inAlign1_ = inAlign1Obj.get<uint8_t>().value_or(inAlignAuto);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void PipeConf::prepare(IPAContext &context, const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       NxpNeoParams *params)
{
	if (frame > 0)
		return;

	/* PIPE_CONF unit configuration */
	auto config = params->block<BlockParamsType::PipeConf>();
	config.setEnabled(true);

	IPAPipelineMode &mode = context.configuration.pipelineMode;
	uint8_t lpAlignAuto = mode != IPAPipelineMode::HdrMerge ? 1 : 0;
	uint8_t lpAlign0 = lpAlign0_.value_or(lpAlignAuto);
	uint8_t lpAlign1 = lpAlign1_.value_or(lpAlignAuto);

	LOG(NxpNeoAlgoPipeConf, Debug)
		<< "inalign0/1 " << +inAlign0_ << "/" << +inAlign1_
		<< " lpalign0/1 " << +lpAlign0 << "/" << +lpAlign1;

	config->img_conf_inalign0 = inAlign0_;
	config->img_conf_lpalign0 = lpAlign0;
	config->img_conf_inalign1 = inAlign1_;
	config->img_conf_lpalign1 = lpAlign1;
}

REGISTER_IPA_ALGORITHM(PipeConf, "PipeConf")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
