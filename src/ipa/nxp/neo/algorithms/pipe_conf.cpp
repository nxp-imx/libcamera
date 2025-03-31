/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * pipe_conf.cpp - NXP NEO PIPE_CONF configuration
 * Copyright 2025 NXP
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
 * other bitfields remain under sole control of the ISP driver.
 * The PIPE_CONF bitfields present in the uAPI are INALIGN0/1 and LPALIGN0/1
 * from the IMG_CONF_CAM0 register, relevant to configuration of the input0
 * and input1 paths of the ISP.
 *
 * INALIGN0/1 defines for the 10, 12 and 14-bit pixel formats if the significant
 * bits should be fetched MSB or LSB-aligned from the 16-bit aligned words in
 * the DDR buffer. On i.MX95 SoC, the DDR buffers produced by the ISI device
 * have the significant data bits MSB-aligned because of an hardware limitation.
 * INALIGN0/1 are currently not exposed to the calibration file as they are
 * related to the underlying SoC - as of now, intent is to keep the calibration
 * file independent from the hardware.
 *
 * LPALIGN0/1 configure for each input path how the N-bit pixel data fetched
 * from the DDR buffer will be stored into the ISP internal pipeline. When using
 * LPALIGN0/1=0, pixel data are stored internally the same as fetched.
 * Conversely, when LPALIGN0/1=1, pixel data are rescaled to be stored left-
 * shifted, with an MSB alignment that depends on the ISP hardware revision:
 *   - ISP rev1 (i.MX95 rev A0/A1):
 *       input0: 20-bit MSB alignment
 *       input1: 16-bit MSB alignment
 *   - ISP rev2 (i.MX95 rev B0):
 *       input0:
 *           12-bit input pixel format: 16-bit MSB alignment
 *           10, 14, 16-bit input pixel format: 20-bit MSB alignment
 *       input1: 16-bit MSB alignment
 *       An other peculiarity of this revision is that for the 12-bit input
 *       pixel format, LPALIGN0/1 configuration value is ignored by the ISP and
 *       a value of 1 is unconditionally applied.
 *
 * Relevant entries in the configuration file is a mapping of the following
 * keys:
 *   lpalign0: LPALIGN0 value (0/1) - default is 1
 *   lpalign1: LPALIGN1 value (0/1) - default is 1
 */

LOG_DEFINE_CATEGORY(NxpNeoAlgoPipeConf)

PipeConf::PipeConf()
	: inAlign0_(kInAlignDefault), lpAlign0_(kLpAlignDefault),
	  inAlign1_(kInAlignDefault), lpAlign1_(kLpAlignDefault)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int PipeConf::init([[maybe_unused]] IPAContext &context,
		   const YamlObject &tuningData)
{
	const YamlObject &lpAlign0Obj = tuningData["lpalign0"];
	lpAlign0_ = lpAlign0Obj.get<uint8_t>().value_or(kLpAlignDefault);
	const YamlObject &lpAlign1Obj = tuningData["lpalign1"];
	lpAlign1_ = lpAlign1Obj.get<uint8_t>().value_or(kLpAlignDefault);

	/* \todo make INALIGN default value SoC-dependent */
	inAlign0_ = kInAlignDefault;
	inAlign1_ = kInAlignDefault;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void PipeConf::prepare([[maybe_unused]] IPAContext &context, const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       [[maybe_unused]] neoisp_meta_params_s *params)
{
	if (frame > 0)
		return;

	LOG(NxpNeoAlgoPipeConf, Debug)
		<< "inalign0/1 " << inAlign0_ << "/" << inAlign1_
		<< "lpalign0/1 " << lpAlign0_ << "/" << lpAlign1_;

	/* PIPE_CONF unit configuration */
	params->features_cfg.pipe_conf_cfg = 1;

	neoisp_pipe_conf_cfg_s *pconf = &params->regs.pipe_conf;
	pconf->img_conf_inalign0 = inAlign0_;
	pconf->img_conf_lpalign0 = lpAlign0_;
	pconf->img_conf_inalign1 = inAlign1_;
	pconf->img_conf_lpalign1 = lpAlign1_;
}

REGISTER_IPA_ALGORITHM(PipeConf, "PipeConf")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
