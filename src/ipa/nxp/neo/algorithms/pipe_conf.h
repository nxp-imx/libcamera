/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * pipe_conf.h - NXP NEO PIPE_CONF configuration
 * Copyright 2025 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class PipeConf : public Algorithm
{
public:
	PipeConf();
	~PipeConf() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     neoisp_meta_params_s *params) override;

private:
	static constexpr size_t kInAlignDefault = 1;
	static constexpr size_t kLpAlignDefault = 1;

	uint8_t inAlign0_;
	uint8_t lpAlign0_;

	uint8_t inAlign1_;
	uint8_t lpAlign1_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
