/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO PIPE_CONF configuration
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

	int init(IPAContext &context, const ValueNode &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     NxpNeoParams *Params) override;

private:
	static constexpr uint8_t kLpAlignDefault = 1;

	uint8_t inAlign0_;
	std::optional<uint8_t> lpAlign0_;

	uint8_t inAlign1_;
	std::optional<uint8_t> lpAlign1_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
