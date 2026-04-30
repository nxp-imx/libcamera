/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * NXP NEO control algorithm interface
 *
 * Based on RkISP1 Image Processing Algorithms
 *     src/ipa/algorithms/algorithm.h
 * Copyright (C) 2021, Ideas On Board
 */

#pragma once

#include <libipa/algorithm.h>

#include "module.h"

namespace libcamera {

namespace ipa::nxpneo {

enum AlgorithmIrOps : uint32_t {
	IrOpNone = 0,
	IrOpPrepare = (1 << 0),
	IrOpProcess = (1 << 1),
	IrOpAll = (IrOpPrepare | IrOpProcess),
};

class Algorithm : public libcamera::ipa::Algorithm<Module>
{
public:
	Algorithm()
		: irOps_(IrOpNone)
	{
	}

	uint32_t irOps() const { return irOps_; }
	void setIrOps(uint32_t ops) { irOps_ = ops; }

private:
	uint32_t irOps_;
};

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
