/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 Image Processing Algorithms
 *     src/ipa/algorithms/algorithm.h
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - NXP NEO control algorithm interface
 * Copyright 2024 NXP
 */

#pragma once

#include <libipa/algorithm.h>

#include "module.h"

namespace libcamera {

namespace ipa::nxpneo {

class Algorithm : public libcamera::ipa::Algorithm<Module>
{
public:
	Algorithm()
		: disabled_(false)
	{
	}

	bool disabled_;
	bool supportsRaw_;
};

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
