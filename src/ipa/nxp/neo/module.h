/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Rockchip ISP1 Module
 *     src/ipa/rkisp1/module.h
 * Copyright (C) 2022, Ideas On Board
 *
 * module.h - NXP NEO IPA Module
 * Copyright 2024-2025 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/ipa/nxpneo_ipa_interface.h>

#include <libipa/module.h>

#include "ipa_context.h"

#include "params.h"
#include "stats.h"

namespace libcamera {

namespace ipa::nxpneo {

using Module = ipa::Module<IPAContext, IPAFrameContext, IPACameraSensorInfo,
			   NxpNeoParams, NxpNeoStats>;

} /* namespace ipa::nxpneo */

} /* namespace libcamera*/
