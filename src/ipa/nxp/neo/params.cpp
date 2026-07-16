/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO ISP Parameters
 *
 * Copyright (C) 2024, Ideas On Board
 */

#include "params.h"

#include <map>
#include <stddef.h>
#include <string.h>

#include <linux/nxp_neoisp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoParams)

namespace ipa::nxpneo {

namespace {

struct BlockParamsTypeInfo {
	enum neoisp_param_block_type_e type;
	size_t size;
};

#define NXPNEO_BLOCK_TYPE_ENTRY(block, id, type, ext)                     \
	{                                                                 \
		BlockParamsType::block,                                   \
		{                                                         \
			NEOISP_PARAM_BLK_##id,                            \
				sizeof(struct neoisp_##type##_##ext##_s), \
		}                                                         \
	}

#define NXPNEO_BLOCK_TYPE_ENTRY_REGS(block, id, type) \
	NXPNEO_BLOCK_TYPE_ENTRY(block, id, type, cfg)

#define NXPNEO_BLOCK_TYPE_ENTRY_MEMS(block, id, type) \
	NXPNEO_BLOCK_TYPE_ENTRY(block, id, type, mem_params)

const std::map<BlockParamsType, BlockParamsTypeInfo> kBlockParamsTypeInfo = {
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(PipeConf, PIPE_CONF, pipe_conf),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(HeadColor, HEAD_COLOR, head_color),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(HdrDec0, HDR_DECOMPRESS0, hdr_decompress0),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(HdrDec1, HDR_DECOMPRESS1, hdr_decompress1),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Obwb0, OBWB0, obwb),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Obwb1, OBWB1, obwb),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Obwb2, OBWB2, obwb),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(HdrMerge, HDR_MERGE, hdr_merge),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(RgbIr, RGBIR, rgbir),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Stat, STAT, stat),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(CTemp, CTEMP, ctemp),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(IrComp, IR_COMPRESS, ir_compress),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Bnr, BNR, bnr),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(VigCtrl, VIGNETTING_CTRL, vignetting_ctrl),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Demosaic, DEMOSAIC, demosaic),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Rgb2Yuv, RGB2YUV, rgb2yuv),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(DrComp, DR_COMP, dr_comp),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Nr, NR, nr),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Af, AF, af),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Ee, EE, ee),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Df, DF, df),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Convmed, CONVMED, convmed),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Cas, CAS, cas),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Gcm, GCM, gcm),
	NXPNEO_BLOCK_TYPE_ENTRY_MEMS(VigTable, VIGNETTING_TABLE, vignetting_table),
	NXPNEO_BLOCK_TYPE_ENTRY_MEMS(DrcGlobalTonemap, DRC_GLOBAL_TONEMAP, drc_global_tonemap),
	NXPNEO_BLOCK_TYPE_ENTRY_MEMS(DrcLocalTonemap, DRC_LOCAL_TONEMAP, drc_local_tonemap),
};

} /* namespace */

Span<uint8_t> NxpNeoParams::block(BlockParamsType type)
{
	auto infoIt = kBlockParamsTypeInfo.find(type);
	if (infoIt == kBlockParamsTypeInfo.end()) {
		LOG(NxpNeoParams, Error)
			<< "Invalid parameters block type "
			<< utils::to_underlying(type);
		return {};
	}

	const BlockParamsTypeInfo &info = infoIt->second;

	/* Make sure to align 8-bytes */
	size_t size = (sizeof(struct v4l2_isp_block_header) + info.size + 7) & ~7;

	return V4L2ParamsBase::block(utils::to_underlying(type), info.type, size);
}

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
