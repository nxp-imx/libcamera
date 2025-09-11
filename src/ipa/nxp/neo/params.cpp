/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 * Copyright 2025 NXP
 *
 * NXP NEOISP Parameters
 */

#include "params.h"

#include <map>
#include <stddef.h>
#include <string.h>

#include <linux/nxp_neoisp.h>
#include <linux/videodev2.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoParams)

namespace ipa::nxpneo {

namespace {

struct BlockParamsTypeInfo {
	enum neoisp_param_block_type_e type;
	size_t size;
	size_t offset;
	void (*setFeatureBit)(struct neoisp_feat_ctrl_s *featureCtrl, bool enable);
};

#define NXPNEO_SET_FEATURE_BIT_ENTRY(block, feature)                             \
	static void setFeatureBit##block(struct neoisp_feat_ctrl_s *featureCtrl, \
					 bool enable)                            \
	{                                                                        \
		featureCtrl->feature##_cfg = (enable == true) ? 1 : 0;           \
	}

NXPNEO_SET_FEATURE_BIT_ENTRY(PipeConf, pipe_conf)
NXPNEO_SET_FEATURE_BIT_ENTRY(HeadColor, head_color)
NXPNEO_SET_FEATURE_BIT_ENTRY(HdrDec0, hdr_decompress_input0)
NXPNEO_SET_FEATURE_BIT_ENTRY(HdrDec1, hdr_decompress_input1)
NXPNEO_SET_FEATURE_BIT_ENTRY(Obwb0, obwb0)
NXPNEO_SET_FEATURE_BIT_ENTRY(Obwb1, obwb1)
NXPNEO_SET_FEATURE_BIT_ENTRY(Obwb2, obwb2)
NXPNEO_SET_FEATURE_BIT_ENTRY(HdrMerge, hdr_merge)
NXPNEO_SET_FEATURE_BIT_ENTRY(RgbIr, rgbir)
NXPNEO_SET_FEATURE_BIT_ENTRY(Stat, stat)
NXPNEO_SET_FEATURE_BIT_ENTRY(CTemp, ctemp)
NXPNEO_SET_FEATURE_BIT_ENTRY(IrComp, ir_compress)
NXPNEO_SET_FEATURE_BIT_ENTRY(Bnr, bnr)
NXPNEO_SET_FEATURE_BIT_ENTRY(VigCtrl, vignetting_ctrl)
NXPNEO_SET_FEATURE_BIT_ENTRY(Demosaic, demosaic)
NXPNEO_SET_FEATURE_BIT_ENTRY(Rgb2Yuv, rgb2yuv)
NXPNEO_SET_FEATURE_BIT_ENTRY(DrComp, dr_comp)
NXPNEO_SET_FEATURE_BIT_ENTRY(Nr, nr)
NXPNEO_SET_FEATURE_BIT_ENTRY(Af, af)
NXPNEO_SET_FEATURE_BIT_ENTRY(Ee, ee)
NXPNEO_SET_FEATURE_BIT_ENTRY(Df, df)
NXPNEO_SET_FEATURE_BIT_ENTRY(Convmed, convmed)
NXPNEO_SET_FEATURE_BIT_ENTRY(Cas, cas)
NXPNEO_SET_FEATURE_BIT_ENTRY(Gcm, gcm)
NXPNEO_SET_FEATURE_BIT_ENTRY(VigTable, vignetting_table)
NXPNEO_SET_FEATURE_BIT_ENTRY(DrcGlobalTonemap, drc_global_tonemap)
NXPNEO_SET_FEATURE_BIT_ENTRY(DrcLocalTonemap, drc_local_tonemap)

#define NXPNEO_BLOCK_TYPE_ENTRY(block, id, type, ext, param, category)         \
	{                                                                      \
		BlockParamsType::block,                                        \
		{                                                              \
			NEOISP_PARAM_BLK_##id,                                 \
			sizeof(struct neoisp_##type##_##ext##_s),              \
			offsetof(struct neoisp_meta_params_s, category.param), \
			setFeatureBit##block,                                  \
		}                                                              \
	}

#define NXPNEO_BLOCK_TYPE_ENTRY_REGS(block, id, type) \
	NXPNEO_BLOCK_TYPE_ENTRY(block, id, type, cfg, type, regs)

#define NXPNEO_BLOCK_TYPE_ENTRY_REGS_ARRAY(block, id, type, idx) \
	NXPNEO_BLOCK_TYPE_ENTRY(block, id, type, cfg, type [idx], regs)

#define NXPNEO_BLOCK_TYPE_ENTRY_MEMS(block, id, type, param) \
	NXPNEO_BLOCK_TYPE_ENTRY(block, id, type, mem_params, param, mems)

const std::map<BlockParamsType, BlockParamsTypeInfo> kBlockParamsTypeInfo = {
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(PipeConf, PIPE_CONF, pipe_conf),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(HeadColor, HEAD_COLOR, head_color),
	NXPNEO_BLOCK_TYPE_ENTRY(HdrDec0, HDR_DECOMPRESS0, hdr_decompress0, cfg, decompress_input0, regs),
	NXPNEO_BLOCK_TYPE_ENTRY(HdrDec1, HDR_DECOMPRESS1, hdr_decompress1, cfg, decompress_input1, regs),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS_ARRAY(Obwb0, OBWB0, obwb, 0),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS_ARRAY(Obwb1, OBWB1, obwb, 1),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS_ARRAY(Obwb2, OBWB2, obwb, 2),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(HdrMerge, HDR_MERGE, hdr_merge),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(RgbIr, RGBIR, rgbir),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Stat, STAT, stat),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(CTemp, CTEMP, ctemp),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(IrComp, IR_COMPRESS, ir_compress),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Bnr, BNR, bnr),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(VigCtrl, VIGNETTING_CTRL, vignetting_ctrl),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Demosaic, DEMOSAIC, demosaic),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Rgb2Yuv, RGB2YUV, rgb2yuv),
	NXPNEO_BLOCK_TYPE_ENTRY(DrComp, DR_COMP, dr_comp, cfg, drc, regs),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Nr, NR, nr),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Af, AF, af),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Ee, EE, ee),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Df, DF, df),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Convmed, CONVMED, convmed),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Cas, CAS, cas),
	NXPNEO_BLOCK_TYPE_ENTRY_REGS(Gcm, GCM, gcm),
	NXPNEO_BLOCK_TYPE_ENTRY_MEMS(VigTable, VIGNETTING_TABLE, vignetting_table, vt),
	NXPNEO_BLOCK_TYPE_ENTRY_MEMS(DrcGlobalTonemap, DRC_GLOBAL_TONEMAP, drc_global_tonemap, gtm),
	NXPNEO_BLOCK_TYPE_ENTRY_MEMS(DrcLocalTonemap, DRC_LOCAL_TONEMAP, drc_local_tonemap, ltm),
};

} /* namespace */

NxpNeoParamsBlockBase::NxpNeoParamsBlockBase(NxpNeoParams *params, BlockParamsType type,
					     const Span<uint8_t> &data)
	: params_(params), type_(type)
{
	if (params->isExtensible()) {
		header_ = data.subspan(0, sizeof(neoisp_ext_params_block_header_s));
		data_ = data.subspan(sizeof(neoisp_ext_params_block_header_s));
	} else {
		data_ = data;
	}
}

void NxpNeoParamsBlockBase::setUpdate(bool update)
{
	/*
	 * For the legacy fixed format, blocks are enabled in the top-level
	 * header. Delegate to the NxpNeoParams class.
	 */
	if (!params_->isExtensible())
		return params_->setBlockUpdate(type_, update);

	/*
	 * For the extensible format, set the flags in the block header
	 * directly.
	 */
	struct neoisp_ext_params_block_header_s *header =
		reinterpret_cast<struct neoisp_ext_params_block_header_s *>(header_.data());
	header->flags &= ~(NEOISP_EXT_PARAMS_BLK_FL_NONE | NEOISP_EXT_PARAMS_BLK_FL_UPDATE);
	header->flags |= update ? NEOISP_EXT_PARAMS_BLK_FL_UPDATE
				: NEOISP_EXT_PARAMS_BLK_FL_NONE;
}

NxpNeoParams::NxpNeoParams(uint32_t apiVersion, Span<uint8_t> data)
	: apiVersion_(apiVersion), data_(data), used_(0)
{
	if (isExtensible()) {
		struct neoisp_ext_params_s *cfg =
			reinterpret_cast<struct neoisp_ext_params_s *>(data.data());

		cfg->version = apiVersion;
		cfg->data_size = 0;

		used_ = offsetof(struct neoisp_ext_params_s, data);
	} else {
		struct neoisp_meta_params_s *cfg =
			reinterpret_cast<struct neoisp_meta_params_s *>(data_.data());

		memset(&cfg->features_cfg, 0, sizeof(struct neoisp_feat_ctrl_s));
		used_ = sizeof(struct neoisp_meta_params_s);
	}
}

void NxpNeoParams::setBlockUpdate(BlockParamsType type, bool update)
{
	const BlockParamsTypeInfo &info = kBlockParamsTypeInfo.at(type);

	struct neoisp_meta_params_s *cfg =
		reinterpret_cast<struct neoisp_meta_params_s *>(data_.data());

	info.setFeatureBit(&cfg->features_cfg, update);
}

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

	/*
	 * For the legacy format, return a block referencing the fixed location
	 * of the data.
	 */
	if (!isExtensible()) {
		/*
		 * Blocks available only in extended parameters have an offset
		 * of 0. Return nullptr in that case.
		 */
		if (info.offset == 0) {
			LOG(NxpNeoParams, Error)
				<< "Block type " << utils::to_underlying(type)
				<< " unavailable in fixed parameters format";
			return {};
		}

		return data_.subspan(info.offset, info.size);
	}

	/*
	 * For the extensible format, allocate memory for the block, including
	 * the header. Look up the block in the cache first. If an algorithm
	 * requests the same block type twice, it should get the same block.
	 */
	auto cacheIt = blocks_.find(type);
	if (cacheIt != blocks_.end())
		return cacheIt->second;

	/* Make sure we don't run out of space, and to align 8-bytes */
	size_t size = (sizeof(struct neoisp_ext_params_block_header_s) + info.size + 7) & ~7;
	if (size > (data_.size() - used_)) {
		LOG(NxpNeoParams, Error)
			<< "Out of memory to allocate block type "
			<< utils::to_underlying(type);
		return {};
	}

	/* Allocate a new block, and initialize its header and set inactive flag. */
	Span<uint8_t> block = data_.subspan(used_, size);
	used_ += size;

	struct neoisp_ext_params_s *cfg =
		reinterpret_cast<struct neoisp_ext_params_s *>(data_.data());
	cfg->data_size += size;

	struct neoisp_ext_params_block_header_s *header =
		reinterpret_cast<struct neoisp_ext_params_block_header_s *>(block.data());
	header->type = info.type;
	header->flags = NEOISP_EXT_PARAMS_BLK_FL_NONE;
	header->size = block.size();

	/* Update the cache. */
	blocks_[type] = block;

	return block;
}

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
