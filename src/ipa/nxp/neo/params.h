/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO ISP Parameters
 *
 * Copyright (C) 2024, Ideas On Board
 */

#pragma once

#include <map>
#include <stdint.h>

#include <linux/nxp_neoisp.h>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>
#include <libipa/v4l2_params.h>

namespace libcamera {

namespace ipa::nxpneo {

enum class BlockParamsType : uint16_t {
	PipeConf,
	HeadColor,
	HdrDec0,
	HdrDec1,
	Obwb0,
	Obwb1,
	Obwb2,
	HdrMerge,
	RgbIr,
	Stat,
	CTemp,
	IrComp,
	Bnr,
	VigCtrl,
	Demosaic,
	Rgb2Yuv,
	DrComp,
	Nr,
	Af,
	Ee,
	Df,
	Convmed,
	Cas,
	Gcm,
	VigTable,
	DrcGlobalTonemap,
	DrcLocalTonemap
};

namespace details {

template<BlockParamsType B>
struct block_params_type {
};

#define NXPNEO_BLOCK_TYPE(id, blockType, blockStruct, cat)             \
	template<>                                                     \
	struct block_params_type<BlockParamsType::blockType> {         \
		using type = struct neoisp_##blockStruct##_##cat##_s;  \
		static constexpr neoisp_param_block_type_e blockType = \
			NEOISP_PARAM_BLK_##id;                         \
	};

#define NXPNEO_BLOCK_TYPE_CFG(id, blockType, blockStruct) \
	NXPNEO_BLOCK_TYPE(id, blockType, blockStruct, cfg)

#define NXPNEO_BLOCK_TYPE_MEM(id, blockType, blockStruct) \
	NXPNEO_BLOCK_TYPE(id, blockType, blockStruct, mem_params)

NXPNEO_BLOCK_TYPE_CFG(PIPE_CONF, PipeConf, pipe_conf)
NXPNEO_BLOCK_TYPE_CFG(HEAD_COLOR, HeadColor, head_color)
NXPNEO_BLOCK_TYPE_CFG(HDR_DECOMPRESS0, HdrDec0, hdr_decompress0)
NXPNEO_BLOCK_TYPE_CFG(HDR_DECOMPRESS1, HdrDec1, hdr_decompress1)
NXPNEO_BLOCK_TYPE_CFG(OBWB0, Obwb0, obwb)
NXPNEO_BLOCK_TYPE_CFG(OBWB1, Obwb1, obwb)
NXPNEO_BLOCK_TYPE_CFG(OBWB2, Obwb2, obwb)
NXPNEO_BLOCK_TYPE_CFG(HDR_MERGE, HdrMerge, hdr_merge)
NXPNEO_BLOCK_TYPE_CFG(RGBIR, RgbIr, rgbir)
NXPNEO_BLOCK_TYPE_CFG(STAT, Stat, stat)
NXPNEO_BLOCK_TYPE_CFG(CTEMP, CTemp, ctemp)
NXPNEO_BLOCK_TYPE_CFG(IR_COMPRESS, IrComp, ir_compress)
NXPNEO_BLOCK_TYPE_CFG(BNR, Bnr, bnr)
NXPNEO_BLOCK_TYPE_CFG(VIGNETTING_CTRL, VigCtrl, vignetting_ctrl)
NXPNEO_BLOCK_TYPE_CFG(DEMOSAIC, Demosaic, demosaic)
NXPNEO_BLOCK_TYPE_CFG(RGB2YUV, Rgb2Yuv, rgb2yuv)
NXPNEO_BLOCK_TYPE_CFG(DR_COMP, DrComp, dr_comp)
NXPNEO_BLOCK_TYPE_CFG(NR, Nr, nr)
NXPNEO_BLOCK_TYPE_CFG(AF, Af, af)
NXPNEO_BLOCK_TYPE_CFG(EE, Ee, ee)
NXPNEO_BLOCK_TYPE_CFG(DF, Df, df)
NXPNEO_BLOCK_TYPE_CFG(CONVMED, Convmed, convmed)
NXPNEO_BLOCK_TYPE_CFG(CAS, Cas, cas)
NXPNEO_BLOCK_TYPE_CFG(GCM, Gcm, gcm)
NXPNEO_BLOCK_TYPE_MEM(VIGNETTING_TABLE, VigTable, vignetting_table)
NXPNEO_BLOCK_TYPE_MEM(DRC_GLOBAL_TONEMAP, DrcGlobalTonemap, drc_global_tonemap)
NXPNEO_BLOCK_TYPE_MEM(DRC_LOCAL_TONEMAP, DrcLocalTonemap, drc_local_tonemap)

struct params_traits {
	using id_type = BlockParamsType;

	template<id_type Id>
	using id_to_details = block_params_type<Id>;
};

} /* namespace details */

class NxpNeoParams;

template<typename T>
class NxpNeoParamsBlock;

class NxpNeoParams : public V4L2Params<details::params_traits>
{
public:
	static constexpr unsigned int kVersion = V4L2_ISP_VERSION_V1;

	NxpNeoParams(Span<uint8_t> data)
		: V4L2Params(data, kVersion)
	{
	}

	template<details::params_traits::id_type id>
	auto block()
	{
		using Type = typename details::block_params_type<id>::type;
		return NxpNeoParamsBlock<Type>(this, id, block(id));
	}

private:
	Span<uint8_t> block(BlockParamsType type);
};

template<typename T>
class NxpNeoParamsBlock final : public V4L2ParamsBlock<T>
{
public:
	NxpNeoParamsBlock(NxpNeoParams *params, BlockParamsType type,
			  const Span<uint8_t> data)
		: V4L2ParamsBlock<T>(data)
	{
		params_ = params;
		type_ = type;

		cfgData_ = data.subspan(sizeof(v4l2_isp_block_header));
	}

	/*
	 * Override the dereference operators to return a reference to the
	 * actual configuration data (struct neoisp_**_cfg_s) skipping the
	 * 'v4l2_isp_block_header' header.
	 */
	virtual const T *operator->() const override
	{
		return reinterpret_cast<const T *>(cfgData_.data());
	}

	virtual T *operator->() override
	{
		return reinterpret_cast<T *>(cfgData_.data());
	}

	virtual const T &operator*() const override
	{
		return *reinterpret_cast<const T *>(cfgData_.data());
	}

	virtual T &operator*() override
	{
		return *reinterpret_cast<T *>(cfgData_.data());
	}

	T *params()
	{
		return reinterpret_cast<T *>(cfgData_.data());
	}

private:
	NxpNeoParams *params_;
	BlockParamsType type_;
	Span<uint8_t> cfgData_;
};

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
