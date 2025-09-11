/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 * Copyright 2025 NXP
 *
 * NXP NEOISP Parameters
 */

#pragma once

#include <map>
#include <stdint.h>

#include <linux/nxp_neoisp.h>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>

namespace libcamera {

namespace ipa::nxpneo {

enum class BlockParamsType {
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

#define NXPNEO_DEFINE_BLOCK_TYPE(blockType, blockStruct, cat)         \
	template<>                                                    \
	struct block_params_type<BlockParamsType::blockType> {        \
		using type = struct neoisp_##blockStruct##_##cat##_s; \
	};

#define NXPNEO_DEFINE_BLOCK_TYPE_CFG(blockType, blockStruct) \
	NXPNEO_DEFINE_BLOCK_TYPE(blockType, blockStruct, cfg)

#define NXPNEO_DEFINE_BLOCK_TYPE_MEM(blockType, blockStruct) \
	NXPNEO_DEFINE_BLOCK_TYPE(blockType, blockStruct, mem_params)

NXPNEO_DEFINE_BLOCK_TYPE_CFG(PipeConf, pipe_conf)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(HeadColor, head_color)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(HdrDec0, hdr_decompress0)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(HdrDec1, hdr_decompress1)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Obwb0, obwb)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Obwb1, obwb)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Obwb2, obwb)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(HdrMerge, hdr_merge)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(RgbIr, rgbir)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Stat, stat)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(CTemp, ctemp)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(IrComp, ir_compress)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Bnr, bnr)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(VigCtrl, vignetting_ctrl)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Demosaic, demosaic)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Rgb2Yuv, rgb2yuv)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(DrComp, dr_comp)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Nr, nr)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Af, af)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Ee, ee)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Df, df)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Convmed, convmed)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Cas, cas)
NXPNEO_DEFINE_BLOCK_TYPE_CFG(Gcm, gcm)
NXPNEO_DEFINE_BLOCK_TYPE_MEM(VigTable, vignetting_table)
NXPNEO_DEFINE_BLOCK_TYPE_MEM(DrcGlobalTonemap, drc_global_tonemap)
NXPNEO_DEFINE_BLOCK_TYPE_MEM(DrcLocalTonemap, drc_local_tonemap)

} /* namespace details */

class NxpNeoParams;

class NxpNeoParamsBlockBase
{
public:
	NxpNeoParamsBlockBase(NxpNeoParams *params, BlockParamsType type,
			      const Span<uint8_t> &data);

	Span<uint8_t> data() const { return data_; }

	void setUpdate(bool update);

private:
	LIBCAMERA_DISABLE_COPY(NxpNeoParamsBlockBase)

	NxpNeoParams *params_;
	BlockParamsType type_;
	Span<uint8_t> header_;
	Span<uint8_t> data_;
};

template<BlockParamsType B>
class NxpNeoParamsBlock : public NxpNeoParamsBlockBase
{
public:
	using Type = typename details::block_params_type<B>::type;

	NxpNeoParamsBlock(NxpNeoParams *params, const Span<uint8_t> &data)
		: NxpNeoParamsBlockBase(params, B, data)
	{
	}

	const Type *operator->() const
	{
		return reinterpret_cast<const Type *>(data().data());
	}

	Type *operator->()
	{
		return reinterpret_cast<Type *>(data().data());
	}

	const Type &operator*() const &
	{
		return *reinterpret_cast<const Type *>(data().data());
	}

	Type &operator*() &
	{
		return *reinterpret_cast<Type *>(data().data());
	}

	Type *params()
	{
		return reinterpret_cast<Type *>(data().data());
	}
};

class NxpNeoParams
{
public:
	NxpNeoParams(uint32_t apiVersion, Span<uint8_t> data);

	template<BlockParamsType B>
	NxpNeoParamsBlock<B> block()
	{
		return NxpNeoParamsBlock<B>(this, block(B));
	}

	size_t size() const { return used_; }

private:
	friend class NxpNeoParamsBlockBase;

	Span<uint8_t> block(BlockParamsType type);
	void setBlockUpdate(BlockParamsType type, bool update);
	bool isExtensible()
	{
		return apiVersion_ != NEOISP_LEGACY_META_BUFFER;
	}

	uint32_t apiVersion_;

	Span<uint8_t> data_;
	size_t used_;

	std::map<BlockParamsType, Span<uint8_t>> blocks_;
};

} /* namespace ipa::nxpneo */

} /* namespace libcamera*/
