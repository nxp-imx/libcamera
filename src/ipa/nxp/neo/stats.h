/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO ISP Statistics
 *
 * Copyright (C) 2024, Ideas On Board
 */

#pragma once

#include <map>
#include <stdint.h>

#include <linux/nxp_neoisp.h>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>

namespace libcamera {

namespace ipa::nxpneo {

enum class BlockStatsType {
	RCTemp,
	RDrc,
	RAf,
	RBnr,
	RNr,
	REe,
	RDf,
	MCTemp,
	MRgbIr,
	MHist,
	MDrc,
};

namespace details {

template<BlockStatsType S>
struct block_stats_type {
};

#define NXPNEO_BLOCK_STATS_TYPE(blockType, blockStruct)             \
	template<>                                                  \
	struct block_stats_type<BlockStatsType::blockType> {        \
		using type = struct neoisp_##blockStruct##_stats_s; \
	};

NXPNEO_BLOCK_STATS_TYPE(RCTemp, ctemp_reg)
NXPNEO_BLOCK_STATS_TYPE(RDrc, drc_reg)
NXPNEO_BLOCK_STATS_TYPE(RAf, af_reg)
NXPNEO_BLOCK_STATS_TYPE(RBnr, bnr_reg)
NXPNEO_BLOCK_STATS_TYPE(RNr, nr_reg)
NXPNEO_BLOCK_STATS_TYPE(REe, ee_reg)
NXPNEO_BLOCK_STATS_TYPE(RDf, df_reg)
NXPNEO_BLOCK_STATS_TYPE(MCTemp, ctemp_mem)
NXPNEO_BLOCK_STATS_TYPE(MRgbIr, rgbir_mem)
NXPNEO_BLOCK_STATS_TYPE(MHist, hist_mem)
NXPNEO_BLOCK_STATS_TYPE(MDrc, drc_mem)

} /* namespace details */

class NxpNeoStats;

class NxpNeoStatsBlockBase
{
public:
	NxpNeoStatsBlockBase(NxpNeoStats *stats,
			     BlockStatsType type,
			     const Span<uint8_t> &data);

	Span<uint8_t> data() const { return data_; }

private:
	LIBCAMERA_DISABLE_COPY(NxpNeoStatsBlockBase)

	NxpNeoStats *stats_;
	BlockStatsType type_;
	Span<uint8_t> header_;
	Span<uint8_t> data_;
};

template<BlockStatsType S>
class NxpNeoStatsBlock : public NxpNeoStatsBlockBase
{
public:
	using Type = typename details::block_stats_type<S>::type;

	NxpNeoStatsBlock(NxpNeoStats *stats, const Span<uint8_t> &data)
		: NxpNeoStatsBlockBase(stats, S, data)
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

	Type *stats()
	{
		return reinterpret_cast<Type *>(data().data());
	}
};

class NxpNeoStats
{
public:
	NxpNeoStats(Span<uint8_t> data);

	template<BlockStatsType S>
	NxpNeoStatsBlock<S> block() const
	{
		return NxpNeoStatsBlock<S>(const_cast<NxpNeoStats *>(this), block(S));
	}

	size_t size() const { return used_; }

private:
	friend class NxpNeoStatsBlockBase;

	Span<uint8_t> block(BlockStatsType type) const;

	Span<uint8_t> data_;
	size_t used_;

	std::map<BlockStatsType, Span<uint8_t>> blocks_;
};

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
