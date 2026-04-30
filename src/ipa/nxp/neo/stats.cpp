/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * NXP NEO ISP Statistics
 *
 * Copyright (C) 2024, Ideas On Board
 */

#include "stats.h"

#include <map>
#include <stddef.h>
#include <string.h>

#include <linux/nxp_neoisp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoStats)

namespace ipa::nxpneo {

namespace {

struct BlockStatsTypeInfo {
	enum neoisp_stats_block_type_e type;
	size_t size;
};

#define NXPNEO_BLOCK_STATS_TYPE_ENTRY(block, id, type)          \
	{                                                       \
		BlockStatsType::block,                          \
		{                                               \
			NEOISP_STATS_BLK_##id,                  \
			sizeof(struct neoisp_##type##_stats_s), \
		}                                               \
	}

const std::map<BlockStatsType, BlockStatsTypeInfo> kBlockStatsTypeInfo = {
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RCTemp, RCTEMP, ctemp_reg),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RDrc, RDRC, drc_reg),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RAf, RAF, af_reg),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RBnr, RBNR, bnr_reg),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RNr, RNR, nr_reg),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(REe, REE, ee_reg),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RDf, RDF, df_reg),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(MCTemp, MCTEMP, ctemp_mem),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(MRgbIr, MRGBIR, rgbir_mem),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(MHist, MHIST, hist_mem),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(MDrc, MDRC, drc_mem),
};

} /* namespace */

NxpNeoStatsBlockBase::NxpNeoStatsBlockBase(NxpNeoStats *stats,
					   BlockStatsType type,
					   const Span<uint8_t> &data)
	: stats_(stats), type_(type)
{
	header_ = data.subspan(0, sizeof(struct v4l2_isp_block_header));
	data_ = data.subspan(sizeof(struct v4l2_isp_block_header));
}

NxpNeoStats::NxpNeoStats(Span<uint8_t> data)
	: data_(data), used_(0)
{
	struct v4l2_isp_buffer *stats =
		reinterpret_cast<struct v4l2_isp_buffer *>(data.data());

	if (stats->version != V4L2_ISP_VERSION_V0 &&
	    stats->version != V4L2_ISP_VERSION_V1) {
		LOG(NxpNeoStats, Error)
			<< "Invalid statistics buffer version "
			<< stats->version;
		return;
	}

	uint32_t offset = offsetof(struct v4l2_isp_buffer, data);
	used_ = stats->data_size + offset;

	/* Parse received blocks. */
	while (offset < stats->data_size) {
		Span<uint8_t> blk = data.subspan(offset);

		auto hdr = reinterpret_cast<struct v4l2_isp_block_header *>(blk.data());
		BlockStatsType type = static_cast<BlockStatsType>(hdr->type);

		auto infoIt = kBlockStatsTypeInfo.find(type);
		if (infoIt == kBlockStatsTypeInfo.end()) {
			LOG(NxpNeoStats, Error)
				<< "Invalid statistics block type "
				<< utils::to_underlying(type);
			break;
		}
		const BlockStatsTypeInfo &info = infoIt->second;

		/* Check we received expected size, and that we are 8-bytes aligned. */
		size_t size = (info.size + sizeof(struct v4l2_isp_block_header) + 7) & ~7;
		if (size != hdr->size) {
			LOG(NxpNeoStats, Error)
				<< "Invalid statistics block size " << hdr->size
				<< ", expected " << size
				<< ", for type " << utils::to_underlying(type);
			break;
		}

		blocks_[type] = blk.subspan(0, hdr->size);
		offset += hdr->size;

		if (hdr->size == 0)
			break;
	}
}

Span<uint8_t> NxpNeoStats::block(BlockStatsType type) const
{
	auto infoIt = kBlockStatsTypeInfo.find(type);
	if (infoIt == kBlockStatsTypeInfo.end()) {
		LOG(NxpNeoStats, Error)
			<< "Invalid statistics block type "
			<< utils::to_underlying(type);
		return {};
	}

	/* Look for the requested block, including the header */
	auto cacheIt = blocks_.find(type);
	if (cacheIt == blocks_.end()) {
		LOG(NxpNeoStats, Error)
			<< "No block found with type "
			<< utils::to_underlying(type);
		return {};
	}

	return cacheIt->second;
}

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
