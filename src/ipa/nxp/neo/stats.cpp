/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 * Copyright 2025 NXP
 *
 * NXP NEOISP Statistics
 */

#include "stats.h"

#include <map>
#include <stddef.h>
#include <string.h>

#include <linux/nxp_neoisp.h>
#include <linux/videodev2.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoStats)

namespace ipa::nxpneo {

namespace {

struct BlockStatsTypeInfo {
	enum neoisp_stats_block_type_e type;
	size_t size;
	size_t offset;
};

#define NXPNEO_BLOCK_STATS_TYPE_ENTRY(block, id, category, type, member)       \
	{                                                                      \
		BlockStatsType::block,                                         \
		{                                                              \
			NEOISP_STATS_BLK_##id,                                 \
			sizeof(struct neoisp_##type##_stats_s),                \
			offsetof(struct neoisp_meta_stats_s, category.member), \
		}                                                              \
	}

const std::map<BlockStatsType, BlockStatsTypeInfo> kBlockTypeInfo = {
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RCTemp, RCTEMP, regs, ctemp_reg, ct),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RDrc, RDRC, regs, drc_reg, drc),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RAf, RAF, regs, af_reg, af),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RBnr, RBNR, regs, bnr_reg, bnr),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RNr, RNR, regs, nr_reg, nr),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(REe, REE, regs, ee_reg, ee),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(RDf, RDF, regs, df_reg, df),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(MCTemp, MCTEMP, mems, ctemp_mem, ctemp),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(MRgbIr, MRGBIR, mems, rgbir_mem, rgbir),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(MHist, MHIST, mems, hist_mem, hist),
	NXPNEO_BLOCK_STATS_TYPE_ENTRY(MDrc, MDRC, mems, drc_mem, drc),
};

} /* namespace */

NxpNeoStatsBlockBase::NxpNeoStatsBlockBase(NxpNeoStats *stats,
					   BlockStatsType type, const Span<uint8_t> &data)
	: stats_(stats), type_(type)
{
	if (stats_->isExtensible()) {
		header_ = data.subspan(0, sizeof(neoisp_ext_stats_block_header_s));
		data_ = data.subspan(sizeof(neoisp_ext_stats_block_header_s));
	} else {
		data_ = data;
	}
}

NxpNeoStats::NxpNeoStats(uint32_t apiVersion, Span<uint8_t> data)
	: apiVersion_(apiVersion), data_(data), used_(0)
{
	uint32_t offset = 0;

	if (this->isExtensible()) {
		struct neoisp_ext_stats_s *stats =
			reinterpret_cast<struct neoisp_ext_stats_s *>(data.data());

		used_ = stats->data_size;
		used_ += offsetof(struct neoisp_ext_stats_s, data);

		offset = offsetof(struct neoisp_ext_stats_s, data);

		/* Parse received blocks. */
		while (offset < stats->data_size) {
			Span<uint8_t> blk = data.subspan(offset);

			auto hdr = reinterpret_cast<struct neoisp_ext_stats_block_header_s *>(blk.data());
			BlockStatsType type = static_cast<BlockStatsType>(hdr->type);

			auto infoIt = kBlockTypeInfo.find(type);
			if (infoIt == kBlockTypeInfo.end()) {
				LOG(NxpNeoStats, Error)
					<< "Invalid statistics block type "
					<< utils::to_underlying(type);
				break;
			}
			const BlockStatsTypeInfo &info = infoIt->second;

			/* Check we received expected size, and that we are 8-bytes aligned. */
			size_t size = (info.size + sizeof(struct neoisp_ext_stats_block_header_s) + 7) & ~7;
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
	} else {
		used_ = sizeof(struct neoisp_meta_stats_s);
	}
}

Span<uint8_t> NxpNeoStats::block(BlockStatsType type) const
{
	auto infoIt = kBlockTypeInfo.find(type);
	if (infoIt == kBlockTypeInfo.end()) {
		LOG(NxpNeoStats, Error)
			<< "Invalid parameters block type "
			<< utils::to_underlying(type);
		return {};
	}

	const BlockStatsTypeInfo &info = infoIt->second;

	/*
	 * For the legacy format, return a block referencing the fixed location
	 * and size of the data.
	 */
	if (!this->isExtensible())
		return data_.subspan(info.offset, info.size);

	/*
	 * For the extensible format, look for the requested block, including
	 * the header.
	 */
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
