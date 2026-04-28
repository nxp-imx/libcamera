/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * NXP NEO ISP enum values
 *
 * Definitions from this file are supposed to be
 * integrated part of the linux kernel UAPI nxp_neoisp.h
 */

#pragma once

#include <array>

namespace libcamera {

namespace ipa::nxpneo {

/**
 * enum neoisp_obwb_instances - instance of the Optical Black Correction and
 * 				White Balance block.
 *
 * @NEO_OBWB_LINE_PATH0:	OB_WB of the line path0
 * @NEO_OBWB_LINE_PATH1:	OB_WB of the line path1
 * @NEO_OBWB_MERGE_PATH:	OB_WB of the merge path
 * @NEO_OBWB_CNT:		OB_WB instances number
 */
enum neoisp_obwb_instances {
	NEO_OBWB_LINE_PATH0 = 0,
	NEO_OBWB_LINE_PATH1,
	NEO_OBWB_MERGE_PATH,
	/* NEO_OBWB_CNT, \todo: should replace the one defined in UAPI */
};

/**
 * enum neoisp_obwb_obpp - size of pixel components outputted from the
 * 			   OB_WB unit.
 *
 * @NEO_OBWB_OBPP_12BPP:	12 bpp
 * @NEO_OBWB_OBPP_14BPP:	14 bpp
 * @NEO_OBWB_OBPP_16BPP:	16 bpp
 * @NEO_OBWB_OBPP_20BPP:	20 bpp
 */
enum neoisp_obwb_obpp {
	NEO_OBWB_OBPP_12BPP = 0,
	NEO_OBWB_OBPP_14BPP = 1,
	NEO_OBWB_OBPP_16BPP = 2,
	NEO_OBWB_OBPP_20BPP = 3,
};

/*
 * Following description should be added for the UAPI struct neoisp_obwb_cfg_s
 *
 * Gain format: UQ8.8 (8bits: integer part, 8bits: fractional part)
 * Offset format: unsigned 16bits
 */

/**
 * enum neoisp_ctemp_ibpp - size of pixel components coming into the
 * 			    COLORTEMP unit.
 *
 * @NEO_CTEMP_IBPP_12BPP:	12 bpp
 * @NEO_CTEMP_IBPP_14BPP:	14 bpp
 * @NEO_CTEMP_IBPP_16BPP:	16 bpp
 * @NEO_CTEMP_IBPP_20BPP:	20 bpp
 */
enum neoisp_ctemp_ibpp {
	NEO_CTEMP_IBPP_12BPP = 0,
	NEO_CTEMP_IBPP_14BPP = 1,
	NEO_CTEMP_IBPP_16BPP = 2,
	NEO_CTEMP_IBPP_20BPP = 3,
};

#define NEO_CTEMP_BLOCK_NB_X 8
#define NEO_CTEMP_BLOCK_NB_Y 8

/**
 * enum neoisp_hist_ctrl_channel - RGGB channel to be included in the
 * 				   histogram.
 *
 * @NEO_HIST_CHANNEL_R:  Red (R) pixels of a RGGB Bayer pattern
 * @NEO_HIST_CHANNEL_GR: Green (Gr) pixels of a RGGB Bayer pattern
 * @NEO_HIST_CHANNEL_GB: Green (Gb) pixels of a RGGB Bayer pattern
 * @NEO_HIST_CHANNEL_B:  Blue (B) pixels of a RGGB Bayer pattern
 */
enum neoisp_hist_ctrl_channel {
	NEO_HIST_CHANNEL_R = 0x1,
	NEO_HIST_CHANNEL_GR = 0x2,
	NEO_HIST_CHANNEL_GB = 0x4,
	NEO_HIST_CHANNEL_B = 0x8,
};

/**
 * enum neoisp_hist_rgbir_channel - RGBIr channels format.
 *
 * @NEO_HIST_CHANNEL1: 1st channel of a 2x2 window of input image
 * @NEO_HIST_CHANNEL2: 2nd channel of a 2x2 window of input image
 * @NEO_HIST_CHANNEL3: 3rd channel of a 2x2 window of input image
 * @NEO_HIST_CHANNEL4: 4th channel of a 2x2 window of input image
 */
enum neoisp_hist_rgbir_channel {
	NEO_HIST_CHANNEL1 = 0x1,
	NEO_HIST_CHANNEL2 = 0x2,
	NEO_HIST_CHANNEL3 = 0x4,
	NEO_HIST_CHANNEL4 = 0x8,
};

/**
 * enum neoisp_hdr_merge_bpp - size of pixel components definition for
 *			       the HDR merge unit.
 *
 * @NEO_HDR_MERGE_BPP_12BPP:	12 bpp
 * @NEO_HDR_MERGE_BPP_14BPP:	14 bpp
 * @NEO_HDR_MERGE_BPP_16BPP:	16 bpp
 * @NEO_HDR_MERGE_BPP_20BPP:	20 bpp
 */
enum neoisp_hdr_merge_bpp {
	NEO_HDR_MERGE_BPP_12BPP = 0,
	NEO_HDR_MERGE_BPP_14BPP = 1,
	NEO_HDR_MERGE_BPP_16BPP = 2,
	NEO_HDR_MERGE_BPP_20BPP = 3,
};

/**
 * Statistics and Histogram (stat)
 */

#define NEO_HIST_BIN_SIZE 64
#define GET_HIST_MEM_OFFSET(histId, roiId)                \
	(histId * NEO_RGBIR_ROI_CNT * NEO_HIST_BIN_SIZE + \
	 roiId * NEO_HIST_BIN_SIZE)

/* This value is used to disable a ROI histogram. */
#define HIST_ROI_INVALID_IMAGE_GEOMETRY 65535

/*
 * Scaling (gain) factor for the histogram bin determination.
 * The value specified is in u8.16 format.
 *
 * The default scaling value is calculated with a default 20-bit range.
 * Indeed the expected bit range to reach at the HDR merge unit
 * (upstream to the STAT and the RGBIR units) is 20-bit range.
 *
 * defaultScaleValue = maxBins * 2^16 / 2^20
 *
 */
#define HIST_SCALE_DEFAULT ((NEO_HIST_BIN_SIZE << 16) >> 20)

/**
 * OBWB shared definitions between algorithms and IPA context.
 */

/* OBWB Color channels: R, Gr, Gb, B */
static constexpr unsigned int kObwbChannelsCount = 4;
/* OBWB instances: OBWB0, OBWB1 and OBWB2 */
static constexpr unsigned int kObwbInstancesCount = 3;

/* OBWB Color channels array: R, Gr, Gb, B */
template<class T>
using ChannelArray = std::array<T, kObwbChannelsCount>;
/* OBWB instances array: OBWB0, OBWB1 and OBWB2 */
template<class T>
using ObwbArray = std::array<T, kObwbInstancesCount>;

/**
 * AutoFocus
 */

#define NEO_AF_BLOCK_NB_X 3
#define NEO_AF_BLOCK_NB_Y 3

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
