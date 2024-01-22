/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Definitions from this file are supposed to be
 * integrated part of the linux kernel UAPI nxp_neoisp.h
 *
 * nxp-neoisp-enums.h - NXP NEO ISP enum values
 * Copyright 2024 NXP
 */

#pragma once

namespace libcamera {

namespace ipa::nxpneo {

/**
 * enum neoisp_obwb_instances -  instance of the Optical Black Correction and White Balance block.
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
	/*NEO_OBWB_CNT,*/ /* /todo: should replace the one defined in UAPI */
};

/**
 * enum neoisp_obwb_obpp -  size of pixel components outputted from the OB_WB unit.
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
 * enum neo_isp_ctemp_ibpp -  size of pixel components coming into the COLORTEMP unit.
 *
 * @NEO_CTEMP_IBPP_12BPP:	12 bpp
 * @NEO_CTEMP_IBPP_14BPP:	14 bpp
 * @NEO_CTEMP_IBPP_16BPP:	16 bpp
 * @NEO_CTEMP_IBPP_20BPP:	20 bpp
 */
enum neo_isp_ctemp_ibpp {
	NEO_CTEMP_IBPP_12BPP = 0,
	NEO_CTEMP_IBPP_14BPP = 1,
	NEO_CTEMP_IBPP_16BPP = 2,
	NEO_CTEMP_IBPP_20BPP = 3,
};

#define NEO_CTEMP_BLOCK_NB_X 8
#define NEO_CTEMP_BLOCK_NB_Y 8

} /* namespace ipa::nxpneo */

} /* namespace libcamera*/

