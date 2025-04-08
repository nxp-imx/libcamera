/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 Color Correction Matrix control algorithm
 *     src/ipa/rkisp1/algorithms/ccm.h
 * Copyright (C) 2024, Ideas On Board
 *
 * ccm.h - Color Correction Matrix control algorithm
 * Copyright 2024 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include "libcamera/internal/matrix.h"

#include "libipa/interpolator.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class Ccm : public Algorithm
{
public:
	Ccm() {}
	~Ccm() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     neoisp_meta_params_s *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const neoisp_meta_stats_s *stats,
		     ControlList &metadata) override;

private:
	void parseYaml(const YamlObject &tuningData);
	void setParameters(neoisp_meta_params_s *params,
			   const Matrix<float, 3, 3> &matrix,
			   const Matrix<int32_t, 3, 1> &offsets);

	unsigned int ct_;
	Interpolator<Matrix<float, 3, 3>> ccm_;
	Interpolator<Matrix<int32_t, 3, 1>> offsets_;
};

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
