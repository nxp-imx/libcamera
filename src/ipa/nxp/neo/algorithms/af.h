/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025-2026 NXP
 *
 * Autofocus control algorithm
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "af_base.h"
#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class Af : public Algorithm
{
public:
	Af();
	~Af() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context,
			  const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     NxpNeoParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const NxpNeoStats *stats,
		     ControlList &metadata) override;

private:
	static void windowsToNative(Span<Rectangle> windows,
				    uint16_t cropX, uint16_t cropY,
				    double scaleX, double scaleY);

	std::unique_ptr<AfBase> algo_;
	AfStatus status_;

	static constexpr unsigned kFilterTapsCount = 9;
	static constexpr unsigned kFiltersCount = 2;
	std::array<std::array<int8_t, kFilterTapsCount>, kFiltersCount> filters_;

	/*
	 * Use Sobel filter as default contrast detection filter.
	 * https://en.wikipedia.org/wiki/Sobel_operator
	 */
	static constexpr std::array<int8_t, kFilterTapsCount>
		kFilter0Default{ -1, 0, 1, -2, 0, 2, -1, 0, 1 };
	static constexpr std::array<int8_t, kFilterTapsCount>
		kFilter1Default{ -1, -2, -1, 0, 0, 0, 1, 2, 1 };

	static constexpr unsigned kShiftDefault = 8;
	static constexpr unsigned kShiftMax = 31;
	std::array<uint8_t, kFiltersCount> shifts_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
