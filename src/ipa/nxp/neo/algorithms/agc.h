/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on RkISP1 AGC/AEC mean-based control algorithm
 *     src/ipa/rkisp1/algorithms/agc.h
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * agc.h - NXP NEO AGC/AEC mean-based control algorithm
 * Copyright 2024-2025 NXP
 */

#pragma once

#include <linux/nxp_neoisp.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "libipa/agc_mean_luminance.h"
#include "libipa/histogram.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

class AgcStats : public AgcMeanLuminance
{
public:
	AgcStats() {}
	virtual ~AgcStats() = default;

	virtual int init(IPAContext &context, const YamlObject &tuningData) = 0;
	virtual void configure(IPAContext &context);
	virtual void setupHistograms(IPAContext &context,
				     NxpNeoParams *params) const = 0;
	virtual void setAwbGains([[maybe_unused]] IPAContext &context,
				 [[maybe_unused]] IPAFrameContext &frameContext)
	{
	}
	virtual void parseStatistics(const NxpNeoStats *stats) = 0;
	const Histogram &histogram() const { return histogram_; }

protected:
	enum HistId {
		HistId0 = 0,
		HistId1,
		HistId2,
		HistId3,
	};
	enum RoiId {
		RoiId0 = 0,
		RoiId1,
	};

	Histogram histogram_;
	IPAContextType contextType_;
};

class AgcStatsRgb : public AgcStats
{
public:
	AgcStatsRgb() { contextType_ = IPAContextTypeRgb; }

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void configure(IPAContext &context) override;
	void setupHistograms(IPAContext &context, NxpNeoParams *params) const override;
	void setAwbGains(IPAContext &context, IPAFrameContext &frameContext) override;
	void parseStatistics(const NxpNeoStats *stats) override;

private:
	int parseTuningDataRgb(const YamlObject &tuningData);
	void configureHistScale(IPAContext &context);
	double estimateLuminance(double gain) const override;

	static const RGB<uint8_t> kHistIds;
	std::vector<uint32_t> histScale_;
	bool userConfig_ = false;
	std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> rgbTriples_;
	RGB<double> awbGains_;
};

class AgcStatsIr : public AgcStats
{
public:
	AgcStatsIr() { contextType_ = IPAContextTypeIr; }

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void setupHistograms(IPAContext &context, NxpNeoParams *params) const override;
	void parseStatistics(const NxpNeoStats *stats) override;

private:
	double estimateLuminance(double gain) const override;

	static constexpr HistId kHistId = HistId0;
	/* The Ir pixel is the 4th channel within 2x2 pattern RGGIr or BGGIr. */
	static constexpr uint8_t kHistChannelIr = NEO_HIST_CHANNEL4;
};

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

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
	void fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
			  ControlList &metadata) const;

	std::map<unsigned, std::unique_ptr<AgcStats>> agcs_;
};

} /* namespace ipa::nxpneo::algorithms */
} /* namespace libcamera */
