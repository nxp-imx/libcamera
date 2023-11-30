/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * neo_utils.h - Helpers for NXP NEO pipeline
 * Copyright 2024 NXP
 */

#pragma once

namespace libcamera {

/* i.MX95 CSI-2/0 mapped to ISI crossbar pad 2, CSI-2/1 to ISI crossbar pad 3 */
constexpr unsigned int kIsiXbarPadDefault = 2;

struct NxpNeoSensorChannel {
	NxpNeoSensorChannel(bool valid, unsigned int xbarPad,
			    unsigned int xbarPadStream)
		: valid_(valid), xbarPad_(xbarPad),
		  xbarPadStream_(xbarPadStream) {}

	bool valid_;
	unsigned int xbarPad_;
	unsigned int xbarPadStream_;

	std::string toString() const;
};

struct NxpNeoSensorChannelDcg : NxpNeoSensorChannel {
	NxpNeoSensorChannelDcg()
		: NxpNeoSensorChannel(true, kIsiXbarPadDefault, 0),
		  skipTopLines(0) {}

	std::string toString() const;
	unsigned int skipTopLines;
};

struct NxpNeoSensorChannelVs : NxpNeoSensorChannel {
	NxpNeoSensorChannelVs()
		: NxpNeoSensorChannel(false, kIsiXbarPadDefault, 0),
		  mbusFormat(0) {}

	std::string toString() const;
	unsigned int mbusFormat;
};

struct NxpNeoSensorChannelEd : NxpNeoSensorChannel {
	NxpNeoSensorChannelEd()
		: NxpNeoSensorChannel(false, kIsiXbarPadDefault, 0),
		  lines(0), mbusFormat(0) {}

	std::string toString() const;
	unsigned int lines;
	unsigned int mbusFormat;
};

struct NxpNeoSensorProperties {
	NxpNeoSensorProperties() {}

	bool hasVs() { return vsChannel_.valid_; }
	bool hasEmbedded() { return edChannel_.valid_; }

	NxpNeoSensorChannelDcg dcgChannel_;
	NxpNeoSensorChannelVs vsChannel_;
	NxpNeoSensorChannelEd edChannel_;
};

} // namespace libcamera
