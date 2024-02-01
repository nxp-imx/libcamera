/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * neo_utils.h - Helpers for NXP NEO pipeline
 * Copyright 2024 NXP
 */

#pragma once

namespace libcamera {

namespace nxpneo {

/* i.MX95 CSI-2/0 mapped to ISI crossbar pad 2, CSI-2/1 to ISI crossbar pad 3 */
constexpr unsigned int kIsiXbarPadDefault = 2;

struct SensorChannel {
	SensorChannel(bool valid, unsigned int xbarPad,
			    unsigned int xbarPadStream)
		: valid_(valid), xbarPad_(xbarPad),
		  xbarPadStream_(xbarPadStream) {}

	bool valid_;
	unsigned int xbarPad_;
	unsigned int xbarPadStream_;

	std::string toString() const;
};

struct SensorChannelInput0 : SensorChannel {
	SensorChannelInput0()
		: SensorChannel(true, kIsiXbarPadDefault, 0),
		  skipTopLines(0) {}

	std::string toString() const;
	unsigned int skipTopLines;
};

struct SensorChannelInput1 : SensorChannel {
	SensorChannelInput1()
		: SensorChannel(false, kIsiXbarPadDefault, 0),
		  mbusFormat(0) {}

	std::string toString() const;
	unsigned int mbusFormat;
};

struct SensorChannelEd : SensorChannel {
	SensorChannelEd()
		: SensorChannel(false, kIsiXbarPadDefault, 0),
		  lines(0), mbusFormat(0) {}

	std::string toString() const;
	unsigned int lines;
	unsigned int mbusFormat;
};

struct SensorProperties {
	SensorProperties() {}

	bool hasInput1() { return input1Channel_.valid_; }
	bool hasEmbedded() { return edChannel_.valid_; }

	SensorChannelInput0 input0Channel_;
	SensorChannelInput1 input1Channel_;
	SensorChannelEd edChannel_;
};

} // namespace nxpneo

} // namespace libcamera
