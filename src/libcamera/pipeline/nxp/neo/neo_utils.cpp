/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * neo-utils.cpp - Helpers for NXP NEO pipeline
 * Copyright 2024 NXP
 */


#include <sstream>
#include <string>

#include "neo_utils.h"

namespace libcamera {

namespace nxpneo {

std::string SensorChannel::toString() const
{
	std::stringstream ss;
	ss << "valid " << valid_
	   << " crossbar pad " << xbarPad_
	   << " virtual channel " << xbarPadStream_;

	return ss.str();
}

std::string SensorChannelInput0::toString() const
{
	std::stringstream ss;
	ss << SensorChannel::toString()
	   << " skipTopLines " << skipTopLines;

	return ss.str();
}

std::string SensorChannelInput1::toString() const
{
	std::stringstream ss;
	ss << SensorChannel::toString()
	   << " mbusFormat " << mbusFormat;

	return ss.str();
}

std::string SensorChannelEd::toString() const
{
	std::stringstream ss;
	ss << SensorChannel::toString()
	   << " mbusFormat " << mbusFormat << " lines " << lines;

	return ss.str();
}

} // namespace nxpneo

} // namespace libcamera
