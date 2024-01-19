/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * neo-utils.cpp - Helpers for NXP NEO pipeline
 * Copyright 2024 NXP
 */


#include <sstream>
#include <string>

#include "neo_utils.h"

namespace libcamera {


std::string NxpNeoSensorChannel::toString() const
{
	std::stringstream ss;
	ss << "valid " << valid_
	   << " crossbar pad " << xbarPad_
	   << " virtual channel " << xbarPadStream_;

	return ss.str();
}

std::string NxpNeoSensorChannelInput0::toString() const
{
	std::stringstream ss;
	ss << NxpNeoSensorChannel::toString()
	   << " skipTopLines " << skipTopLines;

	return ss.str();
}

std::string NxpNeoSensorChannelInput1::toString() const
{
	std::stringstream ss;
	ss << NxpNeoSensorChannel::toString()
	   << " mbusFormat " << mbusFormat;

	return ss.str();
}

std::string NxpNeoSensorChannelEd::toString() const
{
	std::stringstream ss;
	ss << NxpNeoSensorChannel::toString()
	   << " mbusFormat " << mbusFormat << " lines " << lines;

	return ss.str();
}

} // namespace libcamera
