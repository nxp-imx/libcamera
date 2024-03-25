/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * md_parser_ox.cpp
 * MetaData parser class for Omnivision embedded data format found on some
 * sensors of the OX series, not compatible with SMIA parser from RPi.
 * Copyright 2024 NXP
 */

#include <cmath>

#include <libcamera/base/log.h>

#include "md_parser_ox.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpCameraHelper)

namespace nxp {

/**
 * \brief Parser constructor
 * \param[in] registerList ordered list of registers output in embedded data
 *
 * Registers output in embedded data is setup in the sensor as a list of
 * contiguous register addresses.
 * Embedded data format is a list of N ordered <tag, register value> pairs where
 * a register holds 8-bits data.
 * The N register values are followed by two CRC values (4 bytes each). First
 * CRC corresponds to memory group hold CRC, second one to the embedded data
 * CRC. The 4 bytes of a CRC are transmitted MSB first, with a tag preceding
 * each byte.
 * Tag and data format are 8 bits, but the embedded data format width is the
 * same as the one used for pixel lines. Because pixel data width is more than
 * 8 bits, embedded data memory is laid out on 16 bits words MSB-aligned.
 */
MdParserOmniOx::MdParserOmniOx(std::initializer_list<uint32_t> registerList)
	: registerList_{ registerList }
{
	/* Number of registers is multiple of 4 */
	registerCount_ = registerList_.size();
	if (registerCount_ % 3) {
		registerCount_ += 3;
		registerCount_ &= ~0x03U;
		LOG(NxpCameraHelper, Info)
			<< "Number of registers adjusted";
	}
}

MdParserOmniOx::Status
MdParserOmniOx::fetchRegister(libcamera::Span<const uint8_t> buffer,
			      uint32_t registerOffset, uint8_t *value)
{
	/*
	 * Each register value is represented in memory at byte level on 4 bytes:
	 * 0: Tag format padding (undefined)
	 * 1: Tag
	 * 2: Register value format padding (undefined)
	 * 3: Register value
	 */
	size_t byteOffset = registerOffset * 4;
	if (byteOffset >= buffer.size())
		return Status::ERROR;
	if (buffer[byteOffset + 1] != kTag)
		return Status::NOTFOUND;

	*value = buffer[byteOffset + 3];
	return Status::OK;
}

MdParserOmniOx::Status MdParserOmniOx::parse(libcamera::Span<const uint8_t> buffer,
					     RegisterMap &registers)
{
	Status status;
	uint8_t value;
	registers.clear();

	for (uint32_t i = 0; i < registerList_.size(); i++) {
		status = fetchRegister(buffer, i, &value);
		if (status != Status::OK) {
			LOG(NxpCameraHelper, Error)
				<< "Could not read register offset " << i
				<< "/" << registerList_.size();
			return status;
		}
		registers[registerList_[i]] = value;
	}

	/*
	 * Sanity check : 2 CRCs, 4-byte each, are appended after the register
	 * values i.e. 8 bytes. Anything register attempt after that should
	 * report an error as no more tag will be found.
	 * \todo we may compute checksum value
	 */
	for (uint32_t i = 0; i < 8; i++) {
		status = fetchRegister(buffer, registerCount_ + i, &value);
		if (status != Status::OK) {
			LOG(NxpCameraHelper, Error)
				<< "Could not read checksum offset " << i;
			return status;
		}
	}
	status = fetchRegister(buffer, registerCount_ + 8, &value);
	if (status == Status::OK) {
		LOG(NxpCameraHelper, Error)
			<< "Valid tag found after the 8 CRC bytes";
		return Status::ERROR;
	}

	return Status::OK;
}

} /* namespace nxp */

} /* namespace libcamera */
