/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * md_parser_ox.cpp
 * MetaData parser class for Omnivision embedded data format found on some
 * sensors of the OX series, not compatible with SMIA parser from RPi.
 * Copyright 2024-2025 NXP
 */

#include "md_parser_ox.h"

#include <cmath>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpCameraHelper)

namespace nxp {

/**
 * \brief Parser constructor
 * \param[in] registerList The ordered list of registers output in embedded data
 * \param[in] crcParams The parameters definining the CRC type and computation
 *
 * Registers output in embedded data is setup in the sensor as a list of
 * contiguous register addresses.
 * Embedded data format is a list of N ordered <tag, register value> pairs where
 * a register holds 8-bits data.
 * The N register values are followed by two CRC values (4 bytes each). First
 * CRC corresponds to memory group hold CRC, second one to the embedded data
 * CRC. The 4 bytes of a CRC are transmitted MSB first, with a tag preceding
 * each byte.
 */
MdParserOmniOx::MdParserOmniOx(std::initializer_list<uint32_t> registerList,
			       CrcParams &crcParams)
	: registerList_{ registerList }, crcParams_(crcParams)
{
	/* Number of registers is multiple of 4. */
	registerCount_ = registerList_.size();
	if (registerCount_ % 3) {
		registerCount_ += 3;
		registerCount_ &= ~0x03U;
		LOG(NxpCameraHelper, Debug)
			<< "Number of registers adjusted";
	}

	/* Force early init of the CRC LUT. */
	if (crcParams_.type != CrcNone)
		(void)crc32LeLut();
}

MdParserOmniOx::Status
MdParserOmniOx::fetchRegister(libcamera::Span<const uint8_t> buffer,
			      uint32_t registerOffset, uint8_t *value)
{
	/*
	 * Each register value is represented in memory as a 8-bit
	 * <tag, register value> pair. If the metadata was transmitted over a
	 * MIPI-CSI2 channel whose Data Type format is wider than 8 bits, each
	 * element of the pair is prepended by the necessary padding to match
	 * the DT. Using a 16-bit Data Type, the byte-level memory layout is:
	 * 0: Tag format padding (undefined)
	 * 1: Tag
	 * 2: Register value format padding (undefined)
	 * 3: Register value
	 * Conversely, no padding is present when using a 8-bit DT channel.
	 */
	bool padding = !!(bitsPerPixel_ > 8);
	size_t registerSpan = 2 * (padding ? sizeof(uint16_t) : sizeof(uint8_t));
	size_t baseOffset = registerOffset * registerSpan;

	if (baseOffset + registerSpan > buffer.size())
		return Status::ERROR;

	size_t tagOffset = baseOffset + (padding ? 1 : 0);
	if (buffer[tagOffset] != kTag)
		return Status::NOTFOUND;

	size_t valueOffset = baseOffset + (padding ? 3 : 1);
	*value = buffer[valueOffset];
	return Status::OK;
}

MdParserOmniOx::Status MdParserOmniOx::parse(libcamera::Span<const uint8_t> buffer,
					     RegisterMap &registers)
{
	Status status;
	uint8_t value;
	registers.clear();

	status = Status::ERROR;
	if (crcParams_.type == CrcNone) {
		for (uint32_t i = 0; i < registerList_.size(); i++) {
			status = fetchRegister(buffer, i, &value);
			if (status != Status::OK) {
				LOG(NxpCameraHelper, Warning)
					<< "Could not read value offset " << i
					<< "/" << registerList_.size();
				return status;
			}
			registers[registerList_[i]] = value;
		}
	} else if (crcParams_.type == Crc32Le) {
		uint32_t crc32 = ~0U;

		for (uint32_t i = 0; i < registerCount_ + 8; i++) {
			status = fetchRegister(buffer, i, &value);
			if (status != Status::OK) {
				LOG(NxpCameraHelper, Warning)
					<< "Could not read value offset " << i
					<< "/" << registerList_.size();
				return status;
			}

			if (i < registerList_.size())
				registers[registerList_[i]] = value;

			crc32 = crc32Le(crc32, value);
		}

		status = ~crc32 == crcParams_.check ? Status::OK : Status::ERROR;
		if (status != Status::OK)
			LOG(NxpCameraHelper, Warning) << "CRC error";
	}

	return status;
}

/*
 * Little Endian (lsbit-first) CRC-32 computation - Sarwate algorithm
 * Reference: https://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
 */
uint32_t MdParserOmniOx::crc32Le(uint32_t crc32, uint8_t byte) const
{
	size_t index = (crc32 ^ byte) & 0xff;
	return (crc32 >> 8) ^ crc32LeLut()[index];
}

const std::array<uint32_t, 256> &MdParserOmniOx::crc32LeLut() const
{
	/* Initialize once */
	static const std::array<uint32_t, 256> lut = [this]() {
		/* Reverse the polynomial for lsb-first CRC computation */
		uint32_t reverse = 0;
		uint32_t poly = crcParams_.polynomial;
		for (uint32_t i = 0; i < 32; i++) {
			reverse <<= 1;
			reverse |= (poly & 1);
			poly >>= 1;
		}

		std::array<uint32_t, 256> initLut{};
		uint32_t crc32 = 1;
		for (uint32_t i = 128; i; i >>= 1) {
			if (crc32 & 1)
				crc32 = (crc32 >> 1) ^ reverse;
			else
				crc32 = (crc32 >> 1);
			for (int j = 0; j < 256; j += 2 * i)
				initLut[i + j] = crc32 ^ initLut[j];
		}

		return initLut;
	}();

	return lut;
}

} /* namespace nxp */

} /* namespace libcamera */
