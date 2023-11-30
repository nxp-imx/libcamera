/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * md_parser_ox.h
 * MetaData parser class for Omnivision embedded data format found on some
 * sensors of the OX series, not compatible with SMIA parser from RPi.
 * Copyright 2024 NXP
 */
#include <vector>
#include "md_parser.h"

namespace libcamera {

namespace nxp {

using MdParser = RPiController::MdParser;

class MdParserOmniOx : public MdParser
{
public:
	MdParserOmniOx(std::initializer_list<uint32_t> registerList);
	MdParser::Status parse(libcamera::Span<const uint8_t> buffer,
			       RegisterMap &registers) override;

private:
	MdParser::Status fetchRegister(libcamera::Span<const uint8_t> buffer,
				       uint32_t registerOffset, uint8_t *value);

	static constexpr unsigned int kTag = 0xDA;
	std::vector<uint32_t> registerList_;
	uint32_t registerCount_;
};

} /* namespace nxp */

} /* namespace libcamera */
