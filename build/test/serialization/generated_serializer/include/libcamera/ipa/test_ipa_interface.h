/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * test_ipa_interface.h - Image Processing Algorithm interface for test
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once

#include <libcamera/ipa/core_ipa_interface.h>
#include <libcamera/ipa/ipa_interface.h>

#include <map>
#include <vector>

namespace libcamera {

namespace ipa {

namespace test {




enum class _TestCmd {
	Exit = 0,
	Init = 1,
	Start = 2,
	Stop = 3,
	Test = 4,
};

enum class _TestEventCmd {
	DummyEvent = 1,
};


enum IPAOperationCode {
	IPAOperationNone = 0,
	IPAOperationInit = 1,
	IPAOperationStart = 2,
	IPAOperationStop = 3,
};

enum class ErrorFlags {
	Error1 = 1,
	Error2 = 2,
	Error3 = 4,
	Error4 = 8,
};

struct TestStruct
{
public:
#ifndef __DOXYGEN__
	TestStruct()
		: i(0), c(static_cast<IPAOperationCode>(0)), e(static_cast<ErrorFlags>(0))
	{
	}

	TestStruct(const std::map<std::string, std::string> &_m, const std::vector<std::string> &_a, const std::string &_s1, const std::string &_s2, int32_t _i, const std::string &_s3, const IPAOperationCode &_c, const ErrorFlags &_e, const Flags<ipa::test::ErrorFlags> &_f)
		: m(_m), a(_a), s1(_s1), s2(_s2), i(_i), s3(_s3), c(_c), e(_e), f(_f)
	{
	}
#endif


	std::map<std::string, std::string> m;
	std::vector<std::string> a;
	std::string s1;
	std::string s2;
	int32_t i;
	std::string s3;
	IPAOperationCode c;
	ErrorFlags e;
	Flags<ipa::test::ErrorFlags> f;
};

class IPATestInterface : public IPAInterface
{
public:

	virtual int32_t init(
		const IPASettings &settings) = 0;

	virtual int32_t start() = 0;

	virtual void stop() = 0;

	virtual void test(
		const TestStruct &s) = 0;

	Signal<uint32_t> dummyEvent;
};

} /* namespace test */

} /* namespace ipa */

} /* namespace libcamera */