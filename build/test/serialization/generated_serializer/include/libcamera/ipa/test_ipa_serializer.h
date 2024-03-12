/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * test_ipa_serializer.h - Image Processing Algorithm data serializer for test
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once

#include <tuple>
#include <vector>

#include <libcamera/ipa/test_ipa_interface.h>
#include <libcamera/ipa/core_ipa_serializer.h>

#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_data_serializer.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPADataSerializer)

template<>
class IPADataSerializer<ipa::test::TestStruct>
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const ipa::test::TestStruct &data,
		  [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		std::vector<uint8_t> retData;

		std::vector<uint8_t> m;
		std::tie(m, std::ignore) =
			IPADataSerializer<std::map<std::string, std::string>>::serialize(data.m, cs);
		appendPOD<uint32_t>(retData, m.size());
		retData.insert(retData.end(), m.begin(), m.end());

		std::vector<uint8_t> a;
		std::tie(a, std::ignore) =
			IPADataSerializer<std::vector<std::string>>::serialize(data.a, cs);
		appendPOD<uint32_t>(retData, a.size());
		retData.insert(retData.end(), a.begin(), a.end());

		std::vector<uint8_t> s1;
		std::tie(s1, std::ignore) =
			IPADataSerializer<std::string>::serialize(data.s1);
		appendPOD<uint32_t>(retData, s1.size());
		retData.insert(retData.end(), s1.begin(), s1.end());

		std::vector<uint8_t> s2;
		std::tie(s2, std::ignore) =
			IPADataSerializer<std::string>::serialize(data.s2);
		appendPOD<uint32_t>(retData, s2.size());
		retData.insert(retData.end(), s2.begin(), s2.end());

		std::vector<uint8_t> i;
		std::tie(i, std::ignore) =
			IPADataSerializer<int32_t>::serialize(data.i);
		retData.insert(retData.end(), i.begin(), i.end());

		std::vector<uint8_t> s3;
		std::tie(s3, std::ignore) =
			IPADataSerializer<std::string>::serialize(data.s3);
		appendPOD<uint32_t>(retData, s3.size());
		retData.insert(retData.end(), s3.begin(), s3.end());

		std::vector<uint8_t> c;
		std::tie(c, std::ignore) =
			IPADataSerializer<uint32_t>::serialize(data.c);
		retData.insert(retData.end(), c.begin(), c.end());

		std::vector<uint8_t> e;
		std::tie(e, std::ignore) =
			IPADataSerializer<uint32_t>::serialize(static_cast<uint32_t>(data.e));
		retData.insert(retData.end(), e.begin(), e.end());

		std::vector<uint8_t> f;
		std::tie(f, std::ignore) =
			IPADataSerializer<Flags<ipa::test::ErrorFlags>>::serialize(data.f);
		retData.insert(retData.end(), f.begin(), f.end());

		return {retData, {}};
	}

	static ipa::test::TestStruct
	deserialize(std::vector<uint8_t> &data,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::test::TestStruct>::deserialize(data.cbegin(), data.cend(), cs);
	}


	static ipa::test::TestStruct
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		ipa::test::TestStruct ret;
		std::vector<uint8_t>::const_iterator m = dataBegin;

		size_t dataSize = std::distance(dataBegin, dataEnd);

		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "mSize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t mSize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < mSize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "m"
				<< ": not enough data, expected "
				<< (mSize) << ", got " << (dataSize);
			return ret;
		}
		ret.m =
			IPADataSerializer<std::map<std::string, std::string>>::deserialize(m, m + mSize, cs);
		m += mSize;
		dataSize -= mSize;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "aSize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t aSize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < aSize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "a"
				<< ": not enough data, expected "
				<< (aSize) << ", got " << (dataSize);
			return ret;
		}
		ret.a =
			IPADataSerializer<std::vector<std::string>>::deserialize(m, m + aSize, cs);
		m += aSize;
		dataSize -= aSize;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "s1Size"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t s1Size = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < s1Size) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "s1"
				<< ": not enough data, expected "
				<< (s1Size) << ", got " << (dataSize);
			return ret;
		}
		ret.s1 =
			IPADataSerializer<std::string>::deserialize(m, m + s1Size);
		m += s1Size;
		dataSize -= s1Size;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "s2Size"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t s2Size = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < s2Size) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "s2"
				<< ": not enough data, expected "
				<< (s2Size) << ", got " << (dataSize);
			return ret;
		}
		ret.s2 =
			IPADataSerializer<std::string>::deserialize(m, m + s2Size);
		m += s2Size;
		dataSize -= s2Size;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "i"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.i = IPADataSerializer<int32_t>::deserialize(m, m + 4);
		m += 4;
		dataSize -= 4;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "s3Size"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t s3Size = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < s3Size) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "s3"
				<< ": not enough data, expected "
				<< (s3Size) << ", got " << (dataSize);
			return ret;
		}
		ret.s3 =
			IPADataSerializer<std::string>::deserialize(m, m + s3Size);
		m += s3Size;
		dataSize -= s3Size;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "c"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.c = static_cast<ipa::test::IPAOperationCode>(IPADataSerializer<uint32_t>::deserialize(m, m + 4));
		m += 4;
		dataSize -= 4;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "e"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.e = static_cast<ipa::test::ErrorFlags>(IPADataSerializer<uint32_t>::deserialize(m, m + 4));
		m += 4;
		dataSize -= 4;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "f"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.f = IPADataSerializer<Flags<ipa::test::ErrorFlags>>::deserialize(m, m + 4);

		return ret;
	}

	static ipa::test::TestStruct
	deserialize(std::vector<uint8_t> &data,
		    [[maybe_unused]] std::vector<SharedFD> &fds,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::test::TestStruct>::deserialize(data.cbegin(), data.cend(), cs);
	}

	static ipa::test::TestStruct
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::test::TestStruct>::deserialize(dataBegin, dataEnd, cs);
	}
};


} /* namespace libcamera */