/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm data serializer for nxpneo
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once

#include <tuple>
#include <vector>

#include <libcamera/ipa/nxpneo_ipa_interface.h>
#include <libcamera/ipa/core_ipa_serializer.h>

#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_data_serializer.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPADataSerializer)

template<>
class IPADataSerializer<ipa::nxpneo::IPAConfigInfo>
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const ipa::nxpneo::IPAConfigInfo &data,
		  ControlSerializer *cs)
	{
		std::vector<uint8_t> retData;

		std::vector<uint8_t> sensorInfo;
		std::tie(sensorInfo, std::ignore) =
			IPADataSerializer<libcamera::IPACameraSensorInfo>::serialize(data.sensorInfo, cs);
		appendPOD<uint32_t>(retData, sensorInfo.size());
		retData.insert(retData.end(), sensorInfo.begin(), sensorInfo.end());

		if (data.sensorControls.size() > 0) {
			std::vector<uint8_t> sensorControls;
			std::tie(sensorControls, std::ignore) =
				IPADataSerializer<ControlInfoMap>::serialize(data.sensorControls, cs);
			appendPOD<uint32_t>(retData, sensorControls.size());
			retData.insert(retData.end(), sensorControls.begin(), sensorControls.end());
		} else {
			appendPOD<uint32_t>(retData, 0);
		}

		return {retData, {}};
	}

	static ipa::nxpneo::IPAConfigInfo
	deserialize(std::vector<uint8_t> &data,
		    ControlSerializer *cs)
	{
		return IPADataSerializer<ipa::nxpneo::IPAConfigInfo>::deserialize(data.cbegin(), data.cend(), cs);
	}


	static ipa::nxpneo::IPAConfigInfo
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    ControlSerializer *cs)
	{
		ipa::nxpneo::IPAConfigInfo ret;
		std::vector<uint8_t>::const_iterator m = dataBegin;

		size_t dataSize = std::distance(dataBegin, dataEnd);

		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorInfoSize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t sensorInfoSize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < sensorInfoSize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorInfo"
				<< ": not enough data, expected "
				<< (sensorInfoSize) << ", got " << (dataSize);
			return ret;
		}
		ret.sensorInfo =
			IPADataSerializer<libcamera::IPACameraSensorInfo>::deserialize(m, m + sensorInfoSize, cs);
		m += sensorInfoSize;
		dataSize -= sensorInfoSize;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorControlsSize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t sensorControlsSize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < sensorControlsSize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorControls"
				<< ": not enough data, expected "
				<< (sensorControlsSize) << ", got " << (dataSize);
			return ret;
		}
		if (sensorControlsSize > 0)
			ret.sensorControls =
				IPADataSerializer<ControlInfoMap>::deserialize(m, m + sensorControlsSize, cs);

		return ret;
	}

	static ipa::nxpneo::IPAConfigInfo
	deserialize(std::vector<uint8_t> &data,
		    [[maybe_unused]] std::vector<SharedFD> &fds,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::IPAConfigInfo>::deserialize(data.cbegin(), data.cend(), cs);
	}

	static ipa::nxpneo::IPAConfigInfo
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::IPAConfigInfo>::deserialize(dataBegin, dataEnd, cs);
	}
};

template<>
class IPADataSerializer<ipa::nxpneo::DelayedControlsParams>
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const ipa::nxpneo::DelayedControlsParams &data,
		  [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		std::vector<uint8_t> retData;

		std::vector<uint8_t> delay;
		std::tie(delay, std::ignore) =
			IPADataSerializer<uint32_t>::serialize(data.delay);
		retData.insert(retData.end(), delay.begin(), delay.end());

		std::vector<uint8_t> priorityWrite;
		std::tie(priorityWrite, std::ignore) =
			IPADataSerializer<bool>::serialize(data.priorityWrite);
		retData.insert(retData.end(), priorityWrite.begin(), priorityWrite.end());

		return {retData, {}};
	}

	static ipa::nxpneo::DelayedControlsParams
	deserialize(std::vector<uint8_t> &data,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::DelayedControlsParams>::deserialize(data.cbegin(), data.cend(), cs);
	}


	static ipa::nxpneo::DelayedControlsParams
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		ipa::nxpneo::DelayedControlsParams ret;
		std::vector<uint8_t>::const_iterator m = dataBegin;

		size_t dataSize = std::distance(dataBegin, dataEnd);

		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "delay"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.delay = IPADataSerializer<uint32_t>::deserialize(m, m + 4);
		m += 4;
		dataSize -= 4;


		if (dataSize < 1) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "priorityWrite"
				<< ": not enough data, expected "
				<< (1) << ", got " << (dataSize);
			return ret;
		}
		ret.priorityWrite = IPADataSerializer<bool>::deserialize(m, m + 1);

		return ret;
	}

	static ipa::nxpneo::DelayedControlsParams
	deserialize(std::vector<uint8_t> &data,
		    [[maybe_unused]] std::vector<SharedFD> &fds,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::DelayedControlsParams>::deserialize(data.cbegin(), data.cend(), cs);
	}

	static ipa::nxpneo::DelayedControlsParams
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::DelayedControlsParams>::deserialize(dataBegin, dataEnd, cs);
	}
};

template<>
class IPADataSerializer<ipa::nxpneo::SensorConfig>
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const ipa::nxpneo::SensorConfig &data,
		  [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		std::vector<uint8_t> retData;

		std::vector<uint8_t> delayedControlsParams;
		std::tie(delayedControlsParams, std::ignore) =
			IPADataSerializer<std::map<int32_t, ipa::nxpneo::DelayedControlsParams>>::serialize(data.delayedControlsParams, cs);
		appendPOD<uint32_t>(retData, delayedControlsParams.size());
		retData.insert(retData.end(), delayedControlsParams.begin(), delayedControlsParams.end());

		std::vector<uint8_t> embeddedTopLines;
		std::tie(embeddedTopLines, std::ignore) =
			IPADataSerializer<uint32_t>::serialize(data.embeddedTopLines);
		retData.insert(retData.end(), embeddedTopLines.begin(), embeddedTopLines.end());

		std::vector<uint8_t> rgbIr;
		std::tie(rgbIr, std::ignore) =
			IPADataSerializer<bool>::serialize(data.rgbIr);
		retData.insert(retData.end(), rgbIr.begin(), rgbIr.end());

		return {retData, {}};
	}

	static ipa::nxpneo::SensorConfig
	deserialize(std::vector<uint8_t> &data,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::SensorConfig>::deserialize(data.cbegin(), data.cend(), cs);
	}


	static ipa::nxpneo::SensorConfig
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		ipa::nxpneo::SensorConfig ret;
		std::vector<uint8_t>::const_iterator m = dataBegin;

		size_t dataSize = std::distance(dataBegin, dataEnd);

		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "delayedControlsParamsSize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t delayedControlsParamsSize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < delayedControlsParamsSize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "delayedControlsParams"
				<< ": not enough data, expected "
				<< (delayedControlsParamsSize) << ", got " << (dataSize);
			return ret;
		}
		ret.delayedControlsParams =
			IPADataSerializer<std::map<int32_t, ipa::nxpneo::DelayedControlsParams>>::deserialize(m, m + delayedControlsParamsSize, cs);
		m += delayedControlsParamsSize;
		dataSize -= delayedControlsParamsSize;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "embeddedTopLines"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.embeddedTopLines = IPADataSerializer<uint32_t>::deserialize(m, m + 4);
		m += 4;
		dataSize -= 4;


		if (dataSize < 1) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "rgbIr"
				<< ": not enough data, expected "
				<< (1) << ", got " << (dataSize);
			return ret;
		}
		ret.rgbIr = IPADataSerializer<bool>::deserialize(m, m + 1);

		return ret;
	}

	static ipa::nxpneo::SensorConfig
	deserialize(std::vector<uint8_t> &data,
		    [[maybe_unused]] std::vector<SharedFD> &fds,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::SensorConfig>::deserialize(data.cbegin(), data.cend(), cs);
	}

	static ipa::nxpneo::SensorConfig
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::SensorConfig>::deserialize(dataBegin, dataEnd, cs);
	}
};


} /* namespace libcamera */