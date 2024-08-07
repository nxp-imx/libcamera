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
class IPADataSerializer<ipa::nxpneo::IPAColorSpace>
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const ipa::nxpneo::IPAColorSpace &data,
		  [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		std::vector<uint8_t> retData;

		std::vector<uint8_t> primaries;
		std::tie(primaries, std::ignore) =
			IPADataSerializer<ipa::nxpneo::IPAPrimaries>::serialize(data.primaries);
		retData.insert(retData.end(), primaries.begin(), primaries.end());

		std::vector<uint8_t> transferFunction;
		std::tie(transferFunction, std::ignore) =
			IPADataSerializer<ipa::nxpneo::IPATransferFunction>::serialize(data.transferFunction);
		retData.insert(retData.end(), transferFunction.begin(), transferFunction.end());

		std::vector<uint8_t> ycbcrEncoding;
		std::tie(ycbcrEncoding, std::ignore) =
			IPADataSerializer<ipa::nxpneo::IPAYcbcrEncoding>::serialize(data.ycbcrEncoding);
		retData.insert(retData.end(), ycbcrEncoding.begin(), ycbcrEncoding.end());

		std::vector<uint8_t> range;
		std::tie(range, std::ignore) =
			IPADataSerializer<ipa::nxpneo::IPARange>::serialize(data.range);
		retData.insert(retData.end(), range.begin(), range.end());

		return {retData, {}};
	}

	static ipa::nxpneo::IPAColorSpace
	deserialize(std::vector<uint8_t> &data,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::IPAColorSpace>::deserialize(data.cbegin(), data.cend(), cs);
	}


	static ipa::nxpneo::IPAColorSpace
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		ipa::nxpneo::IPAColorSpace ret;
		std::vector<uint8_t>::const_iterator m = dataBegin;

		size_t dataSize = std::distance(dataBegin, dataEnd);

		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "primaries"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.primaries = IPADataSerializer<ipa::nxpneo::IPAPrimaries>::deserialize(m, m + 4);
		m += 4;
		dataSize -= 4;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "transferFunction"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.transferFunction = IPADataSerializer<ipa::nxpneo::IPATransferFunction>::deserialize(m, m + 4);
		m += 4;
		dataSize -= 4;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "ycbcrEncoding"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.ycbcrEncoding = IPADataSerializer<ipa::nxpneo::IPAYcbcrEncoding>::deserialize(m, m + 4);
		m += 4;
		dataSize -= 4;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "range"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.range = IPADataSerializer<ipa::nxpneo::IPARange>::deserialize(m, m + 4);

		return ret;
	}

	static ipa::nxpneo::IPAColorSpace
	deserialize(std::vector<uint8_t> &data,
		    [[maybe_unused]] std::vector<SharedFD> &fds,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::IPAColorSpace>::deserialize(data.cbegin(), data.cend(), cs);
	}

	static ipa::nxpneo::IPAColorSpace
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::IPAColorSpace>::deserialize(dataBegin, dataEnd, cs);
	}
};

template<>
class IPADataSerializer<ipa::nxpneo::InitParams>
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const ipa::nxpneo::InitParams &data,
		  ControlSerializer *cs)
	{
		std::vector<uint8_t> retData;

		std::vector<uint8_t> hwRevision;
		std::tie(hwRevision, std::ignore) =
			IPADataSerializer<uint32_t>::serialize(data.hwRevision);
		retData.insert(retData.end(), hwRevision.begin(), hwRevision.end());

		std::vector<uint8_t> hwCapabilities;
		std::tie(hwCapabilities, std::ignore) =
			IPADataSerializer<uint32_t>::serialize(data.hwCapabilities);
		retData.insert(retData.end(), hwCapabilities.begin(), hwCapabilities.end());

		std::vector<uint8_t> supportedParamsBlocks;
		std::tie(supportedParamsBlocks, std::ignore) =
			IPADataSerializer<uint64_t>::serialize(data.supportedParamsBlocks);
		retData.insert(retData.end(), supportedParamsBlocks.begin(), supportedParamsBlocks.end());

		std::vector<uint8_t> sensorEntity;
		std::tie(sensorEntity, std::ignore) =
			IPADataSerializer<std::string>::serialize(data.sensorEntity);
		appendPOD<uint32_t>(retData, sensorEntity.size());
		retData.insert(retData.end(), sensorEntity.begin(), sensorEntity.end());

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

		if (data.sensorControlList.size() > 0) {
			std::vector<uint8_t> sensorControlList;
			std::tie(sensorControlList, std::ignore) =
				IPADataSerializer<ControlList>::serialize(data.sensorControlList, cs);
			appendPOD<uint32_t>(retData, sensorControlList.size());
			retData.insert(retData.end(), sensorControlList.begin(), sensorControlList.end());
		} else {
			appendPOD<uint32_t>(retData, 0);
		}

		std::vector<uint8_t> lensPresent;
		std::tie(lensPresent, std::ignore) =
			IPADataSerializer<bool>::serialize(data.lensPresent);
		retData.insert(retData.end(), lensPresent.begin(), lensPresent.end());

		return {retData, {}};
	}

	static ipa::nxpneo::InitParams
	deserialize(std::vector<uint8_t> &data,
		    ControlSerializer *cs)
	{
		return IPADataSerializer<ipa::nxpneo::InitParams>::deserialize(data.cbegin(), data.cend(), cs);
	}


	static ipa::nxpneo::InitParams
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    ControlSerializer *cs)
	{
		ipa::nxpneo::InitParams ret;
		std::vector<uint8_t>::const_iterator m = dataBegin;

		size_t dataSize = std::distance(dataBegin, dataEnd);

		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "hwRevision"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.hwRevision = IPADataSerializer<uint32_t>::deserialize(m, m + 4);
		m += 4;
		dataSize -= 4;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "hwCapabilities"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.hwCapabilities = IPADataSerializer<uint32_t>::deserialize(m, m + 4);
		m += 4;
		dataSize -= 4;


		if (dataSize < 8) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "supportedParamsBlocks"
				<< ": not enough data, expected "
				<< (8) << ", got " << (dataSize);
			return ret;
		}
		ret.supportedParamsBlocks = IPADataSerializer<uint64_t>::deserialize(m, m + 8);
		m += 8;
		dataSize -= 8;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorEntitySize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t sensorEntitySize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < sensorEntitySize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorEntity"
				<< ": not enough data, expected "
				<< (sensorEntitySize) << ", got " << (dataSize);
			return ret;
		}
		ret.sensorEntity =
			IPADataSerializer<std::string>::deserialize(m, m + sensorEntitySize);
		m += sensorEntitySize;
		dataSize -= sensorEntitySize;


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
		m += sensorControlsSize;
		dataSize -= sensorControlsSize;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorControlListSize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t sensorControlListSize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < sensorControlListSize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorControlList"
				<< ": not enough data, expected "
				<< (sensorControlListSize) << ", got " << (dataSize);
			return ret;
		}
		if (sensorControlListSize > 0)
			ret.sensorControlList =
				IPADataSerializer<ControlList>::deserialize(m, m + sensorControlListSize, cs);
		m += sensorControlListSize;
		dataSize -= sensorControlListSize;


		if (dataSize < 1) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "lensPresent"
				<< ": not enough data, expected "
				<< (1) << ", got " << (dataSize);
			return ret;
		}
		ret.lensPresent = IPADataSerializer<bool>::deserialize(m, m + 1);

		return ret;
	}

	static ipa::nxpneo::InitParams
	deserialize(std::vector<uint8_t> &data,
		    [[maybe_unused]] std::vector<SharedFD> &fds,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::InitParams>::deserialize(data.cbegin(), data.cend(), cs);
	}

	static ipa::nxpneo::InitParams
	deserialize(std::vector<uint8_t>::const_iterator dataBegin,
		    std::vector<uint8_t>::const_iterator dataEnd,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
		    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
		    ControlSerializer *cs = nullptr)
	{
		return IPADataSerializer<ipa::nxpneo::InitParams>::deserialize(dataBegin, dataEnd, cs);
	}
};

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

		if (data.lensControls.size() > 0) {
			std::vector<uint8_t> lensControls;
			std::tie(lensControls, std::ignore) =
				IPADataSerializer<ControlInfoMap>::serialize(data.lensControls, cs);
			appendPOD<uint32_t>(retData, lensControls.size());
			retData.insert(retData.end(), lensControls.begin(), lensControls.end());
		} else {
			appendPOD<uint32_t>(retData, 0);
		}

		if (data.sensorControlList.size() > 0) {
			std::vector<uint8_t> sensorControlList;
			std::tie(sensorControlList, std::ignore) =
				IPADataSerializer<ControlList>::serialize(data.sensorControlList, cs);
			appendPOD<uint32_t>(retData, sensorControlList.size());
			retData.insert(retData.end(), sensorControlList.begin(), sensorControlList.end());
		} else {
			appendPOD<uint32_t>(retData, 0);
		}

		std::vector<uint8_t> mode;
		std::tie(mode, std::ignore) =
			IPADataSerializer<ipa::nxpneo::IPAPipelineMode>::serialize(data.mode);
		retData.insert(retData.end(), mode.begin(), mode.end());

		std::vector<uint8_t> colorSpace;
		std::tie(colorSpace, std::ignore) =
			IPADataSerializer<ipa::nxpneo::IPAColorSpace>::serialize(data.colorSpace, cs);
		appendPOD<uint32_t>(retData, colorSpace.size());
		retData.insert(retData.end(), colorSpace.begin(), colorSpace.end());

		std::vector<uint8_t> bitsPerPixelAuxiliary;
		std::tie(bitsPerPixelAuxiliary, std::ignore) =
			IPADataSerializer<uint32_t>::serialize(data.bitsPerPixelAuxiliary);
		retData.insert(retData.end(), bitsPerPixelAuxiliary.begin(), bitsPerPixelAuxiliary.end());

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
		m += sensorControlsSize;
		dataSize -= sensorControlsSize;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "lensControlsSize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t lensControlsSize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < lensControlsSize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "lensControls"
				<< ": not enough data, expected "
				<< (lensControlsSize) << ", got " << (dataSize);
			return ret;
		}
		if (lensControlsSize > 0)
			ret.lensControls =
				IPADataSerializer<ControlInfoMap>::deserialize(m, m + lensControlsSize, cs);
		m += lensControlsSize;
		dataSize -= lensControlsSize;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorControlListSize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t sensorControlListSize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < sensorControlListSize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "sensorControlList"
				<< ": not enough data, expected "
				<< (sensorControlListSize) << ", got " << (dataSize);
			return ret;
		}
		if (sensorControlListSize > 0)
			ret.sensorControlList =
				IPADataSerializer<ControlList>::deserialize(m, m + sensorControlListSize, cs);
		m += sensorControlListSize;
		dataSize -= sensorControlListSize;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "mode"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.mode = IPADataSerializer<ipa::nxpneo::IPAPipelineMode>::deserialize(m, m + 4);
		m += 4;
		dataSize -= 4;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "colorSpaceSize"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		const size_t colorSpaceSize = readPOD<uint32_t>(m, 0, dataEnd);
		m += 4;
		dataSize -= 4;
		if (dataSize < colorSpaceSize) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "colorSpace"
				<< ": not enough data, expected "
				<< (colorSpaceSize) << ", got " << (dataSize);
			return ret;
		}
		ret.colorSpace =
			IPADataSerializer<ipa::nxpneo::IPAColorSpace>::deserialize(m, m + colorSpaceSize, cs);
		m += colorSpaceSize;
		dataSize -= colorSpaceSize;


		if (dataSize < 4) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize " << "bitsPerPixelAuxiliary"
				<< ": not enough data, expected "
				<< (4) << ", got " << (dataSize);
			return ret;
		}
		ret.bitsPerPixelAuxiliary = IPADataSerializer<uint32_t>::deserialize(m, m + 4);

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