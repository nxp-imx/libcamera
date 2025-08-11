/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm interface for nxpneo
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <libcamera/base/flags.h>
#include <libcamera/base/signal.h>

#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>

#include <libcamera/ipa/core_ipa_interface.h>
#include <libcamera/ipa/ipa_interface.h>

namespace libcamera {

namespace ipa {

namespace nxpneo {




enum class _NxpNeoCmd {
	Exit = 0,
	Init = 1,
	Start = 2,
	Stop = 3,
	Configure = 4,
	MapBuffers = 5,
	UnmapBuffers = 6,
	QueueRequest = 7,
	ComputeParams = 8,
	ProcessStats = 9,
};

enum class _NxpNeoEventCmd {
	ParamsComputed = 1,
	SetSensorControls = 2,
	MetadataReady = 3,
};


enum IPAPrimaries {
	IPAPrimariesRaw = 0,
	IPAPrimariesSmpte170m = 1,
	IPAPrimariesRec709 = 2,
	IPAPrimariesRec2020 = 3,
};

enum IPATransferFunction {
	IPATransferFunctionLinear = 0,
	IPATransferFunctionSrgb = 1,
	IPATransferFunctionRec709 = 2,
};

enum IPAYcbcrEncoding {
	IPAYcbcrEncodingNone = 0,
	IPAYcbcrEncodingRec601 = 1,
	IPAYcbcrEncodingRec709 = 2,
	IPAYcbcrEncodingRec2020 = 3,
};

enum IPARange {
	IPARangeFull = 0,
	IPARangeLimited = 1,
};

enum IPABufferType {
	IPABufferTypeImage0 = 0,
	IPABufferTypeImage1 = 1,
	IPABufferTypeEData = 2,
	IPABufferTypeParams = 3,
	IPABufferTypeStats = 4,
	IPABufferTypeFrame = 5,
	IPABufferTypeIr = 6,
};

enum IPAContextType {
	IPAContextTypeRgb = 0,
	IPAContextTypeIr = 1,
};

enum IPAModeType {
	IPAModeTypeStandard = 0,
	IPAModeTypeHdrMerge = 1,
	IPAModeTypeRgbIr = 2,
	IPAModeTypeRgbIrDual = 3,
};

struct IPAColorSpace
{
public:
#ifndef __DOXYGEN__
	IPAColorSpace()
		: primaries(static_cast<IPAPrimaries>(0)), transferFunction(static_cast<IPATransferFunction>(0)), ycbcrEncoding(static_cast<IPAYcbcrEncoding>(0)), range(static_cast<IPARange>(0))
	{
	}

	IPAColorSpace(const IPAPrimaries &_primaries, const IPATransferFunction &_transferFunction, const IPAYcbcrEncoding &_ycbcrEncoding, const IPARange &_range)
		: primaries(_primaries), transferFunction(_transferFunction), ycbcrEncoding(_ycbcrEncoding), range(_range)
	{
	}
#endif


	IPAPrimaries primaries;
	IPATransferFunction transferFunction;
	IPAYcbcrEncoding ycbcrEncoding;
	IPARange range;
};

struct InitParams
{
public:
#ifndef __DOXYGEN__
	InitParams()
		: hwRevision(0)
	{
	}

	InitParams(uint32_t _hwRevision, const std::string &_sensorEntity, const IPACameraSensorInfo &_sensorInfo, const ControlInfoMap &_sensorControls, const ControlList &_sensorControlList)
		: hwRevision(_hwRevision), sensorEntity(_sensorEntity), sensorInfo(_sensorInfo), sensorControls(_sensorControls), sensorControlList(_sensorControlList)
	{
	}
#endif


	uint32_t hwRevision;
	std::string sensorEntity;
	IPACameraSensorInfo sensorInfo;
	ControlInfoMap sensorControls;
	ControlList sensorControlList;
};

struct IPAConfigInfo
{
public:
#ifndef __DOXYGEN__
	IPAConfigInfo()
	{
	}

	IPAConfigInfo(const IPACameraSensorInfo &_sensorInfo, const ControlInfoMap &_sensorControls)
		: sensorInfo(_sensorInfo), sensorControls(_sensorControls)
	{
	}
#endif


	IPACameraSensorInfo sensorInfo;
	ControlInfoMap sensorControls;
};

struct DelayedControlsParams
{
public:
#ifndef __DOXYGEN__
	DelayedControlsParams()
		: delay(0), priorityWrite(0)
	{
	}

	DelayedControlsParams(uint32_t _delay, bool _priorityWrite)
		: delay(_delay), priorityWrite(_priorityWrite)
	{
	}
#endif


	uint32_t delay;
	bool priorityWrite;
};

struct SensorConfig
{
public:
#ifndef __DOXYGEN__
	SensorConfig()
		: embeddedTopLines(0), rgbIr(0)
	{
	}

	SensorConfig(const std::map<int32_t, ipa::nxpneo::DelayedControlsParams> &_delayedControlsParams, uint32_t _embeddedTopLines, bool _rgbIr)
		: delayedControlsParams(_delayedControlsParams), embeddedTopLines(_embeddedTopLines), rgbIr(_rgbIr)
	{
	}
#endif


	std::map<int32_t, ipa::nxpneo::DelayedControlsParams> delayedControlsParams;
	uint32_t embeddedTopLines;
	bool rgbIr;
};

class IPANxpNeoInterface : public IPAInterface
{
public:

	virtual int32_t init(
		const IPASettings &settings,
		const InitParams &params,
		ControlInfoMap *ipaControls,
		SensorConfig *sensorConfig) = 0;

	virtual int32_t start() = 0;

	virtual void stop() = 0;

	virtual int32_t configure(
		const IPAConfigInfo &configInfo,
		const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
		const IPAModeType mode,
		const IPAColorSpace &colorSpace,
		ControlInfoMap *ipaControls) = 0;

	virtual void mapBuffers(
		const std::vector<libcamera::IPABuffer> &buffers) = 0;

	virtual void unmapBuffers(
		const std::vector<uint32_t> &ids) = 0;

	virtual void queueRequest(
		const uint32_t frame,
		const ControlList &reqControls) = 0;

	virtual void computeParams(
		const uint32_t frame,
		const IPAContextType context,
		const std::map<uint32_t, uint32_t> &bufferIds) = 0;

	virtual void processStats(
		const uint32_t frame,
		const IPAContextType context,
		const std::map<uint32_t, uint32_t> &bufferIds,
		const ControlList &sensorControls) = 0;

	Signal<uint32_t, const IPAContextType> paramsComputed;

	Signal<uint32_t, const IPAContextType, const ControlList &> setSensorControls;

	Signal<uint32_t, const IPAContextType, const ControlList &> metadataReady;
};

} /* namespace nxpneo */

} /* namespace ipa */

} /* namespace libcamera */