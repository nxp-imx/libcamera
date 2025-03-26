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


enum ISPBufferType {
	TypeInput0 = 0,
	TypeInput1 = 1,
	TypeEData = 2,
	TypeParams = 3,
	TypeStats = 4,
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
		const std::map<uint32_t, uint32_t> &bufferIds) = 0;

	virtual void processStats(
		const uint32_t frame,
		const std::map<uint32_t, uint32_t> &bufferIds,
		const ControlList &sensorControls) = 0;

	Signal<uint32_t> paramsComputed;

	Signal<uint32_t, const ControlList &> setSensorControls;

	Signal<uint32_t, const ControlList &> metadataReady;
};

} /* namespace nxpneo */

} /* namespace ipa */

} /* namespace libcamera */