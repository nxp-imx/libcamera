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
#include <type_traits>
#include <utility>
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
	SetLensControls = 3,
	MetadataReady = 4,
};


enum class IPAPrimaries {
	Raw = 0,
	Smpte170m = 1,
	Rec709 = 2,
	Rec2020 = 3,
};

enum class IPATransferFunction {
	Linear = 0,
	Srgb = 1,
	Rec709 = 2,
};

enum class IPAYcbcrEncoding {
	None = 0,
	Rec601 = 1,
	Rec709 = 2,
	Rec2020 = 3,
};

enum class IPARange {
	Full = 0,
	Limited = 1,
};

enum class IPAStreamType {
	Frame = 0,
	Ir = 1,
};

enum class IPABufferType {
	Image0 = 0,
	Image1 = 1,
	EData = 2,
	Params = 3,
	Stats = 4,
	Frame = 5,
	Ir = 6,
};

enum class IPACameraContext {
	Rgb = 0,
	Ir = 1,
};

enum class IPAPipelineMode {
	Standard = 0,
	HdrMerge = 1,
	RgbIr = 2,
	RgbIrDual = 3,
};

struct IPAColorSpace
{
public:
#ifndef __DOXYGEN__
	IPAColorSpace() = default;

	template<
		typename T1 = IPAPrimaries,
		typename T2 = IPATransferFunction,
		typename T3 = IPAYcbcrEncoding,
		typename T4 = IPARange,
		std::enable_if_t<std::is_convertible_v<T1&&, IPAPrimaries>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T2&&, IPATransferFunction>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T3&&, IPAYcbcrEncoding>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T4&&, IPARange>> * = nullptr
	>
	IPAColorSpace(T1 &&_primaries, T2 &&_transferFunction, T3 &&_ycbcrEncoding, T4 &&_range)
		: primaries(std::forward<T1>(_primaries))
		, transferFunction(std::forward<T2>(_transferFunction))
		, ycbcrEncoding(std::forward<T3>(_ycbcrEncoding))
		, range(std::forward<T4>(_range))
	{
	}
#endif


	IPAPrimaries primaries{ static_cast<IPAPrimaries>(0) };
	IPATransferFunction transferFunction{ static_cast<IPATransferFunction>(0) };
	IPAYcbcrEncoding ycbcrEncoding{ static_cast<IPAYcbcrEncoding>(0) };
	IPARange range{ static_cast<IPARange>(0) };
};

struct InitParams
{
public:
#ifndef __DOXYGEN__
	InitParams() = default;

	template<
		typename T1 = uint32_t,
		typename T2 = uint32_t,
		typename T3 = uint64_t,
		typename T4 = std::string,
		typename T5 = IPACameraSensorInfo,
		typename T6 = ControlInfoMap,
		typename T7 = ControlList,
		typename T8 = bool,
		std::enable_if_t<std::is_convertible_v<T1&&, uint32_t>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T2&&, uint32_t>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T3&&, uint64_t>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T4&&, std::string>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T5&&, IPACameraSensorInfo>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T6&&, ControlInfoMap>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T7&&, ControlList>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T8&&, bool>> * = nullptr
	>
	InitParams(T1 &&_hwRevision, T2 &&_hwCapabilities, T3 &&_supportedParamsBlocks, T4 &&_sensorEntity, T5 &&_sensorInfo, T6 &&_sensorControls, T7 &&_sensorControlList, T8 &&_lensPresent)
		: hwRevision(std::forward<T1>(_hwRevision))
		, hwCapabilities(std::forward<T2>(_hwCapabilities))
		, supportedParamsBlocks(std::forward<T3>(_supportedParamsBlocks))
		, sensorEntity(std::forward<T4>(_sensorEntity))
		, sensorInfo(std::forward<T5>(_sensorInfo))
		, sensorControls(std::forward<T6>(_sensorControls))
		, sensorControlList(std::forward<T7>(_sensorControlList))
		, lensPresent(std::forward<T8>(_lensPresent))
	{
	}
#endif


	uint32_t hwRevision{ 0 };
	uint32_t hwCapabilities{ 0 };
	uint64_t supportedParamsBlocks{ 0 };
	std::string sensorEntity;
	IPACameraSensorInfo sensorInfo;
	ControlInfoMap sensorControls;
	ControlList sensorControlList;
	bool lensPresent{ 0 };
};

struct IPAConfigInfo
{
public:
#ifndef __DOXYGEN__
	IPAConfigInfo() = default;

	template<
		typename T1 = IPACameraSensorInfo,
		typename T2 = ControlInfoMap,
		typename T3 = ControlInfoMap,
		typename T4 = ControlList,
		typename T5 = IPAPipelineMode,
		typename T6 = IPAColorSpace,
		typename T7 = uint32_t,
		std::enable_if_t<std::is_convertible_v<T1&&, IPACameraSensorInfo>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T2&&, ControlInfoMap>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T3&&, ControlInfoMap>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T4&&, ControlList>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T5&&, IPAPipelineMode>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T6&&, IPAColorSpace>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T7&&, uint32_t>> * = nullptr
	>
	IPAConfigInfo(T1 &&_sensorInfo, T2 &&_sensorControls, T3 &&_lensControls, T4 &&_sensorControlList, T5 &&_mode, T6 &&_colorSpace, T7 &&_bitsPerPixelAuxiliary)
		: sensorInfo(std::forward<T1>(_sensorInfo))
		, sensorControls(std::forward<T2>(_sensorControls))
		, lensControls(std::forward<T3>(_lensControls))
		, sensorControlList(std::forward<T4>(_sensorControlList))
		, mode(std::forward<T5>(_mode))
		, colorSpace(std::forward<T6>(_colorSpace))
		, bitsPerPixelAuxiliary(std::forward<T7>(_bitsPerPixelAuxiliary))
	{
	}
#endif


	IPACameraSensorInfo sensorInfo;
	ControlInfoMap sensorControls;
	ControlInfoMap lensControls;
	ControlList sensorControlList;
	IPAPipelineMode mode{ static_cast<IPAPipelineMode>(0) };
	IPAColorSpace colorSpace;
	uint32_t bitsPerPixelAuxiliary{ 0 };
};

struct DelayedControlsParams
{
public:
#ifndef __DOXYGEN__
	DelayedControlsParams() = default;

	template<
		typename T1 = uint32_t,
		typename T2 = bool,
		std::enable_if_t<std::is_convertible_v<T1&&, uint32_t>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T2&&, bool>> * = nullptr
	>
	DelayedControlsParams(T1 &&_delay, T2 &&_priorityWrite)
		: delay(std::forward<T1>(_delay))
		, priorityWrite(std::forward<T2>(_priorityWrite))
	{
	}
#endif


	uint32_t delay{ 0 };
	bool priorityWrite{ 0 };
};

struct SensorConfig
{
public:
#ifndef __DOXYGEN__
	SensorConfig() = default;

	template<
		typename T1 = std::map<int32_t, ipa::nxpneo::DelayedControlsParams>,
		typename T2 = uint32_t,
		std::enable_if_t<std::is_convertible_v<T1&&, std::map<int32_t, ipa::nxpneo::DelayedControlsParams>>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T2&&, uint32_t>> * = nullptr
	>
	SensorConfig(T1 &&_delayedControlsParams, T2 &&_embeddedTopLines)
		: delayedControlsParams(std::forward<T1>(_delayedControlsParams))
		, embeddedTopLines(std::forward<T2>(_embeddedTopLines))
	{
	}
#endif


	std::map<int32_t, ipa::nxpneo::DelayedControlsParams> delayedControlsParams;
	uint32_t embeddedTopLines{ 0 };
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
		const std::map<ipa::nxpneo::IPAStreamType, libcamera::IPAStream> &streamConfig,
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
		const IPACameraContext context,
		const std::map<ipa::nxpneo::IPABufferType, uint32_t> &bufferIds) = 0;

	virtual void processStats(
		const uint32_t frame,
		const IPACameraContext context,
		const std::map<ipa::nxpneo::IPABufferType, uint32_t> &bufferIds,
		const ControlList &sensorControls) = 0;

	Signal<uint32_t, const IPACameraContext, uint32_t> paramsComputed;

	Signal<uint32_t, const ControlList &> setSensorControls;

	Signal<const ControlList &> setLensControls;

	Signal<uint32_t, const IPACameraContext, const ControlList &> metadataReady;
};

} /* namespace nxpneo */

} /* namespace ipa */

} /* namespace libcamera */