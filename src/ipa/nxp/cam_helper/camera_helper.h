/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Helper class that performs sensor-specific parameter computations
 *     src/ipa/libipa/camera_sensor_helper.h
 * Copyright (C) 2021, Google Inc.
 *
 * camera_helper.h
 * Helper class that performs sensor-specific parameter computations
 * Copyright 2024 NXP
 */

#pragma once

#include <map>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/controls.h>

#include "libipa/camera_sensor_helper.h"

namespace libcamera {

namespace nxp {

namespace md {

/*
 * Embedded data controls definition
 */
enum {
	ANALOGUE_GAIN = 0,
	DIGITAL_GAIN = 1,
	EXPOSURE = 2,
	WB_GAIN = 3,
	TEMPERATURE = 4,
};

extern const Control<Span<const float>> AnalogueGain;
extern const Control<Span<const float>> DigitalGain;
extern const Control<Span<const float>> Exposure;
extern const Control<Span<const float>> WhiteBalanceGain;
extern const Control<const float> Temperature;

extern const ControlIdMap controlIdMap;

} /* namespace md */


/* Subset of IPACameraSensorInfo structure*/
struct CameraMode {
	uint64_t pixelRate;
	uint32_t minLineLength;
	uint32_t maxLineLength;
	uint32_t minFrameLength;
	uint32_t maxFrameLength;
};

class CameraHelper : public ipa::CameraSensorHelper
{
public:
	CameraHelper();
	virtual ~CameraHelper() = default;
	virtual void setCameraMode(const CameraMode &mode);

	virtual void controlListSetAGC(
		ControlList *ctrls, double exposure, double gain) const;

	virtual void controlInfoMapGetExposureRange(
		const ControlInfoMap *ctrls, std::vector<double> *minExposure,
		std::vector<double> *maxExposure, std::vector<double> *defExposure) const;

	virtual void controlInfoMapGetAnalogGainRange(
		const ControlInfoMap *ctrls, std::vector<double> *minGain,
		std::vector<double> *maxGain, std::vector<double> *defGain) const;

	virtual void controlListSetAWB(
		ControlList *ctrls, Span<const double, 4> gains) const;

	struct Attributes {
		struct MdParams {
			uint32_t topLines;
		};

		std::map<int32_t, std::pair<uint32_t, bool>> delayedControlParams;
		struct MdParams mdParams;
		bool rgbIr;
	};

	virtual const Attributes *attributes() const { return &attributes_; };

	virtual int parseEmbedded(
		Span<const uint8_t> buffer, ControlList *mdControls);

	virtual int sensorControlsToMetaData(
		const ControlList *sensorCtrls, ControlList *mdCtrls) const;

	virtual double lineDuration() const;

protected:
	static bool controlListHasId(const ControlList *ctrls, unsigned int id);
	Attributes attributes_;
	CameraMode mode_;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraHelper)
};

class CameraHelperFactoryBase
{
public:
	CameraHelperFactoryBase(const std::string name);
	virtual ~CameraHelperFactoryBase() = default;

	static std::unique_ptr<CameraHelper> create(const std::string &name);

	static std::vector<CameraHelperFactoryBase *> &factories();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraHelperFactoryBase)

	static void registerType(CameraHelperFactoryBase *factory);

	virtual std::unique_ptr<CameraHelper> createInstance() const = 0;

	std::string name_;
};

template<typename _Helper>
class CamerarHelperFactory final : public CameraHelperFactoryBase
{
public:
	CamerarHelperFactory(const char *name)
		: CameraHelperFactoryBase(name)
	{
	}

private:
	std::unique_ptr<CameraHelper> createInstance() const
	{
		return std::make_unique<_Helper>();
	}
};

#define REGISTER_CAMERA_HELPER(name, helper) \
	static CamerarHelperFactory<helper> global_##helper##Factory(name);

} /* namespace nxp */

} /* namespace libcamera */
