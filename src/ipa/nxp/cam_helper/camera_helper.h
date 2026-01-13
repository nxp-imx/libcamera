/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Helper class that performs sensor-specific parameter computations
 *     src/ipa/libipa/camera_sensor_helper.h
 * Copyright (C) 2021, Google Inc.
 *
 * camera_helper.h
 * Helper class that performs sensor-specific parameter computations
 * Copyright 2024-2025 NXP
 */

#pragma once

#include <map>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/utils.h>
#include <libcamera/controls.h>

#include "libipa/camera_sensor_helper.h"

using libcamera::utils::Duration;

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

/* Sensor stream modes */
enum SensorStreamModes {
	SensorStreamStandard = 0,
	SensorStreamHdr,
	SensorStreamRgbIr,
	SensorStreamDualContext,
};

/* Sensor frame context types */
enum SensorContextTypes {
	SensorContextRgb = 0,
	SensorContextIr,
};

/* Subset of IPACameraSensorInfo structure*/
struct CameraMode {
	/* bit depth of the raw camera output */
	uint32_t bitdepth;
	/* size in pixels of frames in this mode */
	uint16_t width;
	uint16_t height;
	/* pixel clock rate */
	uint64_t pixelRate;
	/* hblank and vblank in this mode */
	int32_t hblank;
	int32_t vblank;
	/* stream mode */
	SensorStreamModes streamMode;
};

class CameraHelper : public ipa::CameraSensorHelper
{
public:
	CameraHelper();
	virtual ~CameraHelper() = default;
	virtual void setCameraMode(const CameraMode &mode);
	virtual void setControls(const ControlList *sensorCtrls);

	virtual void controlListSetAGC(
		ControlList *ctrls, SensorContextTypes context,
		Duration exposure, double gain);

	virtual void controlInfoMapGetExposureRange(
		const ControlInfoMap *ctrls, std::vector<Duration> *minExposure,
		std::vector<Duration> *maxExposure, std::vector<Duration> *defExposure) const;

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

	virtual const Attributes *attributes() const { return &attributes_; }

	virtual int parseEmbedded(
		Span<const uint8_t> buffer, ControlList *mdControls);

	virtual int sensorControlsToMetaData(
		const ControlList *sensorCtrls, ControlList *mdCtrls) const;

	virtual uint32_t exposureLines(const Duration exposure,
				       const Duration lineLength) const;
	virtual Duration exposure(uint32_t exposureLines,
				  const Duration lineLength) const;
	Duration hblankToLineLength(uint32_t hblank) const;

protected:
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
class CameraHelperFactory final : public CameraHelperFactoryBase
{
public:
	CameraHelperFactory(const char *name)
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
	static CameraHelperFactory<helper> global_##helper##Factory(name);

} /* namespace nxp */

} /* namespace libcamera */
