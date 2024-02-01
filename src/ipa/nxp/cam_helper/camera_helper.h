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

#include <stdint.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/controls.h>

#include "libipa/camera_sensor_helper.h"

namespace libcamera {

namespace nxp {

class CameraHelper : public ipa::CameraSensorHelper
{
public:
	CameraHelper() = default;
	virtual ~CameraHelper() = default;

	virtual uint32_t controlListGetExposure(const ControlList *ctrls) const;
	virtual uint32_t controlListGetGain(const ControlList *ctrls) const;

	virtual void controlListSetExposure(
		ControlList *ctrls, uint32_t exposure) const;
	virtual void controlListSetGain(
		ControlList *ctrls, uint32_t gainCode) const;

	virtual void controlInfoMapGetExposureRange(
		const ControlInfoMap *ctrls, uint32_t *minExposure,
		uint32_t *maxExposure, uint32_t *defExposure = nullptr);

	virtual void controlInfoMapGetGainRange(
		const ControlInfoMap *ctrls, uint32_t *minGainCode,
		uint32_t *maxGainCode, uint32_t *defGainCode = nullptr);

	virtual std::map<int32_t, std::pair<uint32_t, bool>> delayedControlParams();

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
