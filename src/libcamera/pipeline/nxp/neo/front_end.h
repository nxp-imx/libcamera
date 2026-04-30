/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2026 NXP
 *
 * Camera Front End support for neo ISP pipeline
 */

#pragma once

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>
#include <libcamera/orientation.h>
#include <libcamera/pixel_format.h>
#include <libcamera/transform.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

namespace nxpneo {

class FrontEndHandler;
class NeoDevice;
class NeoDeviceAllocator;
class PipelineConfig;

enum class FEStream {
	Image0,
	Image1,
	EData,
};

class FrontEndCamera
{
public:
	FrontEndCamera() = default;
	virtual ~FrontEndCamera() = default;

	virtual const std::string &name() const = 0;
	virtual CameraSensor *sensor() const = 0;

	struct Attributes {
		bool ispBypass;
		bool rgbIrCfa;
		std::optional<utils::Duration> controlsDelay;
	};
	virtual const Attributes &attributes() const = 0;

	virtual Orientation validateOrientation(Orientation orientation) const = 0;

	virtual const std::vector<FEStream> &streams() const = 0;
	virtual bool hasStream(FEStream stream) const = 0;
	virtual V4L2VideoDevice *videoDevice(FEStream stream) const = 0;

	virtual int configure(
		V4L2SubdeviceFormat &subdevFormat, Transform transform,
		std::map<FEStream, V4L2DeviceFormat> *videoFormats,
		const std::map<FEStream, V4L2DeviceFormat> *processedVideoFormats) = 0;

	struct Formats {
		std::map<unsigned int, std::vector<Size>> mbusCodeSizesMap;
		std::map<Size, std::vector<unsigned int>> sizeMbusCodesMap;
		std::map<unsigned int, std::vector<PixelFormat>>
			mbusCodePixelFormatsMap;
	};
	virtual const Formats &formats() const = 0;
	virtual const std::vector<NeoDevice *> &neoDevices() const = 0;
};

class FrontEndHandler
{
public:
	FrontEndHandler(const std::string &name)
		: name_(name) {}
	virtual ~FrontEndHandler() = default;

	struct MatchParams {
		PipelineHandler *pipeline;
		DeviceEnumerator *enumerator;
		NeoDeviceAllocator *neoAllocator;
		PipelineConfig *pipelineConfig;
	};
	virtual bool match(const MatchParams &params) = 0;

	virtual const std::vector<FrontEndCamera *> &cameras() const = 0;
	virtual int acquireDevice(const std::string &name) = 0;
	virtual void releaseDevice(const std::string &name) = 0;
	virtual const std::string &name() const { return name_; }

private:
	std::string name_;
};

class FrontEndHandlerFactoryBase
{
public:
	FrontEndHandlerFactoryBase(const char *name);
	virtual ~FrontEndHandlerFactoryBase() = default;

	std::unique_ptr<FrontEndHandler> create() const;
	const std::string &name() const { return name_; }
	static std::vector<FrontEndHandlerFactoryBase *> &factories();

private:
	static void registerType(FrontEndHandlerFactoryBase *factory);
	virtual std::unique_ptr<FrontEndHandler>
	createInstance(const std::string &name) const = 0;

	std::string name_;
};

template<typename _FrontEndHandler>
class FrontEndHandlerFactory final : public FrontEndHandlerFactoryBase
{
public:
	FrontEndHandlerFactory(const char *name)
		: FrontEndHandlerFactoryBase(name)
	{
	}

private:
	std::unique_ptr<FrontEndHandler> createInstance(
		const std::string &name) const
	{
		return std::make_unique<_FrontEndHandler>(name);
	}
};

#define REGISTER_FRONT_END_HANDLER(name, helper) \
	static FrontEndHandlerFactory<helper> global_##helper##Factory(name);

} /* namespace nxpneo */

} /* namespace libcamera */
