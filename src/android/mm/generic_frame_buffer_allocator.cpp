/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * generic_camera_buffer.cpp - Allocate FrameBuffer using gralloc API
 */

#include <dlfcn.h>
#include <memory>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/shared_fd.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/framebuffer.h"

#include <hardware/camera3.h>
#include <hardware/gralloc.h>
#include <hardware/hardware.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc++98-compat-extra-semi"
// ??? fix me
typedef enum log_id {
  LOG_ID_MIN = 0,

  /** The main log buffer. This is the only log buffer available to apps. */
  LOG_ID_MAIN = 0,
  /** The radio log buffer. */
  LOG_ID_RADIO = 1,
  /** The event log buffer. */
  LOG_ID_EVENTS = 2,
  /** The system log buffer. */
  LOG_ID_SYSTEM = 3,
  /** The crash log buffer. */
  LOG_ID_CRASH = 4,
  /** The statistics log buffer. */
  LOG_ID_STATS = 5,
  /** The security log buffer. */
  LOG_ID_SECURITY = 6,
  /** The kernel log buffer. */
  LOG_ID_KERNEL = 7,

  LOG_ID_MAX,

  /** Let the logging function choose the best log target. */
  LOG_ID_DEFAULT = 0x7FFFFFFF
} log_id_t;

//#include <android/log.h>
//#include <log/log.h>
#include <ui/GraphicBufferAllocator.h>
#pragma GCC diagnostic pop

#include "../camera_device.h"
#include "../frame_buffer_allocator.h"
#include "../hal_framebuffer.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

namespace {
class GenericFrameBufferData : public FrameBuffer::Private
{
	LIBCAMERA_DECLARE_PUBLIC(FrameBuffer)

public:
	GenericFrameBufferData(buffer_handle_t handle,
			       const std::vector<FrameBuffer::Plane> &planes)
		: FrameBuffer::Private(planes),
		  handle_(handle)
	{
	}

	~GenericFrameBufferData() override
	{
		android::GraphicBufferAllocator::get().free(handle_);
	}
private:
	const buffer_handle_t handle_;
};
} /* namespace */

class PlatformFrameBufferAllocator::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(PlatformFrameBufferAllocator)

public:
	Private(CameraDevice *const cameraDevice)
		: cameraDevice_(cameraDevice)
	{
	}

	~Private() override;

	std::unique_ptr<HALFrameBuffer>
	allocate(int halPixelFormat, const libcamera::Size &size, uint32_t usage);

private:
	const CameraDevice *const cameraDevice_;
};

PlatformFrameBufferAllocator::Private::~Private()
{
}

std::unique_ptr<HALFrameBuffer>
PlatformFrameBufferAllocator::Private::allocate(int halPixelFormat,
						const libcamera::Size &size,
						uint32_t usage)
{
	uint32_t stride = 0;
	buffer_handle_t handle = nullptr;

#if 0
	usage |= GRALLOC_USAGE_PRIVATE_3;
	int ret = android::GraphicBufferAllocator::get().allocate(
        size.width, size.height, halPixelFormat, 1u, (unsigned long)usage, &handle,
        &stride, "PlatformFrameBufferAllocator");
	if (ret) {
		LOG(HAL, Error) << "failed buffer allocation: " << ret;
		return nullptr;
	}
	if (!handle) {
		LOG(HAL, Fatal) << "invalid buffer_handle_t";
		return nullptr;
	}
#endif

	/* This code assumes the planes are mapped consecutively. */
	const libcamera::PixelFormat pixelFormat =
		cameraDevice_->capabilities()->toPixelFormat(halPixelFormat);
	const auto &info = PixelFormatInfo::info(pixelFormat);
	std::vector<FrameBuffer::Plane> planes(info.numPlanes());

	SharedFD fd{ handle->data[0] };
	LOG(HAL, Info) << "==== planes " << std::to_string(info.numPlanes()) << ", halPixelFormat " << std::to_string(halPixelFormat) <<
		", stride " << std::to_string(stride) << ", fd " << std::to_string(handle->data[0]) << ", usage " + std::to_string(usage);

	size_t offset = 0;
	for (auto [i, plane] : utils::enumerate(planes)) {
		const size_t planeSize = info.planeSize(size.height, i, stride);

		plane.fd = fd;
		plane.offset = offset;
		plane.length = planeSize;
		offset += planeSize;

		LOG(HAL, Info) << "==== planeSize " << std::to_string(planeSize);
	}

	return std::make_unique<HALFrameBuffer>(
		std::make_unique<GenericFrameBufferData>(
			handle, planes),
		handle);
}

PUBLIC_FRAME_BUFFER_ALLOCATOR_IMPLEMENTATION
