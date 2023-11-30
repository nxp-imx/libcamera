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

#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc++98-compat-extra-semi"
#pragma GCC diagnostic ignored "-Wextra-semi"

#include "graphics_ext.h"
#include "Memory.h"
#include "MemoryDesc.h"
#include "Allocator.h"
// using namespace fsl;

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
		int fd = handle_->data[0];	
		if (fd > 0)
		    close(fd);
		
		fsl::Memory *fslHandle = (fsl::Memory *)handle_;
		if (fslHandle)
		    delete fslHandle;
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

static uint32_t getSizeByForamtRes(int32_t format, uint32_t width, uint32_t height) {
    uint32_t size = 0;

    switch (format) {
        case HAL_PIXEL_FORMAT_YCbCr_420_SP:
        case HAL_PIXEL_FORMAT_YCbCr_420_P:
        case HAL_PIXEL_FORMAT_YCbCr_420_888:
        case HAL_PIXEL_FORMAT_YV12:
            size = width * height * 3 / 2;
            break;

        case HAL_PIXEL_FORMAT_YCbCr_422_I:
        case HAL_PIXEL_FORMAT_YCbCr_422_SP:
        case HAL_PIXEL_FORMAT_RAW16:
            size = width * height * 2;
            break;

        default:
            LOG(HAL, Error) << __func__ << ": format " << std::to_string(format) << " not supported";
            break;
    }

    return size;
}

static void SetBufferHandle(uint32_t size, int fd, buffer_handle_t &handle) {
    fsl::MemoryDesc desc;
    fsl::Memory *fslHandle = NULL;

    desc.mFlag = 0;
    desc.mWidth = desc.mStride = size / 4;
    desc.mHeight = 1;
    desc.mFormat = HAL_PIXEL_FORMAT_RGBA_8888;
    desc.mFslFormat = fsl::FORMAT_RGBA8888;
    desc.mSize = size;
    desc.mProduceUsage = 0;

   	fslHandle = new fsl::Memory(&desc, fd, -1);
    handle = (buffer_handle_t)fslHandle;

		return;
}

std::unique_ptr<HALFrameBuffer>
PlatformFrameBufferAllocator::Private::allocate(int halPixelFormat,
						const libcamera::Size &size,
						uint32_t usage)
{
	uint32_t stride = size.width;
	buffer_handle_t handle = nullptr;

	uint32_t ionSize = getSizeByForamtRes(halPixelFormat, size.width, size.height);
	if (ionSize == 0)
		return nullptr;
	
	int sharedFd;
	uint64_t phyAddr = 0;
	uint64_t virtAddr = 0;

	fsl::Allocator *allocator = fsl::Allocator::getInstance();
	if (allocator == NULL) {
	    LOG(HAL, Error) << __func__ << ": fsl::allocator invalid";
	    return nullptr;
	}
	
	sharedFd = allocator->allocMemory(ionSize, MEM_ALIGN, fsl::MFLAGS_CONTIGUOUS);
	if (sharedFd < 0) {
	    LOG(HAL, Error) << __func__ << ": allocMemory failed.";
	    return nullptr;
	}

#if 0	
	int err = allocator->getVaddrs(sharedFd, ionSize, virtAddr);
	if (err != 0) {
	    LOG(HAL, Error) << __func__ << ": getVaddrs failed.";
	    close(sharedFd);
	    return nullptr;
	}
	
	err = allocator->getPhys(sharedFd, ionSize, phyAddr);
	if (err != 0) {
	    ALOGE("%s: getPhys failed.", __func__);
	    munmap((void *)(uintptr_t)virtAddr, ionSize);
	    close(sharedFd);
	    return nullptr;
	}
#endif

	SetBufferHandle(ionSize, sharedFd, handle);
	
	std::cout << __func__ << ": fd:" << sharedFd << ", virt:" << std::hex << virtAddr << ", phy:" << std::hex << phyAddr << ", ionSize:" << ionSize << \
		", width:" << size.width << ", height:" << size.height << ", format:" << halPixelFormat;
	

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
