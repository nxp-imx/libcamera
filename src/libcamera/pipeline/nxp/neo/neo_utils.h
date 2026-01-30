/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Neo ISP pipeline utilities
 */

#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <libcamera/base/utils.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/orientation.h>

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/value_node.h"

namespace libcamera {

namespace nxpneo {

struct CameraProperties {
	bool image1Stream;
	bool eDataStream;
	bool rgbirCfa;
	std::optional<unsigned int> formatBpp;
	std::optional<Size> formatSize;
	std::optional<Orientation> orientation;
	std::optional<utils::Duration> controlsDelay;

	/*
	 * Vivid instance default configuration values.
	 * Reference: vivid Linux driver implementation vivid-ctrls.c
	 * - Test Pattern (VIVID_CID_TEST_PATTERN): "75% Colorbar" (0)
	 * - Test Pattern Horizontal Movement (VIVID_CID_HOR_MOVEMENT):
	 *   "No Movement" (3)
	 * - Test Pattern Vertical Movement (VIVID_CID_VERT_MOVEMENT):
	 *   "No Movement" (3)
	 */
	static constexpr bool kVividLoopbackDefault = false;
	static constexpr unsigned int kVividTpgPatternDefault = 0;
	static constexpr unsigned int kVividTpgHMovementDefault = 3;
	static constexpr unsigned int kVividTpgVMovementDefault = 3;

	struct VividConfig {
		PixelFormat pixelFormat;
		Size size;
		bool loopback;
		unsigned int tpgPattern;
		unsigned int tpgHorizontalMovement;
		unsigned int tpgVerticalMovement;
	};
	std::optional<std::vector<VividConfig>> vividInstances;
};

struct GlobalInfo {
	static constexpr unsigned int kBufferCount = 4;
	GlobalInfo()
		: bufferCount(kBufferCount) {}

	unsigned int bufferCount;
};

class PipelineConfig
{
public:
	PipelineConfig() = default;
	~PipelineConfig() = default;

	int load(const std::string &file);

	const GlobalInfo &globalInfo() const;
	const CameraProperties &cameraProperties(const std::string &name,
						 const std::string &model);

private:
	int parseCameras(const ValueNode &cameras);
	int parseGlobal(const ValueNode &global);
	int loadFileConfig(const std::string &file);

	std::map<std::string, CameraProperties> camPropertiesMap_;
	GlobalInfo globalInfo_;
};

std::vector<MediaEntity *> locateSensors(MediaDevice *media);

} // namespace nxpneo

} // namespace libcamera
