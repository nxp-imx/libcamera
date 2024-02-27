/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * neo_utils.h - Helpers for NXP NEO pipeline
 * Copyright 2024 NXP
 */

#pragma once

#include <linux/v4l2-subdev.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

namespace nxpneo {


using StreamLink = std::tuple<MediaLink *, unsigned int, unsigned int>;

class PipelineConfig;

class CameraMediaStream {
public:
	CameraMediaStream()
		: isiPipe_(0), mbusCode_(0), embeddedLines_(0) {}
	CameraMediaStream(std::vector<StreamLink> links,
			  unsigned int pipe, uint32_t code, unsigned int lines)
		: streamLinks_(links),
		 isiPipe_(pipe), mbusCode_(code), embeddedLines_(lines) {}
	virtual ~CameraMediaStream() {}

	const std::vector<StreamLink> &streamLinks() const
		{ return streamLinks_; }
	unsigned int pipe() const
		{ return isiPipe_; }

	std::string toString() const;

	/* \todo remove those methods */
	unsigned int mbusCode() const
		{ return mbusCode_; }
	unsigned int embeddedLines() const
		{ return embeddedLines_; }

private:
	std::vector<StreamLink> streamLinks_;
	unsigned int isiPipe_;

	/* \todo remove those fields */
	uint32_t mbusCode_;
	unsigned int embeddedLines_;
};

class CameraInfo {
public:
	CameraInfo() {}
	virtual ~CameraInfo() {}

	bool hasStream(unsigned int id) const;
	bool hasStreamInput0() const { return hasStream(STREAM_INPUT0); }
	bool hasStreamInput1() const { return hasStream(STREAM_INPUT1); }
	bool hasStreamEmbedded() const { return hasStream(STREAM_EMBEDDED); }

	const CameraMediaStream *getStream(unsigned int id) const
		{
			if ((id < STREAM_MAX) && (streams_[id].has_value()))
				return &streams_[id].value();
			else
				return nullptr;
		}
	const CameraMediaStream *getStreamInput0() const
		{ return getStream(STREAM_INPUT0); }
	const CameraMediaStream *getStreamInput1() const
		{ return getStream(STREAM_INPUT1); }
	const CameraMediaStream *getStreamEmbedded() const
		{ return getStream(STREAM_EMBEDDED); }

private:
	enum {
		STREAM_INPUT0 = 0,
		STREAM_INPUT1,
		STREAM_EMBEDDED,
		STREAM_MAX,
	};
	std::array<std::optional<CameraMediaStream>, STREAM_MAX> streams_;

	friend PipelineConfig;
};


using RoutingMap = std::map<std::string, V4L2Subdevice::Routing>;
using CameraMap = std::map<std::string, CameraInfo>;

class PipelineConfig {
public:
	PipelineConfig() {};
	virtual ~PipelineConfig() {};
	int load(std::string file, MediaDevice* media);
	const CameraInfo *getCameraInfo(std::string name) const;
	const RoutingMap &getRoutingMap() const;

private:
	int loadFromFile(std::string file, MediaDevice* media);
	int loadAutoDetect(MediaDevice* media);
	int loadAutoDetectEntity(MediaEntity *entity);
	int loadAutoDetectRouting(MediaEntity *entity,
				  unsigned int sinkPad, unsigned int sourcePad);
	int parsePlatform(const YamlObject &platform, MediaDevice *media);
	int parseMatch(const YamlObject &match, MediaDevice *media);
	int parseRoutings(const YamlObject &platform);
	int parseCameras(const YamlObject &platform, MediaDevice *media);
	std::optional<CameraMediaStream>
	parseMediaStream(const YamlObject &camera, std::string key,
			       MediaDevice *media);

	RoutingMap routingMap_;
	CameraMap cameraMap_;

	enum {
		ROUTE_SINK_PAD = 0,
		ROUTE_SINK_STREAM,
		ROUTE_SOURCE_PAD,
		ROUTE_SOURCE_STREAM,
		ROUTE_FLAGS,
		ROUTE_MAX,
	};
};


} // namespace nxpneo

} // namespace libcamera
