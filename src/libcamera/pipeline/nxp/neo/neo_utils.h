/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * neo_utils.h - Helpers for NXP NEO pipeline
 * Copyright 2024-2025 NXP
 */

#pragma once

#include <map>

#include <linux/v4l2-subdev.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/yaml_parser.h"

#include "isi_device.h"

namespace libcamera {

namespace nxpneo {

class PipelineConfig;

class CameraMediaStream
{
public:
	struct StreamLink {
		StreamLink(MediaLink *mediaLink, unsigned int sourceStream,
			   unsigned int sinkStream)
			: mediaLink_(mediaLink), sourceStream_(sourceStream),
			  sinkStream_(sinkStream) {}
		MediaLink *mediaLink_;
		unsigned int sourceStream_;
		unsigned int sinkStream_;
	};

	CameraMediaStream() {}
	CameraMediaStream(std::vector<StreamLink> &links, unsigned int pipe)
		: streamLinks_(links), isiPipe_(pipe) {}
	virtual ~CameraMediaStream() {}

	const std::vector<StreamLink> &streamLinks() const { return streamLinks_; }
	unsigned int pipe() const { return isiPipe_; }
	std::string toString() const;

private:
	std::vector<StreamLink> streamLinks_;
	unsigned int isiPipe_ = 0;
};

struct CameraProperties {
	CameraProperties()
		: hdrStream(false), eDataStream(false) {}
	bool hdrStream;
	bool eDataStream;
};

class CameraInfo
{
public:
	CameraInfo() {}
	virtual ~CameraInfo() {}

	std::optional<const CameraMediaStream *> getStream(unsigned int id) const;
	bool hasStream(unsigned int id) const { return getStream(id).has_value(); }

	const CameraProperties *getCameraProperties() const { return &properties_; }

	enum {
		STREAM_INPUT0 = 0,
		STREAM_INPUT1,
		STREAM_EDATA,
		STREAM_MAX,
	};
	static constexpr std::array<unsigned int, STREAM_MAX> kCameraStreams = {
		STREAM_INPUT0, STREAM_INPUT1, STREAM_EDATA
	};

private:
	std::map<unsigned int, CameraMediaStream> streams_;
	CameraProperties properties_;

	friend PipelineConfig;
};

using RoutingMap = std::map<MediaEntity *, V4L2Subdevice::Routing>;
using CameraMap = std::map<std::string, CameraInfo>;

class PipelineConfig
{
public:
	PipelineConfig(){};
	virtual ~PipelineConfig();
	int load(std::string file, MediaDevice *media,
		 std::shared_ptr<ISIDevice> isiDevice);
	const CameraInfo *getCameraInfo(std::string name) const;
	const RoutingMap &getRoutingMap() const;

private:
	static constexpr unsigned int kPadAny =
		std::numeric_limits<unsigned int>::max();

	int loadAutoDetect(MediaDevice *media);
	int loadAutoDetectCameraStream(MediaDevice *media, unsigned int pipe,
				       MediaEntity *sensorEntity,
				       unsigned int sensorPad,
				       unsigned int sensorStream,
				       std::map<MediaPad *, unsigned int> *streamMap,
				       RoutingMap *routingMap,
				       CameraMediaStream *cameraMediaStream);
	int loadAutoDetectFindPaths(MediaDevice *media,
				    MediaEntity *fromEntity, unsigned int fromPad,
				    MediaEntity *toEntity, unsigned int toPad,
				    std::vector<std::vector<MediaLink *>> *linkPaths);
	unsigned int loadAutoDetectPadToStream(std::map<MediaPad *, unsigned int> *streamMap,
					       MediaPad *pad);
	int loadAutoDetectAddRoute(MediaEntity *entity,
				   V4L2Subdevice::Stream *sinkStream,
				   V4L2Subdevice::Stream *sourceStream,
				   std::map<MediaEntity *, V4L2Subdevice::Routing> *routingMap);

	int parsePlatformMatch(const YamlObject &match, MediaDevice *media);
	int parsePlatformRoutings(const YamlObject &platform, MediaDevice *media);
	std::optional<CameraMediaStream>
	parsePlatformMediaStream(const YamlObject &camera,
				 std::string key, MediaDevice *media);
	int parsePlatformCameras(const YamlObject &platform, MediaDevice *media);
	int parsePlatformReserveIsi();
	int parseCameras(const YamlObject &cameras);
	int parsePlatforms(const YamlObject &platforms, MediaDevice *media);

	int loadFromFile(std::string file, MediaDevice *media);
	const CameraProperties *getCameraProperties(const std::string &name,
						    const std::string &model);

	RoutingMap routingMap_;
	CameraMap cameraMap_;
	std::shared_ptr<ISIDevice> isiDevice_;

	std::map<std::string, CameraProperties> namePropertiesMap_;
	std::map<std::string, CameraProperties> modelPropertiesMap_;

	/* Configuration file routes sequence elements */
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
