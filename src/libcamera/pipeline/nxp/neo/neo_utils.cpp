/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * neo-utils.cpp - Helpers for NXP NEO pipeline
 * Copyright 2024 NXP
 */

#include "neo_utils.h"

#include <limits>
#include <sstream>
#include <string>

#include <linux/v4l2-subdev.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

#include "isi_device.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpNeo)

namespace nxpneo {

/* -----------------------------------------------------------------------------
 * CameraMediaStream class
 */

/**
 * \brief Assemble and return a string describing the camera media stream
 * \return A string describing the camera media stream
 */
std::string CameraMediaStream::toString() const
{
	std::stringstream ss;

	for (const auto &streamLink : streamLinks_) {
		const MediaLink *_mediaLink = std::get<0>(streamLink);
		const MediaPad *_sourceMediaPad = _mediaLink->source();
		const MediaPad *_sinkMediaPad = _mediaLink->sink();
		std::string _sourceName = _sourceMediaPad->entity()->name();
		std::string _sinkName = _sinkMediaPad->entity()->name();
		unsigned int _sourcePad = _sourceMediaPad->index();
		unsigned int _sinkPad = _sinkMediaPad->index();
		unsigned int _sourceStream = std::get<1>(streamLink);
		unsigned int _sinkStream = std::get<2>(streamLink);
		ss << "source " << _sourceName << " "
		   << _sourcePad << "/" << _sourceStream
		   << " sink " << _sinkName << " "
		   << _sinkPad << "/" << _sinkStream
		   << std::endl;
	}

	ss << "mbus-code " << mbusCode()
	   << " isi-pipe " << pipe()
	   << " embedded-lines " << pipe();

	return ss.str();
}

/* -----------------------------------------------------------------------------
 * CameraInfo class
 */

/**
 * \brief Return if a stream exists for the camera
 * \param[in] streamId The stream identifier STREAM_<XYZ>.
 * \return True if the stream is configured.
 */
bool CameraInfo::hasStream(unsigned int streamId) const
{
	if (streamId >= streams_.size())
		return false;
	return streams_[streamId].has_value();
}

/* -----------------------------------------------------------------------------
 * PipelineConfig class
 */

/**
 * \brief Report the CameraInfo associated to a camera
 * \param[in] name The name of the camera media device entity.
 *
 * The CameraInfo structure carries information related to the integration
 * of the sensor into the media device. Data is acquired either via
 * automatic graph detection or through parsing of a platform configuration
 * file.
 *
 * \return The pointer to CameraInfo structure if it exists, nullptr otherwise
 */
const CameraInfo *PipelineConfig::getCameraInfo(std::string name) const
{
	auto iter = cameraMap_.find(name);

	if (iter != cameraMap_.end())
		return &iter->second;
	else
		return nullptr;
}

/**
 * \brief Report the global routes for the frontend media controller device
 *
 * Routing is to be configured for entities that support streams.
 * RoutingMap is a <subdevice, routing> map that associates to each relevant
 * subdevice the corresponding list of routes to be applied.
 *
 * \return A reference to the RoutingMap
 */
const RoutingMap &PipelineConfig::getRoutingMap() const
{
	return routingMap_;
}

/**
 * \brief Create a default route for a subdevice if it suports streams
 *
 * Create a single route between the sink and the source pad on stream 0.
 * Doing so will also reinitialise the routing table to a known and
 * deterministic state.
 *
 * \param[in] entity The media device entity of the subdevice.
 * \param[in] sinkPad The sink pad index.
 * \param[in] sourcePad The source pad index.
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineConfig::loadAutoDetectRouting(MediaEntity *entity,
					  unsigned int sinkPad,
					  unsigned int sourcePad)
{
	int ret;
	std::string entityName = entity->name();
	std::unique_ptr<V4L2Subdevice> subdev;

	/* Check if subdevices supports streams */
	subdev = std::make_unique<V4L2Subdevice>(entity);
	ret = subdev->open();
	if (ret) {
		LOG(NxpNeo, Error)
			<< "Failed to open " << subdev->deviceNode();
		return ret;
	}

	if (!subdev->caps().hasStreams())
		return 0;

	/* Create default route */
	V4L2Subdevice::Routing routing;
	struct v4l2_subdev_route route = {};
	route.sink_pad = sinkPad,
	route.sink_stream = 0,
	route.source_pad = sourcePad,
	route.source_stream = 0,
	route.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;
	routing.push_back(route);

	LOG(NxpNeo, Debug)
		<< "Default routing for " << entityName
		<< " added " << routing.toString();

	routingMap_[entityName] = routing;
	return 0;
}

/**
 * \brief Extract camera media device graph to the capture video device
 * \param[in] entity The media device entity of the sensor.
 *
 * Discover the camera graph going through a very basic walk-through strategy,
 * starting from the sensor moving downstream until the ISI video device is
 * hit. Next entity selected is the peer connected to the first source
 * pad of the current entity. That is simplistic approach that works with
 * basic linear topology and that do not require media stream configuration.
 * Single camera discovery is currently supported, multiple cameras typically
 * inducing support for deserializers and media stream configuration. That is
 * hardly compatible with automatic graph discovery, and rather requires
 * platform configuration via config file.
 * More complex implementation may be considered though if needed.
 *
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineConfig::loadAutoDetectEntity(MediaEntity *entity)
{
	constexpr unsigned int kGraphDepthMax = 16;
	std::string cameraName = entity->name();
	unsigned int sourcePadIndex = 0;
	unsigned int sinkPadIndex = 0;
	int ret;

	LOG(NxpNeo, Debug) << "Auto discovering path for " << cameraName;

	std::vector<StreamLink> streamLinks;
	for (unsigned int i = 0; i < kGraphDepthMax; i++) {
		MediaPad *sourcePad = nullptr;
		MediaPad *sinkPad;

		for (auto pad : entity->pads()) {
			if (pad->flags() & MEDIA_PAD_FL_SOURCE) {
				sourcePad = pad;
				sourcePadIndex = pad->index();
				break;
			}
		}

		MediaLink *link;
		if (sourcePad && sourcePad->links().size()) {
			link = sourcePad->links()[0];
		} else {
			LOG(NxpNeo, Debug)
				<< "Camera " << cameraName
				<< " entity " << entity->name()
				<< " has no source pad with links";
			return -EINVAL;
		}

		ret = loadAutoDetectRouting(entity,
					    sinkPadIndex, sourcePadIndex);
		if (ret)
			return 0;
		streamLinks.emplace_back(link, 0, 0);

		/* Hop to new entity and record link */
		sinkPad = link->sink();
		entity = sinkPad->entity();
		sinkPadIndex = sinkPad->index();

		/* Stop at video node */
		if (entity->function() == MEDIA_ENT_T_V4L2_VIDEO)
			break;
	}

	if (entity->name() != ISIDevice::kVDevPipeEntityName(0)) {
		LOG(NxpNeo, Error)
			<< "Camera " << cameraName
			<< " No path found to ISI video node";
		return -EINVAL;
	}

	/* Optional input0 stream parameters set to default */
	unsigned int mbusCode = 0;
	unsigned int isiPipe = 0;
	unsigned int embeddedLines = 0;
	CameraMediaStream mediaStream = { streamLinks, isiPipe, mbusCode, embeddedLines };

	LOG(NxpNeo, Debug)
		<< "Camera media stream detected " << std::endl
		<< mediaStream.toString();

	CameraInfo cameraInfo = {};
	cameraInfo.streams_[CameraInfo::STREAM_INPUT0] = mediaStream;
	cameraMap_[cameraName] = cameraInfo;

	return 0;
}

/**
 * \brief Discover a valid camera graph to the capture video device
 * \param[in] media The frontend media controller device.
 *
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineConfig::loadAutoDetect(MediaDevice *media)
{
	int ret = -EINVAL;

	for (MediaEntity *entity : media->entities()) {
		if (entity->function() != MEDIA_ENT_F_CAM_SENSOR)
			continue;
		ret = loadAutoDetectEntity(entity);
		if (!ret)
			break;
	}

	return ret;
}

/**
 * \brief Parse match entries in a platform node
 * \param[in] platform The platform entry node in yaml file.
 * \param[in] media The frontend media controller device.
 * \return 0 in case of match or a negative error code otherwise.
 */
int PipelineConfig::parseMatch(const YamlObject &platform, MediaDevice *media)
{
	const YamlObject &match = platform["match"];
	if (!match.isDictionary()) {
		LOG(NxpNeo, Warning)
			<< "No match dictionary node for platform config";
		return -EINVAL;
	}

	const YamlObject &entities = match["entities"];
	if (!entities.isList()) {
		LOG(NxpNeo, Warning) << "No entities for match";
		return -EINVAL;
	}

	for (const auto &entity : entities.asList()) {
		std::string name = entity.get<std::string>().value_or("");
		bool found = media->getEntityByName(name);
		LOG(NxpNeo, Debug) << "Entity " << name << " found " << found;
		if (!found)
			return -EINVAL;
	}

	return 0;
}

/**
 * \brief Parse route entries in a platform node
 * \param[in] platform The platform entry node in yaml file.
 * \param[in] media The frontend media controller device.
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineConfig::parseRoutings(const YamlObject &platform)
{
	/* Routes definition is optional */
	const YamlObject &routings = platform["routings"];
	if (!routings.isList()) {
		LOG(NxpNeo, Debug) << "No routings list";
		return -EINVAL;
	}

	routingMap_.clear();

	for (const auto &subdev : routings.asList()) {
		const auto &entity = subdev["entity"].get<std::string>();
		if (!entity.has_value()) {
			LOG(NxpNeo, Warning) << "Missing routing entity name";
			return -EINVAL;
		}
		std::string entityName = entity.value();

		const auto &routes = subdev["routes"];
		if (!routes.isList() || routes.size() == 0) {
			LOG(NxpNeo, Debug)
				<< "No routes for entity " << entityName;
			continue;
		}

		V4L2Subdevice::Routing routing;
		for (const auto &route : routes.asList()) {
			std::vector<unsigned int> v =
				route.getList<unsigned int>().value_or(std::vector<unsigned int>{});
			if (v.size() != ROUTE_MAX) {
				LOG(NxpNeo, Warning)
					<< "Unexpected route size " << v.size();
				return -EINVAL;
			}

			struct v4l2_subdev_route r = {};
			r.sink_pad = v[ROUTE_SINK_PAD];
			r.sink_stream = v[ROUTE_SINK_STREAM];
			r.source_pad = v[ROUTE_SOURCE_PAD];
			r.source_stream = v[ROUTE_SOURCE_STREAM];
			r.flags = v[ROUTE_FLAGS];

			routing.emplace_back(std::move(r));
		}

		routingMap_[entityName] = routing;
		LOG(NxpNeo, Debug) << "Entity name " << entityName
				   << " routing " << routing.toString();
	}

	return 0;
}

/**
 * \brief Parse a stream within a camera node
 * \param[in] camera The camera stream node in yaml file.
 * \param[in] key The key of the actual stream for the camera.
 * \param[in] media The frontend media controller device.
 * \return The optional CameraMediaStream if found, otherwise nullopt.
 */

std::optional<CameraMediaStream>
PipelineConfig::parseMediaStream(const YamlObject &camera,
				 std::string key, MediaDevice *media)
{
	/* Streams are optional, so a missing node is not an error */
	const YamlObject &stream = camera[key];
	if (!stream.isDictionary())
		return std::nullopt;

	LOG(NxpNeo, Debug) << "Parsing stream " << key;

	enum {
		LINK_SOURCE_NAME = 0,
		LINK_SOURCE_PAD = 1,
		LINK_SOURCE_STREAM = 2,
		LINK_SINK_NAME = 3,
		LINK_SINK_PAD = 4,
		LINK_SINK_STREAM = 5,
		LINK_SINK_MAX = 6,
	};

	const YamlObject &links = stream["links"];
	if (!links.isList() || links.size() != LINK_SINK_MAX) {
		LOG(NxpNeo, Error) << "Invalid camera links list";
		return std::nullopt;
	}

	std::vector<StreamLink> streamLinks;
	unsigned int maxUint = std::numeric_limits<unsigned int>::max();
	for (const auto &link : links.asList()) {
		std::string sourceEntityName =
			link[LINK_SOURCE_NAME].get<std::string>().value_or("");
		MediaEntity *sourceEntity =
			media->getEntityByName(sourceEntityName);
		if (!sourceEntity) {
			LOG(NxpNeo, Error)
				<< "Source entity not found "
				<< sourceEntityName;
			return std::nullopt;
		}

		unsigned int sourcePad =
			link[LINK_SOURCE_PAD].get<unsigned int>().value_or(maxUint);
		unsigned int sourceStream =
			link[LINK_SOURCE_STREAM].get<unsigned int>().value_or(maxUint);

		std::string sinkEntityName =
			link[LINK_SINK_NAME].get<std::string>().value_or("");
		MediaEntity *sinkEntity =
			media->getEntityByName(sinkEntityName);
		if (!sinkEntity) {
			LOG(NxpNeo, Error)
				<< "Sink entity not found " << sinkEntityName;
			return std::nullopt;
		}

		unsigned int sinkPad =
			link[LINK_SINK_PAD].get<unsigned int>().value_or(maxUint);
		unsigned int sinkStream =
			link[LINK_SINK_STREAM].get<unsigned int>().value_or(maxUint);

		/*
		 * Lookup for matching media link in the graph corresponding to
		 * the link definition from the config file
		 */
		const MediaPad *sourceMediaPad =
			sourceEntity->getPadByIndex(sourcePad);
		if (!sourceMediaPad) {
			LOG(NxpNeo, Error)
				<< "Entity " << sinkEntityName
				<< " source pad " << sourcePad << " not found";
			return std::nullopt;
		}

		MediaLink *mediaLink = nullptr;
		for (MediaLink *_mediaLink : sourceMediaPad->links()) {
			MediaEntity *_sinkEntity = _mediaLink->sink()->entity();
			unsigned int _sinkPad = _mediaLink->sink()->index();
			if ((sinkEntity->name() == _sinkEntity->name()) &&
			    (sinkPad == _sinkPad)) {
				mediaLink = _mediaLink;
			}
		}

		if (!mediaLink) {
			LOG(NxpNeo, Error)
				<< "Link not found"
				<< " source " << sourceEntityName
				<< "/" << sourcePad
				<< " sink " << sinkEntityName
				<< "/" << sinkPad;
			return std::nullopt;
		}

		streamLinks.emplace_back(mediaLink, sourceStream, sinkStream);
	}

	/* mbus-code is optional - may be used for input1 and embedded-data */
	const YamlObject &mbus = stream["mbus-code"];
	unsigned int mbusCode = mbus.get<unsigned int>().value_or(0);

	const YamlObject &pipe = stream["isi-pipe"];
	unsigned int isiPipe = pipe.get<unsigned int>().value_or(maxUint);
	if (isiPipe == maxUint) {
		LOG(NxpNeo, Warning) << "ISI pipe is not defined";
		return std::nullopt;
	}

	/* embedded-lines is optional - may be used for embedded-data */
	const YamlObject &lines = stream["embedded-lines"];
	unsigned int embeddedLines = lines.get<unsigned int>().value_or(0);

	CameraMediaStream mediaStream = { streamLinks, isiPipe, mbusCode, embeddedLines };

	LOG(NxpNeo, Debug)
		<< "Camera media stream parsed " << std::endl
		<< mediaStream.toString();

	return std::make_optional(std::move(mediaStream));
}

/**
 * \brief Parse camera entries in a platform node
 * \param[in] platform The platform entry node in yaml file.
 * \param[in] media The frontend media controller device.
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineConfig::parseCameras(const YamlObject &platform, MediaDevice *media)
{
	cameraMap_.clear();

	const YamlObject &cameras = platform["cameras"];
	if (!cameras.isList()) {
		LOG(NxpNeo, Error) << "No camera listed";
		return -EINVAL;
	}

	for (const auto &camera : cameras.asList()) {
		const auto &entity = camera["entity"].get<std::string>();
		if (!entity.has_value()) {
			LOG(NxpNeo, Error) << "Missing camera entity name";
			return -EINVAL;
		}
		std::string entityName = entity.value();

		LOG(NxpNeo, Debug)
			<< "Parsing camera " << entityName;

		CameraInfo cameraInfo = {};
		std::optional<CameraMediaStream> stream;
		stream = parseMediaStream(camera, "stream-input0", media);
		cameraInfo.streams_[CameraInfo::STREAM_INPUT0] = std::move(stream);
		stream = parseMediaStream(camera, "stream-input1", media);
		cameraInfo.streams_[CameraInfo::STREAM_INPUT1] = std::move(stream);
		stream = parseMediaStream(camera, "stream-embedded", media);
		cameraInfo.streams_[CameraInfo::STREAM_EMBEDDED] = std::move(stream);

		if (!cameraInfo.hasStreamInput0()) {
			LOG(NxpNeo, Error)
				<< "Missing camera stream-input0 definition";
			return -EINVAL;
		}

		LOG(NxpNeo, Debug)
			<< "Camera stream-input1 configured "
			<< cameraInfo.hasStreamInput1();

		LOG(NxpNeo, Debug)
			<< "Camera stream-embedded configured "
			<< cameraInfo.hasStreamEmbedded();

		cameraMap_[entityName] = cameraInfo;
	}

	LOG(NxpNeo, Debug) << "Camera configurations " << cameraMap_.size();
	return cameraMap_.size() > 0 ? 0 : -EINVAL;
}

/**
 * \brief Parse a platform entry in yaml configuration file
 * \param[in] platform The platform entry node in yaml file.
 * \param[in] media The frontend media controller device.
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineConfig::parsePlatform(const YamlObject &platform,
				  MediaDevice *media)
{
	int ret;

	std::string name =
		platform["name"].get<std::string>().value_or("");

	LOG(NxpNeo, Debug) << "Parsing config name " << name;

	ret = parseMatch(platform, media);
	if (ret)
		return ret;

	ret = parseRoutings(platform);
	if (ret)
		return ret;

	ret = parseCameras(platform, media);
	if (ret)
		return ret;

	return 0;
}

/**
 * \brief Load the pipeline configuration from a pipeline configuration file
 * \param[in] filename The path to configuration file.
 * \param[in] media The frontend media controller device.
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineConfig::loadFromFile(std::string filename, MediaDevice *media)
{
	File file(filename);
	int ret;

	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(NxpNeo, Warning)
			<< "Failed to open pipeline config file" << filename;
		return -ENOENT;
	}

	std::unique_ptr<YamlObject> root = YamlParser::parse(file);
	if (!root) {
		LOG(NxpNeo, Warning)
			<< "Failed to parse pipeline config file " << filename;
		return -EINVAL;
	}

	double version = (*root)["version"].get<double>().value_or(0.0);
	if (version != 1.0) {
		LOG(NxpNeo, Warning)
			<< "Unexpected pipeline config file version "
			<< version;
		return -EINVAL;
	}

	const YamlObject &platforms = (*root)["platforms"];
	if (!platforms.isList()) {
		LOG(NxpNeo, Info)
			<< "No platform listed in pipeline config file";
	}

	LOG(NxpNeo, Info) << "Parsing pipeline config file " << filename;

	for (const auto &platform : platforms.asList()) {
		ret = parsePlatform(platform, media);
		if (!ret)
			return 0;
	}

	return -EINVAL;
}

/**
 * \brief Load the pipeline configuration
 * \param[in] file The path to the pipeline configuration file.
 * \param[in] media The frontend media controller device.
 *
 * Build the pipeline configuration from either the config file
 * if it exists and lists a setup corresponding to the frontend media controller
 * device. In case no such predefined is available, default to automatic
 * detection mode that works for pipelines that can be automatically discovered.
 *
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineConfig::load(std::string filename, MediaDevice *media)
{
	int ret;

	ret = loadFromFile(filename, media);
	if (ret)
		ret = loadAutoDetect(media);

	return ret;
}

} // namespace nxpneo

} // namespace libcamera
