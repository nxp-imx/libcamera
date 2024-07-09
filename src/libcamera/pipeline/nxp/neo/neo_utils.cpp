/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * neo-utils.cpp - Helpers for NXP NEO pipeline
 * Copyright 2024 NXP
 */

#include "neo_utils.h"

#include <limits>
#include <regex>
#include <sstream>
#include <string>

#include <linux/v4l2-subdev.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

#include "isi_device.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpNeoPipe)

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
		const MediaLink *_mediaLink = streamLink.mediaLink_;
		const MediaPad *_sourceMediaPad = _mediaLink->source();
		const MediaPad *_sinkMediaPad = _mediaLink->sink();
		std::string _sourceName = _sourceMediaPad->entity()->name();
		std::string _sinkName = _sinkMediaPad->entity()->name();
		unsigned int _sourcePad = _sourceMediaPad->index();
		unsigned int _sinkPad = _sinkMediaPad->index();
		unsigned int _sourceStream = streamLink.sourceStream_;
		unsigned int _sinkStream = streamLink.sinkStream_;
		ss << "source " << _sourceName << " "
		   << _sourcePad << "/" << _sourceStream
		   << " sink " << _sinkName << " "
		   << _sinkPad << "/" << _sinkStream
		   << std::endl;
	}

	ss << "mbus-code " << mbusCode()
	   << " isi-pipe " << pipe()
	   << " embedded-lines " << embeddedLines();

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
 * \brief Discover the valid camera graphs to the capture video device
 * \param[in] media The frontend media controller device
 * \param[in] isiDevice The ISI Device associated to the media controller device
 *
 * For every camera sensor in the media device, look for a valid link path to
 * the capture video device. Also, build the aggregated global routing table for
 * all the cameras detected.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::loadAutoDetect(MediaDevice *media, const ISIDevice *isiDevice)
{
	if (!media)
		return -EINVAL;

	if (!isiDevice)
		return -EINVAL;

	unsigned int pipes = isiDevice->crossbarSourcePads();
	unsigned pipeIndex = 0;

	/* Map aggregating stream identifiers for all pads of the media device */
	std::map<MediaPad *, unsigned int> globalStreamMap;

	/*
	 * Build a set of the sensors entities from the media device to have an
	 * ordered list by name and guarantee a consistent topology detection.
	 * the usual sensor naming convention is 'model xx-yyyy'
	 * where xx is the i2c bus number and yyyy is the bus address.
	 * Sensors set ordering is defined with the criteria below:
	 * - simple alphabetical order when above convention is not used or
	 *   for different sensor model
	 * - increasing i2c bus for identical sensor model on different i2c bus
	 * - increasing i2c address for identical sensor model on the same bus
	 */
	auto compareName =
		[](MediaEntity *a, MediaEntity *b) {
			std::regex r("(\\S+) (\\d+)-(\\d+)");
			const std::string &nameA = a->name();
			const std::string &nameB = b->name();
			std::smatch matchesA, matchesB;
			if (std::regex_search(nameA, matchesA, r) &&
			    std::regex_search(nameB, matchesB, r)) {
				const std::string &modelA = matchesA[1];
				const std::string &modelB = matchesB[1];
				unsigned i2cBusA = std::stoul(matchesA[2]);
				unsigned i2cBusB = std::stoul(matchesB[2]);
				unsigned i2cAddrA = std::stoul(matchesA[3]);
				unsigned i2cAddrB = std::stoul(matchesB[3]);
				if (modelA != modelB)
					return nameA < nameB;
				else if (i2cBusA != i2cBusB)
					return i2cBusA < i2cBusB;
				else
					return i2cAddrA < i2cAddrB;
			} else {
				return nameA < nameB;
			}
		};
	std::set<MediaEntity *, decltype(compareName)> sensors(compareName);
	for (MediaEntity *e : media->entities()) {
		if (e->function() != MEDIA_ENT_F_CAM_SENSOR)
			continue;
		sensors.insert(e);
	}

	/* Discover the topology of every sensor */
	for (MediaEntity *sensor : sensors) {
		if (pipeIndex >= pipes) {
			LOG(NxpNeoPipe, Info) << "No pipes left";
			break;
		}

		LOG(NxpNeoPipe, Debug) << "Auto detect camera " << sensor->name();

		/* Support is limited to input0 for now */
		std::map<MediaEntity *, V4L2Subdevice::Routing> routingMap;
		std::map<MediaPad *, unsigned int> streamMap(globalStreamMap);
		CameraMediaStream cameraMediaStream;
		int ret = loadAutoDetectCameraStream(
			media, isiDevice, pipeIndex, sensor, &streamMap, &routingMap,
			&cameraMediaStream);
		if (ret)
			continue;

		/* Merge the sensor routings into the global routings */
		for (auto &[entity, routing] : routingMap) {
			if (routingMap_.count(entity)) {
				V4L2Subdevice::Routing &dest = routingMap_[entity];
				dest.reserve(dest.size() + routing.size());
				std::move(routing.begin(), routing.end(),
					  std::back_inserter(dest));
			} else {
				routingMap_[entity] = std::move(routing);
			}
		}

		globalStreamMap = std::move(streamMap);

		CameraInfo cameraInfo = {};
		cameraInfo.streams_[CameraInfo::STREAM_INPUT0] =
			std::move(cameraMediaStream);
		cameraMap_[sensor->name()] = std::move(cameraInfo);

		pipeIndex++;
	}

	return cameraMap_.size() ? 0 : -EINVAL;
}

/**
 * \brief Discover a valid stream path from the sensor to the video capture device
 * \param[in] media The frontend media controller device
 * \param[in] isiDevice The ISI device instance associated to the media device
 * \param[in] pipe The ISI pipe associated to that stream
 * \param[in] sensorEntity The targeted sensor media entity
 * \param[inout] streamMap The global map with all media pads already involved
 * in a camera stream
 * \param[out] routingMap The map of routings to be created for that camera
 * \param[out] cameraMediaStream The resulting camera media stream instance
 *
 * The camera media stream discovery requires:
 * - A recursive search of media link paths from the sensor to the capture video
 *   node, that is done concatenating the 2 paths:
 *     1) From the sensor to the ISI crossbar
 *     2) From the crossbar to the capture video node (trivial)
 * - The assignment of stream numbers to every pad involved in the path
 * - Build a base of default routes to be applied to the devices of the graph
 *   if they support streams.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::loadAutoDetectCameraStream(MediaDevice *media,
					       const ISIDevice *isiDevice,
					       unsigned int pipe,
					       MediaEntity *sensorEntity,
					       std::map<MediaPad *, unsigned int> *streamMap,
					       std::map<MediaEntity *, V4L2Subdevice::Routing> *routingMap,
					       CameraMediaStream *cameraMediaStream)
{
	int ret;

	MediaEntity *crossbarEntity =
		media->getEntityByName(isiDevice->kSDevCrossBarEntityName());
	if (!crossbarEntity) {
		LOG(NxpNeoPipe, Error) << "Crossbar not found";
		return -EINVAL;
	}

	/* Discover path from sensor source to crossbar sink */
	std::vector<std::vector<MediaLink *>> xbarPaths;

	/* Assume sensor has single source pad */
	ret = loadAutoDetectFindPaths(media, sensorEntity, 0,
				      crossbarEntity, kPadAny, &xbarPaths);
	if (ret) {
		LOG(NxpNeoPipe, Warning)
			<< "No path found for sensor " << sensorEntity->name();
		return -EINVAL;
	} else if (xbarPaths.size() > 1) {
		LOG(NxpNeoPipe, Warning)
			<< "Multiple paths for sensor " << sensorEntity->name();
	}

	/* Discover path from crossbar to pipe video node */
	std::vector<std::vector<MediaLink *>> pipePaths;
	MediaEntity *pipeEntity =
		media->getEntityByName(isiDevice->kVDevPipeEntityName(pipe));
	if (!pipeEntity)
		return -EINVAL;
	unsigned int crossbarSource =
		isiDevice->crossbarFirstSourcePad() + pipe;

	ret = loadAutoDetectFindPaths(media, crossbarEntity, crossbarSource,
				      pipeEntity, kPadAny, &pipePaths);
	if (ret) {
		LOG(NxpNeoPipe, Error)
			<< "No path found for pipe " << pipeEntity->name();
		return -EINVAL;
	}

	/* Concatenate full path from sensor to video node */
	std::vector<MediaLink *> &path = xbarPaths[0];
	std::vector<MediaLink *> &pipePath = pipePaths[0];
	path.reserve(path.size() + pipePath.size());
	std::move(pipePath.begin(), pipePath.end(), std::back_inserter(path));

	/* Create StreamLink (MediaLink + streams) and routing entries */
	std::vector<CameraMediaStream::StreamLink> slinks;
	const MediaPad *lastSinkPad = nullptr;
	unsigned int lastSinkStreamId = -1;
	for (MediaLink *mlink : path) {
		MediaPad *sourcePad = mlink->source();
		MediaPad *sinkPad = mlink->sink();

		unsigned int sourceStreamId =
			loadAutoDetectPadToStream(streamMap, sourcePad);
		unsigned int sinkStreamId =
			loadAutoDetectPadToStream(streamMap, sinkPad);
		slinks.emplace_back(mlink, sourceStreamId, sinkStreamId);

		/*
		 * Check if a route is needed for the source entity of the media
		 * link. The sink pad and stream information for the source
		 * entity come from the previous link. Thus first link (the
		 * sensor source) is skipped.
		 * \todo revisit when sensor internal pads are supported.
		 */
		if (lastSinkPad) {
			V4L2Subdevice::Stream sinkStream{ lastSinkPad->index(),
							  lastSinkStreamId };
			V4L2Subdevice::Stream sourceStream{ sourcePad->index(),
							    sourceStreamId };
			loadAutoDetectAddRoute(lastSinkPad->entity(),
					       &sinkStream, &sourceStream,
					       routingMap);
		}

		lastSinkPad = sinkPad;
		lastSinkStreamId = sinkStreamId;
	}

	uint32_t mbusCode = 0;
	unsigned int embeddedLines = 0;
	*cameraMediaStream = CameraMediaStream(slinks, pipe, mbusCode, embeddedLines);

	LOG(NxpNeoPipe, Debug)
		<< "Detected CameraMediaStream for " << sensorEntity->name()
		<< std::endl
		<< cameraMediaStream->toString();

	return 0;
}

/**
 * \brief Discover a valid media link path from an entity to an other
 * \param[in] media The frontend media controller device
 * \param[in] fromEntity The start entity
 * \param[in] fromPad The start entity source pad
 * \param[in] toEntity The destination entity
 * \param[in] toPad The destination entity sink pad, or kPadAny if do not care
 * \param[out] linkPaths The resulting list of media links paths
 *
 * Search recursively a path from a media entity source pad to a media entity
 * sink pad, by following the media links from the media device. From the
 * starting entity pad, every media link is visited to reach the remote entity
 * sink pad and continue the recursion from there.
 * A single entry (i.e. path) at most is expected to be reported in linkPaths.
 * Provision for multiple path is kept in order to be able to detect and warn
 * about complex topologies where multiple path candidates would have been
 * found.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::loadAutoDetectFindPaths(MediaDevice *media,
					    MediaEntity *fromEntity, unsigned int fromPad,
					    MediaEntity *toEntity, unsigned int toPad,
					    std::vector<std::vector<MediaLink *>> *linkPaths)
{
	linkPaths->clear();

	if (!fromEntity || (fromPad >= fromEntity->pads().size()) ||
	    !(fromEntity->pads()[fromPad]->flags() & MEDIA_PAD_FL_SOURCE))
		return -EINVAL;

	if (!toEntity || ((toPad != kPadAny) &&
			  ((toPad >= toEntity->pads().size() ||
			    !(toEntity->pads()[toPad]->flags() & MEDIA_PAD_FL_SINK)))))
		return -EINVAL;

	LOG(NxpNeoPipe, Debug)
		<< "Find path from " << fromEntity->name() << "/" << fromPad
		<< " to " << toEntity->name() << "/" << toPad;

	/* Visit every remote entity linked to the current entity source pad */
	std::vector<MediaLink *> mlinks = fromEntity->pads()[fromPad]->links();
	for (auto mlink : mlinks) {
		MediaEntity *remoteEntity = mlink->sink()->entity();
		unsigned int remotePad = mlink->sink()->index();

		/*
		 * In case the remote entity is the destination entity, the
		 * recursion ends with a path consisting in that single link.
		 */
		if ((remoteEntity == toEntity) &&
		    ((toPad == remotePad) || (toPad == kPadAny))) {
			linkPaths->push_back({ mlink });
			LOG(NxpNeoPipe, Debug)
				<< "Found destination via final link "
				<< fromEntity->name() << "/" << fromPad << " -> "
				<< remoteEntity->name() << "/" << remotePad;

			return 0;
		}

		/*
		 * Otherwise, recursively search from every source pad of the
		 * remote entity.
		 */
		for (auto &pad : remoteEntity->pads()) {
			if (!(pad->flags() & MEDIA_PAD_FL_SOURCE))
				continue;

			std::vector<std::vector<MediaLink *>> remotePaths;
			int ret = loadAutoDetectFindPaths(media,
							  remoteEntity, pad->index(),
							  toEntity, toPad,
							  &remotePaths);
			if (ret)
				continue;

			for (auto &remotePath : remotePaths) {
				/*
				 * Paths to destination were found through remote
				 * source pad - prepend the link to the remote
				 * entity, in order to produce the complete path.
				 * This aggregated path is added to the list of
				 * the discovered paths.
				 */
				std::vector<MediaLink *> fullPath = { mlink };
				fullPath.reserve(fullPath.size() + remotePath.size());
				std::move(remotePath.begin(), remotePath.end(),
					  std::back_inserter(fullPath));
				LOG(NxpNeoPipe, Debug)
					<< "Prepending path "
					<< fromEntity->name() << "/" << fromPad << " -> "
					<< remoteEntity->name() << "/" << remotePad
					<< " total links " << fullPath.size();

				linkPaths->push_back(std::move(fullPath));
			}
		}
	}

	return linkPaths->size() ? 0 : -EINVAL;
}

/**
 * \brief Return a stream number allocated for a media device pad
 * \param[in] streamMap The map of all media pads already used and their streams
 * \param[in] pad The targeted media device map
 *
 * Allocate a stream number to use on a media device pad. The basic assumption
 * is that any time a media pad is reused for a new camera graph, the stream
 * number has to be incremented because it is a new stream.
 *
 * \return The stream number, or zero if streams are not supported by the device
 */
unsigned int PipelineConfig::loadAutoDetectPadToStream(std::map<MediaPad *, unsigned int> *streamMap,
						       MediaPad *pad)
{
	std::unique_ptr<V4L2Subdevice> subdev;
	unsigned int stream = 0;
	MediaEntity *entity = pad->entity();
	int ret;

	if (streamMap->count(pad)) {
		/* Check if subdevice supports streams */
		subdev = std::make_unique<V4L2Subdevice>(entity);
		ret = subdev->open();
		if (ret) {
			LOG(NxpNeoPipe, Error)
				<< "Failed to open " << subdev->deviceNode();
		} else if (!subdev->caps().hasStreams()) {
			LOG(NxpNeoPipe, Error)
				<< "Unsupported multi-streams on entity "
				<< entity->name();
		} else {
			unsigned int &previous = streamMap->at(pad);
			previous++;
			stream = previous;
		}
	} else {
		streamMap->insert({ pad, 0 });
	}

	return stream;
}

/**
 * \brief Append a route to the entity routing table
 * \param[in] entity The targeted media entity
 * \param[in] sinkStream The sink stream (pad index and stream number)
 * \param[in] sourceStream The source stream (pad index and stream number)
 * \param[in] routingMap The global map of routings for all the media entities
 *
 * If the media entity supports streams, append a route for the stream to this
 * entity routing table. It does nothing if streams are not supported by the
 * entity.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::loadAutoDetectAddRoute(MediaEntity *entity,
					   V4L2Subdevice::Stream *sinkStream,
					   V4L2Subdevice::Stream *sourceStream,
					   std::map<MediaEntity *, V4L2Subdevice::Routing> *routingMap)
{
	std::unique_ptr<V4L2Subdevice> subdev;
	int ret;

	/* Check if subdevice supports streams */
	subdev = std::make_unique<V4L2Subdevice>(entity);
	ret = subdev->open();
	if (ret) {
		LOG(NxpNeoPipe, Error)
			<< "Failed to open " << subdev->deviceNode();
		return -EINVAL;
	} else if (!subdev->caps().hasStreams()) {
		return 0;
	}

	V4L2Subdevice::Routing *routing;
	if (!routingMap->count(entity))
		routingMap->insert({ entity, {} });
	routing = &(routingMap->at(entity));

	/* Append a default route for this entity */
	unsigned int flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;
	routing->emplace_back(*sinkStream, *sourceStream, flags);

	LOG(NxpNeoPipe, Debug)
		<< "Default route added for " << entity->name() << " "
		<< sinkStream->pad << "/" << sinkStream->stream << "->"
		<< sourceStream->pad << "/" << sourceStream->stream
		<< " [" << flags << "]";

	return 0;
}

/*
 * ---------------------------- Config file parsing ----------------------------
 */

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
		LOG(NxpNeoPipe, Warning)
			<< "No match dictionary node for platform config";
		return -EINVAL;
	}

	const YamlObject &entities = match["entities"];
	if (!entities.isList()) {
		LOG(NxpNeoPipe, Warning) << "No entities for match";
		return -EINVAL;
	}

	for (const auto &entity : entities.asList()) {
		std::string name = entity.get<std::string>().value_or("");
		bool found = media->getEntityByName(name);
		LOG(NxpNeoPipe, Debug) << "Entity " << name << " found " << found;
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
int PipelineConfig::parseRoutings(const YamlObject &platform, MediaDevice *media)
{
	/* Routes definition is optional */
	const YamlObject &routings = platform["routings"];
	if (!routings.isList()) {
		LOG(NxpNeoPipe, Debug) << "No routings list";
		return -EINVAL;
	}

	routingMap_.clear();

	for (const auto &subdev : routings.asList()) {
		const auto &entityNameValue = subdev["entity"].get<std::string>();
		if (!entityNameValue.has_value()) {
			LOG(NxpNeoPipe, Warning) << "Missing routing entity name";
			return -EINVAL;
		}
		std::string entityName = entityNameValue.value();
		MediaEntity *entity = media->getEntityByName(entityName);
		if (!entity) {
			LOG(NxpNeoPipe, Warning) << "Entity not found " << entityName;
			return -EINVAL;
		}

		const auto &routes = subdev["routes"];
		if (!routes.isList() || routes.size() == 0) {
			LOG(NxpNeoPipe, Debug)
				<< "No routes for entity " << entityName;
			continue;
		}

		V4L2Subdevice::Routing routing = {};
		for (const auto &route : routes.asList()) {
			std::vector<unsigned int> v =
				route.getList<unsigned int>().value_or(std::vector<unsigned int>{});
			if (v.size() != ROUTE_MAX) {
				LOG(NxpNeoPipe, Warning)
					<< "Unexpected route size " << v.size();
				return -EINVAL;
			}

			routing.emplace_back(
				V4L2Subdevice::Stream{ v[ROUTE_SINK_PAD], v[ROUTE_SINK_STREAM] },
				V4L2Subdevice::Stream{ v[ROUTE_SOURCE_PAD], v[ROUTE_SOURCE_STREAM] },
				v[ROUTE_FLAGS]);
		}

		routingMap_[entity] = routing;
		LOG(NxpNeoPipe, Debug) << "Entity name " << entityName
				   << " routing " << routing;
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

	LOG(NxpNeoPipe, Debug) << "Parsing stream " << key;

	/* yaml configuration file link sequence elements */
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
		LOG(NxpNeoPipe, Error) << "Invalid camera links list";
		return std::nullopt;
	}

	std::vector<CameraMediaStream::StreamLink> streamLinks;
	unsigned int maxUint = std::numeric_limits<unsigned int>::max();
	for (const auto &link : links.asList()) {
		std::string sourceEntityName =
			link[LINK_SOURCE_NAME].get<std::string>().value_or("");
		MediaEntity *sourceEntity =
			media->getEntityByName(sourceEntityName);
		if (!sourceEntity) {
			LOG(NxpNeoPipe, Error)
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
			LOG(NxpNeoPipe, Error)
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
			LOG(NxpNeoPipe, Error)
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
			LOG(NxpNeoPipe, Error)
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
		LOG(NxpNeoPipe, Warning) << "ISI pipe is not defined";
		return std::nullopt;
	}

	/* embedded-lines is optional - may be used for embedded-data */
	const YamlObject &lines = stream["embedded-lines"];
	unsigned int embeddedLines = lines.get<unsigned int>().value_or(0);

	CameraMediaStream mediaStream = { streamLinks, isiPipe, mbusCode, embeddedLines };

	LOG(NxpNeoPipe, Debug)
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
		LOG(NxpNeoPipe, Error) << "No camera listed";
		return -EINVAL;
	}

	for (const auto &camera : cameras.asList()) {
		const auto &entity = camera["entity"].get<std::string>();
		if (!entity.has_value()) {
			LOG(NxpNeoPipe, Error) << "Missing camera entity name";
			return -EINVAL;
		}
		std::string entityName = entity.value();

		LOG(NxpNeoPipe, Debug)
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
			LOG(NxpNeoPipe, Error)
				<< "Missing camera stream-input0 definition";
			return -EINVAL;
		}

		LOG(NxpNeoPipe, Debug)
			<< "Camera stream-input1 configured "
			<< cameraInfo.hasStreamInput1();

		LOG(NxpNeoPipe, Debug)
			<< "Camera stream-embedded configured "
			<< cameraInfo.hasStreamEmbedded();

		cameraMap_[entityName] = cameraInfo;
	}

	LOG(NxpNeoPipe, Debug) << "Camera configurations " << cameraMap_.size();
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

	LOG(NxpNeoPipe, Debug) << "Parsing config name " << name;

	ret = parseMatch(platform, media);
	if (ret)
		return ret;

	ret = parseRoutings(platform, media);
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
		LOG(NxpNeoPipe, Warning)
			<< "Failed to open pipeline config file" << filename;
		return -ENOENT;
	}

	std::unique_ptr<YamlObject> root = YamlParser::parse(file);
	if (!root) {
		LOG(NxpNeoPipe, Warning)
			<< "Failed to parse pipeline config file " << filename;
		return -EINVAL;
	}

	double version = (*root)["version"].get<double>().value_or(0.0);
	if (version != 1.0) {
		LOG(NxpNeoPipe, Warning)
			<< "Unexpected pipeline config file version "
			<< version;
		return -EINVAL;
	}

	const YamlObject &platforms = (*root)["platforms"];
	if (!platforms.isList()) {
		LOG(NxpNeoPipe, Info)
			<< "No platform listed in pipeline config file";
	}

	LOG(NxpNeoPipe, Info) << "Parsing pipeline config file " << filename;

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
 * \param[in] isiDevice The ISI Device associated to the media controller device
 *
 * Build the pipeline configuration from either the config file
 * if it exists and lists a setup corresponding to the frontend media controller
 * device. In case no such predefined is available, default to automatic
 * detection mode that works for pipelines that can be automatically discovered.
 *
 * \return 0 on success or a negative error code otherwise.
 */
int PipelineConfig::load(std::string filename, MediaDevice *media,
			 const ISIDevice *isiDevice)
{
	int ret;

	ret = loadFromFile(filename, media);
	if (ret)
		ret = loadAutoDetect(media, isiDevice);

	return ret;
}

} // namespace nxpneo

} // namespace libcamera
