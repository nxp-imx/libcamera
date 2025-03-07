/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * neo-utils.cpp - Helpers for NXP NEO pipeline
 * Copyright 2024-2025 NXP
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

/**
 * \struct CameraProperties
 * \brief Camera properties defined by topology discovery or configuration file
 *
 * \var CameraProperties::hdrStream
 * \brief Camera has a dedicated stream enabled for HDR short capture
 *
 * \var CameraProperties::eDataStream
 * \brief Camera has a dedicated stream enabled for embedded data
 *
 * \var CameraProperties::multiCamera
 * \brief Camera is sharing its MIPI CSI-2 port with other cameras
 *
 * This structure reports to the pipeline handler a set of properties defined
 * in the configuration file, or detected during the discovery procedure.
 */

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

	ss << " isi-pipe " << pipe();

	return ss.str();
}

/* -----------------------------------------------------------------------------
 * CameraInfo class
 */

/**
 * \brief Return an optional CameraMediaStream for the camera
 * \param[in] streamId The CameraInfo stream identifier STREAM_<XYZ>
 * \return The CameraMediaStream if it exists, nullptr otherwise
 */
const CameraMediaStream *CameraInfo::stream(unsigned int id) const
{
	if (id >= STREAM_MAX) {
		LOG(NxpNeoPipe, Error) << "Invalid stream " << id;
		return nullptr;
	}

	auto it = streams_.find(id);
	if (it != streams_.end())
		return &it->second;
	else
		return nullptr;
}

/* -----------------------------------------------------------------------------
 * PipelineConfig class
 */

/**
 * \brief Class destructor
 */
PipelineConfig::~PipelineConfig()
{
	/* Release allocated ISI channels */
	if (!isiDevice_)
		return;

	for (auto &[name, cameraInfo] : cameraMap_) {
		for (auto &[id, stream] : cameraInfo.streams_)
			isiDevice_->releasePipe(stream.pipe());
	}
}

/**
 * \brief Load the pipeline configuration
 * \param[in] file The path to the pipeline configuration file
 * \param[in] isiDevice The ISI Device associated to the media controller device
 *
 * Build the pipeline configuration that consists in the parameters that may be
 * defined in the pipeline handler configuration file and the camera streams
 * graphs that are dynamically discovered in the media device.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::load(std::string filename, std::shared_ptr<ISIDevice> isiDevice)
{
	isiDevice_ = isiDevice;

	int ret = loadFileConfig(filename);
	if (ret)
		LOG(NxpNeoPipe, Info) << "Could not parse config file " << filename;

	ret = loadAutoDetect();
	return ret;
}

/**
 * \brief Report the CameraInfo associated to a camera
 * \param[in] name The name of the camera media device entity
 *
 * The CameraInfo structure carries information related to the integration
 * of the sensor into the media device. Data is acquired either via
 * automatic graph detection or through parsing of a platform configuration
 * file.
 *
 * \return The pointer to CameraInfo structure if it exists, nullptr otherwise
 */
const CameraInfo *PipelineConfig::cameraInfo(const std::string &name) const
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
const RoutingMap &PipelineConfig::routingMap() const
{
	return routingMap_;
}

/**
 * \brief Report the global pipeline handler configuration
 *
 * This function reports the global pipeline handler configuration that is not
 * specific to a given camera.
 *
 * \return A reference to the global configuration
 */
const GlobalInfo &PipelineConfig::globalInfo() const
{
	return globalInfo_;
}

/**
 * \brief Discover the valid camera graphs to the capture video device
 *
 * For every camera sensor in the media device, look for valid media links paths
 * to the capture video device. Also, build the aggregated global routing table
 * for all the cameras detected.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::loadAutoDetect()
{
	int ret;

	MediaDevice *media = isiDevice_->media();
	if (!media)
		return -EINVAL;

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

	/*
	 * Merge the routings for entities in the delta map, into the base
	 * map of routings.
	 */
	auto mergeRoutingMap =
		[](RoutingMap &base, RoutingMap &delta) {
			for (auto &[entity, routing] : delta) {
				if (base.count(entity)) {
					V4L2Subdevice::Routing &origin = base[entity];
					origin.reserve(origin.size() + routing.size());
					std::move(routing.begin(), routing.end(),
						  std::back_inserter(origin));
				} else {
					base[entity] = std::move(routing);
				}
			}
		};

	std::set<MediaEntity *, decltype(compareName)> sensorsEntities(compareName);
	for (MediaEntity *e : media->entities()) {
		if (e->function() != MEDIA_ENT_F_CAM_SENSOR)
			continue;
		sensorsEntities.insert(e);
	}

	/* Discover the topology of every sensor */
	ISIDevice *isiDevice = isiDevice_.get();
	for (MediaEntity *entity : sensorsEntities) {
		LOG(NxpNeoPipe, Debug) << "Auto detect camera " << entity->name();

		CameraInfo cameraInfo = {};

		std::unique_ptr<CameraSensor> sensor =
			CameraSensorFactoryBase::create(entity);
		if (!sensor) {
			LOG(NxpNeoPipe, Warning)
				<< "Could not construct camera sensor "
				<< entity->name();
			continue;
		}

		/*
		 * Store the reference to the properties associated to that
		 * camera. Give precedence to the name-based over model-based
		 * properties because it is more specialized.
		 */
		const std::string &name = sensor->entity()->name();
		const std::string &model = sensor->model();
		if (namePropertiesMap_.count(name))
			cameraInfo.properties_ = &namePropertiesMap_[name];
		else
			cameraInfo.properties_ = &modelPropertiesMap_[model];

		Size size = sensor->resolution();

		/* Map for each stream the pipe index and per-entity routing */
		std::map<unsigned int, unsigned int> pipeIndex;
		std::map<unsigned int, RoutingMap> routingMaps;

		/* Copy of the global streams map - revert changes in case of error */
		std::map<MediaPad *, unsigned int> streamMap(globalStreamMap);

		for (auto stream : CameraInfo::kCameraStreams) {
			V4L2Subdevice::Stream sensorStream;
			if (stream == CameraInfo::STREAM_INPUT0) {
				sensorStream = sensor->imageStream();
			} else if (stream == CameraInfo::STREAM_INPUT1) {
				bool enable = cameraInfo.properties_->hdrStream;
				if (!enable)
					continue;
				if (!sensor->auxiliaryStream().has_value()) {
					LOG(NxpNeoPipe, Warning)
						<< "Sensor has no auxiliary stream";
					continue;
				}
				sensorStream = sensor->auxiliaryStream().value();
			} else if (stream == CameraInfo::STREAM_EDATA) {
				bool enable = cameraInfo.properties_->eDataStream;
				if (!enable)
					continue;
				if (!sensor->embeddedDataStream().has_value()) {
					LOG(NxpNeoPipe, Warning)
						<< "Sensor has no embedded data stream";
					continue;
				}
				sensorStream = sensor->embeddedDataStream().value();
			} else {
				LOG(NxpNeoPipe, Warning) << "Invalid sensor stream";
				return -EINVAL;
			}

			unsigned int index;
			ret = isiDevice->reservePipeBySize(size, &index);
			if (ret) {
				LOG(NxpNeoPipe, Warning) << "Input pipe allocation failed";
				goto error;
			}
			pipeIndex[stream] = index;

			CameraMediaStream cameraMediaStream;
			RoutingMap routingMap;

			ret = loadAutoDetectCameraStream(
				index, entity,
				sensorStream.pad, sensorStream.stream,
				&streamMap, &routingMap, &cameraMediaStream);
			if (ret)
				goto error;

			cameraInfo.streams_[stream] = std::move(cameraMediaStream);
			routingMaps[stream] = std::move(routingMap);
		}

		/*
		 * CameraInfo succesfully created
		 * - Store resulting entry into cameras database
		 * - Merge camera streams routings to global routing
		 * - Update the global streams map with the camera streams
		 */
		cameraMap_[entity->name()] = std::move(cameraInfo);

		for (auto &[stream, routingMap] : routingMaps)
			mergeRoutingMap(routingMap_, routingMap);

		globalStreamMap = std::move(streamMap);

		continue;

	error:
		for (auto [stream, index] : pipeIndex)
			isiDevice->releasePipe(index);
	}

	/* Finally, detect multi-camera conditions */
	loadAutoDetectMultiCamera();

	return cameraMap_.size() ? 0 : -EINVAL;
}

/**
 * \brief Discover a valid stream path from the sensor to the video capture device
 * \param[in] pipe The ISI pipe associated to that stream
 * \param[in] sensorEntity The targeted sensor media entity
 * \param[in] sensorPad The targeted sensor source pad
 * \param[in] sensorStream The targeted sensor source stream
 * \param[inout] streamMap The global map with all media pads already involved
 * in a camera stream
 * \param[out] routingMap The map of routings to be created for that camera stream
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
int PipelineConfig::loadAutoDetectCameraStream(unsigned int pipe,
					       MediaEntity *sensorEntity,
					       unsigned int sensorPad,
					       unsigned int sensorStream,
					       std::map<MediaPad *, unsigned int> *streamMap,
					       RoutingMap *routingMap,
					       CameraMediaStream *cameraMediaStream)
{
	int ret;

	MediaDevice *media = isiDevice_->media();
	MediaEntity *crossbarEntity =
		media->getEntityByName(isiDevice_->kSDevCrossBarEntityName());
	if (!crossbarEntity) {
		LOG(NxpNeoPipe, Error) << "Crossbar not found";
		return -EINVAL;
	}

	/* Discover path from sensor source to crossbar sink */
	std::vector<std::vector<MediaLink *>> xbarPaths;

	ret = loadAutoDetectFindPaths(sensorEntity, sensorPad,
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
		media->getEntityByName(isiDevice_->kVDevPipeEntityName(pipe));
	if (!pipeEntity)
		return -EINVAL;
	unsigned int crossbarSource =
		isiDevice_->crossbarFirstSourcePad() + pipe;

	ret = loadAutoDetectFindPaths(crossbarEntity, crossbarSource,
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
	unsigned int sourceStreamId;
	unsigned int sinkStreamId;
	for (MediaLink *mlink : path) {
		MediaPad *sourcePad = mlink->source();
		MediaPad *sinkPad = mlink->sink();

		if (mlink->source()->entity() == sensorEntity) {
			sourceStreamId = sensorStream;
			sinkStreamId = sensorStream;
		} else {
			sourceStreamId = loadAutoDetectPadToStream(streamMap, sourcePad);
			sinkStreamId = loadAutoDetectPadToStream(streamMap, sinkPad);
		}
		slinks.emplace_back(mlink, sourceStreamId, sinkStreamId);

		/*
		 * Check if a route is needed for the source entity of the media
		 * link. The sink pad and stream information for the source
		 * entity come from the previous link. Thus first link (the
		 * sensor source) is skipped.
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

	CameraMediaStream _cameraMediaStream(slinks, pipe);
	*cameraMediaStream = std::move(_cameraMediaStream);

	LOG(NxpNeoPipe, Debug)
		<< "Detected CameraMediaStream " << sensorStream
		<< " for " << sensorEntity->name() << std::endl
		<< cameraMediaStream->toString();

	return 0;
}

/**
 * \brief Discover a valid media link path from an entity to an other
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
int PipelineConfig::loadAutoDetectFindPaths(MediaEntity *fromEntity, unsigned int fromPad,
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
			int ret = loadAutoDetectFindPaths(remoteEntity, pad->index(),
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
 * \param[inout] streamMap The map of all media pads already used and their streams
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

/**
 * \brief Detect cases where a MIPI CSI-2 port is shared by multiple cameras
 *
 * When the same MIPI CSI-2 port is shared by multiple cameras typically through
 * the usage of a SerDes, some restrictions apply regarding the allowed
 * configurations and transitions supported by the front-end media device.
 * The multi-camera use case is detected by counting the number of camera whose
 * main image stream is connected to the same ISI crossbar sink.
 * The CameraProperties structures of those cameras are updated to reflect that
 * condition so that the pipeline handler knows about it.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::loadAutoDetectMultiCamera()
{
	/* Record ISI crossbar sink for every camera */
	MediaDevice *media = isiDevice_->media();
	MediaEntity *crossbarEntity =
		media->getEntityByName(isiDevice_->kSDevCrossBarEntityName());
	std::map<std::string, unsigned int> cameraXbarSink;
	for (auto &[name, cameraInfo] : cameraMap_) {
		const CameraMediaStream *cameraStream =
			cameraInfo.stream(CameraInfo::STREAM_INPUT0);
		if (!cameraStream) {
			LOG(NxpNeoPipe, Error)
				<< "No input0 stream for camera " << name;
			return -EINVAL;
		}

		const std::vector<CameraMediaStream::StreamLink> &streamLinks =
			cameraStream->streamLinks();

		MediaLink *link = nullptr;
		for (const CameraMediaStream::StreamLink &streamLink : streamLinks) {
			link = streamLink.mediaLink_;
			if (link->sink()->entity() == crossbarEntity)
				break;
		}

		if (!link) {
			LOG(NxpNeoPipe, Error)
				<< "No crossbar connection for camera " << name;
			return -EINVAL;
		}

		cameraXbarSink[name] = link->sink()->index();
	}

	/* Count the cameras linked to each sink pad of the ISI crossbar */
	std::map<unsigned int, unsigned int> xbarSinkCount;
	for (auto &[name, sink] : cameraXbarSink)
		xbarSinkCount[sink] += 1;

	/* Record the multi-camera status into the relevant camera properties */
	for (auto &[name, cameraInfo] : cameraMap_) {
		unsigned int sink = cameraXbarSink[name];
		unsigned cameraCount = xbarSinkCount[sink];
		bool multiCamera = cameraCount > 1 ? true : false;
		LOG(NxpNeoPipe, Debug)
			<< "Camera " << name << " sink " << sink
			<< " multi-camera " << multiCamera << " count " << cameraCount;

		cameraInfo.properties_->multiCamera = multiCamera;
	}

	return 0;
}

/*
 * ---------------------------- Config file parsing ----------------------------
 */

/**
 * \brief Parse the cameras section in the yaml configuration file
 * \param[in] cameras The cameras node in yaml file
 * \return 0 if no error was detected, a negative error code otherwise
 */
int PipelineConfig::parseCameras(const YamlObject &cameras)
{
	for (const auto &cameraObj : cameras.asList()) {
		CameraProperties properties = {};

		const YamlObject &nameObj = cameraObj["name"];
		std::string name = nameObj.get<std::string>().value_or("");

		const YamlObject &modelObj = cameraObj["model"];
		std::string model = modelObj.get<std::string>().value_or("");

		const YamlObject &entityObj = cameraObj["entity"];
		std::string entity = entityObj.get<std::string>().value_or("");

		const YamlObject &streamsObj = cameraObj["streams"];
		for (const auto &streamObj : streamsObj.asList()) {
			std::string stream =
				streamObj.get<std::string>().value_or("");
			if (stream == "hdr")
				properties.hdrStream = true;
			else if (stream == "edata")
				properties.eDataStream = true;
		}

		LOG(NxpNeoPipe, Debug)
			<< "Camera entry [" << name
			<< "] model ["  << model << "] entity [" << entity
			<< "] streams hdr " << properties.hdrStream
			<< " edata " << properties.eDataStream;

		if (!model.length() && !entity.length()) {
			LOG(NxpNeoPipe, Warning)
				<< "Camera needs model or entity definition";
			continue;
		}

		if (model.length()) {
			if (modelPropertiesMap_.count(model)) {
				LOG(NxpNeoPipe, Warning) <<
					"Duplicate camera model " << model;
				continue;
			}
			modelPropertiesMap_[model] = properties;
		}

		if (entity.length()) {
			if (namePropertiesMap_.count(entity)) {
				LOG(NxpNeoPipe, Warning) <<
					"Duplicate camera entity " << entity;
				continue;
			}
			namePropertiesMap_[entity] = properties;
		}
	}

	return 0;
}

/**
 * \brief Parse the global section in the yaml configuration file
 * \param[in] global The global node in yaml file
 * \return 0 if no error was detected, a negative error code otherwise
 */
int PipelineConfig::parseGlobal(const YamlObject &global)
{
	const YamlObject &bufferCountObj = global["buffer-count"];
	globalInfo_.bufferCount =
		bufferCountObj.get<unsigned int>().value_or(GlobalInfo::kBufferCount);

	return 0;
}

/**
 * \brief Load the pipeline configuration from a pipeline configuration file
 * \param[in] filename The path to configuration file
 * \return 0 if config file was parsed correctly, a negative error code otherwise
 */
int PipelineConfig::loadFileConfig(std::string filename)
{
	File file(filename);

	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(NxpNeoPipe, Info)
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

	LOG(NxpNeoPipe, Debug) << "Parsing pipeline config file " << filename;

	const YamlObject &global = (*root)["global"];
	int ret = parseGlobal(global);
	if (ret)
		LOG(NxpNeoPipe, Warning)
			<< "Invalid global section in config file";

	const YamlObject &cameras = (*root)["cameras"];
	ret = parseCameras(cameras);
	if (ret)
		LOG(NxpNeoPipe, Warning)
			<< "Invalid cameras section in config file";

	return ret;
}

} // namespace nxpneo

} // namespace libcamera
