/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2026 NXP
 *
 * Media Controller device graphs
 */

#include "media_graph.h"

#include <map>
#include <optional>
#include <set>
#include <vector>

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpNeoMedia)

namespace nxpneo {

/**
 * \class StreamGraph
 * \brief Represents a media graph path for a single stream
 *
 * A StreamGraph tracks the path of a single stream through the media device
 * graph, from a source entity to a destination entity. It manages the entities,
 * pads, streams, and links involved in the path, and handles configuration of
 * the stream routing.
 *
 * This class shares functional similarities with the MediaPipeline class and is
 * a candidate for future merge with it. However, the MediaPipeline class is
 * currently missing some functionalities or has different design choices that
 * make it unsuitable for the neo pipeline:
 * - Graph detection follows only the active routes in the media device, whereas
 *   StreamGraph can discover inactive paths
 * - No assignment/tracking of V4L2 streams on shared media pads
 * - No tracking of the routing configurations for the graph
 */

/**
 * \struct StreamGraph::Entity
 * \brief Represents an entity in the stream graph path
 *
 * \var StreamGraph::Entity::entity
 * \brief Pointer to the MediaEntity
 *
 * \var StreamGraph::Entity::sinkStream
 * \brief The sink pad and stream index for this entity
 *
 * \var StreamGraph::Entity::sourceStream
 * \brief The source pad and stream index for this entity
 *
 * \var StreamGraph::Entity::sourceLink
 * \brief The link from this entity's source pad to the next entity
 */

/**
 * \brief Initialize the stream graph from a source to destination entity
 * \param[in] fromEntity The starting entity in the graph
 * \param[in] fromStream The source pad and stream on the starting entity
 * \param[in] toEntity The destination entity in the graph
 * \param[in,out] padStreamsMap Map tracking stream usage for each pad
 *
 * This function finds a path through the media graph from \a fromEntity to
 * \a toEntity, starting from the specified \a fromStream. It populates the
 * internal list of entities along the path and assigns stream IDs to each
 * pad, tracking usage in \a padStreamsMap.
 *
 * \return 0 on success, negative error code otherwise
 * \retval -EINVAL Invalid parameters
 * \retval -ENOENT No path found between entities
 */
int StreamGraph::init(MediaEntity *fromEntity,
		      const V4L2Subdevice::Stream &fromStream,
		      MediaEntity *toEntity,
		      PadStreamsMap *padStreamsMap)
{
	int ret = 0;

	if (!fromEntity || fromEntity->pads().empty() ||
	    fromStream.pad >= fromEntity->pads().size() ||
	    !(fromEntity->pads()[fromStream.pad]->flags() & MEDIA_PAD_FL_SOURCE))
		return -EINVAL;

	if (!toEntity)
		return -EINVAL;

	LOG(NxpNeoMedia, Debug)
		<< "Find path from " << fromEntity->name() << " stream " << fromStream.pad
		<< "/" << fromStream.stream << " to " << toEntity->name();

	/*
	 * This function implements the graph search recursion: it starts from
	 * an entity sink pad and recursively crawls through the media graph, by
	 * following the source pads media links of that entity.
	 * If one of the downstream entity has a path to the target entity then
	 * the graph is resolved. In such case, preprend the current entity
	 * to the list of entities for that graph and return.
	 */
	std::function<bool(MediaPad *, MediaEntity *)> crawl =
		[this, &crawl](MediaPad *startSinkPad, MediaEntity *stopEntity) -> bool {
		if (!startSinkPad)
			return false;

		ASSERT(startSinkPad->flags() & MEDIA_PAD_FL_SINK);
		MediaEntity *startEntity = startSinkPad->entity();

		if (startEntity == stopEntity) {
			/* No source pad or link for the final entity */
			Entity final{
				stopEntity,
				{ startSinkPad->index(), 0 },
				{ 0, 0 },
				nullptr
			};
			entities_.emplace_back(std::move(final));
			return true;
		}

		for (const MediaPad *startPad : startEntity->pads()) {
			if (!(startPad->flags() & MEDIA_PAD_FL_SOURCE))
				continue;

			for (MediaLink *link : startPad->links()) {
				if (!link)
					continue;

				MediaPad *remoteSinkPad = link->sink();
				if (!remoteSinkPad)
					continue;

				if (!crawl(remoteSinkPad, stopEntity))
					continue;

				/* Final entity found downstream - stop here. */
				Entity e{
					startEntity,
					{ startSinkPad->index(), 0 },
					{ startPad->index(), 0 },
					link
				};
				entities_.emplace_front(std::move(e));
				return true;
			}
		}
		return false;
	};

	entities_.clear();
	routings_.clear();
	for (MediaLink *link : fromEntity->pads()[fromStream.pad]->links()) {
		if (!link)
			continue;

		MediaPad *sinkPad = link->sink();
		if (!sinkPad)
			continue;

		if (crawl(sinkPad, toEntity)) {
			/* Graph complete - no sink pad for the first entity */
			Entity first{
				fromEntity,
				{ 0, 0 },
				{ fromStream.pad, 0 },
				link
			};
			entities_.emplace_front(std::move(first));
			break;
		}
	}

	if (entities_.empty()) {
		LOG(NxpNeoMedia, Debug) << "Stream graph not found";
		return -ENOENT;
	}

	/*
	 * Update the V4L2 stream for each entity pad in the graph. As stream
	 * usage is recorded for each pad of the media device, a free pad index
	 * can be selected when a stream is created.
	 */
	auto assignPadStream = [](PadStreamsMap *streamsMap, MediaPad *pad,
				  std::optional<unsigned int> id = std::nullopt) -> unsigned int {
		auto it = streamsMap->find(pad);
		if (it == streamsMap->end()) {
			unsigned int stream = id.value_or(0);
			(*streamsMap)[pad].insert(stream);
			return stream;
		}

		unsigned int stream;
		std::set<unsigned int> &streams = it->second;
		if (!id.has_value()) {
			/* Pick the first unassigned stream value. */
			stream = 0;
			for (unsigned int allocated : streams) {
				if (allocated != stream)
					break;
				stream++;
			}
		} else {
			stream = id.value();
			if (streams.count(stream)) {
				LOG(NxpNeoMedia, Warning)
					<< "Stream " << stream << " already assigned";
			}
		}
		streams.insert(stream);
		return stream;
	};

	PadStreamsMap originalPadStreamsMap(*padStreamsMap);
	for (Entity &e : entities_) {
		/* Start entity, the source stream value comes from arguments. */
		if (&e == &entities_.front()) {
			MediaPad *sourcePad = fromEntity->pads()[fromStream.pad];
			e.sourceStream.stream =
				assignPadStream(padStreamsMap, sourcePad, { fromStream.stream });
			continue;
		}

		ASSERT(e.sinkStream.pad < e.entity->pads().size());
		MediaPad *sinkPad = e.entity->pads()[e.sinkStream.pad];
		e.sinkStream.stream = assignPadStream(padStreamsMap, sinkPad);

		/* Final entity, no source pad stream or route. */
		if (&e == &entities_.back())
			break;

		ASSERT(e.sourceStream.pad < e.entity->pads().size());
		MediaPad *sourcePad = e.entity->pads()[e.sourceStream.pad];
		e.sourceStream.stream = assignPadStream(padStreamsMap, sourcePad);

		/* Add route if entity supports streams. */
		std::unique_ptr<V4L2Subdevice> subdev =
			std::make_unique<V4L2Subdevice>(e.entity);
		ret = subdev->open();
		if (ret) {
			LOG(NxpNeoMedia, Warning)
				<< "Failed to open subdev " << e.entity->name();
			goto cleanup;
		}
		if (!subdev->caps().hasStreams())
			continue;

		V4L2Subdevice::Routing &routing = routings_[e.entity];
		routing.emplace_back(e.sinkStream,
				     e.sourceStream,
				     V4L2_SUBDEV_ROUTE_FL_ACTIVE);
	}

	LOG(NxpNeoMedia, Debug) << "Found path " << *this;

	return 0;

cleanup:
	entities_.clear();
	routings_.clear();
	*padStreamsMap = originalPadStreamsMap;
	return ret;
}

/**
 * \brief Enable all links in the stream graph
 *
 * This function enables all media links that are part of the stream graph path.
 *
 * \return 0 on success, negative error code otherwise
 */
int StreamGraph::initLinks() const
{
	for (const Entity &e : entities_) {
		if (!e.sourceLink)
			continue;

		int ret = e.sourceLink->setEnabled(true);
		if (ret < 0) {
			LOG(NxpNeoMedia, Error)
				<< "Failed to enable link from "
				<< e.sourceLink->source()->entity()->name()
				<< " to " << e.sourceLink->sink()->entity()->name();
			return ret;
		}
	}

	return 0;
}

/**
 * \brief Check if this stream graph shares resources with another
 * \param[in] other The other StreamGraph to check against
 *
 * This function determines if two stream graphs share any media pads,
 * which would indicate they cannot be used simultaneously.
 *
 * \return True if the graphs share resources, false otherwise
 */
bool StreamGraph::isShared(const StreamGraph *other) const
{
	bool shared = false;

	if (!other)
		return false;

	/* Collect all pads used by this StreamGraph */
	std::set<MediaPad *> thisPads;
	for (const auto &entity : entities_) {
		if (entity.sinkStream.pad < entity.entity->pads().size()) {
			MediaPad *sinkPad =
				entity.entity->pads()[entity.sinkStream.pad];
			thisPads.insert(sinkPad);
		}
		if (entity.sourceStream.pad < entity.entity->pads().size()) {
			MediaPad *sourcePad =
				entity.entity->pads()[entity.sourceStream.pad];
			thisPads.insert(sourcePad);
		}
	}

	/* Check if any pad from other StreamGraph is in this set */
	for (const auto &entity : other->entities_) {
		if (entity.sinkStream.pad < entity.entity->pads().size()) {
			MediaPad *sinkPad =
				entity.entity->pads()[entity.sinkStream.pad];
			if (thisPads.count(sinkPad)) {
				shared = true;
				break;
			}
		}
		if (entity.sourceStream.pad < entity.entity->pads().size()) {
			MediaPad *sourcePad =
				entity.entity->pads()[entity.sourceStream.pad];
			if (thisPads.count(sourcePad)) {
				shared = true;
				break;
			}
		}
	}

	if (!entities().empty() && !other->entities().empty()) {
		LOG(NxpNeoMedia, Debug)
			<< "((" << entities().front().entity->name() << " -> "
			<< entities().back().entity->name() << "), "
			<< "(" << other->entities().front().entity->name() << " -> "
			<< other->entities().back().entity->name() << "))"
			<< " shared " << std::boolalpha << shared;
	}

	return shared;
}

/**
 * \brief Configure the format for all entities in the stream graph
 * \param[in,out] format The format to configure on each entity
 *
 * This function propagates the specified format through all entities in the
 * stream graph, configuring each subdevice's sink pad which also propagates the
 * format to its source pad. The first entity's source pad is assumed to be
 * already configured.
 *
 * \return 0 on success, negative error code otherwise
 */
int StreamGraph::configure(V4L2SubdeviceFormat &format) const
{
	for (const Entity &e : entities_) {
		/* Source pad of the first entity is already configured. */
		if (&e == &entities_.front())
			continue;

		/* Stop at video device node. */
		MediaEntity *entity = e.entity;
		if (entity->function() == MEDIA_ENT_F_IO_V4L)
			break;

		std::unique_ptr<V4L2Subdevice> subdev =
			std::make_unique<V4L2Subdevice>(entity);

		int ret = subdev->open();
		if (ret < 0) {
			LOG(NxpNeoMedia, Error)
				<< "Failed to open subdev " << entity->name();
			return ret;
		}

		/* Set format on sink pad and let it propagate to source pad. */
		LOG(NxpNeoMedia, Debug)
			<< "Configure " << e.entity->name() << " stream ("
			<< e.sinkStream.pad << "/" << e.sinkStream.stream
			<< ") format " << format.toString();

		ret = subdev->setFormat(e.sinkStream, &format);
		if (ret < 0) {
			LOG(NxpNeoMedia, Error)
				<< "Failed to set format on " << entity->name();
			return ret;
		}
	}

	return 0;
}

/**
 * \fn StreamGraph::entities()
 * \brief Retrieve the list of entities in the stream graph
 * \return The list of entities
 */

/**
 * \fn StreamGraph::routings()
 * \brief Retrieve the routing configuration for all entities
 * \return Map of MediaEntity to V4L2Subdevice::Routing
 */

/**
 * \brief Output stream operator for StreamGraph
 * \param[in] out The output stream
 * \param[in] sg The StreamGraph to output
 *
 * This operator outputs a human-readable representation of the StreamGraph,
 * including all entities and their stream configurations.
 *
 * \return The output stream
 */
std::ostream &operator<<(std::ostream &out, const StreamGraph &sg)
{
	out << "StreamGraph with " << sg.entities().size() << " entities:" << std::endl;

	for (const auto &e : sg.entities()) {
		out << "  Entity: " << e.entity->name();
		if (&e != &sg.entities().front())
			out << " sink[" << e.sinkStream.pad << "/"
			    << e.sinkStream.stream << "]";
		if (&e != &sg.entities().back())
			out << " -> source[" << e.sourceStream.pad << "/"
			    << e.sourceStream.stream << "]";

		if (e.sourceLink)
			out << " link to " << e.sourceLink->sink()->entity()->name();
		out << std::endl;

		/* Log routing information for this entity if available */
		auto it = sg.routings().find(e.entity);
		if (it != sg.routings().end()) {
			const V4L2Subdevice::Routing &routing = it->second;
			out << "    Routing (" << routing.size() << " routes):" << std::endl;
			for (const auto &route : routing) {
				out << "      sink[" << route.sink.pad << "/"
				    << route.sink.stream << "] -> source["
				    << route.source.pad << "/" << route.source.stream
				    << "] flags: 0x" << std::hex << route.flags
				    << std::dec << std::endl;
			}
		}
	}

	return out;
}

} /* namespace nxpneo */

} /* namespace libcamera */
