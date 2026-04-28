/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2026 NXP
 *
 * Media Controller device graphs
 */

#pragma once

#include <map>
#include <set>
#include <vector>

#include "libcamera/internal/v4l2_subdevice.h"

namespace libcamera {

class MediaDevice;
class MediaEntity;
class MediaLink;
class MediaPad;
struct V4L2SubdeviceFormat;

namespace nxpneo {

using PadStreamsMap = std::map<MediaPad *, std::set<unsigned int>>;

class StreamGraph
{
public:
	struct Entity {
		MediaEntity *entity;
		V4L2Subdevice::Stream sinkStream;
		V4L2Subdevice::Stream sourceStream;
		MediaLink *sourceLink;
	};

	int init(MediaEntity *fromEntity,
		 const V4L2Subdevice::Stream &fromStream,
		 MediaEntity *toEntity,
		 PadStreamsMap *padStreamsMap);

	int initLinks() const;
	int configure(V4L2SubdeviceFormat &format) const;

	bool isShared(const StreamGraph *other) const;

	const std::list<Entity> &entities() const { return entities_; }
	const std::map<MediaEntity *, V4L2Subdevice::Routing> &
	routings() const { return routings_; }

private:
	std::list<Entity> entities_;
	std::map<MediaEntity *, V4L2Subdevice::Routing> routings_;
};

std::ostream &operator<<(std::ostream &out, const StreamGraph &sg);

} /* namespace nxpneo */

} /* namespace libcamera */
