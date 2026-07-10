/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * sysfs-based device enumerator
 */

#pragma once

#include <list>
#include <map>
#include <set>
#include <string>

#include <libcamera/base/timer.h>
#include <libcamera/base/event_notifier.h>

#include "libcamera/internal/device_enumerator.h"

namespace libcamera {

class EventNotifier;
class MediaDevice;
class MediaEntity;

class DeviceEnumeratorSysfs final : public DeviceEnumerator
{
public:
	DeviceEnumeratorSysfs();
	~DeviceEnumeratorSysfs();

	int init();
	int enumerate();

private:
	using DependencyMap = std::map<dev_t, std::list<MediaEntity *>>;

	struct MediaDeviceDeps {
		MediaDeviceDeps(std::unique_ptr<MediaDevice> media,
				DependencyMap deps)
			: media_(std::move(media)), deps_(std::move(deps))
		{
		}

		bool operator==(const MediaDeviceDeps &other) const
		{
			return media_ == other.media_;
		}

		std::unique_ptr<MediaDevice> media_;
		DependencyMap deps_;
	};

	int addSysfsDevice(const char *devPath);
	int populateMediaDevice(MediaDevice *media, DependencyMap *deps);
	std::string lookupDeviceNode(int major, int minor);

	void retryPendingWintUnknownDeps();

	int addV4L2Device(dev_t devum);
	void sysfsNotify();

	int inotFd_;
	int inotWd_;
	EventNotifier *notifier_;

	std::set<dev_t> orphans_;
	std::list<MediaDeviceDeps> pending_;
	std::set<std::string> pendingWithUnknowns_;
	Timer pendingWithUnknownsTimer_;

	std::chrono::milliseconds pendingWithUnknownsDelayMs_;
	std::map<dev_t, MediaDeviceDeps *> devMap_;
};

} /* namespace libcamera */
