/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * sysfs-based device enumerator
 */

#include "libcamera/internal/device_enumerator_sysfs.h"

#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <sys/inotify.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/media_device.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(DeviceEnumerator)

DeviceEnumeratorSysfs::DeviceEnumeratorSysfs()
	: inotFd_(-1), inotWd_(-1), notifier_(nullptr)
{
}

DeviceEnumeratorSysfs::~DeviceEnumeratorSysfs()
{
	inotify_rm_watch(inotFd_, inotWd_);
	close(inotFd_);
	delete notifier_;
}

int DeviceEnumeratorSysfs::init()
{
	return 0;
}

int DeviceEnumeratorSysfs::addSysfsDevice(const char *name)
{
	struct stat devStat;

	const std::string deviceNode =
		std::string("/dev/") + name;

	if (stat(deviceNode.c_str(), &devStat) < 0) {
		LOG(DeviceEnumerator, Warning)
			<< "Device node " << deviceNode << "should exist but doesn't";
		return -ENODEV;
	}

	if (!strncmp(name, "media", 5)) {
		std::unique_ptr<MediaDevice> media = createDevice(deviceNode.c_str());

		if (!media)
			return -ENODEV;

		DependencyMap deps;

		int ret = populateMediaDevice(media.get(), &deps);
		if (ret < 0) {
			LOG(DeviceEnumerator, Warning)
				<< "Failed to populate media device "
				<< media->deviceNode() << " (" << media->driver() << "), skipping";
			return ret;
		}
		if (!deps.empty()) {
			LOG(DeviceEnumerator, Debug)
				<< "Defer media device " << media->deviceNode()
				<< " due to " << deps.size() << " missing dependencies";

			pending_.emplace_back(std::move(media), std::move(deps));
			MediaDeviceDeps *mediaDeps = &pending_.back();
			for (const auto &dep : mediaDeps->deps_)
				devMap_[dep.first] = mediaDeps;

			return 0;
		}
		addDevice(std::move(media));
		return 0;
	}

	if (!strncmp(name, "video", 5) || !strncmp(name, "v4l-subdev", 10)) {
		addV4L2Device(devStat.st_rdev);
		return 0;
	}

	return -ENODEV;
}

int DeviceEnumeratorSysfs::enumerate()
{
	inotFd_ = inotify_init();

	if (inotFd_ < 0) {
		LOG(DeviceEnumerator, Warning)
			<< "Failed to initialize inotify fd, error:"
			<< strerror(errno);
		return errno;
	}
	inotWd_ = inotify_add_watch(inotFd_, "/dev", IN_CREATE | IN_DELETE);

	if (inotWd_ < 0) {
		LOG(DeviceEnumerator, Warning)
			<< "Failed add inotify watch, error:"
			<< strerror(errno);
	} else {
		notifier_ = new EventNotifier(inotFd_, EventNotifier::Read);
		notifier_->activated.connect(this, [this] {
			sysfsNotify();
		});
	}

	/* Checking /dev/media, /dev/v4l-subdev and /dev/video device nodes. */
	DIR *dir = opendir("/dev/");

	if (!dir) {
		LOG(DeviceEnumerator, Error)
			<< "No valid sysfs media device directory";
		return -ENODEV;
	}

	struct dirent *ent;
	while ((ent = readdir(dir)) != nullptr) {
		addSysfsDevice(ent->d_name);
	}

	closedir(dir);

	return 0;
}

int DeviceEnumeratorSysfs::populateMediaDevice(MediaDevice *media, DependencyMap *deps)
{
	std::set<dev_t> children;

	/* Associate entities to device node paths. */
	for (MediaEntity *entity : media->entities()) {
		if (entity->deviceMajor() == 0 && entity->deviceMinor() == 0)
			continue;

		dev_t devnum = makedev(entity->deviceMajor(), entity->deviceMinor());

		/*
		* If the devnum isn't in the orphans list, add it to the unmet
		* dependencies.
		*/
		if (orphans_.find(devnum) == orphans_.end()) {
			(*deps)[devnum].push_back(entity);
			continue;
		}

		/*
		* Otherwise take it from the orphans list. Don't remove the
		* entry from the list yet as other entities in this media
		* device may need the same device.
		*/
		std::string deviceNode =
			lookupDeviceNode(entity->deviceMajor(), entity->deviceMinor());
		if (deviceNode.empty())
			return -EINVAL;

		int ret = entity->setDeviceNode(deviceNode);
		if (ret)
			return ret;

		children.insert(devnum);
	}

	/* Remove all found children from the orphans list. */
	for (auto it = orphans_.begin(), last = orphans_.end(); it != last;) {
		if (children.find(*it) != children.end())
			it = orphans_.erase(it);
		else
			++it;
	}

	return 0;
}

/**
 * \brief Lookup device node path from device number
 * \param[in] major The device major number
 * \param[in] minor The device minor number
 *
 * Translate a device number given as \a major and \a minor to a device node
 * path.
 *
 * \return The device node path on success, or an empty string if the lookup
 * fails
 */
std::string DeviceEnumeratorSysfs::lookupDeviceNode(int major, int minor)
{
	std::string deviceNode;
	std::string line;
	std::ifstream ueventFile;

	ueventFile.open("/sys/dev/char/" + std::to_string(major) + ":" +
			std::to_string(minor) + "/uevent");
	if (!ueventFile)
		return std::string();

	while (ueventFile >> line) {
		if (line.find("DEVNAME=") == 0) {
			deviceNode = "/dev/" + line.substr(strlen("DEVNAME="));
			break;
		}
	}

	ueventFile.close();

	return deviceNode;
}

/**
 * \brief Add a V4L2 device to the media device that it belongs to
 * \param[in] devnum major:minor number of V4L2 device to add, as a dev_t
 *
 * Add V4L2 device identified by \a devnum to the MediaDevice that it belongs
 * to, if such a MediaDevice has been created. Otherwise add the V4L2 device
 * to the orphan list. If the V4L2 device is added to a MediaDevice, and it is
 * the last V4L2 device that the MediaDevice needs, then the MediaDevice is
 * added to the DeviceEnumerator, where it is available for pipeline handlers.
 *
 * \return 0 on success or a negative error code otherwise
 */
int DeviceEnumeratorSysfs::addV4L2Device(dev_t devnum)
{
	/*
	 * If the devnum doesn't belong to any media device, add it to the
	 * orphans list.
	 */
	auto it = devMap_.find(devnum);
	if (it == devMap_.end()) {
		orphans_.insert(devnum);
		return 0;
	}

	/*
	 * Set the device node for all entities matching the devnum. Multiple
	 * entities can share the same device node, for instance for V4L2 M2M
	 * devices.
	 */
	std::string deviceNode = lookupDeviceNode(major(devnum), minor(devnum));
	if (deviceNode.empty())
		return -EINVAL;

	MediaDeviceDeps *deps = it->second;
	for (MediaEntity *entity : deps->deps_[devnum]) {
		int ret = entity->setDeviceNode(deviceNode);
		if (ret)
			return ret;
	}

	/*
	 * Remove the devnum from the unmet dependencies for this media device.
	 * If no more dependency is unmet, add the media device to the
	 * enumerator.
	 */
	deps->deps_.erase(devnum);
	devMap_.erase(it);

	if (deps->deps_.empty()) {
		LOG(DeviceEnumerator, Debug)
			<< "All dependencies for media device "
			<< deps->media_->deviceNode() << " found";
		addDevice(std::move(deps->media_));
		pending_.remove(*deps);
	}

	return 0;
}

void DeviceEnumeratorSysfs::sysfsNotify()
{
	const size_t INOTIFY_BUF_LEN = 512;
	char eventsBuf[INOTIFY_BUF_LEN];
	int received = read(inotFd_, eventsBuf, INOTIFY_BUF_LEN);

	if (received < 0) {
		LOG(DeviceEnumerator, Warning)
			<< "Fail to read from inotFd_, error:" << strerror(errno);
		return;
	}

	/* Read could return multiple `struct inotify_event`. */
	char *eventPtr = eventsBuf;
	while (eventPtr < (eventsBuf + received)) {
		struct inotify_event *inotifyEvent =
			reinterpret_cast<inotify_event *>(eventPtr);

		eventPtr += sizeof(struct inotify_event) + inotifyEvent->len;

		if (inotifyEvent->mask & IN_CREATE) {
			addSysfsDevice(inotifyEvent->name);
		} else if (inotifyEvent->mask & IN_DELETE) {
			removeDevice(inotifyEvent->name);
		}
	}
}

} /* namespace libcamera */
