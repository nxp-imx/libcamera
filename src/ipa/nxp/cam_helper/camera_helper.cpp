/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on Helper class that performs sensor-specific parameter computations
 *     src/ipa/libipa/camera_sensor_helper.c
 * Copyright (C) 2021, Google Inc.
 *
 * camera_helper.c
 * Helper class that performs sensor-specific parameter computations
 * Copyright 2024 NXP
 */
#include "camera_helper.h"

#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>
#include <libcamera/controls.h>

/**
 * \file camera_helper.h
 * \brief Helper class that performs sensor-specific parameter computations
 *
 * Computation of sensor configuration parameters is a sensor specific
 * operation. Each CameraHelper derived class computes the value of
 * configuration parameters, for example the analogue gain value, using
 * sensor-specific functions and constants.
 *
 * Every subclass of CameraHelper shall be registered using
 * the REGISTER_CAMERA_HELPER() macro.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(NxpCameraHelper)

namespace nxp {

/**
 * \class CameraHelper
 * \brief Base class for computing sensor tuning parameters using
 * sensor-specific constants
 *
 * Instances derived from CameraSensorHelper class are sensor-specific.
 * Each supported sensor will have an associated base class defined.
 */

/**
 * \brief Construct a CameraHelper instance
 *
 * NXpCameraSensorHelper derived class instances shall never be constructed
 * manually but always through the CameraHelperFactoryBase::create()
 * function.
 */


/**
 * \brief Retrieve exposure from the control list
 * \param[in] ctrls The control list to use
 *
 * This function aims to abstract the exposure control for sensor having
 * a proprietary programming model.
 *
 * \return The exposure time in line duration units
 */
uint32_t CameraHelper::controlListGetExposure(const ControlList *ctrls) const
{
	uint32_t exposure = 0;
	if (!ctrls->contains(V4L2_CID_EXPOSURE)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_EXPOSURE cannot be got";
		return exposure;
	}

	/*
	 * \todo Spurious frame loss at stream start() induces invalid
	 * control values reported by pipeline's DelayedControl object.
	 * Occurence is logged by pipeline, workaround the issue to avoid
	 * later assert condition failure.
	 */
	const ControlValue &exposureValue = ctrls->get(V4L2_CID_EXPOSURE);

	if (exposureValue.type() != ControlTypeNone)
		exposure = exposureValue.get<int32_t>();
	else
		/* Keep debug level for now as root cause occurence is logged */
		LOG(NxpCameraHelper, Debug) << "Invalid exposure control";

	return exposure;
}

/**
 * \brief Retrieve gain code from the control list
 * \param[in] ctrls The control list to use
 *
 * This function aims to abstract the gain control for sensor having
 * a proprietary programming model.
 *
 * \return The gain code in sensor format
 */
uint32_t CameraHelper::controlListGetGain(const ControlList *ctrls) const
{

	uint32_t gain = 0;
	if (!ctrls->contains(V4L2_CID_ANALOGUE_GAIN)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_ANALOGUE_GAIN cannot be got";
		return gain;
	}

	/*
	 * \todo Spurious frame loss at stream start() induces invalid
	 * control values reported by pipeline's DelayedControl object.
	 * Occurence is logged by pipeline, workaround the issue to avoid
	 * later assert condition failure.
	 */
	const ControlValue &gainValue = ctrls->get(V4L2_CID_ANALOGUE_GAIN);

	if (gainValue.type() != ControlTypeNone)
		gain = gainValue.get<int32_t>();
	else
		/* Keep debug level for now as root cause occurence is logged */
		LOG(NxpCameraHelper, Debug) << "Invalid gain control";

	return gain;
}

/**
 * \brief Update sensor control list with exposure configuration
 * \param[inout] ctrls The control list to be updated
 * \param[in] exposure The exposure value - unit is horizontal line duration
 *
 * This function aims to abstract the exposure control for sensor having
 * a proprietary programming model.
 */
void CameraHelper::controlListSetExposure(
	ControlList *ctrls, uint32_t exposure) const
{
	if (!controlListHasId(ctrls, V4L2_CID_EXPOSURE)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_EXPOSURE cannot be set";
		return;
	}

	ctrls->set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure));
}

/**
 * \brief Update sensor control list with exposure configuration
 * \param[inout] ctrls The control list to be updated
 * \param[in] gainCode The gain code in sensor format
 *
 * This function aims to abstract the gain control for sensor having
 * a proprietary programming model.
 */
void CameraHelper::controlListSetGain(
	ControlList *ctrls, uint32_t gainCode) const
{
	if (!controlListHasId(ctrls, V4L2_CID_ANALOGUE_GAIN)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_ANALOGUE_GAIN cannot be set";
		return;
	}

	ctrls->set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gainCode));
}

/**
 * \brief Retrieve exposure range from sensor control info map
 * \param[in] ctrls The control list to be updated
 * \param[out] minExposure The minimum exposure time in line duration units
 * \param[out] maxExposure The maximum exposure time in line duration units
 * \param[out] defExposure The default exposure time in line duration units
 *
 * This function aims to abstract the exposure control for sensor having
 * a proprietary programming model.
 */
void CameraHelper::controlInfoMapGetExposureRange(
		const ControlInfoMap *ctrls, uint32_t *minExposure,
		uint32_t *maxExposure, uint32_t *defExposure) const
{
	uint32_t min, max, def;
	const auto it = ctrls->find(V4L2_CID_EXPOSURE);

	if (it != ctrls->end()) {
		min = it->second.min().get<int32_t>();
		max = it->second.max().get<int32_t>();
		def = it->second.def().get<int32_t>();
	} else {
		min = 0;
		max = 0;
		def = 0;
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_EXPOSURE not supported";
	}

	if (minExposure)
		*minExposure = min;
	if (maxExposure)
		*maxExposure = max;
	if (defExposure)
		*defExposure = def;
}

/**
 * \brief Retrieve gain range from sensor control info map
 * \param[in] ctrls The control list to be updated
 * \param[out] minGainCode The minimum gain code in sensor format
 * \param[out] maxGainCode The maximum gain code in sensor format
 * \param[out] defGainCode The default gain code in sensor format
 *
 * This function aims to abstract the gain control for sensor having
 * a proprietary programming model.
 */
void CameraHelper::controlInfoMapGetGainRange(
		const ControlInfoMap *ctrls, uint32_t *minGainCode,
		uint32_t *maxGainCode, uint32_t *defGainCode) const
{
	uint32_t min, max, def;
	const auto it = ctrls->find(V4L2_CID_ANALOGUE_GAIN);

	if (it != ctrls->end()) {
		min = it->second.min().get<int32_t>();
		max = it->second.max().get<int32_t>();
		def = it->second.def().get<int32_t>();
	} else {
		min = 0;
		max = 0;
		def = 0;
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_ANALOGUE_GAIN not supported";
	}

	if (minGainCode)
		*minGainCode = min;
	if (maxGainCode)
		*maxGainCode = max;
	if (defGainCode)
		*defGainCode = def;
}

/**
 * \brief Report the delay for a control to be applied in sensor
 *
 * This function returns a vector of map of (delay, priority) pair definitions
 * for the camera controls handled by 3A.
 * That is typically used to initialise the DelayedControls object instanciated
 * by the pipeline.
 * Delay corresponds to the latency in number of frames for the control value to
 * be applied.
 * Priority indicate that the control must be applied ahead of, and separately
 * from other controls.
 *
 * \return The map of (delay, priority) pairs
 */
std::map<int32_t, std::pair<uint32_t, bool>>
CameraHelper::delayedControlParams() const
{
	/* Common default values used in libcamera code base */
	static const std::map<int32_t, std::pair<uint32_t, bool>> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { 1, false } },
		{ V4L2_CID_EXPOSURE, { 2, false } },
	};

	return params;
}

/**
 * \brief Helper to check if a ControlId is handled by a ControlList
 *
 * This function checks if a ControlList has been constructed with support for
 * a gived ControlId defined by its id.
 * If the ControlId is supported, the ControlList may not have a ControlValue
 * assigned yet, but ControlId will be at least present in the ControlIdMap of
 * the ControlList.
 *
 * \param[in] ctrls The control list
 * \param[in] id The id of the ControlId
 *
 * \return True if the ControlId is handled by the ControlList
 */
bool CameraHelper::controlListHasId(const ControlList *ctrls, unsigned int id) const
{
	auto idMap = ctrls->idMap();
	return (idMap->find(id) != idMap->end());
}


/*-------------------------- Factory definitions --------------------------*/

/**
 * \class CameraHelperFactoryBase
 * \brief Base class for camera sensor helper factories
 *
 * The CameraHelperFactoryBase class is the base of all specializations of
 * the CamerarHelperFactory class template. It implements the factory
 * registration, maintains a registry of factories, and provides access to the
 * registered factories.
 */

/**
 * \brief Construct a camera sensor helper factory base
 * \param[in] name Name of the camera sensor helper class
 *
 * Creating an instance of the factory base registers it with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used to look up factories and shall be unique.
 */
CameraHelperFactoryBase::CameraHelperFactoryBase(const std::string name)
	: name_(name)
{
	registerType(this);
}

/**
 * \brief Create an instance of the CameraHelper corresponding to
 * a named factory
 * \param[in] name Name of the factory
 *
 * \return A unique pointer to a new instance of the CameraHelper subclass
 * corresponding to the named factory or a null pointer if no such factory
 * exists
 */
std::unique_ptr<CameraHelper> CameraHelperFactoryBase::create(const std::string &name)
{
	const std::vector<CameraHelperFactoryBase *> &factories =
		CameraHelperFactoryBase::factories();

	for (const CameraHelperFactoryBase *factory : factories) {
		if (name != factory->name_)
			continue;

		return factory->createInstance();
	}

	return nullptr;
}

/**
 * \brief Add a camera sensor helper class to the registry
 * \param[in] factory Factory to use to construct the camera sensor helper
 *
 * The caller is responsible to guarantee the uniqueness of the camera sensor
 * helper name.
 */
void CameraHelperFactoryBase::registerType(CameraHelperFactoryBase *factory)
{
	std::vector<CameraHelperFactoryBase *> &factories =
		CameraHelperFactoryBase::factories();

	factories.push_back(factory);
}

/**
 * \brief Retrieve the list of all camera sensor helper factories
 * \return The list of camera sensor helper factories
 */
std::vector<CameraHelperFactoryBase *> &CameraHelperFactoryBase::factories()
{
	/*
	 * The static factories map is defined inside the function to ensure
	 * it gets initialized on first use, without any dependency on link
	 * order.
	 */
	static std::vector<CameraHelperFactoryBase *> factories;
	return factories;
}

/**
 * \class CamerarHelperFactory
 * \brief Registration of CamerarHelperFactory classes and creation of instances
 * \tparam _Helper The camera sensor helper class type for this factory
 *
 * To facilitate discovery and instantiation of CameraHelper classes, the
 * CameraSensorHelperFactory class implements auto-registration of camera sensor
 * helpers. Each CameraHelper subclass shall register itself using the
 * REGISTER_CAMERA_HELPER() macro, which will create a corresponding
 * instance of a CameraSensorHelperFactory subclass and register it with the
 * static list of factories.
 */

/**
 * \fn CamerarHelperFactory::CamerarHelperFactory(const char *name)
 * \brief Construct a camera sensor helper factory
 * \param[in] name Name of the camera sensor helper class
 *
 * Creating an instance of the factory registers it with the global list of
 * factories, accessible through the CameraHelperFactoryBase::factories()
 * function.
 *
 * The factory \a name is used to look up factories and shall be unique.
 */

/**
 * \fn CamerarHelperFactory::createInstance() const
 * \brief Create an instance of the CameraSensorHelper corresponding to the
 * factory
 *
 * \return A unique pointer to a newly constructed instance of the
 * CameraHelper subclass corresponding to the factory
 */

/**
 * \def REGISTER_CAMERA_HELPER
 * \brief Register a camera sensor helper with the camera sensor helper factory
 * \param[in] name Sensor model name used to register the class
 * \param[in] helper Class name of CameraHelper derived class to register
 *
 * Register a CameraHelper subclass with the factory and make it available
 * to try and match sensors.
 */

} /* namespace nxp */

} /* namespace libcamera */
