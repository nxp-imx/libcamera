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

#include <cmath>
#include <limits>

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

namespace md {

/*
 * Embedded data controls definition
 * Controls represent the embedded data values that are parsed and reported in a
 * common format to the IPA.
 */

/**
 * \var AnalogueGain
 * \brief Real analogue gain applied by the sensor
 *
 * Values correspond to Long and optionally Short and Very Short captures.
 */
const Control<Span<const float>> AnalogueGain(ANALOGUE_GAIN, "AnalogueGain");

/**
 * \var DigitalGain
 * \brief Real digital gain applied by the sensor
 *
 * Values correspond to Long and optionally Short and Very Short captures.
 */
const Control<Span<const float>> DigitalGain(DIGITAL_GAIN, "DigitalGain");

/**
 * \var Exposure
 * \brief Exposure in seconds applied by the sensor
 *
 * Values correspond to Long and optionally Short and Very Short captures.
 */
const Control<Span<const float>> Exposure(EXPOSURE, "Exposure");

/**
 * \var WhiteBalanceGain
 * \brief Real White Balance gains applied by the sensor
 *
 * Values correspond to Red, GreenR, GreenB and Blue color channels for Long
 * and optionally Short and Very Short captures.
 */
const Control<Span<const float>> WhiteBalanceGain(WB_GAIN, "WhiteBalanceGain");

/**
 * \var Temperature
 * \brief Sensor temperature
 * Temperature format is real degrees Celsius.
 */
const Control<const float> Temperature(TEMPERATURE, "Temperature");

const ControlIdMap controlIdMap{
	{ ANALOGUE_GAIN, &AnalogueGain },
	{ DIGITAL_GAIN, &DigitalGain },
	{ EXPOSURE, &Exposure },
	{ WB_GAIN, &WhiteBalanceGain },
	{ TEMPERATURE, &Temperature },
};

} /* namespace md */

/**
 * \class CameraHelper
 * \brief Base class for computing sensor tuning parameters using
 * sensor-specific constants
 *
 * Instances derived from CameraHelper class are sensor-specific.
 * Each supported sensor will have an associated base class defined.
 */

/**
 * \brief Construct a CameraHelper instance
 *
 * CameraHelper derived class instances shall never be constructed
 * manually but always through the CameraHelperFactoryBase::create()
 * function.
 */

/**
 * \struct Attributes
 * \brief Attributes of the camera sensor
 *
 * Attributes describe all the static attributes related to that sensor.
 */

/**
 * \struct Attributes::MdParams
 * \brief The metadata parameters
 *
 * Metadata parameters describe the optional meta data support of that sensor.
 */

/**
 * \var Attributes::MdParams::topLines
 * \brief The number of top lines
 *
 * Metadata may be prepended as the first lines of the image. This field reports
 * the number of embedded lines, zero if none.
 */

/**
 * \var Attributes::MdParams::controls
 * \brief The ControlIdMap of the controls reported by the metadata parser
 */

/**
 * \var Attributes::delayedControlParams
 * \brief The delayedControls parameters
 *
 * the map of (delay, priority) pair definitions for the camera controls handled
 * by 3A. It is used to initialise the DelayedControls object instanciated
 * by the pipeline. The delay corresponds to the latency in number of frames for
 * the control value to be applied.
 * Priority indicate that the control must be applied ahead of, and separately
 * from other controls.
 *
 */

/**
 * \brief The CameraHelper constructor
 */
CameraHelper::CameraHelper()
	: attributes_{
		  {
			  { V4L2_CID_ANALOGUE_GAIN, { 1, false } },
			  { V4L2_CID_EXPOSURE, { 2, false } },
		  }, /* delayedControlParams */
		  {
			  0, /* topLines */
		  } /* MdParams */,
		  false, /* rgbIr */
	  }
{
}

/**
 * \brief Configure the camera helper for the current sensor mode of operation
 * \param[in] mode The mode to be configured
 */
void CameraHelper::setCameraMode(const CameraMode &mode)
{
	mode_ = mode;
	LOG(NxpCameraHelper, Debug)
		<< " pixel rate: " << mode_.pixelRate
		<< " Line length (min/max) ("
		<< mode.minLineLength << "/" << mode.maxLineLength
		<< ") Frame length (min/max) ("
		<< mode.minFrameLength << "/" << mode.maxFrameLength
		<< ") Line duration " << lineDuration();
}

/**
 * \brief Update sensor control list with AGC configuration
 * \param[inout] ctrls The control list to be updated
 * \param[in] exposure The AGC exposure duration in seconds
 * \param[in] gain The AGC real gain decision
 *
 * This function aims to abstract the AGC control for sensor having
 * a proprietary programming model.
 */
void CameraHelper::controlListSetAGC(
	ControlList *ctrls, double exposure, double gain) const
{
	if (!controlListHasId(ctrls, V4L2_CID_ANALOGUE_GAIN)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_ANALOGUE_GAIN cannot be set";
		return;
	}

	ctrls->set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gainCode(gain)));

	int32_t lines =
		static_cast<int32_t>(std::round(exposure / lineDuration()));

	if (!controlListHasId(ctrls, V4L2_CID_EXPOSURE)) {
		LOG(NxpCameraHelper, Error)
			<< "V4L2_CID_EXPOSURE cannot be set";
		return;
	}

	ctrls->set(V4L2_CID_EXPOSURE, lines);
}

/**
 * \brief Retrieve exposure range from sensor control info map
 * \param[in] ctrls The control list used for control ranges
 * \param[out] minExposure The minimum exposure time in seconds
 * \param[out] maxExposure The maximum exposure time in seconds
 * \param[out] defExposure The default exposure time in seconds
 *
 * Report min, max and default values for exposure. At least one value is
 * reported in each vector corresponding to the main (long) exposure.
 * Implementation may append additional values if it supports multiple captures
 * (short and/or very short).
 */
void CameraHelper::controlInfoMapGetExposureRange(
	const ControlInfoMap *ctrls, std::vector<double> *minExposure,
	std::vector<double> *maxExposure, std::vector<double> *defExposure) const
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

	double line = lineDuration();
	minExposure->clear();
	minExposure->push_back(min * line);

	maxExposure->clear();
	maxExposure->push_back(max * line);

	defExposure->clear();
	defExposure->push_back(def * line);
}

/**
 * \brief Retrieve analog gain range from sensor control info map
 * \param[in] ctrls The control list map used for control ranges
 * \param[out] minGain The minimum analog gain
 * \param[out] maxGain The maximum analog gain
 * \param[out] defGain The default analog gain
 *
 * Report min, max and default values for analog gain. At least one value is
 * reported in each vector corresponding to the main (long) analog gain.
 * Implementation may append additional values if it supports multiple captures
 * (short and/or very short).
 */
void CameraHelper::controlInfoMapGetAnalogGainRange(
	const ControlInfoMap *ctrls, std::vector<double> *minGain,
	std::vector<double> *maxGain, std::vector<double> *defGain) const
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

	minGain->clear();
	minGain->push_back(gain(min));

	maxGain->clear();
	maxGain->push_back(gain(max));

	defGain->clear();
	defGain->push_back(gain(def));
}

/**
 * \brief Configure the sensor with white balance gain
 *
 * This is optional method that is used when following conditions are met:
 * - IPA has the option to control white balance gains in sensors rather than
 *   in the ISP, and that option is enabled.
 * - Sensor driver actually provides a control for white balance gain
 *   configuration.
 * In case white balance gains are controlled in the ISP, this method does
 * nothing.
.*
 * \param[inout] ctrls The control list to be updated
 * \param[in] gains The gains for the color channels in this order: R, Gr, Gb, B
 */
void CameraHelper::controlListSetAWB(
	ControlList *ctrls, Span<const double, 4> gains) const
{
	/* Nothing to do, not supported by default */
	(void)gains;
	(void)ctrls;
}

/**
 * \brief Parse the embedded data buffer to extract metadata
 * \param[in] buffer The buffer with the embedded data
 * \param[out] mdControls The control list where parsed values will be stored
 *
 * \return 0 in case of success (embedded data present and valid)
 */
int CameraHelper::parseEmbedded([[maybe_unused]] Span<const uint8_t> buffer,
				[[maybe_unused]] ControlList *mdControls)
{
	/* No metadata provided by the sensor */
	return -1;
}

/**
 * \brief Convert a sensor control list to its associated metadata control list
 *
 * The purpose of this function is to build a meta data control list matching a
 * sensor control list. It may be used when the sensor does not actually
 * report embedded data, in order to represent the sensor state in the meta
 * data format.
 *
 * \param[in] sensorCtrls The sensor control list
 * \param[out] mdCtrls The metadata control list
 *
 * \return 0 in case of success, embedded data present and decoded
 */
int CameraHelper::sensorControlsToMetaData(const ControlList *sensorCtrls,
					   ControlList *mdCtrls) const
{
	int ret = 0;

	/* Analog gain, consider single capture */
	ASSERT(controlListHasId(mdCtrls, md::AnalogueGain.id()));
	const ControlValue &aGainCtrl = sensorCtrls->get(V4L2_CID_ANALOGUE_GAIN);
	std::array<float, 1> aGainsArray = { 1.0f };
	if (!aGainCtrl.isNone()) {
		uint32_t aGainCode = aGainCtrl.get<int32_t>();
		aGainsArray[0] = gain(aGainCode);
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid analogue gain control";
		ret = -1;
	}
	mdCtrls->set(md::AnalogueGain, Span<float>(aGainsArray));

	/* Unitary gain for digital gain */
	ASSERT(controlListHasId(mdCtrls, md::DigitalGain.id()));
	std::array<float, 1> dGainsArray = { 1.0f };
	mdCtrls->set(md::DigitalGain, Span<float>(dGainsArray));

	/* Exposure, consider single capture */
	ASSERT(controlListHasId(mdCtrls, md::Exposure.id()));
	const ControlValue &exposureCtrl = sensorCtrls->get(V4L2_CID_EXPOSURE);
	std::array<float, 1> exposuresArray = { 0.0f };
	if (!exposureCtrl.isNone()) {
		int32_t exposureLines = exposureCtrl.get<int32_t>();
		exposuresArray[0] = static_cast<float>(exposureLines * lineDuration());
	} else {
		LOG(NxpCameraHelper, Warning) << "Invalid exposure control";
		ret = -1;
	}
	mdCtrls->set(md::Exposure, Span<float>(exposuresArray));

	/* Unitary gains for white balance */
	ASSERT(controlListHasId(mdCtrls, md::WhiteBalanceGain.id()));
	std::array<float, 4> wbGains = { 1.0f, 1.0f, 1.0f, 1.0f };
	mdCtrls->set(md::WhiteBalanceGain, Span<float>(wbGains));

	/* Arbitrary temperature value */
	ASSERT(controlListHasId(mdCtrls, md::Temperature.id()));
	mdCtrls->set(md::Temperature, 25.0);

	return ret;
}

/**
 * \brief Report the line duration in seconds
 *
 * Line duration is computed by dividing the line length in pixels by the pixel
 * rate. By default, the line length is configured to its minimum value, so use
 * that value.
 * \todo make this computation dynamic according to the actual line length.
 *
 * \return The duration in seconds
 */
double CameraHelper::lineDuration() const
{
	return static_cast<double>(mode_.minLineLength) / mode_.pixelRate;
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
bool CameraHelper::controlListHasId(const ControlList *ctrls, unsigned int id)
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
