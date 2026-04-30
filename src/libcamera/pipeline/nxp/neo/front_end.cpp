/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2026 NXP
 *
 * Camera Front End support for neo ISP pipeline
 */

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <libcamera/base/utils.h>

#include "front_end.h"

namespace libcamera {

namespace nxpneo {

/**
 * \enum FEStream
 * \brief Front End stream types
 *
 * This enumeration defines the different stream types that can be produced
 * by a camera front end in the neo ISP pipeline.
 *
 * \var FEStream::Image0
 * \brief Primary image stream output
 *
 * \var FEStream::Image1
 * \brief Secondary image stream output
 *
 * \var FEStream::EData
 * \brief Embedded data stream output
 */

/**
 * \class FrontEndCamera
 * \brief Abstract interface for a camera front end device
 *
 * The FrontEndCamera class provides an abstract interface for camera front end
 * devices in the NXP neo ISP pipeline. It encapsulates the camera sensor,
 * video devices, and configuration capabilities of a front end.
 *
 * A front end camera can support multiple output streams (Image0, Image1,
 * EData) and provides methods to configure formats, query capabilities,
 * and access the underlying hardware devices.
 */

/**
 * \fn FrontEndCamera::name()
 * \brief Get the name of the camera
 * \return The unique camera name as a string reference
 */

/**
 * \fn FrontEndCamera::sensor()
 * \brief Get the CameraSensor object associated with this camera
 * \return Pointer to the CameraSensor object
 */

/**
 * \struct FrontEndCamera::Attributes
 * \brief Camera front end attributes and capabilities
 *
 * This structure contains various attributes that describe the
 * capabilities and characteristics of the camera front end.
 *
 * \var FrontEndCamera::Attributes::ispBypass
 * \brief Camera pipeline is operated without ISP
 *
 * \var FrontEndCamera::Attributes::rgbIrCfa
 * \brief RGB-IR color filter array support
 *
 * \var FrontEndCamera::Attributes::controlsDelay
 * \brief Delay to apply camera control relative to the raw frame capture
 */

/**
 * \fn FrontEndCamera::attributes()
 * \brief Get the attributes of this camera front end
 * \return Reference to the Attributes structure
 */

/**
 * \fn FrontEndCamera::validateOrientation()
 * \brief Validate and adjust the requested orientation
 * \param[in] orientation The requested orientation
 * \return The validated orientation that the camera can support
 *
 * This method validates the requested orientation against the camera's
 * capabilities and returns the closest supported orientation, to integrate a
 * possible orientation constraint at front end level.
 */

/**
 * \fn FrontEndCamera::streams()
 * \brief Get the list of available streams
 * \return Vector of FEStream types supported by this camera
 */

/**
 * \fn FrontEndCamera::hasStream()
 * \brief Check if a specific stream is available
 * \param[in] stream The stream type to check
 * \return True if the stream is supported, false otherwise
 */

/**
 * \fn FrontEndCamera::videoDevice()
 * \brief Get the video device for a specific stream
 * \param[in] stream The stream type
 * \return Pointer to the V4L2VideoDevice for the stream, or nullptr if not
 *     available
 */

/**
 * \fn FrontEndCamera::configure()
 * \brief Configure the camera front end
 * \param[inout] subdevFormat The V4L2 subdevice format to configure
 * \param[in] transform The transform to apply
 * \param[out] videoFormats Optional map of stream types to their configured
 *     video formats
 * \param[in] processedVideoFormats Optional map of pre-processed video formats
 * \return 0 on success, negative error code otherwise
 *
 * This method configures the camera front end with the specified format
 * and transform. It updates the subdevFormat with the actual configured
 * format and populates videoFormats with the resulting video device formats
 * for each stream.
 * The videoFormats map is optional. If provided by the caller, it will be
 * populated with the actual video formats configured for each stream.
 * The processedVideoFormats map is optional and used for ISP bypass operation
 * to select the processed video formats selected, chosen among the set of
 * formats supported by the front end.
 */

/**
 * \struct FrontEndCamera::Formats
 * \brief Supported formats and their relationships
 *
 * This structure contains mappings between media bus codes, sizes,
 * and pixel formats that the camera front end supports.
 *
 * \var FrontEndCamera::Formats::mbusCodeSizesMap
 * \brief Media bus code to sizes mapping
 *
 * \var FrontEndCamera::Formats::sizeMbusCodesMap
 * \brief Size to media bus codes mapping
 *
 * \var FrontEndCamera::Formats::mbusCodePixelFormatsMap
 * \brief Media bus code to pixel formats mapping
 *
 * This map reports for each camera mbus code the list of pixel formats that
 * the front end can support according to its CSC capabilities. That is
 * typically used with ISP bypass operation where user can configure the output
 * pixel format from that list.
 */

/**
 * \fn FrontEndCamera::formats()
 * \brief Get the supported formats
 * \return Reference to the Formats structure containing all supported format
 *     combinations
 */

/**
 * \fn FrontEndCamera::neoDevices()
 * \brief Get the list of NeoDevice instances associated with this camera
 * \return Vector of pointers to NeoDevice objects
 */

/**
 * \struct FrontEndHandler::MatchParams
 * \brief Parameters for front end handler matching and initialization
 *
 * This structure contains the parameters needed by a front end handler to match
 * and initialize devices. The validity of those pointers is limited to the
 * duration of the match() call, so the class implementation should not access
 * these pointers beyond that scope.
 *
 * \var FrontEndHandler::MatchParams::pipeline
 * \brief Pointer to the pipeline handler
 *
 * \var FrontEndHandler::MatchParams::enumerator
 * \brief Pointer to the device enumerator
 *
 * \var FrontEndHandler::MatchParams::neoAllocator
 * \brief Pointer to the NeoDevice allocator
 *
 * \var FrontEndHandler::MatchParams::pipelineConfig
 * \brief Pointer to the pipeline configuration
 */

/**
 * \fn FrontEndHandler::match()
 * \brief Match and initialize front end devices
 * \param[in] params The parameters containing pipeline and device information
 * \return True if matching succeeded, false otherwise
 *
 * This method attempts to match and initialize front end cameras using the
 * provided parameters. It should enumerate available devices, and create
 * a FrontEndCamera instance for each valid camera it finds.
 */

/**
 * \fn FrontEndHandler::cameras()
 * \brief Get the list of cameras managed by this handler
 * \return Vector of pointers to FrontEndCamera objects
 */

/**
 * \fn FrontEndHandler::acquireDevice()
 * \brief Acquire exclusive access to a camera device
 * \param[in] name The name of the camera to acquire
 * \return 0 on success, negative error code otherwise
 */

/**
 * \fn FrontEndHandler::releaseDevice()
 * \brief Release exclusive access to a camera device
 * \param[in] name The name of the camera to release
 */

/**
 * \class FrontEndHandlerFactoryBase
 * \brief Base class for front end factories
 *
 * The FrontEndHandlerFactoryBase class is the base of all specializations of
 * the FrontEndHandlerFactory class template. It implements the factory
 * registration, maintains a registry of factories, and provides access to the
 * registered factories.
 */

/**
 * \brief Construct a front end factory base
 * \param[in] name Name of the front end class
 *
 * Creates an instance of the factory base and registers it with the global list
 * of factories, accessible through the factories() function.
 *
 * The factory \a name is used for debug purpose and shall be unique.
 */
FrontEndHandlerFactoryBase::FrontEndHandlerFactoryBase(const char *name)
	: name_(name)
{
	registerType(this);
}

/**
 * \brief Create an instance of the FrontEndHandler corresponding to the factory
 *
 * \return A unique pointer to a new instance of the FrontEndHandler subclass
 * corresponding to the factory
 */
std::unique_ptr<FrontEndHandler> FrontEndHandlerFactoryBase::create() const
{
	return createInstance(name_);
}

/**
 * \fn FrontEndHandlerFactoryBase::name()
 * \brief Retrieve the factory name
 * \return The factory name
 */

/**
 * \brief Add a front end class to the registry
 * \param[in] factory Factory to use to construct the front end
 *
 * The caller is responsible to guarantee the uniqueness of the front end
 * name.
 */
void FrontEndHandlerFactoryBase::registerType(FrontEndHandlerFactoryBase *factory)
{
	std::vector<FrontEndHandlerFactoryBase *> &factories =
		FrontEndHandlerFactoryBase::factories();

	factories.push_back(factory);
}

/**
 * \brief Retrieve the list of all front end factories
 * \return The list of front end factories
 */
std::vector<FrontEndHandlerFactoryBase *> &FrontEndHandlerFactoryBase::factories()
{
	/*
	 * The static factories map is defined inside the function to ensure
	 * it gets initialized on first use, without any dependency on
	 * link order.
	 */
	static std::vector<FrontEndHandlerFactoryBase *> factories;
	return factories;
}

/**
 * \class FrontEndHandlerFactory
 * \brief Registration of FrontEndHandler classes and creation of instances
 *
 * To facilitate discovery and instantiation of FrontEndHandler classes, the
 * FrontEndHandlerFactory class implements auto-registration of front end
 * handlers. Each FrontEndHandler subclass shall register itself using the
 * REGISTER_FRONT_END_HANDLER() macro, which will create a corresponding
 * instance of a FrontEndHandlerFactory and register it with the static list of
 * factories.
 */

/**
 * \fn FrontEndHandlerFactory::FrontEndHandlerFactory()
 * \brief Construct a front end factory
 * \param[in] name The name of the front end class
 *
 * Creates an instance of the factory and registers it with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used for debug purpose and shall be unique.
 */

/**
 * \fn FrontEndHandlerFactory::createInstance() const
 * \brief Create an instance of the FrontEndHandler corresponding to the factory
 * \param[in] name The name of the front end class
 * \return A unique pointer to a newly constructed instance of the
 * FrontEndHandler subclass corresponding to the factory
 */

/**
 * \def REGISTER_FRONT_END_HANDLER
 * \brief Register a front end with the front end factory
 * \param[in] handler Class name of FrontEndHandler derived class to register
 * \param[in] name Name assigned to the front end
 *
 * Register a FrontEndHandler subclass with the factory and make it available to
 * try and match devices.
 */

} /* namespace nxpneo */

} /* namespace libcamera */
