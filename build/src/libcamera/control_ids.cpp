/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_ids.cpp : Control ID list
 *
 * This file is auto-generated. Do not edit.
 */

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

/**
 * \file control_ids.h
 * \brief Camera control identifiers
 */

namespace libcamera {

/**
 * \brief Namespace for libcamera controls
 */
namespace controls {

/**
 * \var AeEnable
 * \brief Enable or disable the AE.
 *
 * \sa ExposureTime AnalogueGain
 */

/**
 * \var AeLocked
 * \brief Report the lock status of a running AE algorithm.
 *
 * If the AE algorithm is locked the value shall be set to true, if it's
 * converging it shall be set to false. If the AE algorithm is not
 * running the control shall not be present in the metadata control list.
 *
 * \sa AeEnable
 */

/**
 * \enum AeMeteringModeEnum
 * \brief Supported AeMeteringMode values
 *
 * \var MeteringCentreWeighted
 * \brief Centre-weighted metering mode.
 *
 * \var MeteringSpot
 * \brief Spot metering mode.
 *
 * \var MeteringMatrix
 * \brief Matrix metering mode.
 *
 * \var MeteringCustom
 * \brief Custom metering mode.
 */

/**
 * \var AeMeteringModeValues
 * \brief List of all AeMeteringMode supported values
 */

/**
 * \var AeMeteringMode
 * \brief Specify a metering mode for the AE algorithm to use. The metering
 * modes determine which parts of the image are used to determine the
 * scene brightness. Metering modes may be platform specific and not
 * all metering modes may be supported.
 */

/**
 * \enum AeConstraintModeEnum
 * \brief Supported AeConstraintMode values
 *
 * \var ConstraintNormal
 * \brief Default constraint mode. This mode aims to balance the exposure of different parts of the image so as to reach a reasonable average level. However, highlights in the image may appear over-exposed and lowlights may appear under-exposed.
 *
 * \var ConstraintHighlight
 * \brief Highlight constraint mode. This mode adjusts the exposure levels in order to try and avoid over-exposing the brightest parts (highlights) of an image. Other non-highlight parts of the image may appear under-exposed.
 *
 * \var ConstraintShadows
 * \brief Shadows constraint mode. This mode adjusts the exposure levels in order to try and avoid under-exposing the dark parts (shadows) of an image. Other normally exposed parts of the image may appear over-exposed.
 *
 * \var ConstraintCustom
 * \brief Custom constraint mode.
 */

/**
 * \var AeConstraintModeValues
 * \brief List of all AeConstraintMode supported values
 */

/**
 * \var AeConstraintMode
 * \brief Specify a constraint mode for the AE algorithm to use. These determine
 * how the measured scene brightness is adjusted to reach the desired
 * target exposure. Constraint modes may be platform specific, and not
 * all constraint modes may be supported.
 */

/**
 * \enum AeExposureModeEnum
 * \brief Supported AeExposureMode values
 *
 * \var ExposureNormal
 * \brief Default exposure mode.
 *
 * \var ExposureShort
 * \brief Exposure mode allowing only short exposure times.
 *
 * \var ExposureLong
 * \brief Exposure mode allowing long exposure times.
 *
 * \var ExposureCustom
 * \brief Custom exposure mode.
 */

/**
 * \var AeExposureModeValues
 * \brief List of all AeExposureMode supported values
 */

/**
 * \var AeExposureMode
 * \brief Specify an exposure mode for the AE algorithm to use. These specify
 * how the desired total exposure is divided between the shutter time
 * and the sensor's analogue gain. The exposure modes are platform
 * specific, and not all exposure modes may be supported.
 */

/**
 * \var ExposureValue
 * \brief Specify an Exposure Value (EV) parameter. The EV parameter will only be
 * applied if the AE algorithm is currently enabled.
 *
 * By convention EV adjusts the exposure as log2. For example
 * EV = [-2, -1, 0.5, 0, 0.5, 1, 2] results in an exposure adjustment
 * of [1/4x, 1/2x, 1/sqrt(2)x, 1x, sqrt(2)x, 2x, 4x].
 *
 * \sa AeEnable
 */

/**
 * \var ExposureTime
 * \brief Exposure time (shutter speed) for the frame applied in the sensor
 * device. This value is specified in micro-seconds.
 *
 * Setting this value means that it is now fixed and the AE algorithm may
 * not change it. Setting it back to zero returns it to the control of the
 * AE algorithm.
 *
 * \sa AnalogueGain AeEnable
 *
 * \todo Document the interactions between AeEnable and setting a fixed
 * value for this control. Consider interactions with other AE features,
 * such as aperture and aperture/shutter priority mode, and decide if
 * control of which features should be automatically adjusted shouldn't
 * better be handled through a separate AE mode control.
 */

/**
 * \var AnalogueGain
 * \brief Analogue gain value applied in the sensor device.
 * The value of the control specifies the gain multiplier applied to all
 * colour channels. This value cannot be lower than 1.0.
 *
 * Setting this value means that it is now fixed and the AE algorithm may
 * not change it. Setting it back to zero returns it to the control of the
 * AE algorithm.
 *
 * \sa ExposureTime AeEnable
 *
 * \todo Document the interactions between AeEnable and setting a fixed
 * value for this control. Consider interactions with other AE features,
 * such as aperture and aperture/shutter priority mode, and decide if
 * control of which features should be automatically adjusted shouldn't
 * better be handled through a separate AE mode control.
 */

/**
 * \enum AeFlickerModeEnum
 * \brief Supported AeFlickerMode values
 *
 * \var FlickerOff
 * \brief No flicker avoidance is performed.
 *
 * \var FlickerManual
 * \brief Manual flicker avoidance. Suppress flicker effects caused by lighting running with a period specified by the AeFlickerPeriod control. \sa AeFlickerPeriod
 *
 * \var FlickerAuto
 * \brief Automatic flicker period detection and avoidance. The system will automatically determine the most likely value of flicker period, and avoid flicker of this frequency. Once flicker is being corrected, it is implementation dependent whether the system is still able to detect a change in the flicker period. \sa AeFlickerDetected
 */

/**
 * \var AeFlickerModeValues
 * \brief List of all AeFlickerMode supported values
 */

/**
 * \var AeFlickerMode
 * \brief Set the flicker mode, which determines whether, and how, the AGC/AEC
 * algorithm attempts to hide flicker effects caused by the duty cycle of
 * artificial lighting.
 *
 * Although implementation dependent, many algorithms for "flicker
 * avoidance" work by restricting this exposure time to integer multiples
 * of the cycle period, wherever possible.
 *
 * Implementations may not support all of the flicker modes listed below.
 *
 * By default the system will start in FlickerAuto mode if this is
 * supported, otherwise the flicker mode will be set to FlickerOff.
 */

/**
 * \var AeFlickerPeriod
 * \brief Manual flicker period in microseconds. This value sets the current flicker period to avoid. It is used when AeFlickerMode is set to FlickerManual.
 * To cancel 50Hz mains flicker, this should be set to 10000 (corresponding to 100Hz), or 8333 (120Hz) for 60Hz mains.
 * Setting the mode to FlickerManual when no AeFlickerPeriod has ever been set means that no flicker cancellation occurs (until the value of this control is updated).
 * Switching to modes other than FlickerManual has no effect on the value of the AeFlickerPeriod control.
 * \sa AeFlickerMode
 */

/**
 * \var AeFlickerDetected
 * \brief Flicker period detected in microseconds. The value reported here indicates the currently detected flicker period, or zero if no flicker at all is detected.
 * When AeFlickerMode is set to FlickerAuto, there may be a period during which the value reported here remains zero. Once a non-zero value is reported, then this is the flicker period that has been detected and is now being cancelled.
 * In the case of 50Hz mains flicker, the value would be 10000 (corresponding to 100Hz), or 8333 (120Hz) for 60Hz mains flicker.
 * It is implementation dependent whether the system can continue to detect flicker of different periods when another frequency is already being cancelled.
 * \sa AeFlickerMode
 */

/**
 * \var Brightness
 * \brief Specify a fixed brightness parameter. Positive values (up to 1.0)
 * produce brighter images; negative values (up to -1.0) produce darker
 * images and 0.0 leaves pixels unchanged.
 */

/**
 * \var Contrast
 * \brief Specify a fixed contrast parameter. Normal contrast is given by the
 * value 1.0; larger values produce images with more contrast.
 */

/**
 * \var Lux
 * \brief Report an estimate of the current illuminance level in lux. The Lux
 * control can only be returned in metadata.
 */

/**
 * \var AwbEnable
 * \brief Enable or disable the AWB.
 *
 * \sa ColourGains
 */

/**
 * \enum AwbModeEnum
 * \brief Supported AwbMode values
 *
 * \var AwbAuto
 * \brief Search over the whole colour temperature range.
 *
 * \var AwbIncandescent
 * \brief Incandescent AWB lamp mode.
 *
 * \var AwbTungsten
 * \brief Tungsten AWB lamp mode.
 *
 * \var AwbFluorescent
 * \brief Fluorescent AWB lamp mode.
 *
 * \var AwbIndoor
 * \brief Indoor AWB lighting mode.
 *
 * \var AwbDaylight
 * \brief Daylight AWB lighting mode.
 *
 * \var AwbCloudy
 * \brief Cloudy AWB lighting mode.
 *
 * \var AwbCustom
 * \brief Custom AWB mode.
 */

/**
 * \var AwbModeValues
 * \brief List of all AwbMode supported values
 */

/**
 * \var AwbMode
 * \brief Specify the range of illuminants to use for the AWB algorithm. The modes
 * supported are platform specific, and not all modes may be supported.
 */

/**
 * \var AwbLocked
 * \brief Report the lock status of a running AWB algorithm.
 *
 * If the AWB algorithm is locked the value shall be set to true, if it's
 * converging it shall be set to false. If the AWB algorithm is not
 * running the control shall not be present in the metadata control list.
 *
 * \sa AwbEnable
 */

/**
 * \var ColourGains
 * \brief Pair of gain values for the Red and Blue colour channels, in that
 * order. ColourGains can only be applied in a Request when the AWB is
 * disabled.
 *
 * \sa AwbEnable
 */

/**
 * \var ColourTemperature
 * \brief Report the current estimate of the colour temperature, in kelvin, for this frame. The ColourTemperature control can only be returned in metadata.
 */

/**
 * \var Saturation
 * \brief Specify a fixed saturation parameter. Normal saturation is given by
 * the value 1.0; larger values produce more saturated colours; 0.0
 * produces a greyscale image.
 */

/**
 * \var SensorBlackLevels
 * \brief Reports the sensor black levels used for processing a frame, in the
 * order R, Gr, Gb, B. These values are returned as numbers out of a 16-bit
 * pixel range (as if pixels ranged from 0 to 65535). The SensorBlackLevels
 * control can only be returned in metadata.
 */

/**
 * \var Sharpness
 * \brief A value of 0.0 means no sharpening. The minimum value means
 * minimal sharpening, and shall be 0.0 unless the camera can't
 * disable sharpening completely. The default value shall give a
 * "reasonable" level of sharpening, suitable for most use cases.
 * The maximum value may apply extremely high levels of sharpening,
 * higher than anyone could reasonably want. Negative values are
 * not allowed. Note also that sharpening is not applied to raw
 * streams.
 */

/**
 * \var FocusFoM
 * \brief Reports a Figure of Merit (FoM) to indicate how in-focus the frame is.
 * A larger FocusFoM value indicates a more in-focus frame. This singular
 * value may be based on a combination of statistics gathered from
 * multiple focus regions within an image. The number of focus regions and
 * method of combination is platform dependent. In this respect, it is not
 * necessarily aimed at providing a way to implement a focus algorithm by
 * the application, rather an indication of how in-focus a frame is.
 */

/**
 * \var ColourCorrectionMatrix
 * \brief The 3x3 matrix that converts camera RGB to sRGB within the
 * imaging pipeline. This should describe the matrix that is used
 * after pixels have been white-balanced, but before any gamma
 * transformation. The 3x3 matrix is stored in conventional reading
 * order in an array of 9 floating point values.
 */

/**
 * \var ScalerCrop
 * \brief Sets the image portion that will be scaled to form the whole of
 * the final output image. The (x,y) location of this rectangle is
 * relative to the PixelArrayActiveAreas that is being used. The units
 * remain native sensor pixels, even if the sensor is being used in
 * a binning or skipping mode.
 *
 * This control is only present when the pipeline supports scaling. Its
 * maximum valid value is given by the properties::ScalerCropMaximum
 * property, and the two can be used to implement digital zoom.
 */

/**
 * \var DigitalGain
 * \brief Digital gain value applied during the processing steps applied
 * to the image as captured from the sensor.
 *
 * The global digital gain factor is applied to all the colour channels
 * of the RAW image. Different pipeline models are free to
 * specify how the global gain factor applies to each separate
 * channel.
 *
 * If an imaging pipeline applies digital gain in distinct
 * processing steps, this value indicates their total sum.
 * Pipelines are free to decide how to adjust each processing
 * step to respect the received gain factor and shall report
 * their total value in the request metadata.
 */

/**
 * \var FrameDuration
 * \brief The instantaneous frame duration from start of frame exposure to start
 * of next exposure, expressed in microseconds. This control is meant to
 * be returned in metadata.
 */

/**
 * \var FrameDurationLimits
 * \brief The minimum and maximum (in that order) frame duration, expressed in
 * microseconds.
 *
 * When provided by applications, the control specifies the sensor frame
 * duration interval the pipeline has to use. This limits the largest
 * exposure time the sensor can use. For example, if a maximum frame
 * duration of 33ms is requested (corresponding to 30 frames per second),
 * the sensor will not be able to raise the exposure time above 33ms.
 * A fixed frame duration is achieved by setting the minimum and maximum
 * values to be the same. Setting both values to 0 reverts to using the
 * camera defaults.
 *
 * The maximum frame duration provides the absolute limit to the shutter
 * speed computed by the AE algorithm and it overrides any exposure mode
 * setting specified with controls::AeExposureMode. Similarly, when a
 * manual exposure time is set through controls::ExposureTime, it also
 * gets clipped to the limits set by this control. When reported in
 * metadata, the control expresses the minimum and maximum frame
 * durations used after being clipped to the sensor provided frame
 * duration limits.
 *
 * \sa AeExposureMode
 * \sa ExposureTime
 *
 * \todo Define how to calculate the capture frame rate by
 * defining controls to report additional delays introduced by
 * the capture pipeline or post-processing stages (ie JPEG
 * conversion, frame scaling).
 *
 * \todo Provide an explicit definition of default control values, for
 * this and all other controls.
 */

/**
 * \var SensorTemperature
 * \brief Temperature measure from the camera sensor in Celsius. This is typically
 * obtained by a thermal sensor present on-die or in the camera module. The
 * range of reported temperatures is device dependent.
 *
 * The SensorTemperature control will only be returned in metadata if a
 * themal sensor is present.
 */

/**
 * \var SensorTimestamp
 * \brief The time when the first row of the image sensor active array is exposed.
 *
 * The timestamp, expressed in nanoseconds, represents a monotonically
 * increasing counter since the system boot time, as defined by the
 * Linux-specific CLOCK_BOOTTIME clock id.
 *
 * The SensorTimestamp control can only be returned in metadata.
 *
 * \todo Define how the sensor timestamp has to be used in the reprocessing
 * use case.
 */

/**
 * \enum AfModeEnum
 * \brief Supported AfMode values
 *
 * \var AfModeManual
 * \brief The AF algorithm is in manual mode. In this mode it will never
 * perform any action nor move the lens of its own accord, but an
 * application can specify the desired lens position using the
 * LensPosition control.
 *
 * In this mode the AfState will always report AfStateIdle.
 *
 * If the camera is started in AfModeManual, it will move the focus
 * lens to the position specified by the LensPosition control.
 *
 * This mode is the recommended default value for the AfMode control.
 * External cameras (as reported by the Location property set to
 * CameraLocationExternal) may use a different default value.
 *
 * \var AfModeAuto
 * \brief The AF algorithm is in auto mode. This means that the algorithm
 * will never move the lens or change state unless the AfTrigger
 * control is used. The AfTrigger control can be used to initiate a
 * focus scan, the results of which will be reported by AfState.
 *
 * If the autofocus algorithm is moved from AfModeAuto to another
 * mode while a scan is in progress, the scan is cancelled
 * immediately, without waiting for the scan to finish.
 *
 * When first entering this mode the AfState will report
 * AfStateIdle. When a trigger control is sent, AfState will
 * report AfStateScanning for a period before spontaneously
 * changing to AfStateFocused or AfStateFailed, depending on
 * the outcome of the scan. It will remain in this state until
 * another scan is initiated by the AfTrigger control. If a scan is
 * cancelled (without changing to another mode), AfState will return
 * to AfStateIdle.
 *
 * \var AfModeContinuous
 * \brief The AF algorithm is in continuous mode. This means that the lens can
 * re-start a scan spontaneously at any moment, without any user
 * intervention. The AfState still reports whether the algorithm is
 * currently scanning or not, though the application has no ability to
 * initiate or cancel scans, nor to move the lens for itself.
 *
 * However, applications can pause the AF algorithm from continuously
 * scanning by using the AfPause control. This allows video or still
 * images to be captured whilst guaranteeing that the focus is fixed.
 *
 * When set to AfModeContinuous, the system will immediately initiate a
 * scan so AfState will report AfStateScanning, and will settle on one
 * of AfStateFocused or AfStateFailed, depending on the scan result.
 */

/**
 * \var AfModeValues
 * \brief List of all AfMode supported values
 */

/**
 * \var AfMode
 * \brief Control to set the mode of the AF (autofocus) algorithm.
 *
 * An implementation may choose not to implement all the modes.
 */

/**
 * \enum AfRangeEnum
 * \brief Supported AfRange values
 *
 * \var AfRangeNormal
 * \brief A wide range of focus distances is scanned, all the way from
 * infinity down to close distances, though depending on the
 * implementation, possibly not including the very closest macro
 * positions.
 *
 * \var AfRangeMacro
 * \brief Only close distances are scanned.
 *
 * \var AfRangeFull
 * \brief The full range of focus distances is scanned just as with
 * AfRangeNormal but this time including the very closest macro
 * positions.
 */

/**
 * \var AfRangeValues
 * \brief List of all AfRange supported values
 */

/**
 * \var AfRange
 * \brief Control to set the range of focus distances that is scanned. An
 * implementation may choose not to implement all the options here.
 */

/**
 * \enum AfSpeedEnum
 * \brief Supported AfSpeed values
 *
 * \var AfSpeedNormal
 * \brief Move the lens at its usual speed.
 *
 * \var AfSpeedFast
 * \brief Move the lens more quickly.
 */

/**
 * \var AfSpeedValues
 * \brief List of all AfSpeed supported values
 */

/**
 * \var AfSpeed
 * \brief Control that determines whether the AF algorithm is to move the lens
 * as quickly as possible or more steadily. For example, during video
 * recording it may be desirable not to move the lens too abruptly, but
 * when in a preview mode (waiting for a still capture) it may be
 * helpful to move the lens as quickly as is reasonably possible.
 */

/**
 * \enum AfMeteringEnum
 * \brief Supported AfMetering values
 *
 * \var AfMeteringAuto
 * \brief The AF algorithm should decide for itself where it will measure focus.
 *
 * \var AfMeteringWindows
 * \brief The AF algorithm should use the rectangles defined by the AfWindows control to measure focus. If no windows are specified the behaviour is platform dependent.
 */

/**
 * \var AfMeteringValues
 * \brief List of all AfMetering supported values
 */

/**
 * \var AfMetering
 * \brief Instruct the AF algorithm how it should decide which parts of the image
 * should be used to measure focus.
 */

/**
 * \var AfWindows
 * \brief Sets the focus windows used by the AF algorithm when AfMetering is set
 * to AfMeteringWindows. The units used are pixels within the rectangle
 * returned by the ScalerCropMaximum property.
 *
 * In order to be activated, a rectangle must be programmed with non-zero
 * width and height. Internally, these rectangles are intersected with the
 * ScalerCropMaximum rectangle. If the window becomes empty after this
 * operation, then the window is ignored. If all the windows end up being
 * ignored, then the behaviour is platform dependent.
 *
 * On platforms that support the ScalerCrop control (for implementing
 * digital zoom, for example), no automatic recalculation or adjustment of
 * AF windows is performed internally if the ScalerCrop is changed. If any
 * window lies outside the output image after the scaler crop has been
 * applied, it is up to the application to recalculate them.
 *
 * The details of how the windows are used are platform dependent. We note
 * that when there is more than one AF window, a typical implementation
 * might find the optimal focus position for each one and finally select
 * the window where the focal distance for the objects shown in that part
 * of the image are closest to the camera.
 */

/**
 * \enum AfTriggerEnum
 * \brief Supported AfTrigger values
 *
 * \var AfTriggerStart
 * \brief Start an AF scan. Ignored if a scan is in progress.
 *
 * \var AfTriggerCancel
 * \brief Cancel an AF scan. This does not cause the lens to move anywhere else. Ignored if no scan is in progress.
 */

/**
 * \var AfTriggerValues
 * \brief List of all AfTrigger supported values
 */

/**
 * \var AfTrigger
 * \brief This control starts an autofocus scan when AfMode is set to AfModeAuto,
 * and can also be used to terminate a scan early.
 *
 * It is ignored if AfMode is set to AfModeManual or AfModeContinuous.
 */

/**
 * \enum AfPauseEnum
 * \brief Supported AfPause values
 *
 * \var AfPauseImmediate
 * \brief Pause the continuous autofocus algorithm immediately, whether or not
 * any kind of scan is underway. AfPauseState will subsequently report
 * AfPauseStatePaused. AfState may report any of AfStateScanning,
 * AfStateFocused or AfStateFailed, depending on the algorithm's state
 * when it received this control.
 *
 * \var AfPauseDeferred
 * \brief This is similar to AfPauseImmediate, and if the AfState is currently
 * reporting AfStateFocused or AfStateFailed it will remain in that
 * state and AfPauseState will report AfPauseStatePaused.
 *
 * However, if the algorithm is scanning (AfStateScanning),
 * AfPauseState will report AfPauseStatePausing until the scan is
 * finished, at which point AfState will report one of AfStateFocused
 * or AfStateFailed, and AfPauseState will change to
 * AfPauseStatePaused.
 *
 * \var AfPauseResume
 * \brief Resume continuous autofocus operation. The algorithm starts again
 * from exactly where it left off, and AfPauseState will report
 * AfPauseStateRunning.
 */

/**
 * \var AfPauseValues
 * \brief List of all AfPause supported values
 */

/**
 * \var AfPause
 * \brief This control has no effect except when in continuous autofocus mode
 * (AfModeContinuous). It can be used to pause any lens movements while
 * (for example) images are captured. The algorithm remains inactive
 * until it is instructed to resume.
 */

/**
 * \var LensPosition
 * \brief Acts as a control to instruct the lens to move to a particular position
 * and also reports back the position of the lens for each frame.
 *
 * The LensPosition control is ignored unless the AfMode is set to
 * AfModeManual, though the value is reported back unconditionally in all
 * modes.
 *
 * This value, which is generally a non-integer, is the reciprocal of the
 * focal distance in metres, also known as dioptres. That is, to set a
 * focal distance D, the lens position LP is given by
 *
 * \f$LP = \frac{1\mathrm{m}}{D}\f$
 *
 * For example:
 *
 * 0 moves the lens to infinity.
 * 0.5 moves the lens to focus on objects 2m away.
 * 2 moves the lens to focus on objects 50cm away.
 * And larger values will focus the lens closer.
 *
 * The default value of the control should indicate a good general position
 * for the lens, often corresponding to the hyperfocal distance (the
 * closest position for which objects at infinity are still acceptably
 * sharp). The minimum will often be zero (meaning infinity), and the
 * maximum value defines the closest focus position.
 *
 * \todo Define a property to report the Hyperfocal distance of calibrated
 * lenses.
 */

/**
 * \enum AfStateEnum
 * \brief Supported AfState values
 *
 * \var AfStateIdle
 * \brief The AF algorithm is in manual mode (AfModeManual) or in auto mode
 * (AfModeAuto) and a scan has not yet been triggered, or an
 * in-progress scan was cancelled.
 *
 * \var AfStateScanning
 * \brief The AF algorithm is in auto mode (AfModeAuto), and a scan has been
 * started using the AfTrigger control. The scan can be cancelled by
 * sending AfTriggerCancel at which point the algorithm will either
 * move back to AfStateIdle or, if the scan actually completes before
 * the cancel request is processed, to one of AfStateFocused or
 * AfStateFailed.
 *
 * Alternatively the AF algorithm could be in continuous mode
 * (AfModeContinuous) at which point it may enter this state
 * spontaneously whenever it determines that a rescan is needed.
 *
 * \var AfStateFocused
 * \brief The AF algorithm is in auto (AfModeAuto) or continuous
 * (AfModeContinuous) mode and a scan has completed with the result
 * that the algorithm believes the image is now in focus.
 *
 * \var AfStateFailed
 * \brief The AF algorithm is in auto (AfModeAuto) or continuous
 * (AfModeContinuous) mode and a scan has completed with the result
 * that the algorithm did not find a good focus position.
 */

/**
 * \var AfStateValues
 * \brief List of all AfState supported values
 */

/**
 * \var AfState
 * \brief Reports the current state of the AF algorithm in conjunction with the
 * reported AfMode value and (in continuous AF mode) the AfPauseState
 * value. The possible state changes are described below, though we note
 * the following state transitions that occur when the AfMode is changed.
 *
 * If the AfMode is set to AfModeManual, then the AfState will always
 * report AfStateIdle (even if the lens is subsequently moved). Changing to
 * the AfModeManual state does not initiate any lens movement.
 *
 * If the AfMode is set to AfModeAuto then the AfState will report
 * AfStateIdle. However, if AfModeAuto and AfTriggerStart are sent together
 * then AfState will omit AfStateIdle and move straight to AfStateScanning
 * (and start a scan).
 *
 * If the AfMode is set to AfModeContinuous then the AfState will initially
 * report AfStateScanning.
 */

/**
 * \enum AfPauseStateEnum
 * \brief Supported AfPauseState values
 *
 * \var AfPauseStateRunning
 * \brief Continuous AF is running and the algorithm may restart a scan
 * spontaneously.
 *
 * \var AfPauseStatePausing
 * \brief Continuous AF has been sent an AfPauseDeferred control, and will
 * pause as soon as any in-progress scan completes (and then report
 * AfPauseStatePaused). No new scans will be start spontaneously until
 * the AfPauseResume control is sent.
 *
 * \var AfPauseStatePaused
 * \brief Continuous AF is paused. No further state changes or lens movements
 * will occur until the AfPauseResume control is sent.
 */

/**
 * \var AfPauseStateValues
 * \brief List of all AfPauseState supported values
 */

/**
 * \var AfPauseState
 * \brief Only applicable in continuous (AfModeContinuous) mode, this reports
 * whether the algorithm is currently running, paused or pausing (that is,
 * will pause as soon as any in-progress scan completes).
 *
 * Any change to AfMode will cause AfPauseStateRunning to be reported.
 */

/**
 * \brief Namespace for libcamera draft controls
 */
namespace draft {

/**
 * \enum AePrecaptureTriggerEnum
 * \brief Supported AePrecaptureTrigger values
 *
 * \var AePrecaptureTriggerIdle
 * \brief The trigger is idle.
 *
 * \var AePrecaptureTriggerStart
 * \brief The pre-capture AE metering is started by the camera.
 *
 * \var AePrecaptureTriggerCancel
 * \brief The camera will cancel any active or completed metering sequence.
 * The AE algorithm is reset to its initial state.
 */

/**
 * \var AePrecaptureTriggerValues
 * \brief List of all AePrecaptureTrigger supported values
 */

/**
 * \var AePrecaptureTrigger
 * \brief Control for AE metering trigger. Currently identical to
 * ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER.
 *
 * Whether the camera device will trigger a precapture metering sequence
 * when it processes this request.
 */

/**
 * \enum NoiseReductionModeEnum
 * \brief Supported NoiseReductionMode values
 *
 * \var NoiseReductionModeOff
 * \brief No noise reduction is applied
 *
 * \var NoiseReductionModeFast
 * \brief Noise reduction is applied without reducing the frame rate.
 *
 * \var NoiseReductionModeHighQuality
 * \brief High quality noise reduction at the expense of frame rate.
 *
 * \var NoiseReductionModeMinimal
 * \brief Minimal noise reduction is applied without reducing the frame rate.
 *
 * \var NoiseReductionModeZSL
 * \brief Noise reduction is applied at different levels to different streams.
 */

/**
 * \var NoiseReductionModeValues
 * \brief List of all NoiseReductionMode supported values
 */

/**
 * \var NoiseReductionMode
 * \brief Control to select the noise reduction algorithm mode. Currently
 * identical to ANDROID_NOISE_REDUCTION_MODE.
 *
 *  Mode of operation for the noise reduction algorithm.
 */

/**
 * \enum ColorCorrectionAberrationModeEnum
 * \brief Supported ColorCorrectionAberrationMode values
 *
 * \var ColorCorrectionAberrationOff
 * \brief No aberration correction is applied.
 *
 * \var ColorCorrectionAberrationFast
 * \brief Aberration correction will not slow down the frame rate.
 *
 * \var ColorCorrectionAberrationHighQuality
 * \brief High quality aberration correction which might reduce the frame
 * rate.
 */

/**
 * \var ColorCorrectionAberrationModeValues
 * \brief List of all ColorCorrectionAberrationMode supported values
 */

/**
 * \var ColorCorrectionAberrationMode
 * \brief Control to select the color correction aberration mode. Currently
 * identical to ANDROID_COLOR_CORRECTION_ABERRATION_MODE.
 *
 *  Mode of operation for the chromatic aberration correction algorithm.
 */

/**
 * \enum AeStateEnum
 * \brief Supported AeState values
 *
 * \var AeStateInactive
 * \brief The AE algorithm is inactive.
 *
 * \var AeStateSearching
 * \brief The AE algorithm has not converged yet.
 *
 * \var AeStateConverged
 * \brief The AE algorithm has converged.
 *
 * \var AeStateLocked
 * \brief The AE algorithm is locked.
 *
 * \var AeStateFlashRequired
 * \brief The AE algorithm would need a flash for good results
 *
 * \var AeStatePrecapture
 * \brief The AE algorithm has started a pre-capture metering session.
 * \sa AePrecaptureTrigger
 */

/**
 * \var AeStateValues
 * \brief List of all AeState supported values
 */

/**
 * \var AeState
 * \brief Control to report the current AE algorithm state. Currently identical to
 * ANDROID_CONTROL_AE_STATE.
 *
 *  Current state of the AE algorithm.
 */

/**
 * \enum AwbStateEnum
 * \brief Supported AwbState values
 *
 * \var AwbStateInactive
 * \brief The AWB algorithm is inactive.
 *
 * \var AwbStateSearching
 * \brief The AWB algorithm has not converged yet.
 *
 * \var AwbConverged
 * \brief The AWB algorithm has converged.
 *
 * \var AwbLocked
 * \brief The AWB algorithm is locked.
 */

/**
 * \var AwbStateValues
 * \brief List of all AwbState supported values
 */

/**
 * \var AwbState
 * \brief Control to report the current AWB algorithm state. Currently identical
 * to ANDROID_CONTROL_AWB_STATE.
 *
 *  Current state of the AWB algorithm.
 */

/**
 * \var SensorRollingShutterSkew
 * \brief Control to report the time between the start of exposure of the first
 * row and the start of exposure of the last row. Currently identical to
 * ANDROID_SENSOR_ROLLING_SHUTTER_SKEW
 */

/**
 * \enum LensShadingMapModeEnum
 * \brief Supported LensShadingMapMode values
 *
 * \var LensShadingMapModeOff
 * \brief No lens shading map mode is available.
 *
 * \var LensShadingMapModeOn
 * \brief The lens shading map mode is available.
 */

/**
 * \var LensShadingMapModeValues
 * \brief List of all LensShadingMapMode supported values
 */

/**
 * \var LensShadingMapMode
 * \brief Control to report if the lens shading map is available. Currently
 * identical to ANDROID_STATISTICS_LENS_SHADING_MAP_MODE.
 */

/**
 * \var PipelineDepth
 * \brief Specifies the number of pipeline stages the frame went through from when
 * it was exposed to when the final completed result was available to the
 * framework. Always less than or equal to PipelineMaxDepth. Currently
 * identical to ANDROID_REQUEST_PIPELINE_DEPTH.
 *
 * The typical value for this control is 3 as a frame is first exposed,
 * captured and then processed in a single pass through the ISP. Any
 * additional processing step performed after the ISP pass (in example face
 * detection, additional format conversions etc) count as an additional
 * pipeline stage.
 */

/**
 * \var MaxLatency
 * \brief The maximum number of frames that can occur after a request (different
 * than the previous) has been submitted, and before the result's state
 * becomes synchronized. A value of -1 indicates unknown latency, and 0
 * indicates per-frame control. Currently identical to
 * ANDROID_SYNC_MAX_LATENCY.
 */

/**
 * \enum TestPatternModeEnum
 * \brief Supported TestPatternMode values
 *
 * \var TestPatternModeOff
 * \brief No test pattern mode is used. The camera device returns frames from
 * the image sensor.
 *
 * \var TestPatternModeSolidColor
 * \brief Each pixel in [R, G_even, G_odd, B] is replaced by its respective
 * color channel provided in test pattern data.
 * \todo Add control for test pattern data.
 *
 * \var TestPatternModeColorBars
 * \brief All pixel data is replaced with an 8-bar color pattern. The vertical
 * bars (left-to-right) are as follows; white, yellow, cyan, green,
 * magenta, red, blue and black. Each bar should take up 1/8 of the
 * sensor pixel array width. When this is not possible, the bar size
 * should be rounded down to the nearest integer and the pattern can
 * repeat on the right side. Each bar's height must always take up the
 * full sensor pixel array height.
 *
 * \var TestPatternModeColorBarsFadeToGray
 * \brief The test pattern is similar to TestPatternModeColorBars,
 * except that each bar should start at its specified color at the top
 * and fade to gray at the bottom. Furthermore each bar is further
 * subdevided into a left and right half. The left half should have a
 * smooth gradient, and the right half should have a quantized
 * gradient. In particular, the right half's should consist of blocks
 * of the same color for 1/16th active sensor pixel array width. The
 * least significant bits in the quantized gradient should be copied
 * from the most significant bits of the smooth gradient. The height of
 * each bar should always be a multiple of 128. When this is not the
 * case, the pattern should repeat at the bottom of the image.
 *
 * \var TestPatternModePn9
 * \brief All pixel data is replaced by a pseudo-random sequence generated
 * from a PN9 512-bit sequence (typically implemented in hardware with
 * a linear feedback shift register). The generator should be reset at
 * the beginning of each frame, and thus each subsequent raw frame with
 * this test pattern should be exactly the same as the last.
 *
 * \var TestPatternModeCustom1
 * \brief The first custom test pattern. All custom patterns that are
 * available only on this camera device are at least this numeric
 * value. All of the custom test patterns will be static (that is the
 * raw image must not vary from frame to frame).
 */

/**
 * \var TestPatternModeValues
 * \brief List of all TestPatternMode supported values
 */

/**
 * \var TestPatternMode
 * \brief Control to select the test pattern mode. Currently identical to
 * ANDROID_SENSOR_TEST_PATTERN_MODE.
 */

} /* namespace draft */

#ifndef __DOXYGEN__
/*
 * Keep the controls definitions hidden from doxygen as it incorrectly parses
 * them as functions.
 */
extern const Control<bool> AeEnable(AE_ENABLE, "AeEnable");
extern const Control<bool> AeLocked(AE_LOCKED, "AeLocked");
extern const std::array<const ControlValue, 4> AeMeteringModeValues = {
	static_cast<int32_t>(MeteringCentreWeighted),
	static_cast<int32_t>(MeteringSpot),
	static_cast<int32_t>(MeteringMatrix),
	static_cast<int32_t>(MeteringCustom),
};
extern const Control<int32_t> AeMeteringMode(AE_METERING_MODE, "AeMeteringMode");
extern const std::array<const ControlValue, 4> AeConstraintModeValues = {
	static_cast<int32_t>(ConstraintNormal),
	static_cast<int32_t>(ConstraintHighlight),
	static_cast<int32_t>(ConstraintShadows),
	static_cast<int32_t>(ConstraintCustom),
};
extern const Control<int32_t> AeConstraintMode(AE_CONSTRAINT_MODE, "AeConstraintMode");
extern const std::array<const ControlValue, 4> AeExposureModeValues = {
	static_cast<int32_t>(ExposureNormal),
	static_cast<int32_t>(ExposureShort),
	static_cast<int32_t>(ExposureLong),
	static_cast<int32_t>(ExposureCustom),
};
extern const Control<int32_t> AeExposureMode(AE_EXPOSURE_MODE, "AeExposureMode");
extern const Control<float> ExposureValue(EXPOSURE_VALUE, "ExposureValue");
extern const Control<int32_t> ExposureTime(EXPOSURE_TIME, "ExposureTime");
extern const Control<float> AnalogueGain(ANALOGUE_GAIN, "AnalogueGain");
extern const std::array<const ControlValue, 3> AeFlickerModeValues = {
	static_cast<int32_t>(FlickerOff),
	static_cast<int32_t>(FlickerManual),
	static_cast<int32_t>(FlickerAuto),
};
extern const Control<int32_t> AeFlickerMode(AE_FLICKER_MODE, "AeFlickerMode");
extern const Control<int32_t> AeFlickerPeriod(AE_FLICKER_PERIOD, "AeFlickerPeriod");
extern const Control<int32_t> AeFlickerDetected(AE_FLICKER_DETECTED, "AeFlickerDetected");
extern const Control<float> Brightness(BRIGHTNESS, "Brightness");
extern const Control<float> Contrast(CONTRAST, "Contrast");
extern const Control<float> Lux(LUX, "Lux");
extern const Control<bool> AwbEnable(AWB_ENABLE, "AwbEnable");
extern const std::array<const ControlValue, 8> AwbModeValues = {
	static_cast<int32_t>(AwbAuto),
	static_cast<int32_t>(AwbIncandescent),
	static_cast<int32_t>(AwbTungsten),
	static_cast<int32_t>(AwbFluorescent),
	static_cast<int32_t>(AwbIndoor),
	static_cast<int32_t>(AwbDaylight),
	static_cast<int32_t>(AwbCloudy),
	static_cast<int32_t>(AwbCustom),
};
extern const Control<int32_t> AwbMode(AWB_MODE, "AwbMode");
extern const Control<bool> AwbLocked(AWB_LOCKED, "AwbLocked");
extern const Control<Span<const float, 2>> ColourGains(COLOUR_GAINS, "ColourGains");
extern const Control<int32_t> ColourTemperature(COLOUR_TEMPERATURE, "ColourTemperature");
extern const Control<float> Saturation(SATURATION, "Saturation");
extern const Control<Span<const int32_t, 4>> SensorBlackLevels(SENSOR_BLACK_LEVELS, "SensorBlackLevels");
extern const Control<float> Sharpness(SHARPNESS, "Sharpness");
extern const Control<int32_t> FocusFoM(FOCUS_FO_M, "FocusFoM");
extern const Control<Span<const float, 9>> ColourCorrectionMatrix(COLOUR_CORRECTION_MATRIX, "ColourCorrectionMatrix");
extern const Control<Rectangle> ScalerCrop(SCALER_CROP, "ScalerCrop");
extern const Control<float> DigitalGain(DIGITAL_GAIN, "DigitalGain");
extern const Control<int64_t> FrameDuration(FRAME_DURATION, "FrameDuration");
extern const Control<Span<const int64_t, 2>> FrameDurationLimits(FRAME_DURATION_LIMITS, "FrameDurationLimits");
extern const Control<float> SensorTemperature(SENSOR_TEMPERATURE, "SensorTemperature");
extern const Control<int64_t> SensorTimestamp(SENSOR_TIMESTAMP, "SensorTimestamp");
extern const std::array<const ControlValue, 3> AfModeValues = {
	static_cast<int32_t>(AfModeManual),
	static_cast<int32_t>(AfModeAuto),
	static_cast<int32_t>(AfModeContinuous),
};
extern const Control<int32_t> AfMode(AF_MODE, "AfMode");
extern const std::array<const ControlValue, 3> AfRangeValues = {
	static_cast<int32_t>(AfRangeNormal),
	static_cast<int32_t>(AfRangeMacro),
	static_cast<int32_t>(AfRangeFull),
};
extern const Control<int32_t> AfRange(AF_RANGE, "AfRange");
extern const std::array<const ControlValue, 2> AfSpeedValues = {
	static_cast<int32_t>(AfSpeedNormal),
	static_cast<int32_t>(AfSpeedFast),
};
extern const Control<int32_t> AfSpeed(AF_SPEED, "AfSpeed");
extern const std::array<const ControlValue, 2> AfMeteringValues = {
	static_cast<int32_t>(AfMeteringAuto),
	static_cast<int32_t>(AfMeteringWindows),
};
extern const Control<int32_t> AfMetering(AF_METERING, "AfMetering");
extern const Control<Span<const Rectangle>> AfWindows(AF_WINDOWS, "AfWindows");
extern const std::array<const ControlValue, 2> AfTriggerValues = {
	static_cast<int32_t>(AfTriggerStart),
	static_cast<int32_t>(AfTriggerCancel),
};
extern const Control<int32_t> AfTrigger(AF_TRIGGER, "AfTrigger");
extern const std::array<const ControlValue, 3> AfPauseValues = {
	static_cast<int32_t>(AfPauseImmediate),
	static_cast<int32_t>(AfPauseDeferred),
	static_cast<int32_t>(AfPauseResume),
};
extern const Control<int32_t> AfPause(AF_PAUSE, "AfPause");
extern const Control<float> LensPosition(LENS_POSITION, "LensPosition");
extern const std::array<const ControlValue, 4> AfStateValues = {
	static_cast<int32_t>(AfStateIdle),
	static_cast<int32_t>(AfStateScanning),
	static_cast<int32_t>(AfStateFocused),
	static_cast<int32_t>(AfStateFailed),
};
extern const Control<int32_t> AfState(AF_STATE, "AfState");
extern const std::array<const ControlValue, 3> AfPauseStateValues = {
	static_cast<int32_t>(AfPauseStateRunning),
	static_cast<int32_t>(AfPauseStatePausing),
	static_cast<int32_t>(AfPauseStatePaused),
};
extern const Control<int32_t> AfPauseState(AF_PAUSE_STATE, "AfPauseState");

namespace draft {

extern const std::array<const ControlValue, 3> AePrecaptureTriggerValues = {

	static_cast<int32_t>(AePrecaptureTriggerIdle),

	static_cast<int32_t>(AePrecaptureTriggerStart),

	static_cast<int32_t>(AePrecaptureTriggerCancel),

};

extern const Control<int32_t> AePrecaptureTrigger(AE_PRECAPTURE_TRIGGER, "AePrecaptureTrigger");

extern const std::array<const ControlValue, 5> NoiseReductionModeValues = {

	static_cast<int32_t>(NoiseReductionModeOff),

	static_cast<int32_t>(NoiseReductionModeFast),

	static_cast<int32_t>(NoiseReductionModeHighQuality),

	static_cast<int32_t>(NoiseReductionModeMinimal),

	static_cast<int32_t>(NoiseReductionModeZSL),

};

extern const Control<int32_t> NoiseReductionMode(NOISE_REDUCTION_MODE, "NoiseReductionMode");

extern const std::array<const ControlValue, 3> ColorCorrectionAberrationModeValues = {

	static_cast<int32_t>(ColorCorrectionAberrationOff),

	static_cast<int32_t>(ColorCorrectionAberrationFast),

	static_cast<int32_t>(ColorCorrectionAberrationHighQuality),

};

extern const Control<int32_t> ColorCorrectionAberrationMode(COLOR_CORRECTION_ABERRATION_MODE, "ColorCorrectionAberrationMode");

extern const std::array<const ControlValue, 6> AeStateValues = {

	static_cast<int32_t>(AeStateInactive),

	static_cast<int32_t>(AeStateSearching),

	static_cast<int32_t>(AeStateConverged),

	static_cast<int32_t>(AeStateLocked),

	static_cast<int32_t>(AeStateFlashRequired),

	static_cast<int32_t>(AeStatePrecapture),

};

extern const Control<int32_t> AeState(AE_STATE, "AeState");

extern const std::array<const ControlValue, 4> AwbStateValues = {

	static_cast<int32_t>(AwbStateInactive),

	static_cast<int32_t>(AwbStateSearching),

	static_cast<int32_t>(AwbConverged),

	static_cast<int32_t>(AwbLocked),

};

extern const Control<int32_t> AwbState(AWB_STATE, "AwbState");

extern const Control<int64_t> SensorRollingShutterSkew(SENSOR_ROLLING_SHUTTER_SKEW, "SensorRollingShutterSkew");

extern const std::array<const ControlValue, 2> LensShadingMapModeValues = {

	static_cast<int32_t>(LensShadingMapModeOff),

	static_cast<int32_t>(LensShadingMapModeOn),

};

extern const Control<int32_t> LensShadingMapMode(LENS_SHADING_MAP_MODE, "LensShadingMapMode");

extern const Control<int32_t> PipelineDepth(PIPELINE_DEPTH, "PipelineDepth");

extern const Control<int32_t> MaxLatency(MAX_LATENCY, "MaxLatency");

extern const std::array<const ControlValue, 6> TestPatternModeValues = {

	static_cast<int32_t>(TestPatternModeOff),

	static_cast<int32_t>(TestPatternModeSolidColor),

	static_cast<int32_t>(TestPatternModeColorBars),

	static_cast<int32_t>(TestPatternModeColorBarsFadeToGray),

	static_cast<int32_t>(TestPatternModePn9),

	static_cast<int32_t>(TestPatternModeCustom1),

};

extern const Control<int32_t> TestPatternMode(TEST_PATTERN_MODE, "TestPatternMode");

} /* namespace draft */
#endif

/**
 * \brief List of all supported libcamera controls
 *
 * Unless otherwise stated, all controls are bi-directional, i.e. they can be
 * set through Request::controls() and returned out through Request::metadata().
 */
extern const ControlIdMap controls {
	{ AE_ENABLE, &AeEnable },
	{ AE_LOCKED, &AeLocked },
	{ AE_METERING_MODE, &AeMeteringMode },
	{ AE_CONSTRAINT_MODE, &AeConstraintMode },
	{ AE_EXPOSURE_MODE, &AeExposureMode },
	{ EXPOSURE_VALUE, &ExposureValue },
	{ EXPOSURE_TIME, &ExposureTime },
	{ ANALOGUE_GAIN, &AnalogueGain },
	{ AE_FLICKER_MODE, &AeFlickerMode },
	{ AE_FLICKER_PERIOD, &AeFlickerPeriod },
	{ AE_FLICKER_DETECTED, &AeFlickerDetected },
	{ BRIGHTNESS, &Brightness },
	{ CONTRAST, &Contrast },
	{ LUX, &Lux },
	{ AWB_ENABLE, &AwbEnable },
	{ AWB_MODE, &AwbMode },
	{ AWB_LOCKED, &AwbLocked },
	{ COLOUR_GAINS, &ColourGains },
	{ COLOUR_TEMPERATURE, &ColourTemperature },
	{ SATURATION, &Saturation },
	{ SENSOR_BLACK_LEVELS, &SensorBlackLevels },
	{ SHARPNESS, &Sharpness },
	{ FOCUS_FO_M, &FocusFoM },
	{ COLOUR_CORRECTION_MATRIX, &ColourCorrectionMatrix },
	{ SCALER_CROP, &ScalerCrop },
	{ DIGITAL_GAIN, &DigitalGain },
	{ FRAME_DURATION, &FrameDuration },
	{ FRAME_DURATION_LIMITS, &FrameDurationLimits },
	{ SENSOR_TEMPERATURE, &SensorTemperature },
	{ SENSOR_TIMESTAMP, &SensorTimestamp },
	{ AF_MODE, &AfMode },
	{ AF_RANGE, &AfRange },
	{ AF_SPEED, &AfSpeed },
	{ AF_METERING, &AfMetering },
	{ AF_WINDOWS, &AfWindows },
	{ AF_TRIGGER, &AfTrigger },
	{ AF_PAUSE, &AfPause },
	{ LENS_POSITION, &LensPosition },
	{ AF_STATE, &AfState },
	{ AF_PAUSE_STATE, &AfPauseState },
	{ AE_PRECAPTURE_TRIGGER, &draft::AePrecaptureTrigger },
	{ NOISE_REDUCTION_MODE, &draft::NoiseReductionMode },
	{ COLOR_CORRECTION_ABERRATION_MODE, &draft::ColorCorrectionAberrationMode },
	{ AE_STATE, &draft::AeState },
	{ AWB_STATE, &draft::AwbState },
	{ SENSOR_ROLLING_SHUTTER_SKEW, &draft::SensorRollingShutterSkew },
	{ LENS_SHADING_MAP_MODE, &draft::LensShadingMapMode },
	{ PIPELINE_DEPTH, &draft::PipelineDepth },
	{ MAX_LATENCY, &draft::MaxLatency },
	{ TEST_PATTERN_MODE, &draft::TestPatternMode },
};

} /* namespace controls */

} /* namespace libcamera */
