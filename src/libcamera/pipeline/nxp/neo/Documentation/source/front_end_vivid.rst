.. SPDX-License-Identifier: CC-BY-SA-4.0

The Vivid Front End
====================

Overview
--------
Vivid (Virtual Video Test Driver) is a Linux video driver that emulates v4l2 devices in software. Integrating those devices into a dedicated front end is useful to test the image processing pipeline without requiring physical hardware.
See the `Vivid Linux driver documentation <linux_driver_vivid_documentation_>`_ for details.

.. _linux_driver_vivid_documentation: https://docs.kernel.org/admin-guide/media/vivid.html

Usage example can be for testing specific configuration of the pipeline handler or replay offline recorded bayer raw images captured on the field.

By default each vivid instance consists in a Media Controller device that exposes multiple devices, including one capture (input) and one output video device. The input video device provides raw video frames that are fed to the Neo pipeline handler for processing. For single-stream operation of the pipeline handler (SDR), only one raw video stream is necessary that can be provided by the input video device of a single vivid instance.
For dual-stream operation (e.g. HDR), two vivid instances are required to provide two independent video streams from their respective video input devices.
To simulate a multi-camera system, multiple virtual cameras can also be instantiated, using as many vivid instances as there are cameras to emulate.

Each video instance can be used in either of the two modes:

1. **Test Pattern Generation Mode**: vivid driver generates a configurable Bayer test pattern
2. **Loopback mode**: vivid driver loops back the video stream from the output device, injected by the user application

Kernel module usage
-------------------
The vivid driver is typically loaded as a kernel module ``vivid.ko`` with optional parameters to configure. The vivid kernel module build requires some Kconfig options to be enabled:

.. code-block:: shell

	VIDEO_VIVID [=m]

when installed, module can be loaded with the usual:

.. code-block:: shell

	modprobe vivid [<args...>]

and removed with:

.. code-block:: shell

	modprobe -r vivid

In the context of the Neo pipeline handler, there are two important vivid module parameters to be passed as arguments to the ``modprobe vivid`` command:

1. **n_devs**: Number of vivid instances to create (default: 1)
2. **allocators**: Memory allocator to use for video buffers (default: 0 for vmalloc).

See the `Vivid Linux driver documentation <linux_driver_vivid_documentation_>`_ for the complete list of parameters.

The ``n_devs`` parameter should be adjusted to create at least as many instances as needed by the virtual camera system being simulated: one per camera for single-stream operation, two per camera for dual-stream operation (HDR, RGBIr dual-context).

The ``allocators`` should be explicitly set to ``1`` (dma-contig) for each instance, to override the default value ``0`` (vmalloc) that allocates non-contiguous memory buffers, not suitable to the Neo pipeline handler.

.. note::
	The ``allocators`` parameter accepts a comma-separated list of values, one per vivid instance.

As an example, to create a single vivid instance (one camera stream):

.. code-block:: shell

	modprobe vivid n_devs=1 allocators=1

Or to create two vivid instances with the dma-contig allocator:

.. code-block:: shell

	modprobe vivid n_devs=2 allocators=1,1

Following sections assume that the vivid module has been loaded with the appropriate parameters for the desired camera(s) configuration.

Quick start
-----------

The default configuration of the pipeline handler config file `config.yaml` declares a single-stream vivid camera, hence a single vivid instance is sufficient.

Vivid virtual cameras are enumerated like any other camera in the system as visible from a regular ``cam -l`` command:

.. code-block:: shell

	cam -l 2>&1 | grep -i vivid
	[...]
	    [...] INFO [...] Camera: vivid-camera:0 SBGGR12/1920x1080 [stream:0 capture:/dev/video61]
	    [...] INFO [...] Using tuning file /usr/share/libcamera/ipa/nxp/neo/vivid.yaml
	    [...] INFO [...] Adding camera 'vivid-camera:0' for pipeline handler nxp/neo
	[...]
	    5: External camera 'vivid' (vivid-camera:0)

As ``cam -l`` output shows, there is a ``vivid-camera:0`` whose model is ``vivid``, available for use. It has a single stream with video format SBGGR12/1920x1080. This configuration comes from the pipeline handler `config.yaml` config file, that defines a configuration relevant to the ``vivid`` camera model:

.. code-block:: yaml

	[...]
	cameras:
	[...]
	  - model: vivid
	    vivid-instances:
	        - pixel-format: SBGGR12
	          size: [1920, 1080]
	          loopback: false
	          tpg-pattern: 0
	          tpg-hmove: 3
	          tpg-vmove: 3

Bayer format and the resolution of the camera can be modified from there.

.. note::
	The ``pixel-format`` entry is a string representing the Bayer format using libcamera PixelFormat convention: ``SBGGR8``, ``SBGGR10``, ``SBGGR12``, ``SBGGR16``, etc.

It is also possible to select the loopback operation of the vivid instance. When disabled, the Test Pattern Generator (TPG) mode is used, which generates a configurable Bayer test pattern with the parameters ``tpg-pattern``, ``tpg-hmove``, and ``tpg-vmove`` that control the pattern type and motion simulation.

``vivid-instances`` is a list that may contain multiple entries for dual-stream operation. Each entry configures one vivid instance with its own parameters. In present case there is a single stream (SDR) using a single vivid instance.

The camera can be used with the `cam` applet or any libcamera-based applications. For example GStreamer can be used to run the pipeline:

.. code-block:: shell

	CAMERAV0="vivid-camera:0"
	GFORMAT=YUY2
	gst-launch-1.0 \
	    libcamerasrc camera-name="$CAMERAV0" ! \
	    video/x-raw, format="$GFORMAT" ! \
	    queue ! \
	    autovideosink sync=false

.. note::
	The ``sync=false`` parameter in the ``autovideosink`` element disables synchronization and workarounds some possible frame buffer time stamping quirks from the vivid driver.

The pipeline runs with the IPA as with any other camera. The default `vivid.yaml` calibration file selects a minimal set of algorithms for the vivid cameras, but can be amended as needed to match the test needs:

.. code-block:: yaml

	# SPDX-License-Identifier: CC0-1.0
	%YAML 1.1
	---
	version: 1
	algorithms:
	  - PipeConf:
	      # Raw buffers pixel data is LSB aligned (i.MX95 workaround disabled)
	      inalign0: 0
	      inalign1: 0
	  - Awb:
	  - BlackLevelCorrection:
	      R: 0
	      Gr: 0
	      Gb: 0
	      B: 0
	  - Agc:


Loopback mode
-------------

In loopback mode, a vivid instance will no longer generate patterns on the capture (input) device, but loop (copy) frames from a corresponding output device instead. This allows to feed custom frames into the pipeline.

.. note::
	Loopback is enabled when the peer input and output devices are configured with the same format and the same resolution.

Loopback mode is enabled from the config file. As an example, a single stream (SDR) 3840x2160 BGGR 10-bit camera would be declared in the pipeline configuration file as:

.. code-block:: yaml

	[...]
	cameras:
	[...]
	  - model: vivid
	    vivid-instances:
	        - pixel-format: SBGGR10
	          size: [3840, 2160]
	          loopback: true

The test pattern parameters can be omitted as they are irrelevant in loopback mode.
The pipeline can be exercised in a standard way for instance using GStreamer adapter:

.. code-block:: shell

	CAMERAV0="vivid-camera:0"
	GFORMAT=YUY2
	gst-launch-1.0 \
	    libcamerasrc camera-name="$CAMERAV0" ! \
	    video/x-raw, format="$GFORMAT" ! \
	    queue ! \
	    autovideosink sync=false

In loopback mode, some video noise should now be displayed instead of a test pattern. To feed custom frames into the pipeline, first step is to identify the vivid output device associated with the input device. The video devices pairing information is available from the libcamera log:

.. code-block:: shell

	cam -l 2>&1 | grep vivid-camera
	[...]
	    [...] INFO [...] Camera: vivid-camera:0 SBGGR10/3840x2160 [stream:0 capture:/dev/video61 output:/dev/video62]
	[...]

In this example, the ``vivid-camera:0`` has a single stream based on the vivid devices:

#. ``/dev/video61`` : capture (input) stream, correspond to the raw frames injected into the pipeline handler
#. ``/dev/video62`` : output stream, the device to be used to inject frames into the loopback

The actual output device node ``/dev/videoX`` to use is system-dependent and shall be identified from the libcamera log for each setup.

Injecting custom frames into the pipeline handler requires configuring the output device with a format and resolution that match the input device, then queuing buffers into it.
There are multiple ways of doing so - the following subsections give some examples based on standard options.

.. warning::

	The output device must be configured with the same format and resolution as the input device for the loopback to work.

.. warning::

	The default vivid calibration file, `vivid.yaml`, configures the ISP input paths to fetch pixel data from memory buffers with LSB-aligned pixel data (V4L2-compliant).
	If frames are injected with MSB alignment—for example, frames captured from the i.MX95 ISI device— then the ``PipeConf`` section of the calibration file must be adjusted by setting the inalign0 and inalign1 parameters to ``1``.

v4l2-ctl usage for loopback
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Simple option for injecting frames is to use ``v4l2-ctl`` to configure the output device and queue buffers.
As an example, here is a snippet to configure the output device ``/dev/video62`` with SBGGR10 format at 3840x2160 resolution and queue buffers initialized with the raw pixel data read from a file.

.. code-block:: shell

	W=3840
	H=2160
	OUT="/dev/video62"
	VFMT=BG10
	FILE="raw_frame.bin"
	v4l2-ctl -d "$OUT" \
	    --set-fmt-video-out=width="$W",height="$H",pixelformat="$VFMT" \
	    --stream-out-mmap --stream-from "$FILE" --stream-loop

.. note::
	The ``pixelformat`` argument of v4l2-ctl command is a string representation of the Bayer format using convention below (in `RGGB`, `GRBG`, `GBRG`, `BGGR` order):

	* 8-bit: ``RGGB``, ``GRBG``, ``GBRG``, ``BA81``
	* 10-bit: ``RG10``, ``BA10``, ``GB10``, ``BG10``
	* 12-bit: ``RG12``, ``BA12``, ``GB12``, ``BG12``
	* 16-bit: ``RG16``, ``BA16``, ``GB16``, ``BYR2``

The current output device format configuration can be verified with:

.. code-block:: shell

	v4l2-ctl -d $OUT --get-fmt-video-out

The list of formats supported by the output device can be listed with:

.. code-block:: shell

	v4l2-ctl -d $OUT --list-formats-out-ext

GStreamer usage for loopback
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A GStreamer pipeline can also be used to inject frames into the loopback device. This offers great flexibility though the v4l2sink elements caps negotiation is not straightforward.

Here is a snippet to configure the output device ``/dev/video62`` with SBGGR10 format at 3840x2160 resolution and queue buffers initalized with the raw pixel data read from a file.

.. code-block:: shell

	W=3840
	H=2160
	OUT="/dev/video62"
	GFMT=bggr10le
	COLORIMETRY="1:1:5:1"
	FILE="raw_frame.bin"
	CAPS="video/x-bayer,format=$GFMT,width=$W,height=$H,colorimetry=$COLORIMETRY,pixel-aspect-ratio=1/1"
	gst-launch-1.0 multifilesrc -v location="$FILE" stop-index=0 loop=true ! \
	    "$CAPS" ! queue ! \
	    v4l2sink device="$OUT" sync=false

.. note::
	Mind the explicit colorimetry defined in the capsfilter to accommodate with the vivid output video device.

.. note::
	The format field of the GStreamer capsfilter is a string representation of the Bayer format, in its little-endian variant when relevant, using convention below (in `RGGB`, `GRBG`, `GBRG`, `BGGR` order):

	* 8-bit: ``rggb``, ``grgb``, ``gbrg``, ``bggr``
	* 10-bit: ``rggb10le``, ``grgb10le``, ``gbrg10le``, ``bggr10le``
	* 12-bit: ``rggb12le``, ``grgb12le``, ``gbrg12le``, ``bggr12le``
	* 16-bit: ``rggb16``, ``grgb16``, ``gbrg16``, ``bggr16``


Another possibility is to inject a test pattern generated by the GStreamer ``videotestsrc`` element instead of reading the pixel data from a file:

.. code-block:: shell

	W=3840
	H=2160
	OUT="/dev/video62"
	GFMT=bggr10le
	COLORIMETRY="1:1:5:1"
	FILE="raw_frame.bin"
	CAPS="video/x-bayer,format=$GFMT,width=$W,height=$H,colorimetry=$COLORIMETRY,pixel-aspect-ratio=1/1"
	gst-launch-1.0 videotestsrc -v ! \
	    "$CAPS" ! queue ! capssetter replace=true caps="$CAPS" ! \
	    v4l2sink device="$OUT" sync=false

.. note::
	Mind the capsetter element addition that is stripping down the verbose caps of the ``videotestsrc`` element.

Multi-stream operation
----------------------

A vivid camera can simply be configured with a second image stream to support HDR or dual-context modes of operation of the pipeline handler. The main difference from single-stream operation is that a second vivid instance needs to be created at `modprobe` time and configured for that camera.

To illustrate, this is an example of the pipeline config file (`config.yaml`) definition of a vivid camera using two image streams thus two vivid instances to operate in HDR mode, using different test patterns for both.

.. code-block:: yaml

	[...]
	cameras:
	[...]
	  - model: vivid
	    streams: [image1]
	    vivid-instances:
	        - pixel-format: SBGGR10
	          size: [1920, 1080]
	          loopback: false
	          tpg-pattern: 0
	          tpg-hmove: 3
	          tpg-vmove: 3
	        - pixel-format: SBGGR10
	          size: [1920, 1080]
	          loopback: false
	          tpg-pattern: 4
	          tpg-hmove: 2
	          tpg-vmove: 2

In order to merge the two streams into a single HDR image, the IPA algorithm needs to be configured accordingly. This is done by complementing the `vivid.yaml` calibration file with a HDR merge configuration:

.. code-block:: yaml

	# SPDX-License-Identifier: CC0-1.0
	%YAML 1.1
	---
	version: 1
	algorithms:
	  - PipeConf:
	      # Raw buffers pixel data is LSB aligned (i.MX95 workaround disabled)
	      inalign0: 0
	      inalign1: 0
	  - Awb:
	  - BlackLevelCorrection:
	      R: 0
	      Gr: 0
	      Gb: 0
	      B: 0
	  - Agc:
	  - HdrMerge:
	      ratio-long2short: 16

The pipeline with the HDR-enabled vivid camera can now be tested with the same GStreamer pipeline:

.. code-block:: shell

	CAMERAV0="vivid-camera:0"
	GFORMAT=YUY2
	gst-launch-1.0 \
	    libcamerasrc camera-name="$CAMERAV0" ! \
	    video/x-raw, format="$GFORMAT" ! \
	    queue ! \
	    autovideosink sync=false
