.. SPDX-License-Identifier: CC-BY-SA-4.0

Neo IPA algorithms
==================

Introduction
------------

The Neo IPA implements the image processing algorithms for the Neo ISP.
These algorithms provide computed parameters for the ISP blocks based on
sensor tuning data (static configuration) and frame statistics (dynamic
configuration).

The supported algorithms for the Neo ISP are:

- Auto Focus Control (AF)
- Automatic Gain/Exposure Control (AGC/AEC)
- Automatic White Balance (AWB)
- Black Level Correction (BLC)
- Color Correction Matrix (CCM)
- Dynamic Range Compression (DRC)
- Gamma Output Correction control (GOC)
- HDR decompression
- HDR merge
- Lens Shading Correction control (LSC)
- Pipe Conf
- RGBIr

Some algorithms are mandatory for all sensors. Additionally, some algorithms
have dependencies and must be executed in a specific order to ensure proper
operation. Such a dependency exists when the execution of one algorithm relies
on some computation or configuration output by an other algorithm.

Algorithm considerations
-------------------------

The diagram below provides an overview of the first blocks of the ISP hardware
pipeline. These blocks are involved, among other things, in input pixel data
bit depth rescaling, allowing the data to enter the remaining part of the
hardware pipeline at the nominal 20-bit camera pixel bit depth.

.. graphviz:: media/neo_isp_blocks.dot

The following considerations are taken into account by the algorithms to
automatically apply configurations when no explicit configured value is defined
in the calibration file:

* The output pixel bit depth of the HDR-merge block is 20 bits in all cases.
* The input pixel bit depth of the HDR-merge block is:

  * 20-bit (input0) when the HDR-merge block is disabled (SDR case).
  * Respective camera bit depth of respective input0 and input1 when the
    HDR-merge block is enabled (HDR merge case). On both input, bit depth is
    rescaled if necessary to be at least 12-bit to satisfy OBWB saturation
    capability.

Mandatory algorithms
--------------------

The first blocks of the ISP hardware pipeline as shown in the diagram are
configured by default as bypass by the ISP driver.

Since the algorithm considerations dependent on the pipeline mode of operation
(SDR or HDR-merge case), the following algorithms are mandatory and must be
enabled for every sensor:

* PipeConf
* HDR decompression

The OBWB blocks are controlled by the BLC and the AWB algorithms which can be
optionnally enabled.

PipeConf
^^^^^^^^

The PIPE_CONF bitfields available in the uAPI are INALIGN0/1 and LPALIGN0/1
from the IMG_CONF_CAM0 register. Both parameters are relevant to configure the
input0 and input1 paths of the ISP.

* INALIGN0/1: significant bit selection when fetching pixel data from memory
  (used to workaround the ISI limitation on i.MX95).
* LPALIGN0/1: pixel data bit rescaling (left-shift) after memory fetch before
  injecting its value into the ISP pipeline.

Based on the pipeline mode of operation, the LPALIGN0/1 parameters are
configured by the PipeConf algorithm as follows:

* In SDR mode, pixel data are rescaled and stored left-shifted
  (LPALIGN0/1=1).
* In HDR-merge mode, the native camera bit depth is preserved (LPALIGN0/1=0).

HDR decompression
^^^^^^^^^^^^^^^^^

If no configuration is present in the calibration file, the HDR decompression
algorithm falls back on a default configuration logic. It is configured as a
simple linear gain to compensate for the following cases:

* A hardware peculiarity in the ISP: with 12-bit camera pixel bit depth, input0
  and input1 are rescaled to 16-bit regardless of the PIPECONF.LPALIGN setting.
* A constraint from the OBWB block, whose saturation (obpp) is configurable only
  from 12-bit onwards. For lower bit depths (e.g. 10-bit), the required gain is
  applied in the HDR decompression block to rescale the input to 12-bit,
  ensuring compatibility with OBWB constraints.

Algorithm Dependencies
----------------------

AWB dependency with BLC
^^^^^^^^^^^^^^^^^^^^^^^

Due to the black level correction offset applied to pixel data, gain
compensation is required to ensure the full pixel range is used, allowing the
OBWB saturation function to operate properly.
This compensation is performed by the AWB algorithm after the BLC offset removal.
This applies when AWB is mapped either in the same block as BLC or in a
different OBWB block located downstream in the ISP pipeline.

Consequently, the AWB algorithm must run afer the BLC algorithm to account for
the scaled BLC offsets.

AGC dependency with AWB and HDR merge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The AGC algorithm depends on both the AWB and HDR merge algorithms, as it uses:

* AWB gains to adjust the computed luminance.
* The ratio between long and short exposures configured by the HDR merge
  algorithm to scale the histogram.

Therefore, AGC should run after both AWB and HDR merge.

CCM dependency with AWB
^^^^^^^^^^^^^^^^^^^^^^^

The CCM algorithm uses the color temperature computed by the AWB algorithm to
interpolate color correction matrices.

Therefore, CCM should run after AWB.

Algorithms list summary
-----------------------

The table below summarizes each algorithm, its description, supported user
controls, and dependencies:

.. list-table::
   :widths: 20 50 10 15
   :header-rows: 1

   * - Algorithms
     - Description
     - User controls
     - Dependencies
   * - Auto Focus Control (AF)
     - Controls the lens position to adjust focus.
     - ``controls::AfMode``, ``controls::AfRange``, ``controls::AfSpeed``,
       ``controls::AfMetering``, ``controls::AfWindows``, ``controls::AfPause``,
       ``controls::AfTrigger``, ``controls::LensPosition``
     - None
   * - Automatic Gain/Exposure Control (AGC/AEC)
     - Controls the sensor gain and exposure based on ISP histograms.
     - ``controls::AeEnable``, ``controls::ExposureTime``,
       ``controls::AnalogueGain``
     - After AWB and HDR merge
   * - Automatic White Balance (AWB)
     - Controls the ISP white balance based on color temperature statistics.
     - ``controls::AwbEnable``, ``controls::ColourGains``
     - After BLC;
       Before CCM and AGC
   * - Black Level Correction (BLC)
     - Configures the ISP offsets to ensure black pixels map to zero.
     - None
     - Before AWB
   * - Color Correction Matrix (CCM)
     - Configures the ISP RGB to YUV unit based on CCM and color temperature.
     - None
     - After AWB
   * - Dynamic Range Compression (DRC)
     - Controls global tone mapping.
     - None
     - None
   * - Gamma Output Correction control (GOC)
     - Controls gamma in RGB domain.
     - ``controls::Gamma``
     - None
   * - **HDR decompression** - *mandatory*
     - Configures the HDR decompression, required for companded sensors.
     - None
     - None
   * - HDR merge
     - Configures the pixels combination of two image inputs (line path 0 and 1)
       into one output.
     - ``controls::HdrMode``
     - Before AGC
   * - Lens Shading Correction control (LSC)
     - Configures the correction required to compensate for lens shading effect.
     - ``controls::LensShadingCorrectionEnable``
     - None
   * - **Pipe Conf** - *mandatory*
     - Configures INALIGN and LPALIGN parameters from the Pipeline Configuration
       ISP block.
     - None
     - None
   * - RGBIr
     - Configures RGBIR and IR compression for RGBIr sensors.
     - None
     - None
  






