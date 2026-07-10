# NXP Neo ISP Tuning Guide

## Overview

The NXP Neo ISP tuning script (`nxpneo.py`) provides automated calibration for the
NXP Neo Image Signal Processor.
It currently supports two main calibration modules:

- **CCM (Color Correction Matrix)**: Color correction calibration
- **LSC (Lens Shading Correction)**: Lens shading and color shading correction

See https://libcamera.stefanklug.com/guides/tuning/tuning.html for the general tuning
process and libcamera tuning guide documentation.

## Features

### Color Correction Matrix (CCM)

The CCM module calibrates color correction matrices to ensure accurate color reproduction.
Its processing is based on Macbeth color chart detection from RAW images.

No NXP NEO dedicated key features are currently implemented. The calibration uses standard.

### Lens Shading Correction (LSC)

The LSC module corrects non-uniform illumination across the image sensor caused by lens
vignetting. It is based on the rkisp1 LSC implementation but enhanced to address color
shading artifacts.

Key features include:

- **Color Shading Correction (CSC)**: Applied before LSC calibration
  - Prevents greenish corners and other color artifacts
  - Particularly effective for sensors with strong lens shading combined with color imbalance

- **Merged LUT approach**: CSC LUTs are merged with a single LSC LUT (green channel)
  - Preserves both color and luminance uniformity

### Image Preprocessing

- **Configurable image cropping**: Remove unwanted image regions before calibration
  - Useful for removing embedded data lines
  - Configured via `general/load-image/crop-top-lines` key in config file
  - Specified number of rows removed from top of image when loading

## Usage

### Basic Command

- **Prerequisites**:
  - Install Python dependencies from requirements.txt
  - Generate an image data set suitable for calibration

```bash
./nxpneo.py -c <libcamera>/utils/tuning/config-example.yaml -i input_images/ -o output.yaml
```

See `Configuration File` section below for details on YAML template parameters.
See `Input Requirements` section below for details on image naming conventions and format requirements.

### Configuration File

The tuning script uses a YAML configuration file. Example structure:

```yaml
general:
  load-image:
    crop-top-lines: 2  # Optional: Remove top N lines from images

ccm:
  # CCM-specific configuration

lsc:
  # LSC-specific configuration
```

### Input Requirements

#### Image Format
- **Format**: RAW format from the sensor in DNG file format
- **Lighting**: Uniform, controlled lighting conditions recommended

#### LSC Images
- **Content**: Uniform white reference images under consistent illumination
- **Naming convention**: `alsc_<temperature>k_<number>.dng`
  - `<temperature>`: Color temperature in Kelvin (e.g., 2500, 6500)
  - `<number>`: Sequential number (0, 1, 2, ...) for multiple captures at the same temperature
- **Examples**:
  - `alsc_2500k_0.dng`
  - `alsc_2500k_1.dng`
  - `alsc_6500k_0.dng`

#### CCM Images
- **Content**: Captured using a standard color chart (e.g., Macbeth ColorChecker)
- **Naming convention**: `<sensor_name>_<lux>l_<temperature>k_0.dng`
  - `<sensor_name>`: Name of the sensor (e.g., imx335)
  - `<lux>`: Illumination level in lux (e.g., 1000, 1200, 1600)
  - `<temperature>`: Color temperature in Kelvin (e.g., 2500, 4000, 6000)
- **Examples**:
  - `imx335_1000l_2500k_0.dng`
  - `imx335_1200l_4000k_0.dng`
  - `imx335_1600l_6000k_0.dng`

#### Directory Structure Example

After capturing calibration images, the tuning directory should look like this:

```
../tuning-data/
├── alsc_2500k_0.dng
├── alsc_2500k_1.dng
├── alsc_2500k_2.dng
├── alsc_6500k_0.dng
├── imx335_1000l_2500k_0.dng
├── imx335_1200l_4000k_0.dng
└── imx335_1600l_6000k_0.dng
```

### Output

The script generates a YAML file containing calibrated parameters that can be loaded
by the libcamera NXP Neo IPA module.

## Debugging

Both modules support debug visualization:

```python
lsc = LSCNxpNeo(debug=[lt.Debug.Plot], ...)
ccm = CCMNxpNeo(debug=[lt.Debug.Plot])
```

This enables plotting of intermediate results for analysis and verification.
