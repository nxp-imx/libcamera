# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright 2025-2026 NXP
#
# LSC module for tuning NXP Neo ISP
#
# Based on LSC module for tuning rkisp1 - rkisp1.py
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#

from .lsc import LSC

import libtuning as lt
import libtuning.utils as utils

from numbers import Number
import numpy as np


class LSCNxpNeo(LSC):
    """
    Lens Shading Correction (LSC) module.

    This module performs lens shading correction calibration to ensure uniform
    brightness and color across the image sensor.

    Module Configuration Parameters:
        sector_shape (tuple): Grid dimensions for lens shading correction.
            Example: (39, 26)
        sector_x_gradient (lt.Gradient): Distribution method for horizontal sectors.
            Example: Linear distribution with front remainder distribution
        sector_y_gradient (lt.Gradient): Distribution method for vertical sectors.
            Example: Linear distribution with front remainder distribution
        sector_average_function (lt.Average): Method for averaging pixels in each sector.
            Example: Mean averaging of pixels in each sector
        smoothing_function (lt.Smoothing): Smoothing algorithm applied to the correction table.
            Example: Median blur with kernel size 3

    Debug Options:
        Supports debug visualization via lt.Debug.Plot for plotting intermediate
        results for analysis and verification.
    """
    hr_name = 'LSC (NXP Neo ISP)'
    out_name = 'LensShadingCorrection'
    compatible = ['nxpneo']

    def __init__(self, *args, **kwargs):
        super().__init__(**kwargs)

    # We don't actually need anything from the config file
    def validate_config(self, config: dict) -> bool:
        return True

    # @return Image color temperature, flattened array of red calibration table
    #         (containing {sector size} elements), flattened array of blue
    #         calibration table, flattened array of (red's) green calibration
    #         table, flattened array of (blue's) green calibration table

    def _do_single_lsc(self, image: lt.Image):
        # Perform CSC on each channel
        csc_r, csc_g, csc_b = self._do_csc(image)

        # Apply CSC LUTs to the image
        self._apply_csc_lut_to_image_inplace(
            image, csc_r, csc_g, csc_b, self.sector_shape)
        # Perform LSC on green channels
        cgr, _ = self._lsc_single_channel(image.channels[lt.Color.GR], image)
        cgb, _ = self._lsc_single_channel(image.channels[lt.Color.GB], image)
        cg = 0.5 * (cgr + cgb)

        # Merge CSC and LSC LUTs
        merged_r = csc_r * cg
        merged_g = csc_g * cg
        merged_b = csc_b * cg

        return image.color, merged_r.flatten(), merged_b.flatten(), merged_g.flatten()

    # Color Shading Correction is performed before LSC.
    # @return Arrays of R, G and B correction values for CSC.
    #
    # The function computes CSC gains based on ratio between reference G/R and
    # mean block G/R (G/B respectively) so the resulting LUTs are preserving the
    # color ratios across the images. Green gains remain 1.0.
    #
    def _do_csc(self, image: lt.Image, reference_crop_size: int = 64):
        # Get channels
        r = image.channels[lt.Color.R]
        b = image.channels[lt.Color.B]
        g = 0.5 * (image.channels[lt.Color.GR] + image.channels[lt.Color.GB])

        # Get center crop of the image
        h, w = g.shape
        cx, cy = w // 2, h // 2
        half_crop = reference_crop_size // 2

        # Reference crop color values
        ref_r = r[cy - half_crop:cy + half_crop, cx - half_crop:cx + half_crop]
        ref_b = b[cy - half_crop:cy + half_crop, cx - half_crop:cx + half_crop]
        ref_g = g[cy - half_crop:cy + half_crop, cx - half_crop:cx + half_crop]

        ref_r = np.clip(ref_r, 1e-6, None)
        ref_b = np.clip(ref_b, 1e-6, None)
        ref_g = np.clip(ref_g, 1e-6, None)

        # Reference ratios G/R, G/B
        ref_ratio_gr = np.mean(ref_g / ref_r)
        ref_ratio_gb = np.mean(ref_g / ref_b)

        # Gain tables initialization
        table_r = np.zeros(self.sector_shape[::-1])
        table_b = np.zeros(self.sector_shape[::-1])
        table_g = np.zeros(self.sector_shape[::-1])

        block_h = h // self.sector_shape[1]
        block_w = w // self.sector_shape[0]

        # Gain is computed for each block separately
        for j in range(self.sector_shape[1]):
            for i in range(self.sector_shape[0]):
                x0, x1 = i * block_w, (i + 1) * block_w
                y0, y1 = j * block_h, (j + 1) * block_h

                blk_r = np.clip(r[y0:y1, x0:x1], 1e-6, None)
                blk_b = np.clip(b[y0:y1, x0:x1], 1e-6, None)
                blk_g = np.clip(g[y0:y1, x0:x1], 1e-6, None)

                try:
                    # Get mean block ratios
                    blk_ratio_gr = np.mean(blk_g / blk_r)
                    blk_ratio_gb = np.mean(blk_g / blk_b)

                    # Gains are computed as reference ratio to block
                    # ratio to preserve the color uniformity
                    gain_r = ref_ratio_gr / blk_ratio_gr
                    gain_b = ref_ratio_gb / blk_ratio_gb
                    gain_g = np.float64(1.0)

                except Exception:
                    gain_r = gain_b = gain_g = 1.0

                table_r[j, i] = gain_r
                table_b[j, i] = gain_b
                table_g[j, i] = gain_g

        return table_r, table_g, table_b

    # The function applies LUTs to the image channels and overwrite its values
    #
    # The tables contain the gains for each block, so the color values are
    # multiplied by the given gain.
    #
    def _apply_csc_lut_to_image_inplace(self, image: lt.Image, lut_r, lut_g, lut_b, sector_shape):
        h, w = image.channels[lt.Color.R].shape
        block_h = h // sector_shape[1]
        block_w = w // sector_shape[0]

        for color, lut in zip(
            [lt.Color.R, lt.Color.GB, lt.Color.GR, lt.Color.B],
            [lut_r, lut_g, lut_g, lut_b]
        ):
            channel = image.channels[color].astype(np.float32)

            # Channel values are divided into the blocks according to the LUT size
            # and corresponding gains are applied
            for j in range(sector_shape[1]):
                for i in range(sector_shape[0]):
                    x0, x1 = i * block_w, (i + 1) * block_w
                    y0, y1 = j * block_h, (j + 1) * block_h
                    gain = lut[j, i]
                    channel[y0:y1, x0:x1] *= gain

            image.channels[color] = channel

    # Main function performing LSC calibration on input images.
    # @return list of LUT entries for different color temperatures
    #
    # Applies LSC calibration to each image using _do_single_lsc, extracts color
    # temperature and gain tables for red, green, and blue channels. Averages gain
    # tables for images with the same color temperature.
    #
    def _do_all_lsc(self, images: list) -> list:
        output_list = []
        # Maximal gain value so we do not overcompensate
        max_compensation = 4.0
        # Coefficient corresponding to the U3.7 format
        convert_coef = 128
        def output_map_func(values): return np.clip(
            np.round(values * convert_coef), convert_coef, max_compensation * convert_coef)

        # List of colour temperatures
        list_col = []
        # Associated calibration tables
        list_cr = []
        list_cb = []
        list_cg = []

        # Do the LSC calibration for each image
        for image in self._enumerate_lsc_images(images):
            col, cr, cb, cg = self._do_single_lsc(image)
            list_col.append(col)
            list_cr.append(cr)
            list_cb.append(cb)
            list_cg.append(cg)

        # Convert to numpy array for data manipulation
        list_col = np.array(list_col)
        list_cr = np.array(list_cr)
        list_cb = np.array(list_cb)
        list_cg = np.array(list_cg)

        for color_temperature in sorted(set(list_col)):
            # Average tables for the same colour temperature
            indices = np.where(list_col == color_temperature)
            color_temperature = int(color_temperature)

            tables = []
            for lis in [list_cr, list_cg, list_cb]:
                table = np.mean(lis[indices], axis=0)
                # Scale the gains to U3.7 format
                table = output_map_func(table)
                # Pad the LUT with zeros to have expected 1024 values
                table = np.pad(
                    table, (0, 1024 - table.shape[0]), mode='constant')
                table = np.round(table).astype('int32').tolist()
                tables.append(table)

            entry = {
                'ct': color_temperature,
                'r': tables[0],
                'g': tables[1],
                'b': tables[2],
            }

            output_list.append(entry)

        return output_list

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        output = {'profiles': []}

        # Group images by resolution
        resolution_groups = {}
        for image in images:
            res_key = (image.w, image.h)
            if res_key not in resolution_groups:
                resolution_groups[res_key] = []
            resolution_groups[res_key].append(image)

        # Process each resolution group separately
        for (w, h), group_images in resolution_groups.items():
            sets = self._do_all_lsc(group_images)
            if not sets:
                continue

            profile = {
                'resolution': [w, h],
                'block-count': [self.sector_shape[0], self.sector_shape[1]],
                'sets': sets,
            }
            output['profiles'].append(profile)

        if not output['profiles']:
            return None

        return output
