#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright 2025-2026 NXP
#
# CCM module for tuning NXP Neo ISP
#
# Based on CCM module for tuning rkisp1 - rkisp1.py
# Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
# Copyright (C) 2024, Ideas on Board
#

from .ccm import CCM


class CCMNxpNeo(CCM):
    """
    Color Correction Matrix (CCM) module.

    This module performs color correction calibration to ensure accurate color
    reproduction across different lighting conditions.

    Debug Options:
        Supports debug visualization via lt.Debug.Plot for plotting intermediate
        results for analysis and verification.
    """
    hr_name = 'Crosstalk Correction (NXP Neo ISP)'
    out_name = 'Ccm'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # We don't need anything from the config file.
    def validate_config(self, config: dict) -> bool:
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        output = {}

        ccms = self.do_calibration(images)
        output['ccms'] = ccms

        return output
