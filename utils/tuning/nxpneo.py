#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright 2025-2026 NXP
#
# Tuning script for NXP Neo ISP
#
# Based on rkisp1 tuning script rkisp1.py
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
# Copyright (C) 2024, Ideas On Board
#

import logging
import sys

import coloredlogs
import libtuning as lt
from libtuning.generators import YamlOutput
from libtuning.modules.lsc import LSCNxpNeo
from libtuning.modules.ccm import CCMNxpNeo
from libtuning.parsers import YamlParser

coloredlogs.install(level=logging.INFO,
                    fmt='%(name)s %(levelname)s %(message)s')

lsc = LSCNxpNeo(debug=[lt.Debug.Plot],
                # Configurable grid for lens shading correction tables up to 1024 blocks.
                # 39x26 = 1014 blocks maximizes coverage while maintaining 1.5 aspect ratio
                # (matching 1920x1280 sensors) for square-shaped blocks.
                # This grid works well for other resolutions as long as vignetting
                # patterns are smooth and don't require exact aspect matching.
                sector_shape=(39, 26),

                sector_x_gradient=lt.gradient.Linear(
                    lt.Remainder.DistributeFront),
                sector_y_gradient=lt.gradient.Linear(
                    lt.Remainder.DistributeFront),

                # This is the function that will be used to average the pixels in
                # each sector. This can also be a custom function.
                sector_average_function=lt.average.Mean(),

                # This is the function that will be used to smooth the color ratio
                # values.  This can also be a custom function.
                smoothing_function=lt.smoothing.MedianBlur(3),)
ccm = CCMNxpNeo(debug=[lt.Debug.Plot])

tuner = lt.Tuner('NxpNeo')
tuner.add([lsc, ccm])
tuner.set_input_parser(YamlParser())
tuner.set_output_formatter(YamlOutput())

tuner.set_output_order([lsc, ccm])

if __name__ == '__main__':
    sys.exit(tuner.run(sys.argv))
