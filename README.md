<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# QOI Image Transport

This package provides ROS image transport "qoi" which uses [QOI image format](https://qoiformat.org/).

This format is a lossless codec for RGB or RGBA images with similar compression ratio as lossless PNG, but orders of magnitude faster.

This image transport also provides the [image_transport_codecs](https://wiki.ros.org/image_transport_codecs) interface.
This means it is possible to directly encode/decode images from C++/Python code without the need for going through
a pub/sub cycle. You don't even need a ROS master for the conversion.

A benchmark compared to PNG is available [here](https://qoiformat.org/benchmark/).

## Build Status

[![CI](https://github.com/ctu-vras/qoi_image_transport/actions/workflows/ci.yaml/badge.svg)](https://github.com/ctu-vras/qoi_image_transport/actions/workflows/ci.yaml)