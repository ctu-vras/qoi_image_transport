#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for qoi_image_transport."""

import os
import struct
import unittest
import sys

from rosbag import Bag
from sensor_msgs.msg import CompressedImage, Image

from qoi_image_transport import qoi_codec
from image_transport_codecs import decode, encode, get_compressed_image_content, CompressedImageContent


def _byte(b):
    if sys.version_info[0] == 2:
        return ord(b)
    return b


def bytes_to_float(b):
    if len(b) == 4:
        return struct.unpack('<f', b)[0]
    return struct.unpack('<%if' % (len(b) / 4,), b)


def float_to_bytes(f):
    if isinstance(f, float):
        return bytes(struct.pack('<f',  f))
    return bytes(struct.pack('<%if' % (len(f),), *f))


class CodecTest(unittest.TestCase):

    def test_qoi(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "bgr8"
        raw.width = raw.height = 2
        raw.step = 6
        raw.data = b'\x00\x00\x00\x64\x64\x64\xc8\xc8\xc8\xff\xff\xff'

        compressed, err = encode(raw, "qoi")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        self.assertEqual(compressed.format, "qoi; bgr8")

        raw2, err = decode(compressed, "qoi")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        self.assertEqual(raw2.data, raw.data)

        content, err = get_compressed_image_content(compressed, "qoi")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "qoi")
        self.assertEqual(content.data, compressed.data)

        content, err = get_compressed_image_content(compressed, "qoi", "jp2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_compressed_wrong_type(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "32FC1"
        raw.width = raw.height = 2
        raw.step = 8
        raw.data = float_to_bytes((1.0, 2.0, 3.0, 4.0))

        compressed, err = encode(raw, "qoi")
        self.assertNotEqual(err, "")
        self.assertIsNone(compressed)

    def test_bag(self):
        d = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
        raw_bag = os.path.join(d, "raw.bag")
        compressed_bag = os.path.join(d, "compressed.bag")

        with Bag(raw_bag) as bag:
            for topic, msg, _ in bag.read_messages():
                msg.header.seq = 0  # seq number might differ, so we zero it out
                msg.is_bigendian = 0  # Spot driver sets some images to 1 for some reason
                if topic == "/spot/camera/hand_color/image":
                    hand_color_raw = msg

        with Bag(compressed_bag) as bag:
            for topic, msg, _ in bag.read_messages():
                msg.header.seq = 0  # seq number might differ, so we zero it out
                if topic == "/spot/camera/hand_color/image/qoi":
                    hand_color_compressed = msg

        # Hand color

        for it in range(3):  # Test several iterations
            compressed, err = encode(hand_color_raw, "qoi")
            self.assertEqual(err, "")
            self.assertIsNotNone(compressed)
            self.assertIsInstance(compressed, CompressedImage)
            self.assertEqual(compressed.header.stamp, hand_color_compressed.header.stamp)
            self.assertEqual(compressed.header.frame_id, hand_color_compressed.header.frame_id)
            self.assertEqual(compressed.format, hand_color_compressed.format)
            self.assertEqual(compressed.data, hand_color_compressed.data)

            raw2, err = decode(compressed, "qoi")
            self.assertEqual(err, "")
            self.assertIsNotNone(raw2)
            self.assertIsInstance(raw2, Image)
            self.assertEqual(raw2.header, hand_color_raw.header)
            self.assertEqual(raw2.step, hand_color_raw.step)
            self.assertEqual(raw2.width, hand_color_raw.width)
            self.assertEqual(raw2.height, hand_color_raw.height)
            self.assertEqual(raw2.encoding, hand_color_raw.encoding)
            self.assertEqual(raw2.is_bigendian, hand_color_raw.is_bigendian)
            self.assertEqual(raw2.data, hand_color_raw.data)


if __name__ == '__main__':
    unittest.main()
