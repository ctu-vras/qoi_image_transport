# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Encoding and decoding of images compressed with the 'qoi' transport."""

from ctypes import RTLD_GLOBAL, c_bool, c_uint8, c_uint32, c_char_p, c_size_t, POINTER, byref
import time

from sensor_msgs.msg import CompressedImage, Image

from cras.ctypes_utils import load_library, Allocator, StringAllocator, BytesAllocator, LogMessagesAllocator, \
    get_ro_c_buffer


__codec = None


def __get_library():
    global __codec
    if __codec is None:
        if load_library('image_transport_codecs', mode=RTLD_GLOBAL) is not None:
            __codec = load_library('qoi_codec')
            if __codec is None:
                return None

            # Add function signatures

            __codec.qoiCodecEncode.restype = c_bool
            __codec.qoiCodecEncode.argtypes = [
                c_uint32, c_uint32, c_char_p, c_uint8, c_uint32, c_size_t, POINTER(c_uint8),
                Allocator.ALLOCATOR, Allocator.ALLOCATOR,
                Allocator.ALLOCATOR, Allocator.ALLOCATOR
            ]

            __codec.qoiCodecDecode.restype = c_bool
            __codec.qoiCodecDecode.argtypes = [
                c_char_p, c_size_t, POINTER(c_uint8),
                POINTER(c_uint32), POINTER(c_uint32), Allocator.ALLOCATOR, POINTER(c_uint8), POINTER(c_uint32),
                Allocator.ALLOCATOR,
                Allocator.ALLOCATOR, Allocator.ALLOCATOR,
            ]

    return __codec


def encode(raw, config=None):
    """Encode the given raw image into a :sensor_msgs:`CompressedImage` with "qoi" codec.

    :param sensor_msgs.msg.Image raw: The raw image.
    :param dict config: Not used.
    :return: Tuple of compressed image and error string. If the compression fails (e.g. wrong image dimensions or bit
             depth), the image is `None` and error string is filled.
    :rtype: (sensor_msgs.msg.CompressedImage or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load qoi codec library."

    format_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    args = [
        raw.height, raw.width, raw.encoding.encode("utf-8"), raw.is_bigendian, raw.step, len(raw.data),
        get_ro_c_buffer(raw.data), format_allocator.get_cfunc(), data_allocator.get_cfunc(),
        error_allocator.get_cfunc(), log_allocator.get_cfunc()
    ]
    ret = codec.qoiCodecEncode(*args)

    log_allocator.print_log_messages()
    if ret:
        compressed = CompressedImage()
        compressed.header = raw.header
        compressed.format = format_allocator.value
        compressed.data = data_allocator.value
        return compressed, ""
    return None, error_allocator.value


def decode(compressed, config=None):
    """Decode the given :sensor_msgs:`CompressedImage` encoded with "qoi" codec into a raw image.

    :param sensor_msgs.msg.CompressedImage compressed: The compressed image.
    :param dict config: Not used.
    :return: Tuple of raw image and error string. If decoding failed, the image is `None` and error string is filled.
    :rtype: (sensor_msgs.msg.Image or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    encoding_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    raw_height = c_uint32()
    raw_width = c_uint32()
    raw_is_big_endian = c_uint8()
    raw_step = c_uint32()

    args = [
        compressed.format.encode("utf-8"), len(compressed.data), get_ro_c_buffer(compressed.data),
        byref(raw_height), byref(raw_width), encoding_allocator.get_cfunc(), byref(raw_is_big_endian), byref(raw_step),
        data_allocator.get_cfunc(),
        error_allocator.get_cfunc(), log_allocator.get_cfunc(),
    ]

    ret = codec.qoiCodecDecode(*args)

    log_allocator.print_log_messages()
    if ret:
        raw = Image()
        raw.header = compressed.header
        raw.height = raw_height.value
        raw.width = raw_width.value
        raw.encoding = encoding_allocator.value
        raw.is_bigendian = raw_is_big_endian.value
        raw.step = raw_step.value
        raw.data = data_allocator.value
        return raw, ""
    return None, error_allocator.value


if __name__ == '__main__':
    def main():
        import rospy
        raw = Image()
        raw.header.stamp = rospy.Time(10)
        raw.header.frame_id = "test"
        raw.encoding = 'bgr8'
        raw.step = 6
        raw.width = raw.height = 2
        raw.data = [0, 0, 0, 100, 100, 100, 200, 200, 200, 255, 255, 255]

        rospy.init_node("test")
        pub = rospy.Publisher("test/compressed", CompressedImage, queue_size=1, latch=True)
        pub2 = rospy.Publisher("test", Image, queue_size=1, latch=True)
        time.sleep(1)

        compressed, err = encode(raw, {})

        print(bool(compressed), err)
        if compressed is not None:
            pub.publish(compressed)

        raw, err = decode(compressed)

        print(bool(raw), err)
        if raw is not None:
            pub2.publish(raw)

        rospy.spin()

    main()
