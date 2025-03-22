// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for qoi_image_transport.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <string>
#include <utility>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <cras_cpp_common/string_utils.hpp>
#include <image_transport_codecs/image_transport_codecs.h>
#include <qoi_image_transport/qoi_codec.h>

using namespace image_transport_codecs;  // NOLINT(build/namespaces)
using namespace qoi_image_transport;  // NOLINT(build/namespaces)

TEST(QoiImageTransport, Qoi)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "bgr8";
  raw.width = raw.height = 2;
  raw.step = 6;
  raw.data = {0, 0, 0, 100, 100, 100, 200, 200, 200, 255, 255, 255};

  const auto compressedShifter = codecs.encode(raw, "qoi");
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
  EXPECT_EQ("qoi; bgr8", compressed->format);

  const auto raw2 = codecs.decode(compressedShifter.value(), "qoi");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  for (size_t i = 0; i < raw.data.size(); ++i)
  {
    EXPECT_EQ(raw2->data[i], raw.data[i]);
  }

  auto content = codecs.getCompressedImageContent(compressedShifter.value(), "qoi");
  ASSERT_TRUE(content);
  ASSERT_TRUE(content->has_value());
  EXPECT_EQ("qoi", (*content)->format);
  EXPECT_EQ(compressed->data, (*content)->data);

  content = codecs.getCompressedImageContent(compressedShifter.value(), "qoi", "qoi");
  ASSERT_TRUE(content);
  ASSERT_TRUE(content->has_value());
  EXPECT_EQ("qoi", (*content)->format);
  EXPECT_EQ(compressed->data, (*content)->data);

  content = codecs.getCompressedImageContent(compressedShifter.value(), "qoi", "jp2");
  ASSERT_TRUE(content);
  EXPECT_FALSE(content->has_value());
}

TEST(QoiImageTransport, CompressedWrongType)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "32FC1";
  raw.width = raw.height = 2;
  raw.step = 8;
  float floats[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  auto bytes = reinterpret_cast<uint8_t*>(floats);
  raw.data.resize(16);
  memcpy(&raw.data[0], bytes, 16);

  const auto compressedShifter = codecs.encode(raw, "qoi");
  ASSERT_FALSE(compressedShifter);
  ASSERT_NE("", compressedShifter.error());
}

TEST(QoiImageTransport, Bag)
{
  ImageTransportCodecs codecs;

  rosbag::Bag rawBag(std::string(TEST_DATA_DIR) + "/raw.bag");
  rosbag::Bag compressedBag(std::string(TEST_DATA_DIR) + "/compressed.bag");

  sensor_msgs::Image handColorRaw;

  sensor_msgs::CompressedImage handColorCompressed;

  for (const auto& data : rosbag::View(rawBag))
  {
    auto msgPtr = data.instantiate<sensor_msgs::Image>();
    ASSERT_NE(nullptr, msgPtr);
    msgPtr->header.seq = 0;  // seq number might differ, so we zero it out
    msgPtr->is_bigendian = 0;  // Spot driver sets some images to 1 for some reason
    if (data.getTopic() == "/spot/camera/hand_color/image")
      handColorRaw = *msgPtr;
  }

  for (const auto& data : rosbag::View(compressedBag))
  {
    auto msgPtr = data.instantiate<sensor_msgs::CompressedImage>();
    ASSERT_NE(nullptr, msgPtr);
    msgPtr->header.seq = 0;  // seq number might differ, so we zero it out
    if (data.getTopic() == "/spot/camera/hand_color/image/qoi")
      handColorCompressed = *msgPtr;
  }

  for (size_t i = 0; i < 3; ++i)  // test several iterations
  {
    const auto compressedShifter = codecs.encode(handColorRaw, "qoi");
    ASSERT_TRUE(compressedShifter);
    ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
    const auto compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
    EXPECT_EQ(handColorCompressed.header, compressed->header);
    EXPECT_EQ(handColorCompressed.format, compressed->format);
    EXPECT_EQ(handColorCompressed.data.size(), compressed->data.size());
    EXPECT_EQ(handColorCompressed.data, compressed->data);

    const auto rawImg = codecs.decodeTyped(*compressed, "qoi");
    ASSERT_TRUE(rawImg);
    EXPECT_EQ(handColorRaw.header, rawImg->header);
    EXPECT_EQ(handColorRaw.step, rawImg->step);
    EXPECT_EQ(handColorRaw.width, rawImg->width);
    EXPECT_EQ(handColorRaw.height, rawImg->height);
    EXPECT_EQ(handColorRaw.encoding, rawImg->encoding);
    EXPECT_EQ(handColorRaw.is_bigendian, rawImg->is_bigendian);
    ASSERT_EQ(handColorRaw.data.size(), rawImg->data.size());
    ASSERT_EQ(handColorRaw.data, rawImg->data);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
