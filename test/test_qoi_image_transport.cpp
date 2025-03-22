// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for qoi_image_transport.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>
#include <cras_cpp_common/log_utils/macros.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

TEST(QoiImageTransport, Roundtrip)  // NOLINT
{
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

  sensor_msgs::CompressedImageConstPtr receivedQoiMsg;
  sensor_msgs::ImageConstPtr receivedRawMsg;

  ros::NodeHandle nh;
  auto rawPub = nh.advertise<sensor_msgs::Image>("/spot/camera/hand_color/image", 1);
  auto qoiSub = nh.subscribe<sensor_msgs::CompressedImage>("/republished/qoi", 1,
    [&](const sensor_msgs::CompressedImageConstPtr& msg)
    {
      receivedQoiMsg = msg;
    });
  auto raw2Sub = nh.subscribe<sensor_msgs::Image>("/republished2", 1,
    [&](const sensor_msgs::ImageConstPtr& msg)
    {
      receivedRawMsg = msg;
    });

  for (size_t i = 0; i < 100; i++)
  {
    if (rawPub.getNumSubscribers() > 0 && qoiSub.getNumPublishers() > 0 && raw2Sub.getNumPublishers() > 0)
      break;
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(1.0, "Waiting for subscribers and publishers");
    ros::WallDuration(0.1).sleep();
  }

  for (size_t i = 0; i < 100; i++)
  {
    rawPub.publish(handColorRaw);
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(1.0, "Waiting for messages");
    ros::WallDuration(0.1).sleep();
    if (receivedQoiMsg != nullptr && receivedRawMsg != nullptr)
      break;
  }

  ASSERT_NE(nullptr, receivedQoiMsg);
  ASSERT_NE(nullptr, receivedRawMsg);

  EXPECT_EQ(handColorCompressed.header, receivedQoiMsg->header);
  EXPECT_EQ(handColorCompressed.format, receivedQoiMsg->format);
  EXPECT_EQ(handColorCompressed.data.size(), receivedQoiMsg->data.size());
  EXPECT_EQ(handColorCompressed.data, receivedQoiMsg->data);

  EXPECT_EQ(handColorRaw.header, receivedRawMsg->header);
  EXPECT_EQ(handColorRaw.step, receivedRawMsg->step);
  EXPECT_EQ(handColorRaw.width, receivedRawMsg->width);
  EXPECT_EQ(handColorRaw.height, receivedRawMsg->height);
  EXPECT_EQ(handColorRaw.encoding, receivedRawMsg->encoding);
  EXPECT_EQ(handColorRaw.is_bigendian, receivedRawMsg->is_bigendian);
  ASSERT_EQ(handColorRaw.data.size(), receivedRawMsg->data.size());
  ASSERT_EQ(handColorRaw.data, receivedRawMsg->data);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_qoi_image_transport");

  ros::NodeHandle nh;  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}
