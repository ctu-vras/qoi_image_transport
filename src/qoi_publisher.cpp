// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin for qoi transport codec.
 * \author Martin Pecka
 */

#include <image_transport/publisher_plugin.h>
#include <image_transport/simple_publisher_plugin.h>
#include <pluginlib/class_list_macros.hpp>
#include <qoi_image_transport/qoi_codec.h>
#include <sensor_msgs/CompressedImage.h>

namespace qoi_image_transport
{

class Publisher final : public image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage>
{
public:
  ~Publisher() override = default;

  std::string getTransportName() const override
  {
    return this->codec.getTransportName();
  }

protected:
  void publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const override;

private:
  QoiCodec codec;
};

void Publisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
  this->codec.encode(message)
    .or_else([](const std::string& error) { ROS_ERROR("Failed to encode image using QOI codec: %s", error.c_str()); })
    .map(publish_fn);
}

}

PLUGINLIB_EXPORT_CLASS(qoi_image_transport::Publisher, image_transport::PublisherPlugin)
