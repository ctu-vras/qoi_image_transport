// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin for qoi transport codec.
 * \author Martin Pecka
 */

#include <image_transport/simple_subscriber_plugin.h>
#include <pluginlib/class_list_macros.hpp>
#include <qoi_image_transport/qoi_codec.h>
#include <sensor_msgs/CompressedImage.h>

namespace qoi_image_transport
{

class Subscriber final : public image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>
{
public:
  ~Subscriber() override = default;

  std::string getTransportName() const override
  {
    return this->codec.getTransportName();
  }

protected:
  void internalCallback(const sensor_msgs::CompressedImageConstPtr& message, const Callback& user_cb) override;

private:
  QoiCodec codec;
};

void Subscriber::internalCallback(const sensor_msgs::CompressedImageConstPtr& message, const Callback& user_cb)
{
  auto rawImageOrError = this->codec.decode(*message);
  if (!rawImageOrError.has_value())
  {
    ROS_ERROR("Error decoding QOI image: %s", rawImageOrError.error().c_str());
    return;
  }

  const auto rawImage = boost::make_shared<sensor_msgs::Image const>(std::move(*rawImageOrError));
  user_cb(rawImage);
}

}

PLUGINLIB_EXPORT_CLASS(qoi_image_transport::Subscriber, image_transport::SubscriberPlugin)
