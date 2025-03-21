// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Image transport codec working with QOI format.
 * \author Martin Pecka
 */

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_topic_tools/shape_shifter.h>
#include <qoi_image_transport/qoi.h>
#include <qoi_image_transport/qoi_codec.h>

namespace qoi_image_transport
{

namespace enc = sensor_msgs::image_encodings;

struct QoiCodecPrivate
{
};

QoiCodec::QoiCodec(const ::cras::LogHelperPtr& logHelper) : ImageTransportCodec(logHelper),
  data(new QoiCodecPrivate)
{
}

QoiCodec::~QoiCodec() = default;

QoiCodec::EncodeResult QoiCodec::encode(const sensor_msgs::Image& raw) const
{
  if (!enc::isColor(raw.encoding))
    return cras::make_unexpected("QOI codec only supports RGB or RGBA images");
  if (enc::bitDepth(raw.encoding) != 8)
    return cras::make_unexpected("QOI codec only supports 8 bits images");
  const auto numChannels = enc::numChannels(raw.encoding);
  if (numChannels != 3 && numChannels != 4)
    return cras::make_unexpected("QOI codec only support 3- or 4-channel images");

  sensor_msgs::CompressedImage compressed;
  compressed.header = raw.header;
  compressed.format = std::string("qoi; ") + raw.encoding;

  ::qoi_desc desc;
  desc.width = raw.width;
  desc.height = raw.height;
  desc.channels = numChannels;
  desc.colorspace = QOI_SRGB;

  int compressedSize {0};
  auto* encodedData = static_cast<uint8_t*>(::qoi_encode(raw.data.data(), &desc, &compressedSize));
  if (encodedData == nullptr)
    return cras::make_unexpected("QOI encoding failed.");

  compressed.data.assign(encodedData, encodedData + compressedSize);
  ::free(encodedData);

  return compressed;
}

QoiCodec::DecodeResult QoiCodec::decode(const sensor_msgs::CompressedImage& compressed) const
{
  auto encoding = enc::BGR8;
  if (!compressed.format.empty())
  {
    const auto formatParts = cras::split(compressed.format, "; ", 1);
    if (formatParts.size() != 2 || formatParts[0] != "qoi")
      return cras::make_unexpected("Invalid QOI image format: " + compressed.format);

    encoding = formatParts[1];
  }

  sensor_msgs::Image image;
  image.header = compressed.header;
  image.encoding = encoding;
  image.is_bigendian = 1;

  ::qoi_desc desc;
  auto* rawData = static_cast<uint8_t*>(
    ::qoi_decode(compressed.data.data(), static_cast<int>(compressed.data.size()), &desc, 0));
  if (rawData == nullptr)
    return cras::make_unexpected("QOI decoding failed.");

  const auto numRawBytes = desc.width * desc.height * desc.channels;
  image.width = desc.width;
  image.height = desc.height;
  image.step = desc.width * desc.channels;
  image.data.assign(rawData, rawData + numRawBytes);
  ::free(rawData);

  if (image.width > 0 && image.height > 0)
    return image;

  return cras::make_unexpected("Decoding compressed image yielded a zero-size result.");
}

std::string QoiCodec::getTransportName() const
{
  return "qoi";
}

image_transport_codecs::ImageTransportCodec::EncodeResult QoiCodec::encode(
  const sensor_msgs::Image& raw, const dynamic_reconfigure::Config& config) const
{
  const auto compressedImage = this->encode(raw);
  if (!compressedImage)
    return cras::make_unexpected(compressedImage.error());

  cras::ShapeShifter compressed;
  cras::msgToShapeShifter(compressedImage.value(), compressed);
  return compressed;
}

image_transport_codecs::ImageTransportCodec::DecodeResult QoiCodec::decode(
  const topic_tools::ShapeShifter& compressed, const dynamic_reconfigure::Config& config) const
{
  sensor_msgs::CompressedImageConstPtr compressedImage;
  try
  {
    compressedImage = compressed.instantiate<sensor_msgs::CompressedImage>();
  }
  catch (const ros::Exception& e)
  {
    return cras::make_unexpected(cras::format("Invalid shapeshifter passed to compressed decoder: %s.", e.what()));
  }

  return this->decode(*compressedImage);
}

image_transport_codecs::ImageTransportCodec::GetCompressedContentResult QoiCodec::getCompressedImageContent(
  const topic_tools::ShapeShifter& compressed, const std::string& matchFormat) const
{
  sensor_msgs::CompressedImageConstPtr compressedImage;
  try
  {
    compressedImage = compressed.instantiate<sensor_msgs::CompressedImage>();
  }
  catch (const ros::Exception& e)
  {
    return cras::make_unexpected(cras::format("Invalid shapeshifter passed to compressed decoder: %s.", e.what()));
  }
  return this->getCompressedImageContent(*compressedImage, matchFormat);
}

image_transport_codecs::ImageTransportCodec::GetCompressedContentResult QoiCodec::getCompressedImageContent(
  const sensor_msgs::CompressedImage& compressed, const std::string& matchFormat) const
{
  if (!cras::startsWith(cras::toLower(compressed.format), cras::toLower(matchFormat)))
    return cras::nullopt;

  return image_transport_codecs::CompressedImageContent{"qoi", compressed.data};
}

thread_local auto globalLogger = std::make_shared<cras::MemoryLogHelper>();
thread_local QoiCodec qoi_codec_instance(globalLogger);
}

bool qoiCodecEncode(
  sensor_msgs::Image::_height_type rawHeight,
  sensor_msgs::Image::_width_type rawWidth,
  const char* rawEncoding,
  sensor_msgs::Image::_is_bigendian_type rawIsBigEndian,
  sensor_msgs::Image::_step_type rawStep,
  size_t rawDataLength,
  const uint8_t rawData[],
  cras::allocator_t compressedFormatAllocator,
  cras::allocator_t compressedDataAllocator,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator)
{
  sensor_msgs::Image raw;
  raw.height = rawHeight;
  raw.width = rawWidth;
  raw.encoding = rawEncoding;
  raw.is_bigendian = rawIsBigEndian;
  raw.step = rawStep;
  raw.data.resize(rawDataLength);
  memcpy(raw.data.data(), rawData, rawDataLength);

  qoi_image_transport::globalLogger->clear();

  const auto compressed = qoi_image_transport::qoi_codec_instance.encode(raw);

  for (const auto& msg : qoi_image_transport::globalLogger->getMessages())
    cras::outputRosMessage(logMessagesAllocator, msg);
  qoi_image_transport::globalLogger->clear();

  if (!compressed)
  {
    cras::outputString(errorStringAllocator, compressed.error());
    return false;
  }

  cras::outputString(compressedFormatAllocator, compressed->format);
  cras::outputByteBuffer(compressedDataAllocator, compressed->data);

  return true;
}

bool qoiCodecDecode(
  const char* compressedFormat,
  size_t compressedDataLength,
  const uint8_t compressedData[],
  sensor_msgs::Image::_height_type& rawHeight,
  sensor_msgs::Image::_width_type& rawWidth,
  cras::allocator_t rawEncodingAllocator,
  sensor_msgs::Image::_is_bigendian_type& rawIsBigEndian,
  sensor_msgs::Image::_step_type& rawStep,
  cras::allocator_t rawDataAllocator,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
)
{
  sensor_msgs::CompressedImage compressed;
  compressed.format = compressedFormat;
  compressed.data.resize(compressedDataLength);
  memcpy(compressed.data.data(), compressedData, compressedDataLength);

  qoi_image_transport::globalLogger->clear();

  const auto raw = qoi_image_transport::qoi_codec_instance.decode(compressed);

  for (const auto& msg : qoi_image_transport::globalLogger->getMessages())
    cras::outputRosMessage(logMessagesAllocator, msg);
  qoi_image_transport::globalLogger->clear();

  if (!raw)
  {
    cras::outputString(errorStringAllocator, raw.error());
    return false;
  }

  rawHeight = raw->height;
  rawWidth = raw->width;
  rawIsBigEndian = raw->is_bigendian;
  rawStep = raw->step;
  cras::outputString(rawEncodingAllocator, raw->encoding);
  cras::outputByteBuffer(rawDataAllocator, raw->data);

  return true;
}
