#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Image transport codec working with QOI format.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <dynamic_reconfigure/Config.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <image_transport_codecs/image_transport_codec.h>

namespace qoi_image_transport
{

struct QoiCodecPrivate;

/**
 * \brief Image transport codec for QOI format.
 *
 * This codec exposes the functionality of `qoi_image_transport` so that it can be used directly without the
 * need to go through running a node and publishing on a topic. E.g.
 *
 * ```
 * sensor_msgs::Image raw = ...;
 * qoi_image_transport::QoiCodec codec;
 * auto result = codec.encode(raw);
 * if (!result)
 * {
 *   ROS_ERROR_STREAM("Compression failed: " << result.error());
 *   return false;
 * }
 * sensor_msgs::CompressedImage compressed = result.value();
 * ```
 */
class QoiCodec : public image_transport_codecs::ImageTransportCodec
{
public:
  //! \brief Result of image encoding. Either a `sensor_msgs::CompressedImage` message, or error message.
  typedef cras::expected<sensor_msgs::CompressedImage, std::string> EncodeResult;

  /**
   * \brief Create an instance of the codec.
   * \param[in] logHelper The logger to use for error messages not directly related to the currently processed image.
   */
  explicit QoiCodec(const cras::LogHelperPtr& logHelper = std::make_shared<cras::NodeLogHelper>());

  ~QoiCodec() override;

  /**
   * \brief Encode the given raw image using the given publisher config.
   * \param[in] raw The raw image to encode.
   * \sa Corresponding C API function: `qoiCodecEncode()`.
   */
  EncodeResult encode(const sensor_msgs::Image& raw) const;

  /**
   * \brief Decode the given compressed image using the given subscriber config.
   * \param[in] compressed The image to decode.
   * \return The decoded raw image, or an error.
   * \sa Corresponding C API function: `qoiCodecDecode()`.
   */
  ImageTransportCodec::DecodeResult decode(const sensor_msgs::CompressedImage& compressed) const;

  std::string getTransportName() const override;

  ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
                                           const dynamic_reconfigure::Config& config) const override;

  ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                                           const dynamic_reconfigure::Config& config) const override;

  ImageTransportCodec::GetCompressedContentResult getCompressedImageContent(
    const topic_tools::ShapeShifter& compressed, const std::string& matchFormat) const override;

  /**
   * \brief This function returns the bytes of the actual QOI image.
   * \param[in] compressed The compressed image.
   * \param[in] matchFormat If nonempty, the image data is only returned if their `format` field would match the given
   *                        one. The matching should be case-insensitive.
   * \return The contained image bytes.
   */
  ImageTransportCodec::GetCompressedContentResult getCompressedImageContent(
    const sensor_msgs::CompressedImage& compressed, const std::string& matchFormat) const;

private:
  std::unique_ptr<QoiCodecPrivate> data;  //!< \brief Private implementation data
};

}

// ////////
// C API //
// ////////

/**
 * \brief Encode the given raw image using `qoi` codec with the given config.
 *
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 *
 * \param[in] rawHeight Raw image height, that is, number of rows.
 * \param[in] rawWidth Raw image width, that is, number of columns.
 * \param[in] rawEncoding Raw image encoding of pixels -- channel meaning, ordering, size.
 * \param[in] rawIsBigEndian Is raw image bigendian?
 * \param[in] rawStep Raw image full row length in bytes.
 * \param[in] rawDataLength Length of raw image data in bytes, should be `step * rows`.
 * \param[in] rawData The raw image bytes.
 * \param[in,out] compressedFormatAllocator Allocator for the `format` field of the compressed image.
 * \param[in,out] compressedDataAllocator Allocator for the byte data of the compressed image.
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the encoding has succeeded. If yes, `compressedFormatAllocator` and `compressedDataAllocator`
 *         allocate their buffers and write the output to them. If not, `errorStringAllocator` allocates its buffer
 *         and stores the error string in it.
 * \sa Corresponding C++ API function: `qoi_image_transport::QoiCodec::encode()`.
 */
extern "C" bool qoiCodecEncode(
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
  cras::allocator_t logMessagesAllocator
);

/**
 * \brief Decode the given compressed image using `qoi` codec with the given config.
 *
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 *
 * \param[in] compressedFormat The `format` field of the compressed image.
 * \param[in] compressedDataLength Length of the compressed image data in bytes.
 * \param[in] compressedData Bytes of the compressed image.
 * \param[out] rawHeight Raw image height, that is, number of rows.
 * \param[out] rawWidth Raw image width, that is, number of columns.
 * \param[in,out] rawEncodingAllocator Allocator for raw image encoding of pixels -- channel meaning, ordering, size.
 * \param[out] rawIsBigEndian Is raw image bigendian?
 * \param[out] rawStep Raw image full row length in bytes.
 * \param[in,out] rawDataAllocator Allocator for raw image bytes.
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the encoding has succeeded. If yes, output parameters are set, `rawEncodingAllocator` and
 *         `rawDataAllocator` allocate their buffers and write the output to them. If not, `errorStringAllocator`
 *         allocates its buffer and stores the error string in it.
 * \sa Corresponding C++ API function: `qoi_image_transport::QoiCodec::decode()`.
 */
extern "C" bool qoiCodecDecode(
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
);
