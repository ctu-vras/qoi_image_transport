// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin for qoi transport codec.
 * \author Martin Pecka
 */

#include <pluginlib/class_list_macros.h>

#include <image_transport_codecs/image_transport_codec_plugin.h>
#include <qoi_image_transport/qoi_codec.h>

namespace qoi_image_transport
{

class QoiCodecPlugin : public image_transport_codecs::ImageTransportCodecPluginBase<QoiCodec> {};

}

PLUGINLIB_EXPORT_CLASS(qoi_image_transport::QoiCodecPlugin, image_transport_codecs::ImageTransportCodecPlugin)