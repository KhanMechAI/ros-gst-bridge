/* gst_bridge
 * Copyright (C) 2020-2021 Brett Downing <brettrd@brettrd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
/**
 * SECTION:element-gstroscompimagesink
 *
 * The roscompimagesink element pipe video data into ROS2.
 *
 * <refsect2>>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v videotestsrc ! roscompimagesink node_name="gst_image" topic="/imagetopic"
 * ]|
 * Streams test tones as ROS image messages on topic.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/roscompimagesink.h>

#ifndef ROS_COMPRESSED_IMAGE_MSG_CAPS
#define ROS_COMPRESSED_IMAGE_MSG_CAPS          \
  "image/jpeg, "                               \
  "framerate = " GST_VIDEO_FPS_RANGE           \
  ", "                                         \
  "width = " GST_VIDEO_SIZE_RANGE              \
  ", "                                         \
  "height = " GST_VIDEO_SIZE_RANGE " ; "       \
  "image/png, "                                \
  "framerate = " GST_VIDEO_FPS_RANGE           \
  ", "                                         \
  "width = " GST_VIDEO_SIZE_RANGE              \
  ", "                                         \
  "height = " GST_VIDEO_SIZE_RANGE
#endif

GST_DEBUG_CATEGORY_STATIC(roscompimagesink_debug_category);
#define GST_CAT_DEFAULT roscompimagesink_debug_category

/* prototypes */

static void roscompimagesink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void roscompimagesink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec);

static void roscompimagesink_init(Roscompimagesink * sink);

static gboolean roscompimagesink_open(RosBaseSink * sink);
static gboolean roscompimagesink_close(RosBaseSink * sink);
static gboolean roscompimagesink_setcaps(GstBaseSink * gst_base_sink, GstCaps * caps);
static GstFlowReturn roscompimagesink_render(
  RosBaseSink * base_sink, GstBuffer * buffer, rclcpp::Time msg_time);

static void roscompimagesink_finalize(GObject * object);

enum {
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_FRAME_ID,
  PROP_ROS_ENCODING,
};

/* pad templates */

static GstStaticPadTemplate roscompimagesink_sink_template = GST_STATIC_PAD_TEMPLATE(
  "sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS(ROS_COMPRESSED_IMAGE_MSG_CAPS));

/* class initialization */

G_DEFINE_TYPE_WITH_CODE(
  Roscompimagesink, roscompimagesink, GST_TYPE_ROS_BASE_SINK,
  GST_DEBUG_CATEGORY_INIT(
    roscompimagesink_debug_category, "roscompimagesink", 0,
    "debug category for roscompimagesink element"))

static void roscompimagesink_class_init(RoscompimagesinkClass * klass)
{
  GObjectClass * object_class = G_OBJECT_CLASS(klass);
  GstElementClass * element_class = GST_ELEMENT_CLASS(klass);
  GstBaseSinkClass * basesink_class = GST_BASE_SINK_CLASS(klass);
  RosBaseSinkClass * ros_base_sink_class = GST_ROS_BASE_SINK_CLASS(klass);

  object_class->set_property = roscompimagesink_set_property;
  object_class->get_property = roscompimagesink_get_property;
  object_class->finalize = roscompimagesink_finalize;

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template(element_class, &roscompimagesink_sink_template);

  gst_element_class_set_static_metadata(
    element_class, "roscompimagesink", "Sink",
    "a gstreamer sink that publishes compressed image data into ROS",
    "Khan Schroder-Turner <khan@breakerindustries.com>");

  g_object_class_install_property(
    object_class, PROP_ROS_TOPIC,
    g_param_spec_string(
      "ros-topic", "pub-topic", "ROS topic to be published on", "gst_image",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_FRAME_ID,
    g_param_spec_string(
      "ros-frame-id", "frame-id", "frame_id of the image message", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_ENCODING,
    g_param_spec_string(
      "ros-encoding", "encoding-string", "A hack to flexibly set the encoding string", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  //access gstreamer base sink events here
  basesink_class->set_caps =
    GST_DEBUG_FUNCPTR(roscompimagesink_setcaps);  //gstreamer informs us what caps we're using.

  //supply the calls ros base sink needs to negotiate upstream formats and manage the publisher
  ros_base_sink_class->open =
    GST_DEBUG_FUNCPTR(roscompimagesink_open);  //let the base sink know how we register publishers
  ros_base_sink_class->close =
    GST_DEBUG_FUNCPTR(roscompimagesink_close);  //let the base sink know how we destroy publishers
  ros_base_sink_class->render =
    GST_DEBUG_FUNCPTR(roscompimagesink_render);  // gives us a buffer to package
}

static void roscompimagesink_init(Roscompimagesink * sink)
{
  RosBaseSink * ros_base_sink GST_ROS_BASE_SINK(sink);
  ros_base_sink->node_name = g_strdup("gst_compressed_image_sink_node");
  sink->pub_topic = g_strdup("gst_compressed_image_pub");
  sink->frame_id = g_strdup("image_frame");
  sink->encoding = g_strdup("");
  sink->init_caps = g_strdup("");
  sink->format = NULL;  // Initialize format to NULL
}

void roscompimagesink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec)
{
  RosBaseSink * ros_base_sink = GST_ROS_BASE_SINK(object);
  Roscompimagesink * sink = GST_ROSCOMPIMAGESINK(object);

  GST_DEBUG_OBJECT(sink, "set_property");

  switch (property_id) {
    case PROP_ROS_TOPIC:
      if (ros_base_sink->node_if) {
        RCLCPP_ERROR(
          ros_base_sink->node_if->logging->get_logger(), "can't change topic name once opened");
      } else {
        g_free(sink->pub_topic);
        sink->pub_topic = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_FRAME_ID:
      g_free(sink->frame_id);
      sink->frame_id = g_value_dup_string(value);
      break;

    case PROP_ROS_ENCODING:
      g_free(sink->encoding);
      sink->encoding = g_value_dup_string(value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

void roscompimagesink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec)
{
  Roscompimagesink * sink = GST_ROSCOMPIMAGESINK(object);

  GST_DEBUG_OBJECT(sink, "get_property");
  switch (property_id) {
    case PROP_ROS_TOPIC:
      g_value_set_string(value, sink->pub_topic);
      break;

    case PROP_ROS_FRAME_ID:
      g_value_set_string(value, sink->frame_id);
      break;

    case PROP_ROS_ENCODING:
      g_value_set_string(value, sink->encoding);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

/* open the device with given specs */
static gboolean roscompimagesink_open(RosBaseSink * ros_base_sink)
{
  Roscompimagesink * sink = GST_ROSCOMPIMAGESINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "open");
  rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();  //XXX add a parameter for overrides

  sink->pub = rclcpp::create_publisher<sensor_msgs::msg::CompressedImage>(
    ros_base_sink->node_if->parameters, ros_base_sink->node_if->topics, sink->pub_topic, qos);

  return TRUE;
}

/* close the device */
static gboolean roscompimagesink_close(RosBaseSink * ros_base_sink)
{
  Roscompimagesink * sink = GST_ROSCOMPIMAGESINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "close");
  sink->pub.reset();
  return TRUE;
}

/* check the caps, register a node and open an publisher */
static gboolean roscompimagesink_setcaps(GstBaseSink * gst_base_sink, GstCaps * caps)
{
  RosBaseSink * ros_base_sink = GST_ROS_BASE_SINK(gst_base_sink);
  Roscompimagesink * sink = GST_ROSCOMPIMAGESINK(ros_base_sink);

  GstStructure * caps_struct;
  const gchar * mime_type;

  GST_DEBUG_OBJECT(sink, "setcaps");

  if (!gst_caps_is_fixed(caps)) {
    RCLCPP_ERROR(ros_base_sink->node_if->logging->get_logger(), "caps is not fixed");
    return FALSE;
  }

  if (ros_base_sink->node_if)
    RCLCPP_INFO(
      ros_base_sink->node_if->logging->get_logger(), "preparing video with caps '%s'",
      gst_caps_to_string(caps));

  caps_struct = gst_caps_get_structure(caps, 0);
  
  // Get the mime type from the caps
  mime_type = gst_structure_get_name(caps_struct);
  
  if (!mime_type) {
    RCLCPP_ERROR(ros_base_sink->node_if->logging->get_logger(), "setcaps missing mime type");
    return FALSE;
  }
  
  // Check if the mime type is supported (image/jpeg or image/png)
  gboolean is_jpeg = (g_strcmp0(mime_type, "image/jpeg") == 0);
  gboolean is_png = (g_strcmp0(mime_type, "image/png") == 0);
  
  if (!is_jpeg && !is_png) {
    RCLCPP_ERROR(
      ros_base_sink->node_if->logging->get_logger(), 
      "Unsupported format: %s. Only image/jpeg and image/png are supported for CompressedImage", 
      mime_type);
    return FALSE;
  }

  // Free previous format string if it exists
  if (sink->format) {
    g_free(sink->format);
    sink->format = NULL;
  }

  // Store the format based on mime type
  if (is_jpeg) {
    sink->format = g_strdup("jpeg");
  } else if (is_png) {
    sink->format = g_strdup("png");
  }

  // Allow the encoding to be overridden by parameters
  // but update it if it's blank
  if (0 == g_strcmp0(sink->init_caps, "")) {
    g_free(sink->init_caps);
    sink->init_caps = gst_caps_to_string(caps);
  }
  
  if (0 == g_strcmp0(sink->encoding, "")) {
    g_free(sink->encoding);
    sink->encoding = g_strdup(sink->format);
  }

  RCLCPP_INFO(
    ros_base_sink->node_if->logging->get_logger(), 
    "Compressed image format: %s", 
    sink->format);

  return TRUE;
}

static GstFlowReturn roscompimagesink_render(
  RosBaseSink * ros_base_sink, GstBuffer * buf, rclcpp::Time msg_time)
{
  GstMapInfo info;
  sensor_msgs::msg::CompressedImage msg;

  Roscompimagesink * sink = GST_ROSCOMPIMAGESINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "render");

  // Extract the buffer timestamp if available
  if (GST_BUFFER_PTS_IS_VALID(buf)) {
    // Convert GStreamer timestamp (nanoseconds) to ROS time
    GstClockTime pts = GST_BUFFER_PTS(buf);
    int32_t sec = pts / GST_SECOND;
    uint32_t nanosec = pts % GST_SECOND;
    msg.header.stamp = rclcpp::Time(sec, nanosec);
    
    GST_DEBUG_OBJECT(sink, "Using buffer PTS timestamp: %lu ns (%d.%09u)", 
                    pts, sec, nanosec);
  } else if (GST_BUFFER_DTS_IS_VALID(buf)) {
    // Fall back to DTS if PTS is not available
    GstClockTime dts = GST_BUFFER_DTS(buf);
    int32_t sec = dts / GST_SECOND;
    uint32_t nanosec = dts % GST_SECOND;
    msg.header.stamp = rclcpp::Time(sec, nanosec);
    
    GST_DEBUG_OBJECT(sink, "Using buffer DTS timestamp: %lu ns (%d.%09u)", 
                    dts, sec, nanosec);
  } else {
    // Fall back to the provided msg_time if no buffer timestamp is available
    msg.header.stamp = msg_time;
    GST_DEBUG_OBJECT(sink, "No buffer timestamp available, using current time");
  }

  msg.header.frame_id = sink->frame_id;

  // Use the encoding if provided, otherwise use the format
  if (sink->encoding && strlen(sink->encoding) > 0) {
    msg.format = sink->encoding;
  } else if (sink->format) {
    msg.format = sink->format;
  } else {
    // This should never happen as we validate formats in setcaps
    RCLCPP_ERROR(
      ros_base_sink->node_if->logging->get_logger(),
      "No format specified for compressed image");
    return GST_FLOW_ERROR;
  }

  gst_buffer_map(buf, &info, GST_MAP_READ);
  msg.data.assign(info.data, info.data + info.size);
  gst_buffer_unmap(buf, &info);

  // publish
  sink->pub->publish(msg);

  return GST_FLOW_OK;
}

static void roscompimagesink_finalize(GObject * object)
{
  Roscompimagesink * sink = GST_ROSCOMPIMAGESINK(object);

  // Free all allocated strings
  if (sink->pub_topic) {
    g_free(sink->pub_topic);
    sink->pub_topic = NULL;
  }
  
  if (sink->frame_id) {
    g_free(sink->frame_id);
    sink->frame_id = NULL;
  }
  
  if (sink->encoding) {
    g_free(sink->encoding);
    sink->encoding = NULL;
  }
  
  if (sink->init_caps) {
    g_free(sink->init_caps);
    sink->init_caps = NULL;
  }
  
  if (sink->format) {
    g_free(sink->format);
    sink->format = NULL;
  }

  // Chain up to the parent class
  G_OBJECT_CLASS(roscompimagesink_parent_class)->finalize(object);
}
