// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"

// Include cv::Mat type adapter if available.
#ifdef USE_CV_MAT_TYPE_ADAPTER
#include <cv_bridge/cv_mat_sensor_msgs_image_type_adapter.hpp>
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  cv_bridge::ROSCvMatContainer,
  sensor_msgs::msg::Image);
#endif

namespace rs2_ros {
namespace tools {
namespace frame_latency {
class FrameLatencyNode : public rclcpp::Node
{
public:
    explicit FrameLatencyNode( const rclcpp::NodeOptions & node_options
                               = rclcpp::NodeOptions().use_intra_process_comms( true ) );

    FrameLatencyNode( const std::string & node_name,
                      const std::string & ns,
                      const rclcpp::NodeOptions & node_options
                      = rclcpp::NodeOptions().use_intra_process_comms( true ) );

private:
    #ifdef USE_CV_MAT_TYPE_ADAPTER
    using ImgMsg = cv_bridge::ROSCvMatContainer;
    #else
    using ImgMsg = sensor_msgs::msg::Image;
    #endif
    
    rclcpp::Subscription< ImgMsg >::SharedPtr _sub;
    rclcpp::Logger _logger;
};
}  // namespace frame_latency
}  // namespace tools
}  // namespace rs2_ros
