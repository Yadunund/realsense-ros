// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#include <sstream>
#include <string>
#include <frame_latency/frame_latency.h>
#include <constants.h>
// Node which receives sensor_msgs/Image messages and prints the image latency.

#include <cv_bridge/cv_bridge.hpp>

using namespace rs2_ros::tools::frame_latency;

FrameLatencyNode::FrameLatencyNode( const std::string & node_name,
                                    const std::string & ns,
                                    const rclcpp::NodeOptions & node_options )
    : Node( node_name, ns, node_options )
    , _logger( this->get_logger() )
{
}


FrameLatencyNode::FrameLatencyNode( const rclcpp::NodeOptions & node_options )
    : Node( "frame_latency", "/", node_options )
    , _logger( this->get_logger() )
{
    ROS_INFO_STREAM( "frame_latency node is UP!" );
    ROS_INFO_STREAM( "Intra-Process is "
                     << ( this->get_node_options().use_intra_process_comms() ? "ON" : "OFF" ) );
    // Create a subscription on the input topic.
    _sub = this->create_subscription< ImgMsg >(
        "/color/image_raw",  // TODO Currently color only, we can declare and accept the required
                             // streams as ros parameters
        rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( rmw_qos_profile_default ),
                     rmw_qos_profile_default ),
        [&, this]( std::shared_ptr<const ImgMsg > msg) {
            rclcpp::Time curr_time = this->get_clock()->now();
            // Access cv::Mat
            #ifdef USE_CV_MAT_TYPE_ADAPTER
            const cv::Mat& mat = msg->cv_mat();
            auto latency = ( curr_time - msg->header().stamp ).seconds();
            #else
            const auto cv_image = cv_bridge::toCvShare(msg);
            const cv::Mat& mat = cv_image->image;
            auto latency = ( curr_time - msg->header.stamp ).seconds();  
            #endif   
            ROS_INFO_STREAM( "Got msg with address 0x"
                             << std::hex << reinterpret_cast< std::uintptr_t >( msg.get() )
                             << std::dec << " with latency of " << latency << " [sec]."
                             << " cv::Mat size: ["
                             << mat.rows << " x " << mat.cols << "]" );
        } );
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( rs2_ros::tools::frame_latency::FrameLatencyNode )
