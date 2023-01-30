// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#if defined( DASHING ) || defined( ELOQUENT )
#include <image_transport/image_transport.h>
#else
#include <image_transport/image_transport.hpp>
#endif

// Include cv::Mat type adapter if available.
#ifdef USE_CV_MAT_TYPE_ADAPTER
#include <cv_bridge/cv_mat_sensor_msgs_image_type_adapter.hpp>
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  cv_bridge::ROSCvMatContainer,
  sensor_msgs::msg::Image);
#endif

namespace realsense2_camera {
class image_publisher
{
public:
    // Use cv::Mat type adapter if available.
    #ifdef USE_CV_MAT_TYPE_ADAPTER
    virtual void publish(std::unique_ptr<cv_bridge::ROSCvMatContainer> image_ptr) = 0;
    #else
    virtual void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) = 0;
    #endif
    virtual size_t get_subscription_count() const = 0;
    virtual ~image_publisher() = default;
};

// Native RCL implementation of an image publisher (needed for intra-process communication)
class image_rcl_publisher : public image_publisher
{
public:
    image_rcl_publisher( rclcpp::Node & node,
                         const std::string & topic_name,
                         const rmw_qos_profile_t & qos );
    // Use cv::Mat type adapter if available.
    #ifdef USE_CV_MAT_TYPE_ADAPTER
    void publish(std::unique_ptr<cv_bridge::ROSCvMatContainer> image_ptr) override;
    #else
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    #endif                         
    size_t get_subscription_count() const override;

private:
    // Use cv::Mat type adapter if available.
    #ifdef USE_CV_MAT_TYPE_ADAPTER
    rclcpp::Publisher< cv_bridge::ROSCvMatContainer >::SharedPtr image_publisher_impl;
    #else
    rclcpp::Publisher< sensor_msgs::msg::Image >::SharedPtr image_publisher_impl;
    #endif     
};

// image_transport implementation of an image publisher (adds a compressed image topic)
class image_transport_publisher : public image_publisher
{
public:
    image_transport_publisher( rclcpp::Node & node,
                               const std::string & topic_name,
                               const rmw_qos_profile_t & qos );
    // Use cv::Mat type adapter if available.
    #ifdef USE_CV_MAT_TYPE_ADAPTER
    void publish(std::unique_ptr<cv_bridge::ROSCvMatContainer> image_ptr) override;
    #else
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    #endif                                    
    size_t get_subscription_count() const override;

private:
    // Use cv::Mat type adapter if available.
    #ifdef USE_CV_MAT_TYPE_ADAPTER
    rclcpp::Publisher< cv_bridge::ROSCvMatContainer >::SharedPtr image_publisher_impl;
    #else
    std::shared_ptr< image_transport::Publisher > image_publisher_impl;
    #endif   
};

}  // namespace realsense2_camera
