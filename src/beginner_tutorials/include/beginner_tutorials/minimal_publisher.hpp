/**
 * @file minimal_publisher.hpp
 * @brief Declaration of the MinimalPublisher node.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#ifndef BEGINNER_TUTORIALS__MINIMAL_PUBLISHER_HPP_
#define BEGINNER_TUTORIALS__MINIMAL_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "beginner_tutorials/srv/change_string.hpp"

/**
 * @brief Minimal publisher node with service server and TF broadcaster.
 */
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher();

private:
  void TimerCallback();

  void BroadcastTransform();

  rcl_interfaces::msg::SetParametersResult OnSetParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void ChangeStringCallback(
    const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request> request,
    std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> response);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_callback_handle_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  size_t count_;
  std::string base_string_;
};

#endif  // BEGINNER_TUTORIALS__MINIMAL_PUBLISHER_HPP_
