/**
 * @file minimal_publisher.cpp
 * @brief Implementation of the MinimalPublisher node logic.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "beginner_tutorials/minimal_publisher.hpp"

using namespace std::chrono_literals;

MinimalPublisher::MinimalPublisher()
: Node("minimal_publisher"),
  count_(0),
  base_string_("ENPM700 Testing Publisher Output  ")
{
  this->declare_parameter<int>("frequency", 500);
  int frequency = this->get_parameter("frequency").as_int();

  if (frequency <= 0) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Invalid frequency parameter: " << frequency
                                      << ". Using default 500ms");
    frequency = 500;
  }

  this->declare_parameter<std::string>("logger_level", "INFO");
  std::string logger_level_str =
    this->get_parameter("logger_level").as_string();
  rclcpp::Logger::Level logger_level = rclcpp::Logger::Level::Info;
  if (logger_level_str == "DEBUG") {
    logger_level = rclcpp::Logger::Level::Debug;
  } else if (logger_level_str == "INFO") {
    logger_level = rclcpp::Logger::Level::Info;
  } else if (logger_level_str == "WARN") {
    logger_level = rclcpp::Logger::Level::Warn;
  } else if (logger_level_str == "ERROR") {
    logger_level = rclcpp::Logger::Level::Error;
  } else if (logger_level_str == "FATAL") {
    logger_level = rclcpp::Logger::Level::Fatal;
  }
  this->get_logger().set_level(logger_level);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Logger level set to: " << logger_level_str);

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(
      &MinimalPublisher::OnSetParametersCallback, this,
      std::placeholders::_1));

  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Initializing publisher with frequency: " << frequency << "ms");

  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

  auto timer_duration = std::chrono::milliseconds(frequency);
  timer_ = this->create_wall_timer(
    timer_duration, std::bind(&MinimalPublisher::TimerCallback, this));

  service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
    "change_string",
    std::bind(
      &MinimalPublisher::ChangeStringCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Publisher node initialized. Service 'change_string' is "
    "available.");
}

void MinimalPublisher::TimerCallback()
{
  auto message = std_msgs::msg::String();
  message.data = base_string_ + std::to_string(count_++);

  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Preparing to publish message #" << count_);

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Publishing: '" << message.data << "'");

  publisher_->publish(message);
  BroadcastTransform();
}

void MinimalPublisher::BroadcastTransform()
{
  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  geometry_msgs::msg::TransformStamped transform;
  const rclcpp::Time now = this->get_clock()->now();
  transform.header.stamp = now;
  transform.header.frame_id = "world";
  transform.child_frame_id = "talk";

  const double time_seconds = now.seconds();
  transform.transform.translation.x = 1.5;
  transform.transform.translation.y =
    0.5 + 0.25 * std::sin(time_seconds * 0.5);
  transform.transform.translation.z =
    0.25 + 0.15 * std::cos(time_seconds * 0.25);

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.2, 0.1, 0.25 * std::sin(time_seconds * 0.2));
  transform.transform.rotation = tf2::toMsg(quaternion);

  tf_broadcaster_->sendTransform(transform);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Broadcasted TF from /world to /talk at time " << time_seconds);
}

rcl_interfaces::msg::SetParametersResult
MinimalPublisher::OnSetParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "logger_level") {
      std::string logger_level_str = param.as_string();
      rclcpp::Logger::Level logger_level = rclcpp::Logger::Level::Info;
      if (logger_level_str == "DEBUG") {
        logger_level = rclcpp::Logger::Level::Debug;
      } else if (logger_level_str == "INFO") {
        logger_level = rclcpp::Logger::Level::Info;
      } else if (logger_level_str == "WARN") {
        logger_level = rclcpp::Logger::Level::Warn;
      } else if (logger_level_str == "ERROR") {
        logger_level = rclcpp::Logger::Level::Error;
      } else if (logger_level_str == "FATAL") {
        logger_level = rclcpp::Logger::Level::Fatal;
      } else {
        result.successful = false;
        result.reason = "Invalid logger level: " + logger_level_str;
        return result;
      }
      this->get_logger().set_level(logger_level);
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Logger level changed to: " << logger_level_str);
    }
  }
  return result;
}

void MinimalPublisher::ChangeStringCallback(
  const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
  request,
  std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> response)
{
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Service called with new_string: '" << request->new_string
                                        << "'");

  if (request->new_string.empty()) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Received empty string in service request. Keeping "
      "current string.");
    response->success = false;
    return;
  }

  std::string old_string = base_string_;
  base_string_ = request->new_string;

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Base string changed from '" << old_string << "' to '"
                                 << base_string_ << "'");

  response->success = true;

  if (base_string_.length() > 100) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "New base string is very long ("
        << base_string_.length()
        << " chars). May affect performance.");
  }
}
