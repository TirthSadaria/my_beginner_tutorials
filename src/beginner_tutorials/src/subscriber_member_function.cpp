/**
 * @file subscriber_member_function.cpp
 * @brief Subscriber node that listens to string messages
 * 
 * This file implements a ROS 2 subscriber node that subscribes to string
 * messages from a topic and processes them using various logging levels.
 */

// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief Minimal subscriber node
 * 
 * This class creates a subscriber that listens to string messages
 * and processes them with appropriate logging.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalSubscriber
   * 
   * Initializes the node and creates a subscription to the topic.
   */
  explicit MinimalSubscriber()
      : Node("minimal_subscriber") {
    // Declare logger level parameter to allow runtime configuration
    this->declare_parameter<std::string>("logger_level", "INFO");
    std::string logger_level_str = this->get_parameter("logger_level").as_string();
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
    RCLCPP_INFO_STREAM(this->get_logger(), 
      "Logger level set to: " << logger_level_str);
    
    // Add parameter callback to update logger level at runtime
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MinimalSubscriber::OnSetParametersCallback, this, std::placeholders::_1));
    
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10,
        std::bind(&MinimalSubscriber::TopicCallback, this, _1));
    
    RCLCPP_INFO_STREAM(this->get_logger(), 
      "Subscriber node initialized. Listening to topic 'topic'");
  }

 private:
  /**
   * @brief Parameter callback to handle logger level changes at runtime
   * 
   * @param parameters Vector of parameters being set
   * @return Result of parameter setting
   */
  rcl_interfaces::msg::SetParametersResult OnSetParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
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
        RCLCPP_INFO_STREAM(this->get_logger(), 
          "Logger level changed to: " << logger_level_str);
      }
    }
    return result;
  }
  
  /**
   * @brief Topic callback function
   * 
   * Called when a new message is received on the subscribed topic.
   * Uses various logging levels to demonstrate different message types.
   * 
   * @param msg Received string message
   */
  void TopicCallback(const std_msgs::msg::String& msg) {
    message_count_++;
    
    RCLCPP_DEBUG_STREAM(this->get_logger(), 
      "Received message #" << message_count_ << " with length: " << msg.data.length());
    
    // Check for potential issues
    if (msg.data.empty()) {
      RCLCPP_WARN_STREAM(this->get_logger(), 
        "Received empty message! This may indicate a problem with the publisher.");
      return;
    }
    
    if (msg.data.length() > 200) {
      RCLCPP_WARN_STREAM(this->get_logger(), 
        "Received very long message (" << msg.data.length() << " chars). Processing may be slow.");
    }
    
    // Validate message content
    if (msg.data.find("ERROR") != std::string::npos) {
      RCLCPP_ERROR_STREAM(this->get_logger(), 
        "Message contains 'ERROR' keyword: '" << msg.data << "'");
    }
    
    // Check for critical conditions
    if (msg.data.find("FATAL") != std::string::npos) {
      RCLCPP_FATAL_STREAM(this->get_logger(), 
        "FATAL condition detected in message: '" << msg.data << "'. System may be unstable.");
    }
    
    RCLCPP_INFO_STREAM(this->get_logger(), 
      "I heard: '" << msg.data << "'");
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  ///< Subscription to string messages
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;  ///< Parameter callback handle
  size_t message_count_ = 0;  ///< Counter for received messages
};

/**
 * @brief Main function
 * 
 * Initializes ROS 2 and spins the subscriber node.
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return Exit status
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
