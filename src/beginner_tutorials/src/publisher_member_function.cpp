/**
 * @file publisher_member_function.cpp
 * @brief Publisher node with service server to change base output string
 * 
 * This file implements a ROS 2 publisher node that publishes string messages
 * to a topic. It also includes a service server that allows changing the
 * base output string dynamically at runtime.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

using namespace std::chrono_literals;

/**
 * @brief Minimal publisher node with service server
 * 
 * This class creates a publisher that periodically publishes string messages.
 * It also provides a service to change the base output string dynamically.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalPublisher
   * 
   * Initializes the node, creates publisher, timer, and service server.
   * Also declares and gets the publish frequency parameter.
   */
  explicit MinimalPublisher()
      : Node("minimal_publisher"), count_(0), base_string_("ENPM700 Testing Publisher Output  ") {
    // Declare and get frequency parameter
    this->declare_parameter<int>("frequency", 500);
    int frequency = this->get_parameter("frequency").as_int();
    
    // Validate frequency parameter
    if (frequency <= 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), 
        "Invalid frequency parameter: " << frequency << ". Using default 500ms");
      frequency = 500;
    }
    
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
        std::bind(&MinimalPublisher::OnSetParametersCallback, this, std::placeholders::_1));
    
    RCLCPP_DEBUG_STREAM(this->get_logger(), 
      "Initializing publisher with frequency: " << frequency << "ms");
    
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    
    auto timer_duration = std::chrono::milliseconds(frequency);
    timer_ = this->create_wall_timer(
        timer_duration, std::bind(&MinimalPublisher::TimerCallback, this));
    
    // Create service server
    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
        "change_string",
        std::bind(&MinimalPublisher::ChangeStringCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO_STREAM(this->get_logger(), 
      "Publisher node initialized. Service 'change_string' is available.");
  }

 private:
  /**
   * @brief Timer callback function
   * 
   * Called periodically to publish messages to the topic.
   */
  void TimerCallback() {
    auto message = std_msgs::msg::String();
    message.data = base_string_ + std::to_string(count_++);
    
    RCLCPP_DEBUG_STREAM(this->get_logger(), 
      "Preparing to publish message #" << count_);
    
    RCLCPP_INFO_STREAM(this->get_logger(), 
      "Publishing: '" << message.data << "'");
    
    publisher_->publish(message);
  }
  
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
   * @brief Service callback to change the base output string
   * 
   * @param request Service request containing the new string
   * @param response Service response indicating success
   */
  void ChangeStringCallback(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request> request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> response) {
    
    RCLCPP_DEBUG_STREAM(this->get_logger(), 
      "Service called with new_string: '" << request->new_string << "'");
    
    if (request->new_string.empty()) {
      RCLCPP_WARN_STREAM(this->get_logger(), 
        "Received empty string in service request. Keeping current string.");
      response->success = false;
      return;
    }
    
    std::string old_string = base_string_;
    base_string_ = request->new_string;
    
    RCLCPP_INFO_STREAM(this->get_logger(), 
      "Base string changed from '" << old_string << "' to '" << base_string_ << "'");
    
    response->success = true;
    
    if (base_string_.length() > 100) {
      RCLCPP_WARN_STREAM(this->get_logger(), 
        "New base string is very long (" << base_string_.length() << " chars). May affect performance.");
    }
  }
  
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic publishing
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  ///< Publisher for string messages
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;  ///< Service server
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;  ///< Parameter callback handle
  size_t count_;  ///< Message counter
  std::string base_string_;  ///< Base string for published messages
};

/**
 * @brief Main function
 * 
 * Initializes ROS 2 and spins the publisher node.
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return Exit status
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
