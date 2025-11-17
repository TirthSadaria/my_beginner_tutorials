/**
 * @file talker_integration_test.cpp
 * @brief Catch2 integration test for the MinimalPublisher node.
 *
 * This test launches the talker node, subscribes to the `topic`
 * topic, and verifies that at least one message is received within
 * a reasonable timeout. It exercises the full node stack,
 * demonstrating Level 2 (integration) testing as required.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#define CATCH_CONFIG_MAIN
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "catch2/catch.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "beginner_tutorials/minimal_publisher.hpp"

using namespace std::chrono_literals;

/**
 * @brief Helper class capturing messages from the talker node.
 */
class TestSubscriptionNode : public rclcpp::Node
{
public:
  TestSubscriptionNode()
  : Node("talker_integration_test_node"), message_count_(0)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        last_message_ = msg->data;
        ++message_count_;
      });
  }

  size_t GetMessageCount() const {return message_count_;}

  std::string GetLastMessage() const {return last_message_;}

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t message_count_;
  std::string last_message_;
};

TEST_CASE("MinimalPublisher publishes messages on /topic", "[integration]") {
  rclcpp::init(0, nullptr);

  auto talker_node = std::make_shared<MinimalPublisher>();
  auto test_node = std::make_shared<TestSubscriptionNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(talker_node);
  executor.add_node(test_node);

  const auto start_time = std::chrono::steady_clock::now();
  const auto timeout = 3s;

  while ((std::chrono::steady_clock::now() - start_time) < timeout &&
    test_node->GetMessageCount() == 0)
  {
    executor.spin_some();
    std::this_thread::sleep_for(50ms);
  }

  REQUIRE(test_node->GetMessageCount() > 0);
  REQUIRE_FALSE(test_node->GetLastMessage().empty());

  executor.remove_node(talker_node);
  executor.remove_node(test_node);
  rclcpp::shutdown();
}
