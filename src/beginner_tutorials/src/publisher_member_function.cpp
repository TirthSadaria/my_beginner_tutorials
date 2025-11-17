/**
 * @file publisher_member_function.cpp
 * @brief Entry point for MinimalPublisher node executable.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "beginner_tutorials/minimal_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
