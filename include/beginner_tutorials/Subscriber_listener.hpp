/**
 * @file Publisher_talker.hpp
 * @author Anukriti Singh 
 * @brief Implementation of the Minimal publisher
 * @version 0.2
 * @date 2022-11-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <memory>
#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief Class (subclass of Node) and registers a member function as a callback from the topic.
 * 
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
    MinimalSubscriber();

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};