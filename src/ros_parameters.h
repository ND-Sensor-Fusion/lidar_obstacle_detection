#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

struct EnvironmentParams {
public:
  EnvironmentParams(rclcpp::Node::SharedPtr node) {
    renderScene = node->get_parameter("renderScene").as_bool();
    //     t = node->get_parameter_or<bool>("t", false);
  }
  bool renderScene;
};
