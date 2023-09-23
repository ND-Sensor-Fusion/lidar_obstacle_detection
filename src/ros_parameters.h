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
    simpleHighway = node->get_parameter("simpleHighway").as_bool();
    publish_input_cloud = node->get_parameter("publish_input_cloud").as_bool();
    downroi.filterRes = node->get_parameter("downroi.filterRes").as_double();
    data_folder = node->get_parameter("data_folder").as_string();
    downroi.minx = node->get_parameter("downroi.minx").as_double();
    downroi.miny = node->get_parameter("downroi.miny").as_double();
    downroi.minz = node->get_parameter("downroi.minz").as_double();
    downroi.mini = node->get_parameter("downroi.mini").as_double();
    downroi.maxx = node->get_parameter("downroi.maxx").as_double();
    downroi.maxy = node->get_parameter("downroi.maxy").as_double();
    downroi.maxz = node->get_parameter("downroi.maxz").as_double();
    downroi.maxi = node->get_parameter("downroi.maxi").as_double();
    ransac.distanceThreshold = node->get_parameter("ransac.distanceThreshold").as_double();
    ransac.maxIterations = node->get_parameter("ransac.maxIterations").as_int();
    clustering.distanceTol = node->get_parameter("clustering.distanceTol").as_double();
    clustering.minPoints = node->get_parameter("clustering.minPoints").as_int();
    clustering.maxPoints = node->get_parameter("clustering.maxPoints").as_int();
  }
  bool renderScene, simpleHighway, publish_input_cloud;
  std::string data_folder;
  struct clusteringparam {
    float distanceTol;
    int minPoints, maxPoints;
  } clustering;
  struct ransacparam {
    float distanceThreshold;
    int maxIterations;
  } ransac;
  struct downandroicropparams {
    float filterRes;
    float minx, miny, minz, mini;
    float maxx, maxy, maxz, maxi;
  } downroi;
};
