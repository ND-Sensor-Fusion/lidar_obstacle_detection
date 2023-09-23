#pragma once
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
using namespace std::chrono_literals;

class RVizViewer {
public:
  RVizViewer(rclcpp::Node::SharedPtr node) : logger_{rclcpp::get_logger("RVIZ_VIEWER")} {
    node_ = node;
    pub_pcl_floor_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_floor", 1);
    pub_pcl_filtered_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_flitered", 1);
    pub_pcl_clusters_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_clusters", 1);
    pub_pcl_input_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_input", 1);
    pub_clusters_bb_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("marker_bb_clusters", 1);
  }

  void publishFloor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_cloud, std::string name, Color color) {
    // Convert PCL point cloud to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = "laser3d";
    ros_cloud.header.stamp = node_->get_clock()->now();
    pub_pcl_floor_->publish(ros_cloud);
  }

  void publishInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_cloud, std::string name, Color color) {
    // Convert PCL point cloud to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = "laser3d";
    ros_cloud.header.stamp = node_->get_clock()->now();
    pub_pcl_input_->publish(ros_cloud);
  }

  void publishFliteredCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_cloud, std::string name, Color color) {
    // Convert PCL point cloud to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = "laser3d";
    ros_cloud.header.stamp = node_->get_clock()->now();
    pub_pcl_filtered_->publish(ros_cloud);
  }

  void addClusterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_cloud, std::string name, Color color) {
    // Convert pcl_cloud to colored RGB CLOUD
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pcl_cloud, *output_cloud);

    //     output_cloud->points.resize(pcl_cloud->points.size());
    //     output_cloud->header = pcl_cloud->header;
    //     output_cloud->is_dense = pcl_cloud->is_dense;
    // output_cloud->height = pcl_cloud->height;
    for (size_t i = 0; i < pcl_cloud->points.size(); i++) {

      //       output_cloud->points[i].x = pcl_cloud->points[i].x;
      //       output_cloud->points[i].y = pcl_cloud->points[i].y;
      //       output_cloud->points[i].z = pcl_cloud->points[i].z;
      output_cloud->points[i].r = static_cast<int>(color.r * 255.0f);
      output_cloud->points[i].g = static_cast<int>(color.g * 255.0f);
      output_cloud->points[i].b = static_cast<int>(color.b * 255.0f);
    }
    colored_clusters_.push_back(output_cloud);
  }

  void publishClustersAndBoxes() {
    if (colored_clusters_.size() == 0)
      return;
    // Concatenate all the colored clusters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < colored_clusters_.size(); i++) {
      *concatenated_cloud += *colored_clusters_[i];
    }
    concatenated_cloud->header = colored_clusters_[0]->header;
    concatenated_cloud->is_dense = colored_clusters_[0]->is_dense;
    // publish cloud
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*concatenated_cloud, ros_cloud);
    ros_cloud.header.frame_id = "laser3d";
    ros_cloud.header.stamp = node_->get_clock()->now();
    pub_pcl_clusters_->publish(ros_cloud);
    // clear the clusters after publishing
    colored_clusters_.clear();
    // publish bounding boxes
    pub_clusters_bb_->publish(marker_bounding_boxes_);
    marker_bounding_boxes_.markers.clear();
  }

  void addBox(Box box, int id, Color color, float opacity) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser3d";
    marker.header.stamp = node_->get_clock()->now();
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.type = marker.CUBE;
    marker.ns = "clusters_bb";
    marker.id = id;
    marker.scale.x = box.x_max - box.x_min;
    marker.scale.y = box.y_max - box.y_min;
    marker.scale.z = box.z_max - box.z_min;
    marker.color.r = 1.0;     // color.r;
    marker.color.g = 0.0;     // color.g;
    marker.color.b = 0.0;     // color.b;
    marker.color.a = opacity; // Alpha (transparency)
    marker.pose.position.x = (box.x_min + box.x_max) / 2.0;
    marker.pose.position.y = (box.y_min + box.y_max) / 2.0;
    marker.pose.position.z = (box.z_min + box.z_max) / 2.0;
    marker_bounding_boxes_.markers.push_back(marker);
  }

  void clearVisualizations() {
    marker_bounding_boxes_.markers.clear();
    pub_clusters_bb_->publish(marker_bounding_boxes_);
  }

private:
  //
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Logger logger_;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> colored_clusters_;
  visualization_msgs::msg::MarkerArray marker_bounding_boxes_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_input_, pub_pcl_floor_, pub_pcl_filtered_, pub_pcl_clusters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_clusters_bb_;
}; // class RVizViewer