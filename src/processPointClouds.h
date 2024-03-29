// PCL lib Functions for processing point clouds
#pragma once

#include "kdtree_3d.h"
#include "render/box.h"
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <chrono>
#include <ctime>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <vector>
template <typename PointT> class ProcessPointClouds {
public:
  // constructor
  ProcessPointClouds();
  // deconstructor
  ~ProcessPointClouds();

  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint,
                                                    Eigen::Vector4f maxPoint);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers,
                                                                                                         typename pcl::PointCloud<PointT>::Ptr cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                                       int maxIterations, float distanceThreshold);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
  BoxQ MyBoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

  void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

  std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

  pcl::PointIndices::Ptr MyRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
};