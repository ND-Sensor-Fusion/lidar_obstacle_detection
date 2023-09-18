// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT> void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT> typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloud;
}

template <typename PointT> std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCLoud(new pcl::PointCloud<PointT>());

  for (int index : inliers->indices)
    planeCLoud->points.push_back(cloud->points[index]);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCLoud);

  return segResult;
}

template <typename PointT> std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  // TODO:: Fill in this function to find inliers for the cloud
  pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients}; // can be used to render the plane
  //   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true); // optional
  //  Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a planar model for the given cloud " << std::endl;
    // PCL_ERROR("Could not estimate a planar model for the given cl.\n");
    // return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
  // for (const auto &idx : inliers->indices)
  //   std::cerr << idx << "    " << cloud->points[idx].x << " " << cloud->points[idx].y << " " << cloud->points[idx].z << std::endl;

  //
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT> std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance); // 2cm
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  int j = 0;
  for (const auto &cluster : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (const auto &idx : cluster.indices) {
      cloud_cluster->push_back((*cloud)[idx]);
    } //*
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
    // std::stringstream ss;
    // ss << std::setw(4) << std::setfill('0') << j;
    // writer.write<PointT>("cloud_cluster_" + ss.str() + ".pcd", *cloud_cluster, false); //*
    // j++;
    clusters.push_back(cloud_cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

template <typename PointT> Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT> void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT> typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

  return cloud;
}

template <typename PointT> std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  std::sort(paths.begin(), paths.end());

  return paths;
}