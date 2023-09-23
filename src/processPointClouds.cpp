// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT> void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                                                              Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

  // Downsample
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter((*cloudFiltered));

  typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  // remove also roof
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloudFiltered);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);
  //

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
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

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers; //{new pcl::PointIndices};
  // TODO:: Fill in this function to find inliers for the cloud

  //   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object

  ///* MY RANSAC IMPLEMENTATION
  inliers = MyRansac3D(cloud, maxIterations, distanceThreshold);
  //*/

  /* PCL FUNCT
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients}; // can be used to render the plane
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
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " <<
  coefficients->values[3] << std::endl;
  //*/

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

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
                                                                                          int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles
  // Creating the KdTree object for the search method of the extraction
  std::vector<pcl::PointIndices> cluster_indices;
  /* pcl kdtree
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance); // 2cm
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  //*/
  ///*
  MyKd3DTree *mytree = new MyKd3DTree;
  std::vector<std::vector<float>> points_f(cloud->points.size(), std::vector<float>(3)); // resize in one go
  uint p_id = 0;
  for (PointT point : cloud->points) {
    points_f[p_id][0] = point.x;
    points_f[p_id][1] = point.y;
    points_f[p_id][2] = point.z;
    // std::vector<float> point_f = {point.x, point.y, point.z};
    // points_f.push_back(point_f);
    mytree->insert(points_f[p_id], p_id);
    ++p_id;
  }
  EuclideanCluster *mycluster = new EuclideanCluster;
  std::vector<std::vector<int>> clusters_i = mycluster->euclideanCluster(points_f, mytree, clusterTolerance, minSize, maxSize);
  for (auto cluster_i : clusters_i) {
    pcl::PointIndices clust_ind;
    for (auto it = cluster_i.begin(); it != cluster_i.end(); ++it)
      clust_ind.indices.push_back(*it);
    cluster_indices.push_back(clust_ind);
  }

  //*/
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

// template <typename PointT> BoxQ ProcessPointClouds<PointT>::MyBoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster) {
//   BoxQ box;

//   pcl::PointXYZ minPoint, maxPoint;
//   pcl::getMinMax3D(*cluster, minPoint, maxPoint);

//   // Calculate the dimensions of the bounding box
//   float length = maxPoint.x - minPoint.x;
//   float width = maxPoint.y - minPoint.y;
//   float height = maxPoint.z - minPoint.z;

//   // Set the dimensions of the bounding box
//   box.cube_length = length;
//   box.cube_width = width;
//   box.cube_height = height;

//   // Calculate the center of the bounding box
//   box.bboxTransform = Eigen::Vector3f((maxPoint.x + minPoint.x) / 2, (maxPoint.y + minPoint.y) / 2, (maxPoint.z + minPoint.z) / 2);

//   // Set the quaternion for rotation around the Z-axis (XY plane)
//   Eigen::Quaternionf bboxQuaternion;
//   bboxQuaternion = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()); // Adjust the angle for desired rotation

//   box.bboxQuaternion = bboxQuaternion;

//   return box;
// }

// My bounding Box Q min area with PCA
template <typename PointT> BoxQ ProcessPointClouds<PointT>::MyBoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster) {

  // 1) compute the centroid (c0, c1, c2) and the normalized covariance
  // 2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
  // 3) move the points in that RF --- note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
  // 4) compute the max, the min and the center of the diagonal
  // 5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z) the transformation you have to apply is
  // Rotation = (e0, e1, e0 X e1) & Translation = Rotation * center_diag + (c0, c1, c2)

  // Find bounding box for one of the clusters
  // PointT minPoint, maxPoint;
  // pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  BoxQ box;

  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cluster, pcaCentroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);

  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

  // Set the X and Y components of the eigenvectors to zero to align with the Z-axis
  // eigenVectorsPCA.col(0).head<2>().setZero();
  // eigenVectorsPCA.col(1).head<2>().setZero();
  // // Set the Z component to zero for all the eigenvectors to align with the XY plane
  // eigenVectorsPCA.col(0)(2) = 0;
  // eigenVectorsPCA.col(1)(2) = 0;
  // eigenVectorsPCA.col(2)(2) = 0;

  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

  // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
  ///    the signs are different and the box doesn't get correctly oriented in some cases.
  // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PCA<pcl::PointXYZ> pca;
  // pca.setInputCloud(cloudSegmented);
  // pca.project(*cloudSegmented, *cloudPCAprojection);
  // std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
  // std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
  // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.

  //=================================================================
  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
  //=================================================================
  // Final transform
  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
  //=================================================================
  box.bboxTransform = bboxTransform;
  box.bboxQuaternion = bboxQuaternion;
  box.cube_length = maxPoint.x - minPoint.x;
  box.cube_width = maxPoint.y - minPoint.y;
  box.cube_height = maxPoint.z - minPoint.z;
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