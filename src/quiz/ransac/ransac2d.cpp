/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include "../../ros_parameters.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  std::string pkg_path = ament_index_cpp::get_package_share_directory("lidar_obstacle_detection");
  std::string filename = pkg_path + "/data/pcd/simpleHighway.pcd";
  return pointProcessor.loadPcd(filename);
//   return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function

  // For max iterations

  // Randomly sample subset and fit line

  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  // Return indicies of inliers from fitted line with most inliers

  while (maxIterations--) {
    std::unordered_set<int> inliers; // set unique
    while (inliers.size() < 2)
      inliers.insert(rand() % (cloud->points.size()));

    float x1, y1, x2, y2;
    //       for (int index : inliers->indices)
    auto itr = inliers.begin();
    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    itr++;
    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;
    float a = (y1 - y2);
    float b = (x2 - x1);
    float c = (x1 * y2 - x2 * y1);

    for (int index = 0; index < cloud->points.size(); index++) {
      if (inliers.count(index) > 0) // do not include the random points
        continue;
      pcl::PointXYZ point = cloud->points[index];
      float x3 = point.x;
      float y3 = point.y;
      float d = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);
      if (d <= distanceTol)
        inliers.insert(index);
    }
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function

  // For max iterations

  // Randomly sample subset and fit line

  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  // Return indicies of inliers from fitted line with most inliers

  while (maxIterations--) {
    std::unordered_set<int> inliers; // set unique

    // extract 3 random points
    while (inliers.size() < 3)
      inliers.insert(rand() % (cloud->points.size()));

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    auto itr = inliers.begin();
    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    z1 = cloud->points[*itr].z;
    itr++;
    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;
    z2 = cloud->points[*itr].z;
    itr++;
    x3 = cloud->points[*itr].x;
    y3 = cloud->points[*itr].y;
    z3 = cloud->points[*itr].z;
    // use point1 as reference and calculate the two vectors
    pcl::PointXYZ v1 = pcl::PointXYZ(x2 - x1, y2 - y1, z2 - z1);
    pcl::PointXYZ v2 = pcl::PointXYZ(x3 - x1, y3 - y1, z3 - z1);
    // Find the normal vector to the plane by calculate v1xv2 = <i,j,k>
    float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    // Plane Ax + By + Ck + D = 0
    float A = i;
    float B = j;
    float C = k;
    float D = -1. * (i * x1 + j * y1 + k * z1);
    // Iterate over all the other points
    for (int index = 0; index < cloud->points.size(); index++) {
      if (inliers.count(index) > 0) // do not include the random points
        continue;
      pcl::PointXYZ point = cloud->points[index];
      float x3 = point.x;
      float y3 = point.y;
      float z3 = point.z;
      float d = fabs(A * x3 + B * y3 + C * z3 + D) / sqrt(A * A + B * B + C * C);
      if (d <= distanceTol)
        inliers.insert(index);
    }
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  return inliersResult;
}

int main() {

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
  // TODO: Change the max iteration and distance tolerance arguments for Ransac function
  //   std::unordered_set<int> inliers = Ransac2D(cloud, 10, 1.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
  std::unordered_set<int> inliers = Ransac(cloud, 20, 0.25);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  //   while (!viewer->wasStopped()) {
  //     viewer->spinOnce();
  //   }
  viewer->spin();
}
