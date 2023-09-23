#include "processPointClouds.h"
#include <unordered_set>

template <typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::MyRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {
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
    PointT v1 = PointT(x2 - x1, y2 - y1, z2 - z1);
    PointT v2 = PointT(x3 - x1, y3 - y1, z3 - z1);
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
      PointT point = cloud->points[index];
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

  pcl::PointIndices::Ptr pointindices_inliers{new pcl::PointIndices};
  for (auto it = inliersResult.begin(); it != inliersResult.end(); ++it)
    pointindices_inliers->indices.push_back(*it);
  return pointindices_inliers;
}