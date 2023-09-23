#pragma once
#include <chrono>
#include <ctime>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <string>
#include <vector>

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