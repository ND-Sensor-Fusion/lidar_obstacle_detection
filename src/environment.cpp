/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "ransac_3d.cpp"

#include "ros_parameters.h"
#include "rviz_viewer.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer, std::shared_ptr<EnvironmentParams> params) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  std::vector<Car> cars = initHighway(params->renderScene, viewer);

  // TODO:: Create lidar sensor
  Lidar *lidar = new Lidar(cars, 0); // in the heap
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

  // renderRays(viewer, lidar->position, inputCloud);
  // renderPointCloud(viewer, inputCloud, "lidar_pointcloud", Color(1, 1, 1));
  // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  // ransac
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
      pointProcessor.SegmentPlane(inputCloud, params->ransac.maxIterations, params->ransac.distanceThreshold); // contains MyRansac3D
  // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
  // clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
      pointProcessor.Clustering(segmentCloud.first, params->clustering.distanceTol, params->clustering.minPoints, params->clustering.maxPoints);

  u_int clusterId = 0;
  // Set a static seed value, so we get the same color list everytime, without repeating (achieved also by time(NULL) ) or getting similar colors
  unsigned int seed = 42;
  srand(seed);
  // srand((unsigned)time(NULL));
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size ";
    pointProcessor.numPoints(cluster);
    // i see why you hardcoded since they can be similar
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                     Color((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX));
    // render box
    Box box = pointProcessor.BoundingBox(cluster);
    // My bounding Box Q min area with PCA
    // BoxQ box = pointProcessor.MyBoundingBoxQ(cluster);

    renderBox(viewer, box, clusterId);

    ++clusterId;
  }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, std::shared_ptr<RVizViewer> rviz_viewer, std::shared_ptr<EnvironmentParams> params,
               ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {

  //====================================================================================================================
  if (params->publish_input_cloud)
    rviz_viewer->publishInputCloud(inputCloud, "inputCLoud", Color(1, 1, 0));
  //  filter downsample and crop
  Eigen::Vector4f minroipoint(params->downroi.minx, params->downroi.miny, params->downroi.minz, params->downroi.mini);
  Eigen::Vector4f maxroipoint(params->downroi.maxx, params->downroi.maxy, params->downroi.maxz, params->downroi.maxi);
  double filterRes;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, params->downroi.filterRes, minroipoint, maxroipoint);
  rviz_viewer->publishFliteredCloud(filteredCloud, "filteredCloud", Color(1, 1, 0));
  // renderPointCloud(viewer, filteredCloud, "filteredCloud");
  // extract plane and objects // First Obj Second Objects
  // ransac
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
      pointProcessorI->SegmentPlane(filteredCloud, params->ransac.maxIterations, params->ransac.distanceThreshold); // contains MyRansac3D
  // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
  rviz_viewer->publishFloor(segmentCloud.second, "planeCloud", Color(0, 1, 0));
  // clustering
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      pointProcessorI->Clustering(segmentCloud.first, params->clustering.distanceTol, params->clustering.minPoints, params->clustering.maxPoints);
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 1, 1)};
  u_int clusterId = 0;
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    // std::cout << "cluster size ";
    // pointProcessorI->numPoints(cluster);
    rviz_viewer->addClusterCloud(cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);
    // renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);
    // render box
    Box box = pointProcessorI->BoundingBox(cluster);
    rviz_viewer->addBox(box, clusterId, Color(1, 1, 1), 0.3);
    // renderBox(viewer, box, clusterId);

    ++clusterId;
  }
  rviz_viewer->publishClustersAndBoxes();
  //==================================================================================================================
}

//=========================================================================================================================

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
  case XY:
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
    break;
  case TopDown:
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
    break;
  case Side:
    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
    break;
  case FPS:
    viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}

// USER SIGNAL
static bool EXIT_VAR = false;
void signalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "Ctrl+C detected. Exiting..." << std::endl;
    EXIT_VAR = true;
  }
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("lidar_obstacle_detection", node_options);
  std::shared_ptr<EnvironmentParams> env_params = std::make_shared<EnvironmentParams>(node);
  // end ros

  std::cout << "starting enviroment" << std::endl;
  // pcl viewer
  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (env_params->simpleHighway)
    viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  // rviz viewer
  std::shared_ptr<RVizViewer> rviz_viewer = std::make_shared<RVizViewer>(node);
  //=== spin the pcl viewer, ! CAN NOT USE spinOnce() in this version of pcl
  // std::unique_ptr<std::thread> viewer_thread = std::make_unique<std::thread>([&]() { viewer->spin(); });
  // viewer_thread->detach(); //  The main thread doesn't need to wait for the detached thread to finish, and there's no need to call
  // while (!viewer->wasStopped ())
  // {
  // viewer->spinOnce ();
  // }

  //=== spin also ros
  rclcpp::executors::MultiThreadedExecutor executor;
  std::unique_ptr<std::thread> node_thread = std::make_unique<std::thread>([&]() {
    executor.add_node(node->get_node_base_interface());
    executor.spin();
  });
  node_thread->detach();

  //===============================================================
  if (env_params->simpleHighway) {
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
  }
  if (env_params->simpleHighway)
    simpleHighway(viewer, env_params);
  else {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("lidar_obstacle_detection");
    std::string base_path = pkg_path + "/data/pcd/" + env_params->data_folder; // data_1 data_2
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(base_path);
    auto streamIterator = stream.begin(); // iterator of pcd data stream.

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    while (!EXIT_VAR) {
      // Load pcd and run obstacle detection process
      // viewer->removeAllPointClouds();
      // viewer->removeAllShapes();
      inputCloud = pointProcessorI->loadPcd((*streamIterator).string());
      auto startTime = std::chrono::steady_clock::now();
      cityBlock(viewer, rviz_viewer, env_params, pointProcessorI, inputCloud);
      auto endTime = std::chrono::steady_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
      std::cout << "City Block took " << elapsedTime.count() << " milliseconds" << std::endl;
      // Updates streamIterator. Loops from beginning if reach end.
      streamIterator++;
      if (streamIterator == stream.end())
        streamIterator = stream.begin();
      // viewer->spinOnce(1000, true);

      // std::this_thread::sleep_for(std::chrono::microseconds(100));
      rviz_viewer->clearVisualizations();
    }
  }
  //===============================================================

  // wait for ctrl+c
  std::cout << EXIT_VAR << std::endl;
  while (!EXIT_VAR)
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  viewer->close();    // close the viewer
  rclcpp::shutdown(); // shutdown ros
  return 0;
}