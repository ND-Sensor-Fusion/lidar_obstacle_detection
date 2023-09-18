cmake_minimum_required(VERSION 3.5)
project(lidar_obstacle_detection)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
set(dependencies
  ament_cmake
  ament_cmake_python
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  ament_index_cpp
  pcl_conversions
  pcl_ros
  pcl_msgs
)

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PCL 1.12 REQUIRED COMPONENTS common visualization io)
find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# include_directories(
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

#### MAIN
add_executable(environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
ament_target_dependencies(environment ${dependencies})
target_link_libraries(environment ${PCL_LIBRARIES})
####

#### Ransac 2d
add_executable(ransac2d src/quiz/ransac/ransac2d.cpp src/render/render.cpp src/processPointClouds.cpp)
ament_target_dependencies(ransac2d ${dependencies})
target_link_libraries(ransac2d ${PCL_LIBRARIES} ${EIGEN_LIBRARIES})
####

#### Cluster
add_executable(cluster src/quiz/cluster/cluster.cpp src/render/render.cpp src/processPointClouds.cpp)
ament_target_dependencies(cluster ${dependencies})
target_link_libraries(cluster ${PCL_LIBRARIES} ${EIGEN_LIBRARIES})
####

# INSTALL

install(DIRECTORY data config launch
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

install(TARGETS environment ransac2d cluster
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

## EXPORTS
ament_export_include_directories(
  include
)

ament_export_dependencies(${dependencies})

ament_package()