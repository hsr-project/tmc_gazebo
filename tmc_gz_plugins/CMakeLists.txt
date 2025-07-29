# This package has no tests
cmake_minimum_required(VERSION 3.5)
project(tmc_gz_plugins)

find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-plugin1 REQUIRED)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

add_library(${PROJECT_NAME}-odometry-publisher-system SHARED src/odometry_publisher.cpp)
target_link_libraries(${PROJECT_NAME}-odometry-publisher-system
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${PROJECT_NAME}-odometry-publisher-system
  DESTINATION lib
)

ament_export_libraries(${PROJECT_NAME}-odometry-publisher-system)
ament_package()