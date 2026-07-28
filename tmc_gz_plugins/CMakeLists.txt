# This package has no tests
cmake_minimum_required(VERSION 3.5)
project(tmc_gz_plugins)

find_package(ament_cmake REQUIRED)

find_package(gz-sim8 REQUIRED)
set(GZ_GAZEBO_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

add_library(${PROJECT_NAME}-odometry-publisher-system SHARED src/odometry_publisher.cpp)
target_link_libraries(${PROJECT_NAME}-odometry-publisher-system
  gz-sim${GZ_GAZEBO_VER}::core
  gz-plugin${GZ_PLUGIN_VER}::register
)

install(TARGETS ${PROJECT_NAME}-odometry-publisher-system
  DESTINATION lib
)

ament_export_libraries(${PROJECT_NAME}-odometry-publisher-system)
ament_package()