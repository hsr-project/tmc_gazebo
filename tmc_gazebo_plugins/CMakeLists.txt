cmake_minimum_required(VERSION 3.8)
project(tmc_gazebo_plugins)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(angles REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tmc_control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(tmc_gazebo_ros_mimic_joint SHARED src/gazebo_ros_mimic_joint.cpp)
ament_target_dependencies(tmc_gazebo_ros_mimic_joint gazebo_dev gazebo_ros rclcpp)

add_library(tmc_gazebo_ros_hrh_gripper SHARED src/gazebo_ros_hrh_gripper.cpp)
ament_target_dependencies(tmc_gazebo_ros_hrh_gripper angles control_msgs control_toolbox gazebo_dev gazebo_ros rclcpp rclcpp_action tmc_control_msgs trajectory_msgs)

add_library(tmc_gazebo_ros_bumper SHARED src/gazebo_ros_bumper.cpp)
ament_target_dependencies(tmc_gazebo_ros_bumper gazebo_ros gazebo_msgs)

add_library(tmc_gazebo_grasp_hack_hrh_gripper SHARED src/gazebo_grasp_hack_base.cpp src/gazebo_grasp_hack_hrh_gripper.cpp)
ament_target_dependencies(tmc_gazebo_grasp_hack_hrh_gripper gazebo_ros gazebo_msgs)

add_library(tmc_gazebo_grasp_hack_rh_p12_rn SHARED src/gazebo_grasp_hack_base.cpp src/gazebo_grasp_hack_rh_p12_rn.cpp)
ament_target_dependencies(tmc_gazebo_grasp_hack_rh_p12_rn gazebo_ros gazebo_msgs)

add_library(tmc_gazebo_ros_p3d SHARED thirdparty/gazebo_ros_p3d.cpp)
ament_target_dependencies(tmc_gazebo_ros_p3d gazebo_dev gazebo_ros nav_msgs rclcpp tf2_geometry_msgs)

install(TARGETS tmc_gazebo_ros_mimic_joint tmc_gazebo_ros_hrh_gripper tmc_gazebo_ros_bumper tmc_gazebo_grasp_hack_hrh_gripper tmc_gazebo_grasp_hack_rh_p12_rn tmc_gazebo_ros_p3d
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(angles)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(rclcpp)
ament_export_dependencies(trajectory_msgs)

ament_export_libraries(tmc_gazebo_mimic_plugin)
ament_export_libraries(tmc_gazebo_ros_hrh_gripper)
ament_export_libraries(tmc_gazebo_ros_bumper)
ament_export_libraries(tmc_gazebo_grasp_hack_hrh_gripper)
ament_export_libraries(tmc_gazebo_grasp_hack_rh_p12_rn)
ament_export_libraries(tmc_gazebo_ros_p3d)

ament_package()
