# This package has no tests
cmake_minimum_required(VERSION 3.8)
project(tmc_gazebo_worlds)

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
find_package(xacro REQUIRED)

file(GLOB _world_xacros "${PROJECT_SOURCE_DIR}/worlds/*.world.xacro")
file(GLOB_RECURSE _component_xacros "${PROJECT_SOURCE_DIR}/worlds/include/*.xacro")

# Generate world files from each *.world.xacro files.
foreach(_xacro ${_world_xacros})
    get_filename_component(_dir ${_xacro} PATH)
    get_filename_component(_name ${_xacro} NAME_WE)
    set(_default_world ${_dir}/${_name}.world)
    set(_fast_world ${_dir}/${_name}_fast.world)
    add_custom_command(
        OUTPUT ${_default_world}
        DEPENDS ${_xacro} ${_component_xacros}
        COMMAND xacro ${_xacro} fast_physics:=false > ${_default_world})
    add_custom_command(
        OUTPUT ${_fast_world}
        DEPENDS ${_xacro} ${_component_xacros}
        COMMAND xacro ${_xacro} fast_physics:=true > ${_fast_world})
    add_custom_target(${PROJECT_NAME}_${_name}_gen_default_world ALL DEPENDS ${_default_world})
    add_custom_target(${PROJECT_NAME}_${_name}_gen_fast_world ALL DEPENDS ${_fast_world})
endforeach()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.world"
  PATTERN "include" EXCLUDE)

ament_package()
