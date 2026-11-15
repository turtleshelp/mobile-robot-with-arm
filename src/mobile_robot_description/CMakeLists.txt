# Copyright 2026 Muhammad Ali Sadiq (github.com/turtleshelp)
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)

project(mobile_robot_description)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# Gazebo world plugin: attaches the cube to the gripper once it is
# between the jaws (see src/grasp_assist_plugin.cpp for why)
add_library(grasp_assist_plugin SHARED src/grasp_assist_plugin.cpp)
target_include_directories(grasp_assist_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
ament_target_dependencies(grasp_assist_plugin gazebo_dev gazebo_ros rclcpp std_srvs)
install(TARGETS grasp_assist_plugin LIBRARY DESTINATION lib)

install(
  DIRECTORY
    launch
    urdf
    meshes
    worlds
    config
    media
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/arm_gripper_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
