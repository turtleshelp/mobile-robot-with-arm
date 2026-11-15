# Copyright 2026 Muhammad Ali Sadiq (github.com/turtleshelp)
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)

project(mobile_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/mobile_manipulation_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
