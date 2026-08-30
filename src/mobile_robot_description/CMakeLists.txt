cmake_minimum_required(VERSION 3.8)

project(mobile_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    rviz
    meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/arm_gripper_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
