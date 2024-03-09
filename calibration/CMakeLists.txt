cmake_minimum_required(VERSION 3.0.2)
project(calibration)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  rosbag
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    rosbag
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    scripts/create_bag.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
