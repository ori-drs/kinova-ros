cmake_minimum_required(VERSION 3.0.2)
project(kinova_hri_estop)

find_package(catkin REQUIRED COMPONENTS
  hri_safety_sense
  kinova_msgs
  rospy
)

catkin_package()