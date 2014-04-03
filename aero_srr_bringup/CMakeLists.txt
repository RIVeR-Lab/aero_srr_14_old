cmake_minimum_required(VERSION 2.8.3)
project(aero_srr_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  CATKIN_DEPENDS aero_bringup base_local_planner
)
