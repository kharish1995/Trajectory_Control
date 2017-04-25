cmake_minimum_required(VERSION 2.8.3)
project(fetch_nav)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf_conversions
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fetch_nav src/fetch_nav.cpp)

target_link_libraries(fetch_nav
   ${catkin_LIBRARIES}
 )

