cmake_minimum_required(VERSION 2.8.3)
project(fetch_nav)

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

