cmake_minimum_required(VERSION 2.8.3)
project(fetch_moveit_tasks)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  fetch_moveit_config   
  moveit_commander   
  moveit_core   
  moveit_fake_controller_manager   
  moveit_msgs   
  moveit_planners_ompl   
  moveit_ros_benchmarks   
  moveit_ros_benchmarks_gui   
  moveit_ros_control_interface   
  moveit_ros_manipulation   
  moveit_ros_move_group   
  moveit_ros_perception   
  moveit_ros_planning   
  moveit_ros_planning_interface   
  moveit_ros_robot_interaction   
  moveit_ros_visualization   
  moveit_ros_warehouse   
  moveit_simple_controller_manager   
  trac_ik_examples   
  trac_ik_kinematics_plugin   
  trac_ik_lib
  roscpp
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_moveit src/test_moveit.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(test_moveit ${catkin_LIBRARIES})

