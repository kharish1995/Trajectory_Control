# Trajectory_Control
Trajectory Control for Differential Drive Mobile Manipulators

Setting up the workspace,

1) Initialize a catkin workspace in your home folder,

mkdir -p ~/fetch_ws/src

cd ~/fetch_ws/src

catkin_init_wokspace

2) Download all the packages from https://github.com/fetchrobotics and place them in the src folder

3) Add the following line to the CMakeLists.txt file in the fetch_gazebo and fetch_gazebo_demo package. 

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

4) Clone the Trajectory_Control repository from your src folder.

5) Then,

cd ~/fetch_ws

catkin_make


Team Members:
Harish Karunakaran
Gopeshh Raaj
