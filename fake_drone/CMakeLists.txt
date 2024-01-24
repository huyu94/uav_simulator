cmake_minimum_required(VERSION 3.0.2)
project(fake_drone)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_drone
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (poscmd_2_odom src/poscmd_2_odom.cpp )
target_link_libraries(poscmd_2_odom
  ${catkin_LIBRARIES})



