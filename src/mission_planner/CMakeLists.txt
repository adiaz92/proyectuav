cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  fiducial_msgs
  final_aerial_project
  tf
  tf2_geometry_msgs
  tf2
  message_generation
)


## Generate messages in the 'msg' folder
  add_message_files(
  FILES
  transform_type.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

SET(SRCS
       src/mission_planner_main.cpp
       src/mission_planner.cpp)

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    fiducial_msgs
    final_aerial_project
    tf
    tf2_geometry_msgs
    tf2
    message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

add_executable(mission_planner ${SRCS})
add_dependencies(mission_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_planner ${catkin_LIBRARIES})
