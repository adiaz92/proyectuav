cmake_minimum_required(VERSION 2.8.3)
project(transform_pose)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  fiducial_msgs
  tf
  tf2_geometry_msgs
  tf2
  mission_planner
)

SET(SRCS
       src/transform_pose_main.cpp
       src/transform_pose.cpp)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    fiducial_msgs
    tf
    tf2_geometry_msgs
    tf2
    mission_planner
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

add_executable(transform_pose ${SRCS})
add_dependencies(transform_pose ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_pose ${catkin_LIBRARIES})
