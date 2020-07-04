cmake_minimum_required(VERSION 2.8.3)
project(transform_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  fiducial_msgs
  tf
  tf2_geometry_msgs
  tf2
)

SET(SRCS
       src/transform_manager_main.cpp
       src/transform_manager.cpp)

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
    fiducial_msgs
    tf
    tf2_geometry_msgs
    tf2
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

add_executable(transform_manager ${SRCS})
add_dependencies(transform_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_manager ${catkin_LIBRARIES})
