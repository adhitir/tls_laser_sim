cmake_minimum_required(VERSION 3.0.2)
project(sim_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  rviz
  tf
  urdf
  xacro
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    gazebo_ros
    geometry_msgs
    roscpp
    rviz
    tf
    urdf
    xacro
)


add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_library(tc_sim_plugin src/sim_plugin.cpp)
target_link_libraries(tc_sim_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

