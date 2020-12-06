cmake_minimum_required(VERSION 3.0.2)
project(process_lidar)


find_package(catkin REQUIRED COMPONENTS 
    roscpp
    pcl_conversions
    pcl_ros
)

#find_package(PCL 1.4 REQUIRED)


catkin_package()


#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})



include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(process_lidar src/cylinder_segmentation.cpp)
target_link_libraries(process_lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES})


