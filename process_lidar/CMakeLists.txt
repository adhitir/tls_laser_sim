cmake_minimum_required(VERSION 3.0.2)
project(process_lidar)


find_package(catkin REQUIRED COMPONENTS 
    roscpp
    pcl_conversions
    pcl_ros
    message_generation
    message_filters
)

find_package(Boost REQUIRED COMPONENTS system)


add_message_files(
   FILES
   Sphere.msg
   Sphere_array.msg

 )

#find_package(PCL 1.4 REQUIRED)
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

#catkin_package(
#  CATKIN_DEPENDS message_filters roscpp sensor_msgs
#)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})



include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(process_lidar src/sphere_segmentation.cpp)
add_executable(process_lidar_timesync src/time_sync.cpp)

target_link_libraries(process_lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(process_lidar_timesync ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#add_dependencies(sphere_segmentation ${catkin_EXPORTED_TARGETS})
