// sphere_segmentation.h
#ifndef SPHERE_SEGMENTATION_H
#define SPHERE_SEGMENTATION_H

#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <Eigen/Geometry>
#include <gazebo_msgs/LinkStates.h>
#include <process_lidar/Sphere.h>
#include <process_lidar/Sphere_array.h>

typedef pcl::PointXYZ PointT;


class SubscribeProcessPublish{
    public:
    SubscribeProcessPublish(ros::NodeHandle* nodehandle); //Constructor
    void TruePositionCallBack(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void processLidarMeasurementCallBack(const pcl::PCLPointCloud2ConstPtr& cloud);      

    private:
    ros::NodeHandle _nh;
    ros::Subscriber vel_subscriber;
    ros::Subscriber odom_subscriber;

    ros::Publisher filter_publisher;
    ros::Publisher truepos_publisher;
    ros::Publisher measuredpos_publisher;

    process_lidar::Sphere true_data;
    process_lidar::Sphere mes_data;
    process_lidar::Sphere_array true_data_vec;
    process_lidar::Sphere_array mes_data_vec;

};

#endif