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


typedef pcl::PointXYZ PointT;

class SubscribeProcessPublish {
    public:
        SubscribeProcessPublish() {
            // assign subscriber
            this->subscriber = this->nh.subscribe<pcl::PCLPointCloud2>("/velodyne_points", 5, &SubscribeProcessPublish::processLidarMeasurementCallBack, this);
            
            // assign publisher
            this->publisher = this->nh.advertise<pcl::PCLPointCloud2> ("output", 1);
            //this->publisher = nh.advertise<vector<pcl_msgs::ModelCoefficients>> ("output", 1);


        }
        void processLidarMeasurementCallBack(const pcl::PCLPointCloud2ConstPtr& cloud) {      
            //std::cout << "Received lidar measurement made at " << cloud->header.seq << std::endl;

            /*            
            // define a new container for the data
            pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
            
            // define a voxelgrid
            pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
            // set input to cloud
            voxelGrid.setInputCloud(cloud);
            // set the leaf size (x, y, z)
            voxelGrid.setLeafSize(0.1, 0.1, 0.1);
            // apply the filter to dereferenced cloudVoxel
            voxelGrid.filter(*cloudVoxel);
            */

            // cascade the floor removal filter and define a container for floorRemoved	
            pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());
            
            // define a PassThrough
            pcl::PassThrough<pcl::PCLPointCloud2> pass;
            
            // set input to cloudVoxel
            pass.setInputCloud(cloud);
            // filter along z-axis
            pass.setFilterFieldName("z");
            // set z-limits
            pass.setFilterLimits(-0.2, 1.0);
            pass.filter(*floorRemoved);

            // Publish the data for visualisation
            //this->publisher.publish (*floorRemoved);

            // define a point cloud of type PointXYZ
            pcl::PointCloud<pcl::PointXYZ> pclXYZ;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclXYZ_ptr;
            
            // copy the contents of the floorRemoved to pclXYZ
            pcl::fromPCLPointCloud2(*floorRemoved, pclXYZ);
            pclXYZ_ptr = pclXYZ.makeShared ();

            std::cerr << "PointCloud has: " << pclXYZ_ptr->size () << " data points." << std::endl;

            //define an object for normal estimator
            pcl::NormalEstimation<PointT, pcl::Normal> ne;
        
            //define a container for the normalized cloud and KDtree points
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  
            // Estimate point normals

            ne.setSearchMethod (tree);
            //ne.setInputCloud (pclXYZ_ptr);
            ne.setKSearch (50);
            //ne.compute (*cloud_normals);

            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

            // Create the segmentation object for sphere segmentation and set all the parameters
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_SPHERE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight (0.1);
            seg.setMaxIterations (10000);
            seg.setDistanceThreshold (0.05);
            seg.setRadiusLimits (0, 1);
            //seg.setInputCloud (pclXYZ_ptr);
            //seg.setInputNormals (cloud_normals);

            pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);

            //seg.segment (*inliers_sphere, *coefficients_sphere);
            //std::cerr << "sphere coefficients: " << *coefficients_sphere << std::endl;


            pcl::ExtractIndices<PointT> extract;

            // Write the sphere inliers to disk
            //extract.setInputCloud (pclXYZ_ptr);
            //extract.setIndices (inliers_sphere);
            //extract.setNegative (false);
            //pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
            //extract.filter (*cloud_sphere);
            //if (cloud_sphere->points.empty ()) 
            //    std::cerr << "Can't find the spherical component." << std::endl;
            //else
            //{
            //    std::cerr << "PointCloud representing the spherical component: " << cloud_sphere->size () << " data points." << std::endl;
                //writer.write ("table_scene_mug_stereo_textured_sphere.pcd", *cloud_sphere, false);
            //}

            int original_size(pclXYZ_ptr->height*pclXYZ_ptr->width);
            int n_spheres(0);

            //vector<pcl_msgs::ModelCoefficients> all_spheres;
            std::vector<pcl::ModelCoefficients> all_spheres;

            while(pclXYZ_ptr->height*pclXYZ_ptr->width > original_size*0.05){

                // Estimate point normals

                ne.setInputCloud (pclXYZ_ptr);
                ne.compute (*cloud_normals);

                seg.setInputCloud (pclXYZ_ptr);
                seg.setInputNormals (cloud_normals);
                seg.segment (*inliers_sphere, *coefficients_sphere);
                std::cerr << "sphere coefficients: " << *coefficients_sphere << std::endl;

                //pcl_msgs::ModelCoefficients ros_coefficients;
                //pcl_conversions::fromPCL(*coefficients_sphere, ros_coefficients);

                all_spheres.push_back(*coefficients_sphere);

                // Check result
                if (inliers_sphere->indices.size() == 0)
                    break;

                // Write the sphere inliers to disk
                extract.setInputCloud (pclXYZ_ptr);
                extract.setIndices (inliers_sphere);
                extract.setNegative (true);

                pcl::PointCloud<PointT> cloud_sphere;
                extract.filter (cloud_sphere);
                pclXYZ_ptr->swap(cloud_sphere);

                ROS_INFO("fitted sphere %i: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)", n_spheres,
                 coefficients_sphere->values[0],(coefficients_sphere->values[1]>=0?"+":""),
                 coefficients_sphere->values[1],(coefficients_sphere->values[2]>=0?"+":""),
                 coefficients_sphere->values[2],(coefficients_sphere->values[3]>=0?"+":""),
                 coefficients_sphere->values[3],
                 inliers_sphere->indices.size(),original_size);

                n_spheres++;              
                
            }
            //this->publisher.publish (ros_coefficients);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;

};


int main(int argv, char** argc)
{
	// initialise the node	
	ros::init(argv, argc, "process_lidar");

	std::cout << "Process_lidar node initialised" << std::endl;

	// create instance of PublishSubscribe
	SubscribeProcessPublish process;

	// Handle ROS communication events
	ros::spin();

	return 0;
}