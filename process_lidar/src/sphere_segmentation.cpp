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


typedef pcl::PointXYZ PointT;

class SubscribeProcessPublish {
    public:
        SubscribeProcessPublish() {
            // assign subscriber
            this->vel_subscriber = this->nh.subscribe<pcl::PCLPointCloud2>("/velodyne_points", 5, &SubscribeProcessPublish::processLidarMeasurementCallBack, this);
            this->odom_subscriber = this->nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 5, &SubscribeProcessPublish::TruePositionCallBack, this);
            
            // assign publisher
            this->publisher = this->nh.advertise<pcl::PCLPointCloud2> ("output", 1);
            //this->publisher = nh.advertise<vector<pcl_msgs::ModelCoefficients>> ("output", 1);


        }
        void TruePositionCallBack(const gazebo_msgs::LinkStates::ConstPtr& msg){
            const geometry_msgs::Pose lidar_pose = msg->pose[1];

            Eigen::Quaterniond q_lidar;
            Eigen::Matrix3d R_lidar;
            Eigen::Vector3d t_lidar;

            q_lidar.w() = lidar_pose.orientation.w;
            q_lidar.x() = lidar_pose.orientation.x;
            q_lidar.y() = lidar_pose.orientation.y;
            q_lidar.z() = lidar_pose.orientation.z;
            R_lidar = q_lidar.normalized().toRotationMatrix();
            t_lidar(0) = lidar_pose.position.x;
            t_lidar(1) = lidar_pose.position.y;
            t_lidar(2) = lidar_pose.position.z;

            Eigen::Matrix4d T_lidar; // Your Transformation Matrix
            T_lidar.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
            T_lidar.block<3,3>(0,0) = R_lidar;
            T_lidar.block<3,1>(0,3) = t_lidar;


            Eigen::Quaterniond q;
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            Eigen::Matrix4d T; // Your Transformation Matrix
            Eigen::Matrix4d T_ls; // Tranformation from Lidar to sphere


            for(size_t i=1; i<msg->name.size()-1; i++){
                const geometry_msgs::Pose sphere_pose = msg->pose[i];
                q.w() = sphere_pose.orientation.w;
                q.x() = sphere_pose.orientation.x;
                q.y() = sphere_pose.orientation.y;
                q.z() = sphere_pose.orientation.z;
                R = q.normalized().toRotationMatrix();
                t(0) = sphere_pose.position.x;
                t(1) = sphere_pose.position.y;
                t(2) = sphere_pose.position.z;
                T.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
                T.block<3,3>(0,0) = R;
                T.block<3,1>(0,3) = t;

                //Relative Transformation from lidar to sphere:
                T_ls = T_lidar.inverse()*T;

                int j = i-2;

                // Since we are only interested in the distance value:
                ROS_INFO("pose of sphere (laser frame) %i: x = %f, y = %f z = %f ", j, T_ls(0,3), T_ls(1,3), T_ls(2,3));
                ROS_INFO("pose of sphere (ground truth) %i: x = %f, y = %f z = %f ", j, T(0,3), T(1,3), T(2,3));

            }


        }
        void processLidarMeasurementCallBack(const pcl::PCLPointCloud2ConstPtr& cloud) {      

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
            this->publisher.publish (*floorRemoved);

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

            pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);


            pcl::ExtractIndices<PointT> extract;


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
                //std::cerr << "sphere coefficients: " << *coefficients_sphere << std::endl;

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

                ROS_INFO("fitted sphere %i: x = %f, y = %f, z = %f radius = %f (inliers: %zu/%i)", n_spheres,
                 coefficients_sphere->values[0],coefficients_sphere->values[1],coefficients_sphere->values[2],coefficients_sphere->values[3],
                 inliers_sphere->indices.size(),original_size);

                n_spheres++;              
                
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber vel_subscriber;
        ros::Subscriber odom_subscriber;

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