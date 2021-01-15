#include "sphere_segmentation.h"
#include <process_lidar/Sphere.h>
#include <process_lidar/Sphere_array.h>

//typedef pcl::PointXYZ PointT;

SubscribeProcessPublish::SubscribeProcessPublish(ros::NodeHandle* nh):_nh(*nh) {
    ROS_INFO("Entering class constructor");
    // assign subscriber
    this->vel_subscriber = this->_nh.subscribe<pcl::PCLPointCloud2>("/velodyne_points", 5, &SubscribeProcessPublish::processLidarMeasurementCallBack, this);
    this->odom_subscriber = this->_nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 5, &SubscribeProcessPublish::TruePositionCallBack, this);
    
    // assign publisher
    this->filter_publisher = this->_nh.advertise<pcl::PCLPointCloud2> ("output", 1);
    this->truepos_publisher = this->_nh.advertise<process_lidar::Sphere_array> ("/truepos", 1);
    this->measuredpos_publisher = this->_nh.advertise<process_lidar::Sphere_array> ("/measuredpos", 1);
}

void SubscribeProcessPublish::TruePositionCallBack(const gazebo_msgs::LinkStates::ConstPtr& msg){
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
    
    this->true_data_vec.sphere_vec.clear();

    for(size_t i=2; i<msg->name.size(); i++){
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
        //ROS_INFO("pose of sphere (ground truth) %i: x = %f, y = %f z = %f ", j, T(0,3), T(1,3), T(2,3));
        
        //this->true_data.header.stamp = ros::Time::now ();


        this->true_data.label = "TruePos";
        this->true_data.radius = 0.5;
        this->true_data.index = j;


        this->true_data.x = T_ls(0,3);
        this->true_data.y = T_ls(1,3);
        this->true_data.z = T_ls(2,3);

        this->true_data_vec.sphere_vec.push_back(true_data);

    }

    ros::spinOnce();

    this->true_data_vec.header.stamp = ros::Time::now ();
    this->truepos_publisher.publish(this->true_data_vec);


    //std::cout<<true_data_vec.size()<<std::endl;
}

void SubscribeProcessPublish::processLidarMeasurementCallBack(const pcl::PCLPointCloud2ConstPtr& cloud) {      

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
    this->filter_publisher.publish (*floorRemoved);

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

    mes_data_vec.sphere_vec.clear();

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
        

        //std::cout<<this->true_data_vec.size()<<std::endl;


        this->mes_data.x = coefficients_sphere->values[0];
        this->mes_data.y = coefficients_sphere->values[1];
        this->mes_data.z = coefficients_sphere->values[2];

        double diff;

        for(size_t i=0; i<this->true_data_vec.sphere_vec.size(); i++){
            diff = std::abs(this->true_data_vec.sphere_vec[i].x - this->mes_data.x) + std::abs(this->true_data_vec.sphere_vec[i].y - this->mes_data.y) + std::abs(this->true_data_vec.sphere_vec[i].z - this->mes_data.z);
            if(diff < 0.1){
                n_spheres = i;
            }
            
            //std::cout << diff << " ";
        }

        //std::cout << std::endl; 
        //this->mes_data.header.stamp = ros::Time::now ();

        this->mes_data.label = "RANSAC";
        this->mes_data.radius = coefficients_sphere->values[3];
        this->mes_data.index = n_spheres;

        
        ROS_INFO("fitted sphere %i: x = %f, y = %f, z = %f radius = %f (inliers: %zu/%i)", n_spheres,
            coefficients_sphere->values[0],coefficients_sphere->values[1],coefficients_sphere->values[2],coefficients_sphere->values[3],
            inliers_sphere->indices.size(),original_size);
        


        //this->measuredpos_publisher.publish(this->mes_data);

        this->mes_data_vec.sphere_vec.push_back(mes_data);
        

        n_spheres++;              
        
    }

    this->mes_data_vec.header.stamp = ros::Time::now ();
    this->measuredpos_publisher.publish(this->mes_data_vec);
}

    

int main(int argv, char** argc)
{
	// initialise the node	
	ros::init(argv, argc, "process_lidar");
    
    ros::NodeHandle nh;

	std::cout << "Process_lidar node initialised" << std::endl;

	// create instance of PublishSubscribe
	SubscribeProcessPublish process(&nh);

	// Handle ROS communication events
	ros::spin();

	return 0;
}