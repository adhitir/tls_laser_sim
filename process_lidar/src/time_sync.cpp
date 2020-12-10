#include <ros/ros.h>
#include <process_lidar/Sphere.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

using namespace message_filters;

ros::Publisher data1_pub;
ros::Publisher data2_pub;


void callback(const process_lidar::Sphere::ConstPtr& data1, const process_lidar::Sphere::ConstPtr& data2)
{

  ROS_INFO("Sync_Callback");

  data1_pub.publish(data1);
  data2_pub.publish(data2);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TimeSynchronizer");
    std::string data1_topic = "/truepos";
    std::string data2_topic = "/measuredpos";


    ros::NodeHandle nh;
    message_filters::Subscriber<process_lidar::Sphere> data1_sub(nh, data1_topic, 100);
    message_filters::Subscriber<process_lidar::Sphere> data2_sub(nh, data2_topic, 100);

    data1_pub = nh.advertise<process_lidar::Sphere>("/synchronized" + data1_topic, 100);
    data2_pub = nh.advertise<process_lidar::Sphere>("/synchronized" + data2_topic, 100);

    typedef sync_policies::ApproximateTime<process_lidar::Sphere, process_lidar::Sphere> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), data1_sub, data2_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        ros::spin();
        loop_rate.sleep(); // Don't forget this! *
    }

    return 0;
}