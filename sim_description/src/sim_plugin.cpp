#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo{

    class six_dof_motion : public ModelPlugin {
        public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
            // store the model pointer to the model
            this->model = _parent;

            //Listen to the update event. The event is broadcast in every simualtion iteration
            this->updateConnection = event::Events::ConnectWorldUpdateBegin( std::bind(&six_dof_motion::OnUpdate, this));
            
            //this->old_secs = ros::Time::now().toSec();

            std::string vel_topicname = "/cmd_vel";

            if (!ros::isInitialized()){
                int argc = 0;
                char **argv = NULL;
                ros::init( argc,argv, "sim_vel_node", ros::init_options::NoSigintHandler);
            }
            
            // Initialize the rosnode
            this->rosNode.reset(new ros::NodeHandle("sim_vel_node"));

            // Initialize the subscriber for the cmd_vel topic
            ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
                vel_topicname,
                1, 
                boost::bind(&six_dof_motion::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue );

            this->rosSub = this->rosNode->subscribe(so);

            this->rosQueueThread = std::thread(std::bind(&six_dof_motion::QueueThread, this));

            ROS_WARN("Loaded six_dof_motion plugin with parent... %s ", this->model->GetName().c_str());
        }

        public: void OnUpdate(){

        }

        void MoveSim3D(float linear_x_vel, float linear_y_vel, float linear_z_vel, float angular_x_vel, float angular_y_vel, float angular_z_vel){
            std::string model_name = this->model->GetName();
            ROS_DEBUG("Moving model = %s", model_name.c_str());

            this->model->SetLinearVel(ignition::math::Vector3d(linear_x_vel,linear_y_vel,linear_z_vel));
            this->model->SetAngularVel(ignition::math::Vector3d(angular_x_vel,angular_y_vel,angular_z_vel));

            ROS_DEBUG("Moving model = %s...END", model_name.c_str());

        }

        public: void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg){
            this->MoveSim3D(_msg->linear.x, _msg->linear.y, _msg->linear.z, _msg->angular.x, _msg->angular.y, _msg->angular.z);
            //ROS_DEBUG("Recieving message...END");
        }

        private: void QueueThread(){
            static const double timeout = 0.01;
            while(this->rosNode->ok()){
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }


        private: physics::ModelPtr model;

        private: event::ConnectionPtr updateConnection;
        
        //double old_secs;

        private: std::unique_ptr<ros::NodeHandle> rosNode;
        
        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;
        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;
        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;    };

    GZ_REGISTER_MODEL_PLUGIN(six_dof_motion)

}
