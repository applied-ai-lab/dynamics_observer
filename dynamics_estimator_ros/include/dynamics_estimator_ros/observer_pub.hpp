#ifndef DYNAMICS_ESTIMATOR_ROS__OBSERVER_PUB
#define DYNAMICS_ESTIMATOR_ROS__OBSERVER_PUB

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/fwd.hpp>

#include <Eigen/Eigen>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <dynamics_estimator/dynamics_observer.hpp>
#include <dynamics_estimator/model_utils.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


namespace dynamics_estimator_ros
{
    class ObserverPub
    {
        public:
            ObserverPub(ros::NodeHandle& nh);

            void resetContainers();

            void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

            void publish();

            void createMapFromJointStateMsg(const sensor_msgs::JointState::ConstPtr &msg);

        private:

            // Ros related things
            ros::NodeHandle nh_;

            // Ros time
            ros::Time time_;

            // Joint state subscriber
            ros::Subscriber jointStateSub_;
            
            // Publisher
            ros::Publisher framePosePub_;
            ros::Publisher frameTwistPub_;
            ros::Publisher jointStatePub_;

            // Dynamics estimator   
            std::unique_ptr<dynamics_estimator::DynamicsObserver> dynamics_observer_;

            // Containers
            Eigen::VectorXd q_, qdot_, tau_; 

            Eigen::Vector3d framePosition_;
            Eigen::Quaterniond frameQuat_;

            pinocchio::SE3 framePose_;
            Eigen::Matrix<double, 6, 1> frameVel_;

            // Joint names
            std::vector<std::string> joint_names_;
            std::string frame_name_parameter_;
            std::string odom_frame_name_;
            
            // Message indexing map
            std::unordered_map<std::string, int> joint_index_map_;

            // Frame Pose msgs
            geometry_msgs::PoseStamped framePoseStamped_;
            // Frame velocity msgs
            geometry_msgs::TwistStamped frameTwistStamped_;
            // Joint State mgs
            sensor_msgs::JointState frameJointMsgs_;
    };

} //dynamics_estimator_ros

#endif //DYNAMICS_ESTIMATOR_ROS__OBSERVER_PUB