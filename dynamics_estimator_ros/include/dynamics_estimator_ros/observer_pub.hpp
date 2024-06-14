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


namespace dynamics_estimator_ros
{
    class ObserverPub
    {
        public:
            ObserverPub();

            void resetContainers();

        private:
            ros::NodeHandle nh_;

            std::unique_ptr<dynamics_estimator::DynamicsObserver> dynamics_observer_;

            // Containers
            Eigen::Vector3d framePosition_;
            Eigen::Quaterniond frameQuat_;

            Eigen::Matrix<double, 6, 1> frameVel_;
    };

} //dynamics_estimator_ros

#endif //DYNAMICS_ESTIMATOR_ROS__OBSERVER_PUB