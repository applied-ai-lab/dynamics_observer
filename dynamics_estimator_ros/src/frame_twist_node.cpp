#include "dynamics_estimator_ros/observer_pub.hpp"

#include <ros/ros.h>


int main(int argc, char* argv[])
{       
    ros::init(argc, argv, "twist_node");

    // Create node
    dynamics_estimator_ros::ObserverPub();

    ros::spin();

    return 0;
}

