#include "dynamics_estimator_ros/observer_pub.hpp"


namespace dynamics_estimator_ros
{
    ObserverPub::ObserverPub(ros::NodeHandle& nh) // : nh_{}
    {
        // Reset the containers
        resetContainers();
        
        // Get the robot description 
        std::string robot_description_parameter {"/robot_description"};
        if (!nh.getParam("robot_description_parameter", robot_description_parameter)) 
        {
            ROS_WARN_STREAM("Failed getting robot description parameter, defaulting to '" << robot_description_parameter << "'!");
        }
        std::string robot_description {};
        if (!nh.getParam(robot_description_parameter, robot_description)) 
        {
            ROS_ERROR_STREAM("Failed getting robot description from '" << robot_description_parameter << "'!");
        }
        // Create the Robot Model
        auto robot_model = std::make_shared<pinocchio::Model>();
        pinocchio::urdf::buildModelFromXML(robot_description, *robot_model.get());
        if (!robot_model) 
        {
            ROS_ERROR_STREAM("Failed to load Pinocchio model from robot description '" << robot_description << "'!");
        }
        
        // Load joint names
        if (!nh.getParam("joints", joint_names_))
        {
            ROS_ERROR_STREAM("Failed to load Joint Names!");    
        }
        
        // Get parameters for the frame addition
        frame_name_parameter_ = "frame_name";
        std::string frame_parent_parameter {"/parent_frame_name"};

        if (!nh.getParam("frame_name", frame_name_parameter_)) 
        {
            ROS_WARN_STREAM("Failed getting frame name parameter, defaulting to '" << frame_name_parameter_ << "'!");
        }
        if (!nh.getParam("parent_frame_name", frame_parent_parameter)) 
        {
            ROS_WARN_STREAM("Failed getting frame parent parameter, defaulting to '" << frame_parent_parameter << "'!");
        }

        // Load the position offset
        std::vector<double> position_offset_lst;
        Eigen::Vector3d position_offset;
        position_offset.setZero();
        if (!nh.getParam("offset_position", position_offset_lst))
        {
            ROS_WARN_STREAM("Failed getting offset position, defaulting to '" << position_offset.transpose() << "'!");
            position_offset.setZero();
        }
        else
        {
            for (int k=0; k<3; ++k)
            {
                position_offset[k] = position_offset_lst[k];
            }
        }
        // Load the orientation offset quaternion
        std::vector<double> orientation_offset_lst;
        Eigen::Quaterniond orientation_offset(Eigen::Matrix3d::Identity());        
        if (!nh.getParam("offset_orientation", orientation_offset_lst))
        {
            ROS_WARN_STREAM("Failed getting frame name parameter, defaulting to '" << frame_name_parameter_ << "'!");
        }
        else
        {
            orientation_offset.w() = orientation_offset_lst[0];
            orientation_offset.x() = orientation_offset_lst[1];
            orientation_offset.y() = orientation_offset_lst[2];
            orientation_offset.z() = orientation_offset_lst[3];  
        }
        // Create the target offset pose
        pinocchio::SE3 tar_offset(orientation_offset.normalized().toRotationMatrix(), position_offset);

        // Create model utils 
        dynamics_estimator::ModelUtils model_utils;
        model_utils.addFrameToFrame(*robot_model, 
                                     frame_name_parameter_,
                                     frame_parent_parameter,
                                     tar_offset,
                                     pinocchio::FrameType::OP_FRAME);

        // Check the new frame is there
        if (!robot_model->existFrame(frame_name_parameter_))
        {
            ROS_WARN_STREAM("New Frame failed to be added to robot model!!!");
        }

        // Create the observer
        dynamics_observer_ = std::make_unique<dynamics_estimator::DynamicsObserver>(robot_model);

        // Set vector sizes
        q_.setZero(dynamics_observer_->getNq());
        qdot_.setZero(dynamics_observer_->getNv());
        tau_.setZero(dynamics_observer_->getNv());

        // Create subscriber
        std::string joint_state_topic;
        if (!nh.getParam("joint_states_topic", joint_state_topic))
        {
            ROS_WARN_STREAM("Could not read Joint State Topic from param server");
        }

        // Ros wait for msg
        const sensor_msgs::JointState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_topic, nh, ros::Duration(5.0));
        createMapFromJointStateMsg(msg);

        ROS_INFO_STREAM("Joint State Msg received");

        // Create publisher
        frameTwistPub_ = nh.advertise<geometry_msgs::TwistStamped>("frame_twist", 1);
        frameTwistStamped_.header.stamp = ros::Time::now();
        frameTwistStamped_.header.frame_id = "base_link";

        nh.subscribe(joint_state_topic, 1, &ObserverPub::jointStateCallback, this, ros::TransportHints().tcpNoDelay());

    }

    void ObserverPub::resetContainers()
    {
        framePosition_.setZero();
        frameQuat_.setIdentity();

        frameVel_.setZero();
    }

    void ObserverPub::createMapFromJointStateMsg(const sensor_msgs::JointState::ConstPtr &msg)
    {
        // Get joint names and positions from the JointState message
        std::vector<std::string> msg_names = msg->name;
        // Get the index of each joint and store in a map
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto it = std::find(msg_names.begin(), msg_names.end(), joint_names_[i]);
            if (it != msg_names.end()) 
            {
                joint_index_map_[joint_names_[i]] = std::distance(joint_names_.begin(), it);
            }
            else
            {
                ROS_WARN_STREAM("Could not find the joint name in the joint state topic!");
            }
        }
    }

    void ObserverPub::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            q_(i)    = msg->position[joint_index_map_[joint_names_[i]]];
            qdot_(i) = msg->velocity[joint_index_map_[joint_names_[i]]];
            tau_(i)  = msg->effort[joint_index_map_[joint_names_[i]]];
        }
        // Update the frame vel
        frameVel_ = dynamics_observer_->computeFrameVelocity(q_, 
                                                             qdot_,
                                                             frame_name_parameter_,
                                                             pinocchio::ReferenceFrame::WORLD);
        publishFrameTwist();
    }

    void ObserverPub::publishFrameTwist()
    {
        frameTwistStamped_.header.stamp = ros::Time::now();
        frameTwistStamped_.twist.linear.x = frameVel_[0];
        frameTwistStamped_.twist.linear.y = frameVel_[1];
        frameTwistStamped_.twist.linear.z = frameVel_[2];
        frameTwistStamped_.twist.angular.x = frameVel_[3];
        frameTwistStamped_.twist.angular.y = frameVel_[4];
        frameTwistStamped_.twist.angular.z = frameVel_[5];

        frameTwistPub_.publish(frameTwistStamped_);
    }

    
} // namespace dynamics_estimator_ros

