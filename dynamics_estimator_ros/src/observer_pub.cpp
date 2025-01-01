#include "dynamics_estimator_ros/observer_pub.hpp"


namespace dynamics_estimator_ros
{
    ObserverPub::ObserverPub(ros::NodeHandle& nh) : nh_{nh}
    {
        // Reset the containers
        resetContainers();
        
        // Get the robot description 
        std::string robot_description_parameter {"/robot_description"};
        if (!nh_.getParam("robot_description_parameter", robot_description_parameter)) 
        {
            ROS_WARN_STREAM("Failed getting robot description parameter, defaulting to '" << robot_description_parameter << "'!");
        }
        else
        {
            ROS_INFO_STREAM("==== ROBOT DESCRIPTION PARAMETER:  " << robot_description_parameter);
        }
        std::string robot_description {};
        if (!nh_.getParam(robot_description_parameter, robot_description)) 
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
        if (!nh_.getParam("joints", joint_names_))
        {
            ROS_ERROR_STREAM("Failed to load Joint Names!");    
        }
        
        // Get parameters for the frame addition
        odom_frame_name_ = {"world"};
        frame_name_parameter_ = "frame_name";
        std::string frame_parent_parameter {"/parent_frame_name"};

        if (!nh_.getParam("odom_frame", odom_frame_name_)) 
        {
            ROS_WARN_STREAM("Failed getting odom frame name, defaulting to '" << odom_frame_name_ << "'!");
        }
        else
        {
            ROS_INFO_STREAM("==== ODOM FRAME NAME:  " << odom_frame_name_);
        }
        if (!nh_.getParam("frame_name", frame_name_parameter_)) 
        {
            ROS_WARN_STREAM("Failed getting frame name parameter, defaulting to '" << frame_name_parameter_ << "'!");
        }
        else
        {
            ROS_INFO_STREAM("==== TARGET FRAME NAME:  " << frame_name_parameter_);
        }
        if (!nh_.getParam("parent_frame_name", frame_parent_parameter)) 
        {
            ROS_WARN_STREAM("Failed getting frame parent parameter, defaulting to '" << frame_parent_parameter << "'!");
        }
        else
        {
            ROS_INFO_STREAM("==== PARENT FRAME NAME:  " << frame_parent_parameter);
        }
        // Load the position offset
        std::vector<double> position_offset_lst;
        Eigen::Vector3d position_offset;
        position_offset.setZero();
        if (!nh_.getParam("offset_position", position_offset_lst))
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
        if (!nh_.getParam("offset_orientation", orientation_offset_lst))
        {
            ROS_WARN_STREAM("Failed getting offset orientation, defaulting to '" << orientation_offset.w() << 
                                                                                    orientation_offset.x() << 
                                                                                    orientation_offset.y() << 
                                                                                    orientation_offset.z() << "'!");
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
        dynamics_estimator::ModelUtils model_utils {};
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
        force_.setZero();

        // Create subscriber
        std::string joint_state_topic;
        if (!nh_.getParam("joint_states_topic", joint_state_topic))
        {
            ROS_WARN_STREAM("Could not read Joint State Topic from param server");
        }

        // Ros wait for msg
        const sensor_msgs::JointState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_topic, nh_, ros::Duration(5.0));
        createMapFromJointStateMsg(msg);

        ROS_INFO_STREAM("Joint State Msg received");

        // Create publisher
        framePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("frame_pose", 1);
        framePoseStamped_.header.stamp = ros::Time::now();
        framePoseStamped_.header.frame_id = "base_link";

        frameTwistPub_ = nh_.advertise<geometry_msgs::TwistStamped>("frame_twist", 1);
        frameTwistStamped_.header.stamp = ros::Time::now();
        frameTwistStamped_.header.frame_id = "base_link";

        jointStatePub_ = nh_.advertise<sensor_msgs::JointState>("frame_joint_states", 1);
        frameJointMsgs_.header.stamp = ros::Time::now();
        frameJointMsgs_.name = joint_names_;

        frameJointMsgs_.position.resize(joint_names_.size(), 0.0);
        frameJointMsgs_.velocity.resize(joint_names_.size(), 0.0);
        frameJointMsgs_.effort.resize(joint_names_.size(), 0.0);

        forceStatePub_ = nh_.advertise<sensor_msgs::JointState>("frame_force_states", 1);
        eeForceMsgs_.header.stamp = ros::Time::now();
        eeForceMsgs_.name = forceStateNames;

        eeForceMsgs_.position.resize(forceStateNames.size(), 0.0);
        eeForceMsgs_.velocity.resize(forceStateNames.size(), 0.0);
        eeForceMsgs_.effort.resize(forceStateNames.size(), 0.0);

        jointStateSub_ = nh_.subscribe(joint_state_topic, 1, &ObserverPub::jointStateCallback, this, ros::TransportHints().tcpNoDelay());

        // Get the world name
        pinocchio::FrameIndex world_frame_index = 0;
        auto frames = robot_model->frames;

        ROS_INFO_STREAM("==== FRAME NAMES ====");
        for (size_t i=0; i<frames.size(); ++i)
        {
            ROS_INFO_STREAM(frames[i].name);
        }

        ROS_INFO_STREAM("==== OBSERVER CREATED ====");

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
                joint_index_map_[joint_names_[i]] = std::distance(msg_names.begin(), it);
                std::cout << joint_names_[i] << " " << joint_index_map_[joint_names_[i]] << std::endl;
            }
            else
            {
                ROS_WARN_STREAM("Could not find the joint name in the joint state topic!");
                ROS_WARN_STREAM(joint_names_[i]);
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

        // Check frame still exists
        if (!dynamics_observer_->existFrame(frame_name_parameter_))
        {
            ROS_WARN_STREAM(" TARGET FRAME DOES NOT EXIST: " << frame_name_parameter_);
        }

       
        // Update the frame pose
        framePose_ = dynamics_observer_->findFramePose(q_,
                                                       qdot_,
                                                       odom_frame_name_,
                                                       frame_name_parameter_);

        
        // Update the frame vel
        frameVel_ = dynamics_observer_->computeFrameVelocity(q_, 
                                                             qdot_,
                                                             frame_name_parameter_,
                                                             pinocchio::ReferenceFrame::WORLD);


        // Cacluate the end effector force
        force_ = dynamics_observer_->computeFrameJacobian(q_, 
                                                          qdot_,
                                                          frame_name_parameter_,
                                                          pinocchio::ReferenceFrame::WORLD).transpose().block(0, 0, 6, q_.rows()) * tau_;


        publish();
    }

    void ObserverPub::publish()
    {
        time_ = ros::Time::now();

        frameJointMsgs_.header.stamp = time_;
        for (size_t i=0; i<joint_names_.size(); ++i)
        {
            frameJointMsgs_.position[i] = q_(i);
            frameJointMsgs_.velocity[i] = qdot_(i);
            frameJointMsgs_.effort[i] = tau_(i);
        }

        eeForceMsgs_.header.stamp = time_;
        for (size_t i=0; i<forceStateNames.size(); ++i)
        {
            eeForceMsgs_.effort[i] = force_(i);
        }
        
        framePoseStamped_.header.stamp = time_;

        framePosition_ = framePose_.translation();
        frameQuat_ = Eigen::Quaterniond(framePose_.rotation());

        framePoseStamped_.pose.position.x = framePosition_(0);
        framePoseStamped_.pose.position.y = framePosition_(1);
        framePoseStamped_.pose.position.z = framePosition_(2);

        framePoseStamped_.pose.orientation.w = frameQuat_.w();
        framePoseStamped_.pose.orientation.x = frameQuat_.x();
        framePoseStamped_.pose.orientation.y = frameQuat_.y();
        framePoseStamped_.pose.orientation.z = frameQuat_.z();

        frameTwistStamped_.header.stamp = time_;
        frameTwistStamped_.twist.linear.x = frameVel_[0];
        frameTwistStamped_.twist.linear.y = frameVel_[1];
        frameTwistStamped_.twist.linear.z = frameVel_[2];
        frameTwistStamped_.twist.angular.x = frameVel_[3];
        frameTwistStamped_.twist.angular.y = frameVel_[4];
        frameTwistStamped_.twist.angular.z = frameVel_[5];

        // Publish msgs
        framePosePub_.publish(framePoseStamped_);
        frameTwistPub_.publish(frameTwistStamped_);
        jointStatePub_.publish(frameJointMsgs_);
        forceStatePub_.publish(eeForceMsgs_);
    }

    
} // namespace dynamics_estimator_ros

