#include "dynamics_estimator_ros/observer_pub.hpp"


namespace dynamics_estimator_ros
{
    ObserverPub::ObserverPub() : nh_{}
    {
        // Reset the containers
        resetContainers();
        
        // Get the robot description 
        std::string robot_description_parameter {"/robot_description"};
        if (!nh_.getParam("robot_description_parameter", robot_description_parameter)) 
        {
            ROS_WARN_STREAM("Failed getting robot description parameter, defaulting to '" << robot_description_parameter << "'!");
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
        // Get parameters for the frame addition
        std::string frame_name_parameter {"/frame_name"};
        std::string frame_parent_parameter {"/parent_frame_name"};

        if (!nh_.getParam("frame_name_parameter", frame_name_parameter)) 
        {
            ROS_WARN_STREAM("Failed getting frame name parameter, defaulting to '" << frame_name_parameter << "'!");
        }
        if (!nh_.getParam("frame_parent_parameter", frame_parent_parameter)) 
        {
            ROS_WARN_STREAM("Failed getting frame parent parameter, defaulting to '" << frame_parent_parameter << "'!");
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
        if (!nh_.getParam("offset_oreintation", orientation_offset_lst))
        {
            ROS_WARN_STREAM("Failed getting frame name parameter, defaulting to '" << frame_name_parameter << "'!");
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
                                     frame_name_parameter,
                                     frame_parent_parameter,
                                     tar_offset,
                                     pinocchio::FrameType::OP_FRAME);

        // Check the new frame is there
        if (!robot_model->existFrame(frame_name_parameter))
        {
            ROS_WARN_STREAM("New Frame failed to be added to robot model!!!");
        }

        // Create the observer
        dynamics_observer_ = std::make_unique<dynamics_estimator::DynamicsObserver>(robot_model);
    }

    void ObserverPub::resetContainers()
    {
        framePosition_.setZero();
        frameQuat_.setIdentity();

        frameVel_.setZero();
    }

    
} // namespace dynamics_estimator_ros

