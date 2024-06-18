#include "dynamics_estimator/model_utils.hpp"

namespace dynamics_estimator 
{   
    
    const pinocchio::FrameIndex ModelUtils::addFrameToFrame(pinocchio::Model& model, 
                                                            std::string const& frame_name,
                                                            std::string const& parent_frame_name,
                                                            pinocchio::SE3 const& transform,
                                                            pinocchio::FrameType frame_type)
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
        pinocchio::Data data = pinocchio::Data(model);
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        // Find the parent joint from the parent frame name
        const pinocchio::FrameIndex parent_frame = model.getFrameId(parent_frame_name); 
        const pinocchio::JointIndex parent_joint = model.frames[parent_frame].parent;

        // Find the transform from the parent to join to the frame
        const pinocchio::SE3 joint_to_parent_frame = data.oMi[parent_joint].inverse() * data.oMf[parent_frame];
        
        // std::cout << model.names[parent_joint] << std::endl;
        // std::cout << "Joint to parent frame: " << std::endl;
        // std::cout << data.oMi[parent_joint].toHomogeneousMatrix() << std::endl;
        // std::cout << "Joint to parent frame: " << std::endl;
        // std::cout << joint_to_parent_frame.toHomogeneousMatrix() << std::endl;
        // std::cout << "Joint to parent frame: " << std::endl;
        // std::cout << data.oMf[parent_frame].toHomogeneousMatrix() << std::endl;
        

        const pinocchio::SE3 joint_to_target_frame = joint_to_parent_frame * transform;

        pinocchio::Frame target_frame(frame_name, 
                                      parent_joint, 
                                      parent_frame, 
                                      joint_to_target_frame, 
                                      frame_type);

        return model.addFrame(target_frame);        
    }


} //namespace dynamics_estimator
