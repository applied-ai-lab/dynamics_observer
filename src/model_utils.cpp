#include "dynamics_estimator/model_utils.hpp"

namespace dynamics_estimator 
{   
    
    const pinocchio::FrameIndex ModelUtils::addFrameToFrame(pinocchio::Model& model, 
                                                            std::string const& frame_name,
                                                            std::string const& parent_frame_name,
                                                            pinocchio::SE3 const& transform,
                                                            pinocchio::FrameType frame_type)
    {
        
        pinocchio::Data data = pinocchio::Data(model);
        // Find the parent joint from the parent frame name
        const pinocchio::FrameIndex parent_frame = model.getFrameId(parent_frame_name); 
        const pinocchio::JointIndex parent_joint = model.frames[parent_frame].parent;

        // Find the transform from the parent to join to the frame
        const pinocchio::SE3 joint_to_parent_frame = data.oMi[parent_joint].inverse() * data.oMf[parent_frame];
        const pinocchio::SE3 joint_to_target_frame = joint_to_parent_frame * transform;

        pinocchio::Frame target_frame(frame_name, 
                                      parent_joint, 
                                      parent_frame, 
                                      joint_to_target_frame, 
                                      frame_type);

        return model.addFrame(target_frame);        
    }


} //namespace dynamics_estimator
