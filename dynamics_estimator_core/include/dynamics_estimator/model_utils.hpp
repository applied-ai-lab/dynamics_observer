#ifndef DYNAMICS_ESTIMATOR__MODEL_UTILS
#define DYNAMICS_ESTIMATOR__MODEL_UTILS

#include <chrono>
#include <memory>
#include <string>

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Eigen>

namespace dynamics_estimator
{
    class ModelUtils
    {
        public:
            
            ModelUtils() = default;

            const pinocchio::FrameIndex addFrameToFrame(pinocchio::Model& model, 
                                                        std::string const& frame_name,
                                                        std::string const& parent_frame_name,
                                                        pinocchio::SE3 const& transform,
                                                        pinocchio::FrameType frame_type=pinocchio::FrameType::OP_FRAME);
    };

} // namespace dynamics_estimator

#endif //DYNAMICS_ESTIMATOR__MODEL_UTILS