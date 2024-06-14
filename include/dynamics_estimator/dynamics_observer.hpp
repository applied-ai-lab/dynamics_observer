#ifndef DYNAMICS_ESTIMATOR__DYNAMICS_OBSERVER
#define DYNAMICS_ESTIMATOR__DYNAMICS_OBSERVER

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
    class DynamicsObserver
    {
        public:
            DynamicsObserver(std::shared_ptr<pinocchio::Model> robot_model);

            void advanceKinematics(Eigen::VectorXd const& q,
                                   Eigen::VectorXd const& qdot);

            /*
            enum ReferenceFrame
            {
              WORLD = 0, 
              LOCAL = 1, 
              LOCAL_WORLD_ALIGNED = 2 
            };
            */
            Eigen::MatrixXd computeFrameJacbian(Eigen::VectorXd const& q,
                                                Eigen::VectorXd const& qdot,
                                                const std::string frame_name,
                                                const pinocchio::ReferenceFrame reference_frame);

            Eigen::Matrix<double, 6, 1> computeFrameVelocity(Eigen::VectorXd const& q,
                                                             Eigen::VectorXd const& qdot,
                                                             const std::string frame_name,
                                                             const pinocchio::ReferenceFrame reference_frame);

            std::shared_ptr<pinocchio::Model> const& getRobotModel() { return robot_model_; }
            std::shared_ptr<pinocchio::Data>  const& getData() { return data_; }    

        private:
            std::shared_ptr<pinocchio::Model> robot_model_;
            std::shared_ptr<pinocchio::Data> data_;

            Eigen::VectorXd q_;            
            Eigen::VectorXd qdot_;
            Eigen::VectorXd tau_;

            // Fame velocity
            Eigen::Matrix<double, 6, 1> frameVel_;

            // Frame Jacobian
            Eigen::MatrixXd J_;
    };

} // namespace dynamics_estimator

#endif //DYNAMICS_ESTIMATOR__DYNAMICS_OBSERVER