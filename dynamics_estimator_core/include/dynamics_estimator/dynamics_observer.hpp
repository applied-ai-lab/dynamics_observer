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
            /**\fn DynamicsObserver
             * \brief
             *   Constructor that allocates all necessary structures
             * 
             * \param[in] robot_model
             *   The robot model
            */
            DynamicsObserver(std::shared_ptr<pinocchio::Model> robot_model);


            /**\fn advanceKinematics
             * \brief
             *   Update the kinematic tree
             * 
             * \param[in] q
             *   Joint state
             * \param[in] qdot
             *   Joint velocities
            */
            void advanceKinematics(Eigen::VectorXd const& q,
                                   Eigen::VectorXd const& qdot);


            /**\fn computeFrameJacobian
             * \brief
             *   Compute jacobian for the named frame
             * 
             * \param[in] q
             *   Joint state
             * \param[in] qdot
             *   Joint velocities
             * \param[in] frame_name
             *   Name of frame for the jacobian
             * \param[in] reference_frame 
             *    The reference frame for the velocity    
            */
            Eigen::MatrixXd computeFrameJacobian(Eigen::VectorXd const& q,
                                                Eigen::VectorXd const& qdot,
                                                const std::string frame_name,
                                                const pinocchio::ReferenceFrame reference_frame);


            /**\fn computeFrameVelocity
             * \brief
             *   Computes classical velocity of frame 
             * 
             * \param[in] q
             *   Joint state
             * \param[in] qdot
             *   Joint velocities
             * \param[in] frame_name
             *   Name of frame for the jacobian
             * \param[in] reference_frame 
             *    The reference frame for the velocity
            */ 
            Eigen::Matrix<double, 6, 1> computeFrameVelocity(Eigen::VectorXd const& q,
                                                             Eigen::VectorXd const& qdot,
                                                             const std::string frame_name,
                                                             const pinocchio::ReferenceFrame reference_frame);

            pinocchio::SE3 findFramePose(Eigen::VectorXd const& q,
                                         Eigen::VectorXd const& qdot,
                                         const std::string frame_name);
            

            // Get vector sizes
            const int getNq() { return robot_model_->nq; }
            const int getNv() { return robot_model_->nv; }
            
            std::shared_ptr<pinocchio::Model> const& getRobotModel() { return robot_model_; }
            std::shared_ptr<pinocchio::Data>  const& getData() { return data_; }    

        private:
            std::shared_ptr<pinocchio::Model> robot_model_;
            std::shared_ptr<pinocchio::Data> data_;

            Eigen::VectorXd q_;            
            Eigen::VectorXd qdot_;
            Eigen::VectorXd tau_;

            // Pose of the frame
            pinocchio::SE3 framePose_;

            // Frame velocity
            Eigen::Matrix<double, 6, 1> frameVel_;

            // Frame Jacobian
            Eigen::MatrixXd J_;
    };

} // namespace dynamics_estimator

#endif //DYNAMICS_ESTIMATOR__DYNAMICS_OBSERVER