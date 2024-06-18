#include "dynamics_estimator/dynamics_observer.hpp"

namespace dynamics_estimator
{
    DynamicsObserver::DynamicsObserver(std::shared_ptr<pinocchio::Model> robot_model) : robot_model_{robot_model},
                                                                                        data_{std::make_shared<pinocchio::Data>(*robot_model_.get())}    
    {
        q_.setZero(robot_model_->nq);
        qdot_.setZero(robot_model_->nv);
        tau_.setZero(robot_model_->nv);

        // Init Pose
        framePose_.translation(Eigen::Vector3d::Zero());
        framePose_.rotation(Eigen::Matrix3d::Identity());

        // Frame velocity hence 6 units
        frameVel_.setZero();
        J_.setZero(6, robot_model_->nv);
    }

    void DynamicsObserver::advanceKinematics(Eigen::VectorXd const& q,
                                             Eigen::VectorXd const& qdot)
    {
        q_ = q;
        qdot_ = qdot;

        pinocchio::forwardKinematics(*robot_model_.get(), *data_.get(), q_, qdot_);        
        return;
    }
    
    Eigen::MatrixXd DynamicsObserver::computeFrameJacobian(Eigen::VectorXd const& q,
                                                           Eigen::VectorXd const& qdot,
                                                           const std::string frame_name,
                                                           const pinocchio::ReferenceFrame reference_frame)
    {
        // Advance Kinematics
        advanceKinematics(q, qdot); 
        // Update all joint Jacobians
        pinocchio::computeJointJacobians(*robot_model_.get(),
                                         *data_.get(),
                                          q);
        // Set the Jacobian to zero
        J_.setZero();
        // Get the Jacobian after computing all of them
        pinocchio::getFrameJacobian(*robot_model_.get(),
                                    *data_.get(),
                                    robot_model_->getFrameId(frame_name),
                                    reference_frame,
                                    J_);
        return J_;
    }

    Eigen::Matrix<double, 6, 1> DynamicsObserver::computeFrameVelocity(Eigen::VectorXd const& q,
                                                                       Eigen::VectorXd const& qdot,
                                                                       const std::string frame_name,
                                                                       const pinocchio::ReferenceFrame reference_frame)
    {
        J_ = computeFrameJacobian(q, qdot, frame_name, reference_frame);
        frameVel_.noalias() = J_ * qdot;
        return frameVel_;
    }

    pinocchio::SE3 DynamicsObserver::findFramePose(Eigen::VectorXd const& q,
                                                   Eigen::VectorXd const& qdot,
                                                   const std::string frame_name)
    {
        // Advance Kinematics
        advanceKinematics(q, qdot);

        framePose_ = data_->oMf[robot_model_->getFrameId(frame_name)];
        return framePose_;
    }

} //namespace dynamics_estimator
