#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "dynamics_estimator/dynamics_observer.hpp"
#include "dynamics_estimator/model_utils.hpp"


int main(int /* argc */, char ** /* argv */)
{

    // Filename
    std::string urdf_path = PATH_TO_DIR + std::string("/test/resources/oxf20_right_arm.urdf");
    std::cout << "Path to URDF: " << urdf_path << std::endl;

    auto robot_model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_path, *robot_model.get());

    // Add frame to the model 
    dynamics_estimator::ModelUtils model_utils;
    const pinocchio::SE3 tarOffset(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 0.08));
    auto target_id = model_utils.addFrameToFrame(*robot_model.get(),
                                                 "right_target",
                                                 "right_kinova_arm_tool_frame",
                                                 tarOffset);

    // Check that the new frame exists
    std::cout << robot_model->existFrame("right_target") << std::endl;
    std::cout << robot_model->getFrameId("right_target") << std::endl;

    // Observer
    dynamics_estimator::DynamicsObserver observer(robot_model);
    

    Eigen::MatrixXd J(6, robot_model->nv);
    J.setZero(); 

    // Create random joint configuration
    Eigen::VectorXd q = 0.0 * Eigen::VectorXd::Ones(robot_model->nq);
    Eigen::VectorXd qdot = 0.5 * Eigen::VectorXd::Ones(robot_model->nv);
    Eigen::VectorXd xdot(6);


    // Get the Jacobian
    J = observer.computeFrameJacobian(q, 
                                 qdot, 
                                 "right_kinova_arm_joint_4",
                                 pinocchio::ReferenceFrame::WORLD);

    std::cout << J << std::endl;

    xdot.noalias() = J * qdot;

    // Use get frame vel
    auto xdot_vel = observer.computeFrameVelocity(q, 
                                              qdot, 
                                              "right_kinova_arm_joint_4",
                                              pinocchio::ReferenceFrame::WORLD);

    std::cout << " ===== X DOT ======" << std::endl;
    std::cout << xdot.transpose() << std::endl;
    std::cout << " ===== X DOT VEL ==" << std::endl;
    std::cout << xdot_vel.transpose() << std::endl;

    return 0;
}