#include "dynamics_estimator/utils.hpp"

#include <cmath>

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/algorithm/model.hpp>

#include <Eigen/Eigen>


namespace dynamics_estimator {

  Eigen::VectorXd joint_ros_to_pinocchio(Eigen::VectorXd const& q, pinocchio::Model const& model) {
    Eigen::VectorXd q_pin {model.nq};
    for (std::size_t i = 0; i < model.joints.size() - 1; i++) {
      pinocchio::JointIndex const jidx {model.getJointId(model.names[i + 1])};
      int const qidx {model.idx_qs[jidx]};
      // 2 corresponds to continuous joints in Pinocchio
      if (model.nqs[jidx] == 2) {
        q_pin(qidx) = std::cos(q(i));
        q_pin(qidx + 1) = std::sin(q(i));
      } else {
        q_pin(qidx) = q(i);
      }
    }
    return q_pin;
  }

} // dynamics_estimator