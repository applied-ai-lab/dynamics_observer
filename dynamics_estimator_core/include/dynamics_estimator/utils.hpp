#ifndef DYNAMICS_ESTIMATOR__UTILS
#define DYNAMICS_ESTIMATOR__UTILS

#include <string>
#include <vector>

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/algorithm/model.hpp>

#include <Eigen/Eigen>


namespace dynamics_estimator {

  /**\fn joint_ros_to_pinocchio
   * \brief
   *   Convert the joint states to the Pinocchio convention
   * 
   * \param[in] q
   *   The input joint angles
   * \param[in] model
   *   The Pinocchio robot model
   * \return
   *   The joint angles converted to the Pinocchio convention
  */
  [[nodiscard]]
  Eigen::VectorXd joint_ros_to_pinocchio(Eigen::VectorXd const& q, pinocchio::Model const& model);

}

#endif // DYNAMICS_ESTIMATOR__UTILS