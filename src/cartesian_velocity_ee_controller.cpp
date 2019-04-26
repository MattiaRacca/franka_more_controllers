// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_more_controllers/cartesian_velocity_ee_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace franka_more_controllers {

bool CartesianVelocityEEController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  robot_hardware_ = robot_hardware;
  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id_ + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id_ + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

    auto O_T_EE = state_handle.getRobotState().O_T_EE;
    Eigen::Matrix<double, 4, 4> transform_matrix(O_T_EE.data());
    std::cout << "Transformation Matrix:\n" << transform_matrix << std::endl;

    Eigen::Matrix<double, 4, 1> new_pose, O_new_pose;
    new_pose << 0.0, 0.0, 0.1, 1.0; // 10 cm on the z axis

    O_new_pose = transform_matrix * new_pose;
    std::cout << "New pose in O frame:\n" << O_new_pose << std::endl;

  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityEEController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianVelocityEEController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianVelocityEEController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;

  double time_max = 4.0;
  double v_max = 0.005;
  double angle = M_PI / 4.0;
  double cycle = std::floor(
      pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max));
  double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));

  double v_x = std::cos(angle) * v;

  auto state_interface = robot_hardware_->get<franka_hw::FrankaStateInterface>();
  auto state_handle = state_interface->getHandle(arm_id_ + "_robot");
  auto O_T_EE = state_handle.getRobotState().O_T_EE;
  Eigen::Matrix<double, 4, 4> transform_matrix(O_T_EE.data());

  Eigen::Matrix<double, 3, 1> velocity_in_EE, velocity_in_O;
  velocity_in_EE << 0.0, 0.0, v_max;

  velocity_in_O = transform_matrix.block<3,3>(0,0) * velocity_in_EE;

  std::array<double, 6> command = {{velocity_in_O(0), velocity_in_O(1), velocity_in_O(2),
                                    0.0, 0.0, 0.0}};
  velocity_cartesian_handle_->setCommand(command);
}

void CartesianVelocityEEController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_more_controllers

PLUGINLIB_EXPORT_CLASS(franka_more_controllers::CartesianVelocityEEController,
                       controller_interface::ControllerBase)
