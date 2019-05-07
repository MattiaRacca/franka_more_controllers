// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_more_controllers/cartesian_impedance_trajectory_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "pseudo_inversion.h"

namespace franka_more_controllers {

bool CartesianImpedanceTrajectoryController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  /*
  sub_target_pose_ = node_handle.subscribe(
          "/target_pose", 20, &CartesianImpedanceTrajectoryController::targetPoseCallback, this,
          ros::TransportHints().reliable().tcpNoDelay());
  */

  move_to_ee_server_ = new actionlib::SimpleActionServer<franka_more_controllers::LinearMotionAction>(
          node_handle, "move_to_ee_server",boost::bind(&CartesianImpedanceTrajectoryController::moveToEECallback, this, _1), false);
  move_to_ee_server_->start();

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceTrajectoryController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceTrajectoryController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceTrajectoryController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceTrajectoryController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceTrajectoryController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceTrajectoryController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceTrajectoryController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceTrajectoryController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_cartesian_impedance_trajectory_param_node_ =
          ros::NodeHandle("dynamic_reconfigure_cartesian_impedance_trajectory_param_node");

  dynamic_server_cartesian_impedance_trajectory_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
      dynamic_reconfigure_cartesian_impedance_trajectory_param_node_);

  dynamic_server_cartesian_impedance_trajectory_param_->setCallback(
          boost::bind(&CartesianImpedanceTrajectoryController::cartesianImpedanceTrajectoryParamCallback, this, _1, _2));

  position_d_.setZero();
  position_d_target_.setZero();
  starting_position_.setZero();

  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  starting_orientation_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  desired_time_ = 1.0; // TODO: better way to avoid division by 0?
  progression_ = 0.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void CartesianImpedanceTrajectoryController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  starting_position_ = initial_transform.translation();
  position_d_target_ = initial_transform.translation();

  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  starting_orientation_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianImpedanceTrajectoryController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // Update desired pose
  progression_ += period.toSec();
  double tau = std::min(progression_/desired_time_, 1.0);
  // Position
  position_d_ = starting_position_*(1 - tau) + position_d_target_*tau;
  // Orientation
  orientation_d_ = starting_orientation_.slerp(tau, orientation_d_target_);

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }

  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering

  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;

  /*
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  Eigen::AngleAxisd aa_orientation_d(orientation_d_);
  Eigen::AngleAxisd aa_orientation_d_target(orientation_d_target_);
  aa_orientation_d.axis() = filter_params_ * aa_orientation_d_target.axis() +
                            (1.0 - filter_params_) * aa_orientation_d.axis();
  aa_orientation_d.angle() = filter_params_ * aa_orientation_d_target.angle() +
                             (1.0 - filter_params_) * aa_orientation_d.angle();
  orientation_d_ = Eigen::Quaterniond(aa_orientation_d);

  speed_ = filter_params_ * speed_target_ + (1.0 - filter_params_) * speed_;
  rotation_speed_ = filter_params_ * rotation_speed_target_ + (1.0 - filter_params_) * rotation_speed_;
  */
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceTrajectoryController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceTrajectoryController::cartesianImpedanceTrajectoryParamCallback(
        franka_example_controllers::compliance_paramConfig &config,
        uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
          << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
          << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
          << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
          << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianImpedanceTrajectoryController::targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
          msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
  // TODO: speed has to be set by the user
  double position_speed_target = 0.01; // m/s
  double rotation_speed_target = 0.5; // rad/s

  // Save current position as starting
  starting_position_ = position_d_;

  // Save current orientation as starting
  starting_orientation_ = orientation_d_;

  // Compute "pace" of motion
  Eigen::Vector3d position_difference = position_d_target_ - position_d_;
  Eigen::Quaterniond rotation_difference(orientation_d_ * last_orientation_d_target.inverse());
  // convert to axis angle
  Eigen::AngleAxisd rotation_difference_angle_axis(rotation_difference);

  desired_time_ = std::max(rotation_difference_angle_axis.angle()/rotation_speed_target,
          position_difference.norm()/position_speed_target);
  progression_ = 0.0;
}

void CartesianImpedanceTrajectoryController::moveToEECallback(const franka_more_controllers::LinearMotionGoalConstPtr &goal){
  position_d_target_ << goal->pose.pose.position.x, goal->pose.pose.position.y, goal->pose.pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << goal->pose.pose.orientation.x, goal->pose.pose.orientation.y,
          goal->pose.pose.orientation.z, goal->pose.pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }

  double position_speed_target = goal->position_speed; // m/s
  double rotation_speed_target = goal->rotation_speed; // rad/s

  // Save current position as starting
  starting_position_ = position_d_;

  // Save current orientation as starting
  starting_orientation_ = orientation_d_;

  // Compute "pace" of motion
  Eigen::Vector3d position_difference = position_d_target_ - position_d_;
  Eigen::Quaterniond rotation_difference(orientation_d_ * last_orientation_d_target.inverse());
  // convert to axis angle
  Eigen::AngleAxisd rotation_difference_angle_axis(rotation_difference);

  desired_time_ = std::max(rotation_difference_angle_axis.angle()/rotation_speed_target,
                           position_difference.norm()/position_speed_target);
  progression_ = 0.0;

  bool motion_done = false;
  while(!motion_done)
  {
    double tau = progression_/desired_time_;
    if(tau < 1.0){
      move_to_ee_feedback_.progression = progression_;
      move_to_ee_server_->publishFeedback(move_to_ee_feedback_);
    } else {
      motion_done = true;
    }
  }

  move_to_ee_result_.final_pose.pose.position.x = position_d_[0];
  move_to_ee_result_.final_pose.pose.position.y = position_d_[1];
  move_to_ee_result_.final_pose.pose.position.z = position_d_[2];
  move_to_ee_result_.final_pose.pose.orientation.x = orientation_d_.x();
  move_to_ee_result_.final_pose.pose.orientation.y = orientation_d_.y();
  move_to_ee_result_.final_pose.pose.orientation.z = orientation_d_.z();
  move_to_ee_result_.final_pose.pose.orientation.w = orientation_d_.w();

  move_to_ee_server_->setSucceeded(move_to_ee_result_);
}

}  // namespace franka_more_controllers

PLUGINLIB_EXPORT_CLASS(franka_more_controllers::CartesianImpedanceTrajectoryController,
                       controller_interface::ControllerBase)