// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_controllers/scaled_joint_trajectory_controller.hpp"

#include "angles/angles.h"
#include "fanuc_controllers/ros2_control_compat.hpp"
#include "fanuc_robot_driver/constants.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace fanuc_controllers
{

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_init()
{
  folag_state_ = time_scale_value_.load();
  return JointTrajectoryController::on_init();
}

controller_interface::InterfaceConfiguration ScaledJointTrajectoryController::state_interface_configuration() const
{
  auto state_interface_config = JointTrajectoryController::state_interface_configuration();
  using fanuc_robot_driver::kConnectionStatusName;
  using fanuc_robot_driver::kRobotStatusInterfaceName;
  using fanuc_robot_driver::kStatusCollaborativeSpeedScalingType;
  using fanuc_robot_driver::kStatusMotionPossibleType;
  state_interface_config.names.push_back(std::string(kRobotStatusInterfaceName) + "/" +
                                         kStatusCollaborativeSpeedScalingType);
  state_interface_config.names.push_back(std::string(kRobotStatusInterfaceName) + "/" + kStatusMotionPossibleType);
  return state_interface_config;
}

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  // Initialize slider subscriber for time scaling control
  time_scale_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int32>(
      time_scale_topic_name_, 10,
      std::bind(&ScaledJointTrajectoryController::time_scale_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Time scale subscriber created for topic: %s", time_scale_topic_name_.c_str());

  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type ScaledJointTrajectoryController::update(const rclcpp::Time& time,
                                                                          const rclcpp::Duration& period)
{
  if (!state_interfaces_.empty() && fanuc_controllers::state_value(state_interfaces_.back()) == 0.0)
  {
    last_is_connected_ = false;
    // The robot state indicated that STMO does not have the motion control.
    const auto active_goal = *rt_active_goal_.readFromRT();
    if (active_goal)
    {  // abort the trajectory
      auto result = std::make_shared<FollowJTrajAction::Result>();
      result->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
      result->set__error_string("Aborted: STMO is inactive (!motion_possible)");
      active_goal->setAborted(result);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
      rt_has_pending_goal_ = false;
      RCLCPP_WARN(this->get_node()->get_logger(), "Aborted: STMO is inactive (!motion_possible)");
      new_trajectory_msg_.reset();
      new_trajectory_msg_.initRT(set_hold_position());
    }
    return controller_interface::return_type::OK;
  }
  if (state_interfaces_.size() >= 2)
  {
    // The collaborative speed scaling interface is the second-to-last state interface
    const double tmp_scaling_factor = fanuc_controllers::state_value(state_interfaces_[state_interfaces_.size() - 2]);
    if (std::isfinite(tmp_scaling_factor))
    {
      robot_scaling_factor_ = tmp_scaling_factor;
    }
  }

  if (!last_is_connected_)
  {
    // The controller should update its reference to use the current state
    read_state_from_state_interfaces(state_current_);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    rt_has_pending_goal_ = false;
    pending_traj_msg_ = nullptr;
    rt_active_goal_deferred_ = false;
    new_trajectory_msg_.reset();
    new_trajectory_msg_.initRT(set_hold_position());
  }
  last_is_connected_ = true;

  folag_h_ = period.seconds();
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  // The base controller advances trajectory time by period * scaling_factor_ (since ros2_controllers 4.x
  // it supports speed scaling natively). Feed it the topic scale (low-pass filtered) * the robot's
  // collaborative speed scaling, then let it run the trajectory, tolerance and action-goal logic.
  const double scale_percentage = 0.01 * first_order_lag_filter(static_cast<double>(time_scale_value_.load()));
  scaling_factor_ = scale_percentage * robot_scaling_factor_.load();

  return JointTrajectoryController::update(time, period);
}

double ScaledJointTrajectoryController::first_order_lag_filter(const double filter_input)
{
  // discrete time first order lag filter
  folag_state_ = (1 - folag_h_ / folag_tau_) * folag_state_ + (folag_h_ / folag_tau_) * filter_input;
  return folag_state_;
}

void ScaledJointTrajectoryController::time_scale_callback(const std::shared_ptr<std_msgs::msg::Int32> msg)
{
  // Clamp the slider value between 1 and 100 (1% to 100%)
  int clamped_value = std::clamp(msg->data, 0, 100);
  time_scale_value_.store(clamped_value);
}

}  // namespace fanuc_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_controllers::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
