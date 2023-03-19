// Copyright 2019, FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-18
 *
 */
//----------------------------------------------------------------------

#include <memory>
#include <vector>
#include <thread>
#include <angles/angles.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_action/create_server.hpp>
#include <joint_trajectory_controller/trajectory.hpp>

#include "mr_controllers/mr_joint_trajectory_controller.hpp"

std::string set_traj_service_name("/set_trajectory_state");

namespace mr_controllers
{
controller_interface::CallbackReturn MRJointTrajectoryController::on_init()
{
  robot_name_ = get_node()->get_namespace();
  set_traj_client_ = get_node()->create_client<
      mr_msgs::srv::SetTrajectoryState>(set_traj_service_name);
  return JointTrajectoryController::on_init();
}

controller_interface::CallbackReturn MRJointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State& state)
{
  controller_interface::CallbackReturn result = 
    JointTrajectoryController::on_configure(state);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(&MRJointTrajectoryController::goal_received_callback, this, _1, _2),
    std::bind(&MRJointTrajectoryController::goal_cancelled_callback, this, _1),
    std::bind(&MRJointTrajectoryController::goal_accepted_callback, this, _1));
  return result;
}

controller_interface::return_type MRJointTrajectoryController::update(
  const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  auto mark_traj_end = [this]()
  {
    auto req = std::make_shared<mr_msgs::srv::SetTrajectoryState::Request>();
    req->header.stamp = get_node()->now();
    req->header.frame_id = robot_name_;
    req->action = mr_msgs::srv::SetTrajectoryState::Request::TRAJECTORY_DONE_OR_CANCEL;
    std::thread t(&MRJointTrajectoryController::set_trajectory_state, this, req);
    t.detach();
  };

  auto compute_error_for_joint = [&](
                                   JointTrajectoryPoint & error, int index,
                                   const JointTrajectoryPoint & current,
                                   const JointTrajectoryPoint & desired)
  {
    // error defined as the difference between current and desired
    error.positions[index] =
      angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
    if (has_velocity_state_interface_ && has_velocity_command_interface_)
    {
      error.velocities[index] = desired.velocities[index] - current.velocities[index];
    }
    if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
    {
      error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
    }
  };

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  if (current_external_msg != *new_external_msg)
  {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    traj_external_point_ptr_->update(*new_external_msg);
  }

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point =
    [&](auto & joint_interface, const std::vector<double> & trajectory_point_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
    {
      joint_interface[index].get().set_value(trajectory_point_interface[index]);
    }
  };

  // current state update
  state_current_.time_from_start.set__sec(0);
  read_state_from_hardware(state_current_);

  // currently carrying out a trajectory
  if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg())
  {
    bool first_sample = false;
    // if sampling the first time, set the point before you sample
    if (!(*traj_point_active_ptr_)->is_sampled_already())
    {
      first_sample = true;
      if (params_.open_loop_control)
      {
        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, last_commanded_state_);
      }
      else
      {
        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, state_current_);
      }
    }

    // find segment for current timestamp
    joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point =
      (*traj_point_active_ptr_)
        ->sample(time, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);

    if (valid_point)
    {
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      double time_difference = 0.0;
      const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index)
      {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        if (
          (before_last_point || first_sample) &&
          !check_state_tolerance_per_joint(
            state_error_, index, default_tolerances_.state_tolerance[index], false))
        {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (
          !before_last_point &&
          !check_state_tolerance_per_joint(
            state_error_, index, default_tolerances_.goal_state_tolerance[index], false))
        {
          outside_goal_tolerance = true;

          if (default_tolerances_.goal_time_tolerance != 0.0)
          {
            // if we exceed goal_time_tolerance set it to aborted
            const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
            const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

            time_difference = get_node()->now().seconds() - traj_end.seconds();

            if (time_difference > default_tolerances_.goal_time_tolerance)
            {
              within_goal_time = false;
            }
          }
        }
      }

      // set values for next hardware write() if tolerance is met
      if (!tolerance_violated_while_moving && within_goal_time)
      {
        if (use_closed_loop_pid_adapter_)
        {
          // Update PIDs
          for (auto i = 0ul; i < dof_; ++i)
          {
            tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +
                              pids_[i]->computeCommand(
                                state_desired_.positions[i] - state_current_.positions[i],
                                state_desired_.velocities[i] - state_current_.velocities[i],
                                (uint64_t)period.nanoseconds());
          }
        }

        // set values for next hardware write()
        if (has_position_command_interface_)
        {
          assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);
        }
        if (has_velocity_command_interface_)
        {
          if (use_closed_loop_pid_adapter_)
          {
            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
          }
          else
          {
            assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
          }
        }
        if (has_acceleration_command_interface_)
        {
          assign_interface_from_point(joint_command_interface_[2], state_desired_.accelerations);
        }
        if (has_effort_command_interface_)
        {
          if (use_closed_loop_pid_adapter_)
          {
            assign_interface_from_point(joint_command_interface_[3], tmp_command_);
          }
          else
          {
            assign_interface_from_point(joint_command_interface_[3], state_desired_.effort);
          }
        }

        // store the previous command. Used in open-loop control mode
        last_commanded_state_ = state_desired_;
      }
      const auto active_goal = *rt_active_goal_.readFromRT();
      if (active_goal)
      {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = params_.joints;

        feedback->actual = state_current_;
        feedback->desired = state_desired_;
        feedback->error = state_error_;
        active_goal->setFeedback(feedback);

        // check abort
        if (tolerance_violated_while_moving)
        {
          set_hold_position();
          auto result = std::make_shared<FollowJTrajAction::Result>();

          RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
          mark_traj_end();
          // check goal tolerance
        }
        else if (!before_last_point)
        {
          if (!outside_goal_tolerance)
          {
            auto res = std::make_shared<FollowJTrajAction::Result>();
            res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            active_goal->setSucceeded(res);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            mark_traj_end();
            RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
          }
          else if (!within_goal_time)
          {
            set_hold_position();
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            mark_traj_end();
            RCLCPP_WARN(
              get_node()->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds",
              time_difference);
          }
          // else, run another cycle while waiting for outside_goal_tolerance
          // to be satisfied or violated within the goal_time_tolerance
        }
      }
      else if (tolerance_violated_while_moving)
      {
        set_hold_position();
        mark_traj_end();
        RCLCPP_ERROR(get_node()->get_logger(), "Holding position due to state tolerance violation");
      }
    }
  }

  publish_state(state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse MRJointTrajectoryController::goal_received_callback(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  return JointTrajectoryController::goal_received_callback(uuid, goal);
}

rclcpp_action::CancelResponse MRJointTrajectoryController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  const rclcpp_action::CancelResponse res = JointTrajectoryController::goal_cancelled_callback(goal_handle);
  auto req = std::make_shared<mr_msgs::srv::SetTrajectoryState::Request>();
  req->header.stamp = get_node()->now();
  req->header.frame_id = robot_name_;
  req->action = mr_msgs::srv::SetTrajectoryState::Request::TRAJECTORY_DONE_OR_CANCEL;
  set_trajectory_state(req);
  return res;
}

void MRJointTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  JointTrajectoryController::goal_accepted_callback(goal_handle);

  auto req = std::make_shared<mr_msgs::srv::SetTrajectoryState::Request>();
  req->header.stamp = get_node()->now();
  req->header.frame_id = robot_name_;
  req->action = mr_msgs::srv::SetTrajectoryState::Request::MARK_TRAJECTORY_START_TIME;
  set_trajectory_state(req);
}

void MRJointTrajectoryController::set_trajectory_state(
    std::shared_ptr<mr_msgs::srv::SetTrajectoryState::Request> req)
{
  while(!set_traj_client_->wait_for_service(std::chrono::milliseconds(500))){
    RCLCPP_ERROR(get_node()->get_logger(),
      "'%s' wait for service exist '%s' failed!", 
      robot_name_.c_str(), set_traj_service_name.c_str());
  }
  set_traj_client_->async_send_request(req, [this](
    const rclcpp::Client<mr_msgs::srv::SetTrajectoryState>::SharedFuture future) -> void {
      if(future.get()->success){
        RCLCPP_INFO(get_node()->get_logger(),
          "'%s' Success to set trajectory state!", robot_name_.c_str());
      }else{
        RCLCPP_ERROR(get_node()->get_logger(),
          "'%s' Failed to set trajectory state!", robot_name_.c_str());
      }
    }
  );
}
}  // namespace mr_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mr_controllers::MRJointTrajectoryController, controller_interface::ControllerInterface)
