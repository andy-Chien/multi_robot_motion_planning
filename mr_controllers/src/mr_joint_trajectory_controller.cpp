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
#include <rclcpp_action/create_server.hpp>
#include "mr_controllers/mr_joint_trajectory_controller.hpp"

#include "lifecycle_msgs/msg/state.hpp"

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
  return JointTrajectoryController::update(time, period);
}

rclcpp_action::GoalResponse MRJointTrajectoryController::goal_received_callback(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  return JointTrajectoryController::goal_received_callback(uuid, goal);
}

rclcpp_action::CancelResponse MRJointTrajectoryController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  return JointTrajectoryController::goal_cancelled_callback(goal_handle);
}

void MRJointTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  JointTrajectoryController::goal_accepted_callback(goal_handle);

  auto req = std::make_shared<mr_msgs::srv::SetTrajectoryState::Request>();
  req->header.stamp = get_node()->now();
  req->header.frame_id = robot_name_;
  req->action = mr_msgs::srv::SetTrajectoryState::Request::MARK_TRAJECTORY_START_TIME;
  while(!set_traj_client_->wait_for_service(std::chrono::milliseconds(500))){
    RCLCPP_ERROR(get_node()->get_logger(),
      "'%s' wait for service exist '%s' failed!", 
      robot_name_.c_str(), set_traj_service_name.c_str());
  }
  set_traj_client_->async_send_request(req, [this](
    const rclcpp::Client<mr_msgs::srv::SetTrajectoryState>::SharedFuture future) -> void {
      if(future.get()->success){
        RCLCPP_INFO(get_node()->get_logger(),
          "'%s' Success to set trajectory start time!", robot_name_.c_str());
      }else{
        RCLCPP_ERROR(get_node()->get_logger(),
          "'%s' Failed to set trajectory start time!", robot_name_.c_str());
      }
    }
  );
}
}  // namespace mr_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mr_controllers::MRJointTrajectoryController, controller_interface::ControllerInterface)
