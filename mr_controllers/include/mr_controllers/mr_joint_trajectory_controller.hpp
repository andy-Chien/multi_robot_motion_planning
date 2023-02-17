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
 * \author  Andy Chien 808790017@gms.tku.edu.tw
 * \date    2023-02-17
 *
 */
//----------------------------------------------------------------------
#ifndef MR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define MR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_

// #include "angles/angles.h"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
// #include "joint_trajectory_controller/trajectory.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp/time.hpp"
// #include "rclcpp/duration.hpp"

#include "mr_msgs/srv/mark_trajectory_state.hpp"

namespace mr_controllers
{
class MRJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  MRJointTrajectoryController() = default;
  ~MRJointTrajectoryController() override = default;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& state) override;
  controller_interface::return_type update(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // callbacks for action_server_
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);
  
private:
  std::string robot_name_;
  rclcpp::Client<mr_msgs::srv::MarkTrajectoryState>::SharedPtr mark_traj_client_;
};
}  // namespace mr_controllers

#endif  // MR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
