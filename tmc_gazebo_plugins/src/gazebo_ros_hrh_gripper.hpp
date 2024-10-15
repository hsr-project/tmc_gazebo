/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @brief HIGH-RATIO-HYPOID Hand control plugin for grippers
#ifndef TMC_GAZEBO_PLUGINS_SRC_GAZEBO_ROS_HRH_GRIPPER_HPP_
#define TMC_GAZEBO_PLUGINS_SRC_GAZEBO_ROS_HRH_GRIPPER_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_toolbox/pid.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tmc_control_msgs/action/gripper_apply_effort.hpp>


namespace tmc_gazebo_plugins {

class PositionCommand {
 public:
  using Ptr = std::shared_ptr<PositionCommand>;

  PositionCommand(double start_position, double goal_position, const rclcpp::Time& start_stamp,
                  const rclcpp::Duration goal_stamp_from_start)
      : start_position_(start_position), goal_position_(goal_position),
        velocity_((goal_position - start_position) / goal_stamp_from_start.seconds()),
        start_stamp_(start_stamp), goal_stamp_(start_stamp + goal_stamp_from_start) {}

  double ComputeCommand(const rclcpp::Time& stamp) const {
    if (stamp < start_stamp_) {
      return start_position_;
    } else if (stamp > goal_stamp_) {
      return goal_position_;
    } else {
      const auto duration = (stamp - start_stamp_).seconds();
      return start_position_ + velocity_ * duration;
    }
  }

  double goal_position() const { return goal_position_; }
  rclcpp::Time goal_stamp() const { return goal_stamp_; }

 private:
  double start_position_;
  double goal_position_;
  double velocity_;
  rclcpp::Time start_stamp_;
  rclcpp::Time goal_stamp_;
};

class GazeboRosHrhGripper : public gazebo::ModelPlugin {
 public:
  GazeboRosHrhGripper();
  virtual ~GazeboRosHrhGripper();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  gazebo_ros::Node::SharedPtr model_nh_;
  gazebo::event::ConnectionPtr update_connection_;
  void Update();

  bool UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory& msg);
  void UpdateEffortCommand(double effort);

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_command_sub_;
  void TrajectoryCommanCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
      follow_joint_trajectory_goal_handle_;

  rclcpp_action::GoalResponse FollowJointTrajectoryActionGoalCallback(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse FollowJointTrajectoryActionCancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
  void FollowJointTrajectoryActionFeedbackSetupCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

  rclcpp_action::Server<tmc_control_msgs::action::GripperApplyEffort>::SharedPtr grasp_action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>> grasp_goal_handle_;

  rclcpp_action::GoalResponse GraspActionGoalCallback(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const tmc_control_msgs::action::GripperApplyEffort::Goal> goal);
  rclcpp_action::CancelResponse GraspActionCancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>> goal_handle);
  void GraspActionFeedbackSetupCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>> goal_handle);

  rclcpp_action::Server<tmc_control_msgs::action::GripperApplyEffort>::SharedPtr apply_effort_action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>>
  apply_effort_goal_handle_;

  rclcpp_action::GoalResponse ApplyEffortActionGoalCallback(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const tmc_control_msgs::action::GripperApplyEffort::Goal> goal);
  rclcpp_action::CancelResponse ApplyEffortActionCancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>> goal_handle);
  void ApplyEffortActionFeedbackSetupCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>> goal_handle);

  void CancelAllGoals();

  std::vector<std::string> joint_names_;
  std::vector<gazebo::physics::JointPtr> gazebo_joints_;
  std::vector<control_toolbox::Pid> pid_controllers_;

  PositionCommand::Ptr command_position_;
  double command_torque_;

  double max_torque_;
  double error_tolerance_;
  double goal_time_tolerance_;
  double sensitiveness_;
  std::string robot_namespace_;
  std::string motor_joint_name_;

  // The update cycle of GAZEBO's CLOCK is at most the simulator STEP cycle, so it is troublesome to control, so we calculate a troublesome time.
  // TODO(Takeshita) 時間処理をクラスに切り出す
  std::queue<rclcpp::Time> update_stamps_;
  std::queue<uint64_t> update_counts_;
  uint64_t current_count_;
  uint64_t total_count_;
  uint64_t dt_;
};

}  // namespace tmc_gazebo_plugins

#endif  // TMC_GAZEBO_PLUGINS_SRC_GAZEBO_ROS_HRH_GRIPPER_HPP_
