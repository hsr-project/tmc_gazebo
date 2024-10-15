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
#include "gazebo_ros_hrh_gripper.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <angles/angles.h>
#include <rclcpp_action/create_server.hpp>

namespace {

// /// Hand maximum opening width (interval between both fingers) [mm]
// /// See the design document of the mechanical part for details
// const double kMaxOpenWidth = 140.0 / 1000.0;

/// Hand's minimum opening width [DEG]
/// See the design document of the mechanical part for details.This value is an approximate value.
constexpr double kMinFingerDegrees = -7.0;

/// Maximum opening width of hand [DEG]
/// See the design document of the mechanical part for details.This value is an approximate value.
constexpr double kMaxFingerDegrees = 70.0;

/// Maximum output torque of each hand joint [NM]
/// This value is applied not only to the directly served joint but also to other joints.
/// [NM] No design basis.If you raise it more than this, the physical operation will not be stable
constexpr double kDefaultMaxTorque = 10.0;

/// Sensitivity for hand errors [RAD]
constexpr double kDefaultSensitiveness = 0.01;

/// Hand's goal reach acceptance time [S]
const double kDefaultGoalTimeTolerance = 5.0;

/// Maximum output torque of hand frame joint [NM]
/// [NM] No design basis.If you raise it more than this, the physical operation will not be stable
constexpr double kDefaultMaxSpringJointTorque = 7.0;

// /// Timeout time of gripping action [S]
// const double kDefaultStallTimeout = 10.0;

/// Gain value to demonstrate the same torque performance as the actual machine
const double kMaxTorqueGain = 1000.0;

/// Prefix for the same name as the actual machine
const char* const kGripperNamePrefix = "gripper_controller/";

// Joint type
const std::vector<std::string> kJointBaseNames = {
    "motor_joint",      "l_proximal_joint",        "l_spring_proximal_joint", "l_mimic_distal_joint", "l_distal_joint",
    "r_proximal_joint", "r_spring_proximal_joint", "r_mimic_distal_joint",    "r_distal_joint"};

enum {
  kMotorJoint,
  kLeftProximalJoint,
  kLeftSpringProximalJoint,
  kLeftMimicDistalJoint,
  kLeftDistalJoint,
  kRightProximalJoint,
  kRightSpringProximalJoint,
  kRightMimicDistalJoint,
  kRightDistalJoint,
  kNumJoints
};


template<typename TYPE>
TYPE LoadParameter(sdf::ElementPtr sdf, const std::string& name, const TYPE& default_value) {
  if (sdf->HasElement(name)) {
    return sdf->GetElement(name)->Get<TYPE>();
  } else {
    return default_value;
  }
}

inline double clamp(double value, double min, double max) {
  assert(min < max);
  return std::min(max, std::max(value, min));
}

double GetGain(sdf::ElementPtr sdf, const std::string& name, double default_value) {
  if (sdf->HasAttribute(name)) {
    double gain = default_value;
    sdf->GetAttribute(name)->Get<double>(gain);
    return gain;
  } else {
    return default_value;
  }
}

control_toolbox::Pid GeneratePidControl(sdf::ElementPtr sdf, const std::string& name) {
  if (!sdf->HasElement("parameters")) {
    return control_toolbox::Pid();
  }
  auto param_sdf = sdf->GetElement("parameters");

  if (!param_sdf->HasElement(name)) {
    return control_toolbox::Pid();
  }
  auto joint_sdf = param_sdf->GetElement(name);

  return control_toolbox::Pid(GetGain(joint_sdf, "p", 1.0),
                              GetGain(joint_sdf, "i", 0.0),
                              GetGain(joint_sdf, "d", 0.0));
}

}  // namespace

namespace tmc_gazebo_plugins {

GazeboRosHrhGripper::GazeboRosHrhGripper()
    : command_torque_(kDefaultMaxTorque) {}

GazeboRosHrhGripper::~GazeboRosHrhGripper() {
  update_connection_.reset();
}

void GazeboRosHrhGripper::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_nh_ = gazebo_ros::Node::Get(sdf);

  robot_namespace_ = LoadParameter<std::string>(sdf, "robot_namespace", "");
  if (robot_namespace_.empty()) {
    // If the name space is not specified, put it from the model name in Gazebo.
    std::vector<std::string> splitted;
    boost::split(splitted, model->GetScopedName(), boost::is_any_of("::"));
    robot_namespace_ = splitted.back();
  }

  max_torque_ = LoadParameter<double>(sdf, "max_torque", kDefaultMaxTorque);
  sensitiveness_ = LoadParameter<double>(sdf, "sensitiveness", kDefaultSensitiveness);
  error_tolerance_ = LoadParameter<double>(sdf, "error_tolerance", angles::from_degrees(1.0));
  goal_time_tolerance_ = LoadParameter<double>(sdf, "goal_time_tolerance", kDefaultGoalTimeTolerance);
  motor_joint_name_ = LoadParameter<std::string>(sdf, "motor_joint_name", "hand_motor_joint");

  joint_names_.clear();
  for (const auto& name : kJointBaseNames) {
    joint_names_.push_back(LoadParameter<std::string>(sdf, name + "_name", "hand_" + name));
  }

  gazebo_joints_.clear();
  for (const auto& name : joint_names_) {
    gazebo::physics::JointPtr joint = model->GetJoint(name);
    if (!joint) {
      RCLCPP_ERROR_STREAM(model_nh_->get_logger(), "Joint(" << name << ") was not found");
      return;
    }
    joint->SetEffortLimit(0, kMaxTorqueGain * max_torque_);
    gazebo_joints_.push_back(joint);
    pid_controllers_.push_back(GeneratePidControl(sdf, name));
  }

  follow_joint_trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
    model_nh_, kGripperNamePrefix + std::string("follow_joint_trajectory"),
    std::bind(&GazeboRosHrhGripper::FollowJointTrajectoryActionGoalCallback,
              this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GazeboRosHrhGripper::FollowJointTrajectoryActionCancelCallback, this, std::placeholders::_1),
    std::bind(&GazeboRosHrhGripper::FollowJointTrajectoryActionFeedbackSetupCallback, this, std::placeholders::_1));

  grasp_action_server_ = rclcpp_action::create_server<tmc_control_msgs::action::GripperApplyEffort>(
    model_nh_, kGripperNamePrefix + std::string("grasp"),
    std::bind(&GazeboRosHrhGripper::GraspActionGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GazeboRosHrhGripper::GraspActionCancelCallback, this, std::placeholders::_1),
    std::bind(&GazeboRosHrhGripper::GraspActionFeedbackSetupCallback, this, std::placeholders::_1));

  apply_effort_action_server_ = rclcpp_action::create_server<tmc_control_msgs::action::GripperApplyEffort>(
    model_nh_, kGripperNamePrefix + std::string("apply_force"),
    std::bind(&GazeboRosHrhGripper::ApplyEffortActionGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GazeboRosHrhGripper::ApplyEffortActionCancelCallback, this, std::placeholders::_1),
    std::bind(&GazeboRosHrhGripper::ApplyEffortActionFeedbackSetupCallback, this, std::placeholders::_1));

  trajectory_command_sub_ = model_nh_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    kGripperNamePrefix + std::string("joint_trajectory"), 1,
    std::bind(&GazeboRosHrhGripper::TrajectoryCommanCallback, this, std::placeholders::_1));

  UpdateEffortCommand(0.0);
  // There is no particular meaning for queue size 5, it is not too large or too small
  update_stamps_.push(model_nh_->get_clock()->now());
  for (int i = 0; i < 5; ++i) {
    update_stamps_.push(update_stamps_.back());
    update_counts_.push(0);
  }
  current_count_ = 0;
  total_count_ = 0;
  dt_ = 1000000;

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosHrhGripper::Update, this));
}


void GazeboRosHrhGripper::Update() {
  const auto current_stamp = model_nh_->get_clock()->now();
  if (current_stamp != update_stamps_.back()) {
    update_stamps_.push(current_stamp);
    update_counts_.push(current_count_);

    total_count_ += current_count_;
    total_count_ -= update_counts_.front();

    update_stamps_.pop();
    update_counts_.pop();

    current_count_ = 0;

    if (total_count_ != 0) {
      dt_ = (update_stamps_.back() - update_stamps_.front()).nanoseconds() / total_count_;
    }
  } else {
    ++current_count_;
  }

  const double command_torque = clamp(std::abs(command_torque_), 0, max_torque_);
  for (auto joint : gazebo_joints_) {
    joint->SetEffortLimit(0, kMaxTorqueGain * command_torque);
  }

  double command_position;
  if (command_position_) {
    command_position = command_position_->ComputeCommand(model_nh_->get_clock()->now());
  } else {
    command_position = gazebo_joints_[kMotorJoint]->Position(0);
  }
  command_position = clamp(
      command_position, angles::from_degrees(kMinFingerDegrees), angles::from_degrees(kMaxFingerDegrees));

  // The joints defined in ENUM
  // kMotorJoint,
  // kLeftProximalJoint,
  // kLeftSpringProximalJoint,
  // kLeftMimicDistalJoint,
  // kLeftDistalJoint,
  // kRightProximalJoint,
  // kRightSpringProximalJoint,
  // kRightMimicDistalJoint,
  // kRightDistalJoint,
  std::vector<double> targets = {command_position,
                                 command_position,
                                 0.0,
                                 0.0,
                                 -angles::normalize_angle(gazebo_joints_[kLeftProximalJoint]->Position(0)) -
                                     angles::normalize_angle(gazebo_joints_[kLeftSpringProximalJoint]->Position(0)),
                                 command_position,
                                 0.0,
                                 0.0,
                                 -angles::normalize_angle(gazebo_joints_[kRightProximalJoint]->Position(0)) -
                                     angles::normalize_angle(gazebo_joints_[kRightSpringProximalJoint]->Position(0))};

  for (auto i = 0u; i < joint_names_.size(); ++i) {
    const double target = angles::normalize_angle(targets[i]);
    const double state = angles::normalize_angle(gazebo_joints_[i]->Position(0));
    const double error = target - state;
    if (std::abs(error) >= sensitiveness_) {
      const double effort = clamp(pid_controllers_[i].computeCommand(error, dt_), -max_torque_, max_torque_);
      gazebo_joints_[i]->SetForce(0, effort);
    }
  }

  if (!command_position_) {
    return;
  }

  // TODO(Takeshita) Feedbackの発行
  // TODO(Takeshita) 設計をちゃんとやる
  const double error = std::abs(command_position_->goal_position() - gazebo_joints_[kMotorJoint]->Position(0));
  const auto goal_stamp = command_position_->goal_stamp();
  if (current_stamp >= goal_stamp) {
    if (error < error_tolerance_) {
      if (follow_joint_trajectory_goal_handle_) {
        auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        follow_joint_trajectory_goal_handle_->succeed(result);
        follow_joint_trajectory_goal_handle_.reset();
      }
      if (grasp_goal_handle_) {
        auto result = std::make_shared<tmc_control_msgs::action::GripperApplyEffort::Result>();
        result->stalled = true;
        grasp_goal_handle_->succeed(result);
        grasp_goal_handle_.reset();
      }
      if (apply_effort_goal_handle_) {
        auto result = std::make_shared<tmc_control_msgs::action::GripperApplyEffort::Result>();
        result->stalled = true;
        apply_effort_goal_handle_->succeed(result);
        apply_effort_goal_handle_.reset();
      }
    } else if ((current_stamp - goal_stamp).seconds() > goal_time_tolerance_) {
      if (follow_joint_trajectory_goal_handle_) {
        auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        follow_joint_trajectory_goal_handle_->abort(result);
        follow_joint_trajectory_goal_handle_.reset();
      }
      if (grasp_goal_handle_) {
        auto result = std::make_shared<tmc_control_msgs::action::GripperApplyEffort::Result>();
        result->stalled = false;
        grasp_goal_handle_->abort(result);
        grasp_goal_handle_.reset();
      }
      if (apply_effort_goal_handle_) {
        auto result = std::make_shared<tmc_control_msgs::action::GripperApplyEffort::Result>();
        result->stalled = true;
        apply_effort_goal_handle_->succeed(result);
        apply_effort_goal_handle_.reset();
      }
    }
  }
}

bool GazeboRosHrhGripper::UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory& msg) {
  auto motor_joint_iter = std::find(msg.joint_names.begin(), msg.joint_names.end(), motor_joint_name_);
  if (motor_joint_iter == msg.joint_names.end()) {
    RCLCPP_ERROR(model_nh_->get_logger(), "Failed: invalid joint names.");
    return false;
  }

  double goal_position = gazebo_joints_[kMotorJoint]->Position(0);
  auto goal_time_from_start = rclcpp::Duration(1, 0);
  if (!msg.points.empty()) {
    uint32_t distance = std::distance(msg.joint_names.begin(), motor_joint_iter);
    if (distance < msg.points.back().positions.size()) {
      goal_position = msg.points.back().positions[distance];
      goal_time_from_start = msg.points.back().time_from_start;
    }
  }

  if (goal_time_from_start.nanoseconds() == 0) {
    RCLCPP_ERROR(model_nh_->get_logger(), "Failed: invalid time from start.");
    return false;
  }

  auto stamp = rclcpp::Time(msg.header.stamp);
  if (stamp.nanoseconds() == 0) {
    stamp = model_nh_->get_clock()->now();
  }
  command_position_ = std::make_shared<PositionCommand>(
      gazebo_joints_[kMotorJoint]->Position(0), goal_position, stamp, goal_time_from_start);
  command_torque_ = 0.020;

  return true;
}

void GazeboRosHrhGripper::UpdateEffortCommand(double effort) {
  double goal_position;
  if (effort > 0.0) {
    goal_position = angles::from_degrees(kMaxFingerDegrees);
    command_torque_ = effort;
  } else if (effort < 0.0) {
    goal_position = angles::from_degrees(kMinFingerDegrees);
    command_torque_ = effort;
  } else {
    goal_position = gazebo_joints_[kMotorJoint]->Position(0);
    command_torque_ = kDefaultMaxTorque;
  }

  command_position_ = std::make_shared<PositionCommand>(
      gazebo_joints_[kMotorJoint]->Position(0), goal_position, model_nh_->get_clock()->now(), rclcpp::Duration(1, 0));
}


void GazeboRosHrhGripper::TrajectoryCommanCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  // Ignore the success or fail because there is no need to return the result
  UpdateTrajectory(*msg);
}

rclcpp_action::GoalResponse GazeboRosHrhGripper::FollowJointTrajectoryActionGoalCallback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
  // TODO(Takeshita) ホントはここでゴールがvalidかを検証すべき
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GazeboRosHrhGripper::FollowJointTrajectoryActionCancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
  // TODO(Takeshita) ちゃんと単体でキャンセルする
  CancelAllGoals();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GazeboRosHrhGripper::FollowJointTrajectoryActionFeedbackSetupCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
  CancelAllGoals();
  if (UpdateTrajectory(goal_handle->get_goal()->trajectory)) {
    follow_joint_trajectory_goal_handle_ = goal_handle;
  } else {
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL;
    goal_handle->abort(result);
  }
}

rclcpp_action::GoalResponse GazeboRosHrhGripper::GraspActionGoalCallback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const tmc_control_msgs::action::GripperApplyEffort::Goal> goal) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GazeboRosHrhGripper::GraspActionCancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>> goal_handle) {
  // TODO(Takeshita) ちゃんと単体でキャンセルする
  CancelAllGoals();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GazeboRosHrhGripper::GraspActionFeedbackSetupCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>> goal_handle) {
  CancelAllGoals();
  UpdateEffortCommand(goal_handle->get_goal()->effort);
  grasp_goal_handle_ = goal_handle;
}

rclcpp_action::GoalResponse GazeboRosHrhGripper::ApplyEffortActionGoalCallback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const tmc_control_msgs::action::GripperApplyEffort::Goal> goal) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GazeboRosHrhGripper::ApplyEffortActionCancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>> goal_handle) {
  // TODO(Takeshita) ちゃんと単体でキャンセルする
  CancelAllGoals();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GazeboRosHrhGripper::ApplyEffortActionFeedbackSetupCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<tmc_control_msgs::action::GripperApplyEffort>> goal_handle) {
  CancelAllGoals();
  // Reversal
  UpdateEffortCommand(-(goal_handle->get_goal()->effort));
  apply_effort_goal_handle_ = goal_handle;
}

void GazeboRosHrhGripper::CancelAllGoals() {
  command_position_.reset();

  follow_joint_trajectory_goal_handle_.reset();
  grasp_goal_handle_.reset();
  apply_effort_goal_handle_.reset();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosHrhGripper);

}  // namespace tmc_gazebo_plugins
