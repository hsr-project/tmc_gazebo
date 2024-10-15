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

#include "gazebo_grasp_hack_rh_p12_rn.hpp"

#include <string>

namespace {
/// The threshold of the deviation that is not applied [RAD]
const double kNoForceDiff = 0.1;
}  // namespace

namespace tmc_gazebo_plugins {

bool GazeboGraspHackRhP12Rn::LoadImpl(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  const auto grasp_check = sdf->GetElement("grasp_check");
  if (grasp_check->HasElement("no_force_diff")) {
    no_force_diff_ = grasp_check->Get<double>("no_force_diff");
  } else {
    no_force_diff_ = kNoForceDiff;
  }
  if (grasp_check->HasElement("virtual_joint")) {
    virtual_joint_ = model->GetJoint(sdf->GetElement("virtual_joint")->Get<std::string>());
  } else {
    virtual_joint_ = model->GetJoint("gripper_virtual_joint");
  }
  if (grasp_check->HasElement("base_left_joint")) {
    base_left_joint_ = model->GetJoint(sdf->GetElement("base_left_joint")->Get<std::string>());
  } else {
    base_left_joint_ = model->GetJoint("rh_l1");
  }
  if (grasp_check->HasElement("base_right_joint")) {
    base_right_joint_ = model->GetJoint(sdf->GetElement("base_right_joint")->Get<std::string>());
  } else {
    base_right_joint_ = model->GetJoint("rh_r1");
  }
  if (!virtual_joint_ || !base_left_joint_ || !base_right_joint_) {
    RCLCPP_FATAL(model_nh_->get_logger(), "Base Joint was not found");
    return false;
  }
  return true;
}

bool GazeboGraspHackRhP12Rn::AreFingersLoaded() const {
  // The joint angle is 1.1-0.0 [RAD], compared to 0.0-0.106 [m] for opening and closing.
  const double command_pos = 1.1 - 10.377358491 * virtual_joint_->Position(0);
  return (((command_pos - base_left_joint_->Position(0)) > no_force_diff_) ||
          ((command_pos - base_left_joint_->Position(0)) > no_force_diff_));
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGraspHackRhP12Rn);

}  // namespace tmc_gazebo_plugins
