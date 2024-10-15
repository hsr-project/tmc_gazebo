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

#include "gazebo_grasp_hack_hrh_gripper.hpp"

#include <string>

namespace {
/// Spring threshold that determines that it is not powered [RAD]
const double kSpringNoForce = 0.05;
}  // namespace

namespace tmc_gazebo_plugins {

bool GazeboGraspHackHrhGripper::LoadImpl(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  const auto grasp_check = sdf->GetElement("grasp_check");
  if (grasp_check->HasElement("spring_no_force")) {
    spring_no_force_ = grasp_check->Get<double>("spring_no_force");
  } else {
    spring_no_force_ = kSpringNoForce;
  }
  if (grasp_check->HasElement("left_spring")) {
    left_spring_ = model->GetJoint(sdf->GetElement("left_spring")->Get<std::string>());
  } else {
    left_spring_ = model->GetJoint("hand_l_spring_proximal_joint");
  }
  if (grasp_check->HasElement("right_spring")) {
    right_spring_ = model->GetJoint(sdf->GetElement("right_spring")->Get<std::string>());
  } else {
    right_spring_ = model->GetJoint("hand_r_spring_proximal_joint");
  }
  if (!left_spring_ || !right_spring_) {
    RCLCPP_FATAL(model_nh_->get_logger(), "Spring Joint was not found");
    return false;
  }
  return true;
}

bool GazeboGraspHackHrhGripper::AreFingersLoaded() const {
  return ((fabs(left_spring_->Position(0)) > spring_no_force_) ||
          (fabs(right_spring_->Position(0)) > spring_no_force_));
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGraspHackHrhGripper);

}  // namespace tmc_gazebo_plugins
