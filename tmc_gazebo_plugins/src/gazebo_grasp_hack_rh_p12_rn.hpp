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
#ifndef TMC_GAZEBO_PLUGINS_SRC_GAZEBO_GRASP_HACK_HRH_GRIPPER_HPP_
#define TMC_GAZEBO_PLUGINS_SRC_GAZEBO_GRASP_HACK_HRH_GRIPPER_HPP_

#include "gazebo_grasp_hack_base.hpp"

namespace tmc_gazebo_plugins {

class GazeboGraspHackRhP12Rn : public GazeboGraspHackBase {
 public:
  GazeboGraspHackRhP12Rn() : GazeboGraspHackBase() {}
  virtual ~GazeboGraspHackRhP12Rn() = default;

 protected:
  bool LoadImpl(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  bool AreFingersLoaded() const override;

 private:
  double no_force_diff_;
  gazebo::physics::JointPtr virtual_joint_;
  gazebo::physics::JointPtr base_left_joint_;
  gazebo::physics::JointPtr base_right_joint_;
};

}  // namespace tmc_gazebo_plugins
#endif  // TMC_GAZEBO_PLUGINS_SRC_GAZEBO_GRASP_HACK_HRH_GRIPPER_HPP_
