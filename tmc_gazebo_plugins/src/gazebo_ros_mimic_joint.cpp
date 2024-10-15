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
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tmc_gazebo_plugins {

class GazeboRosMimicJoint : public gazebo::ModelPlugin {
 public:
  GazeboRosMimicJoint() {}
  virtual ~GazeboRosMimicJoint();

  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override;

 private:
  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  // Joint update
  void Update();

  // joint
  gazebo::physics::JointPtr joint_;
  gazebo::physics::JointPtr mimic_joint_;

  // NodeHandle
  gazebo_ros::Node::SharedPtr model_nh_;

  // Information required for control
  double multiplier_;
  double offset_;
};

GazeboRosMimicJoint::~GazeboRosMimicJoint() {
  update_connection_.reset();
}


void GazeboRosMimicJoint::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
  model_nh_ = gazebo_ros::Node::Get(sdf);

  // TODO(Takeshita) 繰り返しているので関数化する
  std::string joint_name;
  if (sdf->HasElement("joint")) {
    joint_name = sdf->GetElement("joint")->Get<std::string>();
  } else {
    RCLCPP_ERROR_STREAM(model_nh_->get_logger(), "joint is not set");
    return;
  }
  joint_ = parent->GetJoint(joint_name);
  if (!joint_) {
    RCLCPP_ERROR_STREAM(model_nh_->get_logger(), joint_name << " is not in URDF");
    return;
  }

  std::string mimic_joint_name;
  if (sdf->HasElement("mimic_joint")) {
    mimic_joint_name = sdf->GetElement("mimic_joint")->Get<std::string>();
  } else {
    RCLCPP_ERROR_STREAM(model_nh_->get_logger(), "mimic_joint is not set");
    return;
  }
  mimic_joint_ = parent->GetJoint(mimic_joint_name);
  if (!mimic_joint_) {
    RCLCPP_ERROR_STREAM(model_nh_->get_logger(), mimic_joint_name << " is not in URDF");
    return;
  }

  if (sdf->HasElement("multiplier")) {
    multiplier_ = sdf->GetElement("multiplier")->Get<double>();
  } else {
    multiplier_ = 1.0;
  }

  if (sdf->HasElement("offset")) {
    offset_ = sdf->GetElement("offset")->Get<double>();
  } else {
    offset_ = 0.0;
  }

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosMimicJoint::Update, this));
}

void GazeboRosMimicJoint::Update() {
  const double position = mimic_joint_->Position(0);
  joint_->SetPosition(0, position * multiplier_ + offset_, true);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMimicJoint);

}  // namespace tmc_gazebo_plugins
