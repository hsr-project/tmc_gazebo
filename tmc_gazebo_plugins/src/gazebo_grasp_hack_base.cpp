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

#include "gazebo_grasp_hack_base.hpp"

#include <algorithm>
#include <set>

namespace {
/// ATTACH Link weight threshold (not judged to be large)
const double kMaxAttachWeight = 1.2;

/// Buffer number of position deviation
const uint32_t kNumDiffBuffer = 2;
/// Contact link-Variance threshold (for Attach judgment) of difference from the front frame of palm link
const double kPoseDiffVarThreshold = 1e-2;
/// Contact link-MAX threshold (for ATTACH judgment) of difference from the front frame of palm link
const double kPoseDiffMaxThreshold = 1e-2;
}  // namespace

namespace tmc_gazebo_plugins {

GazeboGraspHackBase::GazeboGraspHackBase()
    : gazebo::ModelPlugin(),
      update_period_(0.3),  // Adjusted value, if you shorten it, the reaction when you let go, but the knowledge is not stable.
      prev_update_time_(0, 0),
      pos_count_(0),
      zero_count_(0),
      diff_index_(0) {
  diffs_.resize(kNumDiffBuffer);
}

GazeboGraspHackBase::~GazeboGraspHackBase() {}

void GazeboGraspHackBase::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  world_ = model->GetWorld();
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init(world_->Name());

  model_nh_ = gazebo_ros::Node::Get(sdf);
  prev_update_time_ = rclcpp::Time(0, 0, model_nh_->get_clock()->get_clock_type());

  const auto grasp_check = sdf->GetElement("grasp_check");
  min_contact_count_ = grasp_check->Get<uint32_t>("min_contact_count");
  attach_steps_ = grasp_check->Get<uint32_t>("attach_steps");
  detach_steps_ = grasp_check->Get<uint32_t>("detach_steps");

  if (grasp_check->HasElement("max_attach_weight")) {
    attach_weight_ = grasp_check->Get<double>("max_attach_weight");
  } else {
    attach_weight_ = kMaxAttachWeight;
  }

  if (sdf->HasElement("palm_link")) {
    palm_link_ = model->GetLink(sdf->GetElement("palm_link")->Get<std::string>());
  } else {
    palm_link_ = model->GetLink("hand_palm_link");
  }
  if (!palm_link_) {
    RCLCPP_ERROR(model_nh_->get_logger(), "palm link not found");
    return;
  }

  revolute_joint_ = world_->Physics()->CreateJoint("revolute", model);

  if (!LoadImpl(model, sdf)) {
    return;
  }

  auto gripper_link_elem = sdf->GetElement("gripper_link");
  while (gripper_link_elem) {
    const auto gripper_link = model->GetLink(gripper_link_elem->Get<std::string>());
    for (auto i = 0u; i < gripper_link->GetChildCount(); ++i) {
      const auto collision = gripper_link->GetCollision(i);
      if (collisions_.find(collision->GetScopedName()) == collisions_.end()) {
        collisions_[collision->GetScopedName()] = collision;
      }
    }
    gripper_link_elem = gripper_link_elem->GetNextElement("gripper_link");
  }
  if (!collisions_.empty()) {
    auto contact_manager = world_->Physics()->GetContactManager();
    const auto topic = contact_manager->CreateFilter(sdf->Get<std::string>("name"), collisions_);
    contact_sub_ = gazebo_node_->Subscribe(topic, &GazeboGraspHackBase::ContactsCallback, this);
  }

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboGraspHackBase::UpdateCallback, this));

  RCLCPP_INFO(model_nh_->get_logger(), "GazeboGraspHack loaded");
}

void GazeboGraspHackBase::UpdateCallback() {
  const auto current_stamp = model_nh_->get_clock()->now();
  if ((current_stamp - prev_update_time_).seconds() < update_period_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_contacts_);
  if ((contacts_.size() >= min_contact_count_) && AreFingersLoaded()) {
    pos_count_++;
    zero_count_ = 0;
  } else {
    zero_count_++;
    pos_count_ = std::max(0u, pos_count_ - 1);
  }

  if (pos_count_ > attach_steps_ && !attached_) {
    HandleAttach();
  } else if (zero_count_ > detach_steps_ && attached_) {
    HandleDetach();
  }

  contacts_.clear();
  prev_update_time_ += rclcpp::Duration::from_seconds(update_period_);
}

void GazeboGraspHackBase::ContactsCallback(ConstContactsPtr& msg) {
  for (auto i = 0u; i < msg->contact_size(); ++i) {
    const auto collision1 =
        boost::dynamic_pointer_cast<gazebo::physics::Collision>(world_->EntityByName(msg->contact(i).collision1()));
    const auto collision2 =
        boost::dynamic_pointer_cast<gazebo::physics::Collision>(world_->EntityByName(msg->contact(i).collision2()));
    if ((collision1 && !collision1->IsStatic()) && (collision2 && !collision2->IsStatic())) {
      std::lock_guard<std::mutex> lock(mutex_contacts_);
      contacts_.push_back(msg->contact(i));
    }
  }
}

void GazeboGraspHackBase::HandleAttach() {
  if (!palm_link_) {
    RCLCPP_WARN_THROTTLE(model_nh_->get_logger(), *model_nh_->get_clock(), 1000, "palm link not found");
    return;
  }

  // This function is only called from the UpdateCallback function so
  // the call to contacts.clear() is not going to happen in
  // parallel with the reads in the following code, no mutex needed.

  std::set<std::string> collide_links;
  std::map<std::string, gazebo::physics::CollisionPtr> contact_collisions;
  std::map<std::string, uint32_t> contact_counts;
  for (auto i = 0u; i < contacts_.size(); ++i) {
    const auto name1 = contacts_[i].collision1();
    const auto name2 = contacts_[i].collision2();
    // Save the contact link name without duplicate
    collide_links.insert(name1);
    collide_links.insert(name2);
    if (collisions_.find(name1) == collisions_.end()) {
      contact_collisions[name1] = boost::dynamic_pointer_cast<gazebo::physics::Collision>(world_->EntityByName(name1));
      contact_counts[name1] += 1;
    }
    if (collisions_.find(name2) == collisions_.end()) {
      contact_collisions[name2] = boost::dynamic_pointer_cast<gazebo::physics::Collision>(world_->EntityByName(name2));
      contact_counts[name2] += 1;
    }
  }

  for (auto iter = contact_counts.begin(); iter != contact_counts.end(); ) {
    // What is these 2?Since it was in the original Grasp Hack, it is as it is
    // If the number of collision points is one point, do you mean not Attach?
    if (iter->second < 2) {
      iter = contact_counts.erase(iter);
      continue;
    }

    if (contact_collisions[iter->first]) {
      const auto links = contact_collisions[iter->first]->GetModel()->GetLinks();
      double model_weight = 0.0;
      for (auto i = 0u; i < links.size(); ++i) {
        model_weight += links[i]->GetInertial()->Mass();
      }
      if (model_weight > attach_weight_) {
        // Is this return?Not Continue?
        return;
      }
      const auto diff = contact_collisions[iter->first]->GetLink()->WorldPose() - palm_link_->WorldPose();
      const double x = (diff - prev_diff_).Pos().X();
      const double y = (diff - prev_diff_).Pos().Y();
      const double z = (diff - prev_diff_).Pos().Z();
      const double dd = x * x + y * y + z * z;

      prev_diff_ = diff;
      diffs_[diff_index_] = dd;
      diff_index_ = (diff_index_ + 1) % kNumDiffBuffer;

      const double var = ignition::math::variance<double>(diffs_);
      const double max = ignition::math::max<double>(diffs_);

      if (var < kPoseDiffVarThreshold && max < kPoseDiffMaxThreshold) {
        if (collide_links.size() > 2) {
          attached_ = true;

          revolute_joint_->Load(palm_link_, contact_collisions[iter->first]->GetLink(), ignition::math::Pose3d());
          revolute_joint_->Init();
          revolute_joint_->SetUpperLimit(0, 0);
          revolute_joint_->SetLowerLimit(0, 0);
          return;
        }
      }
    }
    ++iter;
  }
}

void GazeboGraspHackBase::HandleDetach() {
  attached_ = false;
  revolute_joint_->Detach();
}

}  // namespace tmc_gazebo_plugins
