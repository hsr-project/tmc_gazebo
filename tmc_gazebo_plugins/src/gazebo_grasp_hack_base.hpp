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
#ifndef TMC_GAZEBO_PLUGINS_SRC_GAZEBO_GRASP_HACK_BASE_HPP_
#define TMC_GAZEBO_PLUGINS_SRC_GAZEBO_GRASP_HACK_BASE_HPP_

#include <map>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_ros/node.hpp>

namespace tmc_gazebo_plugins {

class GazeboGraspHackBase : public gazebo::ModelPlugin {
 public:
  GazeboGraspHackBase();
  virtual ~GazeboGraspHackBase();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

 protected:
  gazebo_ros::Node::SharedPtr model_nh_;

  virtual bool LoadImpl(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) = 0;
  virtual bool AreFingersLoaded() const = 0;

 private:
  gazebo::event::ConnectionPtr update_connection_;
  void UpdateCallback();

  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::SubscriberPtr contact_sub_;
  std::map<std::string, gazebo::physics::CollisionPtr> collisions_;
  void ContactsCallback(ConstContactsPtr& msg);

  void HandleAttach();
  void HandleDetach();

  gazebo::physics::WorldPtr world_;

  gazebo::physics::LinkPtr palm_link_;
  gazebo::physics::JointPtr revolute_joint_;

  double update_period_;
  rclcpp::Time prev_update_time_;

  bool attached_;
  uint32_t pos_count_;
  uint32_t zero_count_;
  double attach_weight_;

  uint32_t min_contact_count_;
  uint32_t attach_steps_;
  uint32_t detach_steps_;

  ignition::math::Pose3d prev_diff_;
  std::vector<double> diffs_;
  uint32_t diff_index_;

  std::vector<gazebo::msgs::Contact> contacts_;
  std::mutex mutex_contacts_;
};

}  // namespace tmc_gazebo_plugins
#endif  // TMC_GAZEBO_PLUGINS_SRC_GAZEBO_GRASP_HACK_BASE_HPP_
