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
/*
 *
*/
#ifndef TMC_GAZEBO_PLUGINS_GAZEBO_ROS_BUMPER_HPP_
#define TMC_GAZEBO_PLUGINS_GAZEBO_ROS_BUMPER_HPP_

#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/ContactPlugin.hh>
#include <gazebo_msgs/msg/contact_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>


namespace tmc_gazebo_plugins {
/// @brief A Bumper plugin
class GazeboROSBumper : public gazebo::SensorPlugin {
 public:
  /// Constructor
  GazeboROSBumper();

  /// Destructor
  virtual ~GazeboROSBumper() = default;

  /// Load the SDF elements and initialize
  void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

 private:
  /// Update the bumper data
  void OnContact();

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_pub_;

  gazebo::sensors::ContactSensorPtr parent_sensor_;

  gazebo::event::ConnectionPtr update_connection_;
};
}  // namespace tmc_gazebo_plugins

#endif  // TMC_GAZEBO_PLUGINS_GAZEBO_ROS_BUMPER_HPP_
