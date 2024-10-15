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

#include "gazebo_ros_bumper.hpp"

#include <memory>

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>


namespace tmc_gazebo_plugins {

// Constructor
GazeboROSBumper::GazeboROSBumper()
    : SensorPlugin() {}

// Load the SDF and initialize
void GazeboROSBumper::Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf) {
  gazebo_ros::Node::SharedPtr rosnode = gazebo_ros::Node::Get(sdf);

  parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(parent);
  if (!parent_sensor_) {
    RCLCPP_ERROR(rosnode->get_logger(), "The bumper plugin's parent sensor is not of type ContactSensor.");
    rosnode.reset();
    return;
  }

  contact_pub_ = rosnode->create_publisher<std_msgs::msg::Bool>("bumper_states", 1);

  update_connection_ = parent_sensor_->ConnectUpdated(
      std::bind(&GazeboROSBumper::OnContact, this));

  parent_sensor_->SetActive(true);
}

// Update the bumper data
void GazeboROSBumper::OnContact() {
  auto contacts = parent_sensor_->Contacts();

  std_msgs::msg::Bool msg;
  msg.data = contacts.contact_size() > 0;
  contact_pub_->publish(msg);
}

// Register this plugin with gazebo
GZ_REGISTER_SENSOR_PLUGIN(GazeboROSBumper)

}  // namespace tmc_gazebo_plugins
