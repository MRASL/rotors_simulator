/****************************************************************************
 *
 *   Copyright (c) 2017 Mobile Robotics and Autonomous Systems Laboratory
 *   (MRASL), Polytechnique Montreal. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name MRASL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *  Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtlca>
 *
 ****************************************************************************/

#include "rotors_gazebo_plugins/gazebo_motor_fault_plugin.hpp"

namespace gazebo {

void GazeboMotorFaultPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->model_ =_model;

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  // Use the robot namespace to create the node handle.
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_fault_plugin] Please specify a robotNamespace.\n";

  std::string joint_name;
  if (_sdf->HasElement("jointName"))
    joint_name = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_fault_plugin] Please specify a linkName.\n";
  // Get the pointer to the link.
  joint_ = model_->GetJoint(joint_name);
  if (joint_ == NULL)
  gzthrow("[gazebo_motor_fault_plugin] Couldn't find specified joint \"" << joint_name << "\".");

  getSdfParam<std::string>(_sdf, "faultTopic", fault_topic_,
                           kDefaultFaultTopic);

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMotorFaultPlugin::OnUpdate, this, _1));

  nh_ = new ros::NodeHandle();
  sub_ = nh_->subscribe(namespace_ + "/" + fault_topic_, 1,
                &GazeboMotorFaultPlugin::FaultCallback, this);

  gzdbg << "\n\n\n\n\n\n\n\n" << "JESUS FUCKING CHRIST WILL THIS WORK ALREADY"
        << namespace_ << "/" << fault_topic_ << "\n\n\n\n\n\n\n\n";
}

GazeboMotorFaultPlugin::~GazeboMotorFaultPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if(nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void GazeboMotorFaultPlugin::OnUpdate(const common::UpdateInfo &) {
  if(stop_) return;
  //if(!pubs_and_subs_created_) CreatePubsAndSubs();
  if(trigger_) {
    joint_->Detach();
    stop_ = true;
  }
}

void GazeboMotorFaultPlugin::CreatePubsAndSubs() {

  pubs_and_subs_created_ = true;
}

void GazeboMotorFaultPlugin::FaultCallback(std_msgs::Empty::ConstPtr msg) {
  trigger_ = true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFaultPlugin);

}