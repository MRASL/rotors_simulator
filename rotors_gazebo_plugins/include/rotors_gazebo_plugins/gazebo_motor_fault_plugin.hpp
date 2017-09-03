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
#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_MOTOR_FAULT_PLUGIN_HPP
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_MOTOR_FAULT_PLUGIN_HPP

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "rotors_gazebo_plugins/common.h"
#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

static const std::string kDefaultFaultTopic = "motor_fault";

class GazeboMotorFaultPlugin : public ModelPlugin {
public:
  GazeboMotorFaultPlugin()
      : ModelPlugin(),
        pubs_and_subs_created_(false),
        trigger_(false),
        stop_(false),
        nh_(0)
  {}

  virtual ~GazeboMotorFaultPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);

private:
  physics::ModelPtr model_;

  /// \brief    Transport namespace.
  std::string namespace_;

  /// \brief    Topic name for fault activation messages.
  std::string fault_topic_;

  /// \brief    Pointer to the joint.
  physics::JointPtr joint_;

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;
  bool trigger_;
  bool stop_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();

  ros::NodeHandle* nh_;
  ros::Subscriber sub_;
  void FaultCallback(std_msgs::Empty::ConstPtr msg);
};

}

#endif //ROTORS_GAZEBO_PLUGINS_GAZEBO_MOTOR_FAULT_PLUGIN_HPP
