/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface that performs a perfect control loop for simulation
*/

// This file is the base class that defines your hardware elements
// Later, you need to implement a hardware real/sim interfaces.

#ifndef ROS_CONTROL__RRBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__RRBOT_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

namespace rrbot_hw
{

/// \brief Hardware interface for a robot
class RRBOTHardware : public hardware_interface::RobotHW 
{
/*public:

  /// \brief Constructor.
  RRBOTHardware();

  /// \brief Destructor.
  ~RRBOTHardware();
*/
protected:

  // Interfaces
  hardware_interface::JointStateInterface      joint_state_interface_;
  hardware_interface::PositionJointInterface   position_joint_interface_;
  hardware_interface::VelocityJointInterface   velocity_joint_interface_;
  hardware_interface::EffortJointInterface     effort_joint_interface_;

  joint_limits_interface::EffortJointSaturationInterface   effort_joint_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   effort_joint_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

  // Custom or available transmissions
  // transmission_interface::RRBOTTransmission rrbot_trans_;
  // std::vector<transmission_interface::SimpleTransmission> simple_trans_;

  // Shared memory
  int                                          num_joints_;
  int                                          joint_mode_; // position, velocity, or effort
  std::vector<std::string>                     joint_names_;
  std::vector<int>                             joint_types_;
  std::vector<double>                          joint_position_;
  std::vector<double>                          joint_velocity_;
  std::vector<double>                          joint_effort_;
  std::vector<double>                          joint_position_command_;
  std::vector<double>                          joint_velocity_command_;
  std::vector<double>                          joint_effort_command_;
  std::vector<double>                          joint_lower_limits_;
  std::vector<double>                          joint_upper_limits_;
  std::vector<double>                          joint_effort_limits_;

}; // class

} // namespace

#endif
