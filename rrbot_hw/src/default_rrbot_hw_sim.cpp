/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/* Author: Dave Coleman, Johnathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#ifndef _RRBOT_HW___DEFAULT_RRBOT_HW_SIM_H_
#define _RRBOT_HW___DEFAULT_RRBOT_HW_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// URDF
#include <urdf/model.h>

// RRBOT definition in simulation
#include <rrbot_hw/rrbot_hw_sim.h>

namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace rrbot_hw
{

class DefaultRRBOTHWSim : public rrbot_hw::RRBOTHWSim
{
public:

  bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
    // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
    const ros::NodeHandle joint_limit_nh(model_nh, robot_namespace);

    // Resize vectors to our DOF
    num_joints_ = transmissions.size();
    double num_non_driven_joints = urdf_model->joints_.size() - num_joints_;
    std::cout << "# OF NON DRIVEN JOINTS: " << num_non_driven_joints << std::endl;
    joint_names_.resize(num_joints_);
    joint_types_.resize(num_joints_);
    joint_lower_limits_.resize(num_joints_);
    joint_upper_limits_.resize(num_joints_);
    joint_effort_limits_.resize(num_joints_);
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);

    joint_control_methods_.resize(1);
    pid_controllers_.resize(1);
    joint_effort_command_.resize(1);
    joint_position_command_.resize(1);
    joint_velocity_command_.resize(1);

    // Initialize values
    for(unsigned int j=0; j < num_joints_; j++)
    {

      std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
      if (joint_interfaces.empty() &&
          !(transmissions[j].actuators_.empty()) &&
          !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
      {
        // TODO: Deprecate HW interface specification in actuators in ROS J
        joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
        ROS_WARN_STREAM_NAMED("default_rrbot_hw_sim", "The <hardware_interface> element of tranmission " <<
          transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
          "The transmission will be properly loaded, but please update " <<
          "your robot model to remain compatible with future versions of the plugin.");
      }

      // Add data from transmission
      joint_names_[j] = transmissions[j].joints_[0].name_;
      joint_position_[j] = 1.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 1.0;  // N/m for continuous joints

      const std::string& hardware_interface = joint_interfaces.front();

      // Debug
      // ROS_WARN_STREAM_NAMED("default_rrbot_hw_sim","Loading joint '" << joint_names_[j]
      //  << "' of type '" << hardware_interface << "'");

      // Create joint state interface for all joints
      joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      // Get the gazebo joint that corresponds to the robot joint.
      // ROS_WARN_STREAM_NAMED("default_rrbot_hw_sim", "Getting pointer to gazebo joint: "
       // << joint_names_[j]);

      gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
      if (!joint)
      {
        ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
          << "\" which is not in the gazebo model.");
        return false;
      }
      sim_joints_.push_back(joint);

    }

    joint_effort_command_[0] = 0.0;
    joint_position_command_[0] = 0.0;
    joint_velocity_command_[0] = 0.0;

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;
    
    // Create position joint interface
    joint_control_methods_[0] = POSITION;
    joint_handle = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[1]),
                                                   &joint_position_command_[0]);
    position_joint_interface_.registerHandle(joint_handle);

    registerJointLimits(joint_names_[1], joint_handle, POSITION,
                    joint_limit_nh, urdf_model,
                    &joint_types_[1], &joint_lower_limits_[1], &joint_upper_limits_[1],
                    &joint_effort_limits_[1]);

    // Register interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    return true;
  }

  void readSim(ros::Time time, ros::Duration period)
  {
    for(unsigned int j=0; j < num_joints_; j++)
    {
      // Gazebo has an interesting API...
      if (joint_types_[j] == urdf::Joint::PRISMATIC)
      {
        joint_position_[j] = sim_joints_[j]->GetAngle(0).Radian();
      }
      else
      {
        joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                              sim_joints_[j]->GetAngle(0).Radian());
      }
      joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
      joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
    }
  }

  void writeSim(ros::Time time, ros::Duration period)
  {
    // effort_joint_saturation_interface_.enforceLimits(period);
    // effort_joint_limits_interface_.enforceLimits(period);
    position_joint_saturation_interface_.enforceLimits(period);
    position_joint_limits_interface_.enforceLimits(period);
    // velocity_joint_saturation_interface_.enforceLimits(period);
    // velocity_joint_limits_interface_.enforceLimits(period);

    // sim_joints_[0]->Update();
    // sim_joints_[0]->Reset();
    sim_joints_[1]->SetPosition(0, joint_position_command_[0]);
    

  }

private:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit)
  {
    *joint_type = urdf::Joint::UNKNOWN;
    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    *effort_limit = std::numeric_limits<double>::max();

    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != NULL)
    {
      const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
      if (urdf_joint != NULL)
      {
        *joint_type = urdf_joint->type;
        // Get limits from the URDF file.
        if (joint_limits_interface::getJointLimits(urdf_joint, limits))
          has_limits = true;
        if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
          has_soft_limits = true;
      }
    }
    // Get limits from the parameter server.
    if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
      has_limits = true;

    if (!has_limits)
      return;

    if (*joint_type == urdf::Joint::UNKNOWN)
    {
      // Infer the joint type.

      if (limits.has_position_limits)
      {
        *joint_type = urdf::Joint::REVOLUTE;
      }
      else
      {
        if (limits.angle_wraparound)
          *joint_type = urdf::Joint::CONTINUOUS;
        else
          *joint_type = urdf::Joint::PRISMATIC;
      }
    }

    if (limits.has_position_limits)
    {
      *lower_limit = limits.min_position;
      *upper_limit = limits.max_position;
    }
    if (limits.has_effort_limits)
      *effort_limit = limits.max_effort;

    if (has_soft_limits)
    {
      switch (ctrl_method)
      {
        case EFFORT:
          {
            const joint_limits_interface::EffortJointSoftLimitsHandle
              limits_handle(joint_handle, limits, soft_limits);
            effort_joint_limits_interface_.registerHandle(limits_handle);
          }
          break;
        case POSITION:
          {
            const joint_limits_interface::PositionJointSoftLimitsHandle
              limits_handle(joint_handle, limits, soft_limits);
            position_joint_limits_interface_.registerHandle(limits_handle);
          }
          break;
        case VELOCITY:
          {
            const joint_limits_interface::VelocityJointSoftLimitsHandle
              limits_handle(joint_handle, limits, soft_limits);
            velocity_joint_limits_interface_.registerHandle(limits_handle);
          }
          break;
      }
    }
    else
    {
      switch (ctrl_method)
      {
        case EFFORT:
          {
            const joint_limits_interface::EffortJointSaturationHandle
              sat_handle(joint_handle, limits);
            effort_joint_saturation_interface_.registerHandle(sat_handle);
          }
          break;
        case POSITION:
          {
            const joint_limits_interface::PositionJointSaturationHandle
              sat_handle(joint_handle, limits);
            position_joint_saturation_interface_.registerHandle(sat_handle);
          }
          break;
        case VELOCITY:
          {
            const joint_limits_interface::VelocityJointSaturationHandle
              sat_handle(joint_handle, limits);
            velocity_joint_saturation_interface_.registerHandle(sat_handle);
          }
          break;
      }
    }
  }

  /*
  This part is now in the base class since it is the same for real and sim interfaces
  unsigned int num_joints_;
  */
  /*
  hardware_interface::JointStateInterface    joint_state_interface_;
  hardware_interface::EffortJointInterface   effort_joint_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  joint_limits_interface::EffortJointSaturationInterface   effort_joint_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   effort_joint_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
*/

  // but this, eventhough it can be useful in a real interface as well
  // so it might be useful to have it in the base class
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;

};

typedef boost::shared_ptr<DefaultRRBOTHWSim> DefaultRRBOTHWSimPtr;

}

PLUGINLIB_EXPORT_CLASS(rrbot_hw::DefaultRRBOTHWSim, rrbot_hw::RRBOTHWSim)

#endif // #ifndef __RRBOT_HW____PLUGIN_DEFAULT_RRBOT_HW_SIM_H_
