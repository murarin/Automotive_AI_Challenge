/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "twist_gate3/twist_gate.h"
#include <ros_observer/lib_ros_observer.h>

#include "ros/ros.h"

#include <chrono>
#include <string>
#include <autoware_system_msgs/DiagnosticStatus.h>




using AwDiagStatus = autoware_system_msgs::DiagnosticStatus;

TwistGate::TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , timeout_period_(10.0)
  , command_mode_(CommandMode::AUTO)
  , previous_command_mode_(CommandMode::AUTO)
{
  private_nh_.param<double>("loop_rate", loop_rate_, 30.0);
  private_nh_.param<bool>("use_decision_maker", use_decision_maker_, false);

  health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_);
  control_command_pub_ = nh_.advertise<std_msgs::String>("/ctrl_mode", 1);
  vehicle_cmd_pub_ = nh_.advertise<vehicle_cmd_msg_t>("/vehicle_cmd", 1, true);
  remote_cmd_sub_ = nh_.subscribe("/remote_cmd", 1, &TwistGate::remote_cmd_callback, this);
  config_sub_ = nh_.subscribe("config/twist_filter", 1, &TwistGate::config_callback, this);

  timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_), &TwistGate::timer_callback, this);

  auto_cmd_sub_stdmap_["twist_cmd"] = nh_.subscribe("/twist_cmd", 1, &TwistGate::auto_cmd_twist_cmd_callback, this);
  auto_cmd_sub_stdmap_["mode_cmd"] = nh_.subscribe("/mode_cmd", 1, &TwistGate::mode_cmd_callback, this);
  auto_cmd_sub_stdmap_["gear_cmd"] = nh_.subscribe("/gear_cmd", 1, &TwistGate::gear_cmd_callback, this);
  auto_cmd_sub_stdmap_["accel_cmd"] = nh_.subscribe("/accel_cmd", 1, &TwistGate::accel_cmd_callback, this);
  auto_cmd_sub_stdmap_["steer_cmd"] = nh_.subscribe("/steer_cmd", 1, &TwistGate::steer_cmd_callback, this);
  auto_cmd_sub_stdmap_["brake_cmd"] = nh_.subscribe("/brake_cmd", 1, &TwistGate::brake_cmd_callback, this);
  auto_cmd_sub_stdmap_["lamp_cmd"] = nh_.subscribe("/lamp_cmd", 1, &TwistGate::lamp_cmd_callback, this);
  auto_cmd_sub_stdmap_["ctrl_cmd"] = nh_.subscribe("/ctrl_cmd", 1, &TwistGate::ctrl_cmd_callback, this);
  auto_cmd_sub_stdmap_["state"] = nh_.subscribe("/decision_maker/state", 1, &TwistGate::state_callback, this);
  auto_cmd_sub_stdmap_["emergency_velocity"] =
      nh_.subscribe("emergency_velocity", 1, &TwistGate::emergency_cmd_callback, this);




  auto_cmd_sub_stdmap_["ndt_pose"] = nh_.subscribe("/ndt_pose", 1, &TwistGate::ndt_pose_callback, this);
  auto_cmd_sub_stdmap_["obstacle_waypoint"] = nh_.subscribe("/obstacle_waypoint", 1, &TwistGate::obstacle_callback, this);



  twist_gate_msg_.header.seq = 0;
  emergency_stop_msg_.data = false;
  health_checker_ptr_->ENABLE();
  health_checker_ptr_->NODE_ACTIVATE();

  remote_cmd_time_ = ros::Time::now();
  emergency_handling_time_ = ros::Time::now();
  state_time_ = ros::Time::now();
  watchdog_timer_thread_ = std::thread(&TwistGate::watchdog_timer, this);
  is_alive = true;


  obstacle = false;
  obstacle2 = false;
  obstacle_num.data = -1;

}

TwistGate::~TwistGate()
{
  is_alive = false;
  watchdog_timer_thread_.join();
}

void TwistGate::reset_vehicle_cmd_msg()
{
  twist_gate_msg_.twist_cmd.twist.linear.x = 0;
  twist_gate_msg_.twist_cmd.twist.angular.z = 0;
  twist_gate_msg_.mode = 0;
  twist_gate_msg_.gear = 0;
  twist_gate_msg_.lamp_cmd.l = 0;
  twist_gate_msg_.lamp_cmd.r = 0;
  twist_gate_msg_.accel_cmd.accel = 0;
  twist_gate_msg_.brake_cmd.brake = 0;
  twist_gate_msg_.steer_cmd.steer = 0;
  twist_gate_msg_.ctrl_cmd.linear_velocity = -1;
  twist_gate_msg_.ctrl_cmd.steering_angle = 0;
  twist_gate_msg_.emergency = 0;
}

void TwistGate::check_state()
{
  const double state_msg_timeout = 0.5;
  double state_time_diff = ros::Time::now().toSec() - state_time_.toSec();

  if (use_decision_maker_ && (!is_state_drive_ || state_time_diff >= state_msg_timeout) )
  {
    twist_gate_msg_.twist_cmd.twist = geometry_msgs::Twist();
    twist_gate_msg_.ctrl_cmd = autoware_msgs::ControlCommand();
  }
}

void TwistGate::watchdog_timer()
{
  ShmVitalMonitor shm_vmon("TwistGate", 100);

  while (is_alive)
  {
    ros::Time now_time = ros::Time::now();

    // check command mode
    if (previous_command_mode_ != command_mode_)
    {
      if (command_mode_ == CommandMode::AUTO)
      {
        command_mode_topic_.data = "AUTO";
      }
      else if (command_mode_ == CommandMode::REMOTE)
      {
        command_mode_topic_.data = "REMOTE";
      }
      else
      {
        command_mode_topic_.data = "UNDEFINED";
      }

      control_command_pub_.publish(command_mode_topic_);
      previous_command_mode_ = command_mode_;
    }

    shm_vmon.run();

    // check lost Communication
    if (command_mode_ == CommandMode::REMOTE)
    {
      const double dt = (now_time - remote_cmd_time_).toSec() * 1000;
      health_checker_ptr_->CHECK_MAX_VALUE("remote_cmd_interval", dt, 700, 1000, 1500, "remote cmd interval is too "
                                                                                       "long.");
    }

    // check push emergency stop button
    const int level = (emergency_stop_msg_.data) ? AwDiagStatus::ERROR : AwDiagStatus::OK;
    health_checker_ptr_->CHECK_TRUE("emergency_stop_button", emergency_stop_msg_.data, level, "emergency stop button "
                                                                                              "is pushed.");

    // if no emergency message received for more than timeout_period_
    if ((now_time - emergency_handling_time_) > timeout_period_)
    {
      emergency_handling_active_ = false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}


//////
void TwistGate::obstacle_callback(const std_msgs::Int32& msg)
{
  obstacle_num = msg;
}


//////
void TwistGate::ndt_pose_callback(const geometry_msgs::PoseStamped& msg)
{

  current_pose_ = msg;

}


void TwistGate::remote_cmd_callback(const remote_msgs_t::ConstPtr& input_msg)
{
  remote_cmd_time_ = ros::Time::now();
  command_mode_ = static_cast<CommandMode>(input_msg->control_mode);
  emergency_stop_msg_.data = static_cast<bool>(input_msg->vehicle_cmd.emergency);

  // Update Emergency Mode
  twist_gate_msg_.emergency = input_msg->vehicle_cmd.emergency;
  if (command_mode_ == CommandMode::REMOTE && emergency_stop_msg_.data == false)
  {
    twist_gate_msg_.header.frame_id = input_msg->vehicle_cmd.header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->vehicle_cmd.header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->vehicle_cmd.twist_cmd.twist;
    twist_gate_msg_.ctrl_cmd = input_msg->vehicle_cmd.ctrl_cmd;
    twist_gate_msg_.accel_cmd = input_msg->vehicle_cmd.accel_cmd;
    twist_gate_msg_.brake_cmd = input_msg->vehicle_cmd.brake_cmd;
    twist_gate_msg_.steer_cmd = input_msg->vehicle_cmd.steer_cmd;
    twist_gate_msg_.gear = input_msg->vehicle_cmd.gear;
    twist_gate_msg_.lamp_cmd = input_msg->vehicle_cmd.lamp_cmd;
    twist_gate_msg_.mode = input_msg->vehicle_cmd.mode;
  }
}

void TwistGate::auto_cmd_twist_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& input_msg)
{
  health_checker_ptr_->CHECK_RATE("topic_rate_twist_cmd_slow", 8, 5, 1, "topic twist_cmd subscribe rate slow.");
  health_checker_ptr_->CHECK_MAX_VALUE("twist_cmd_linear_high", input_msg->twist.linear.x,
    DBL_MAX, DBL_MAX, DBL_MAX, "linear twist_cmd is too high");

  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->twist;

    check_state();
  }
}

void TwistGate::mode_cmd_callback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    // TODO(unknown): check this if statement
    if (input_msg->mode == -1 || input_msg->mode == 0)
    {
      reset_vehicle_cmd_msg();
    }
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.mode = input_msg->mode;
  }
}

void TwistGate::gear_cmd_callback(const tablet_socket_msgs::gear_cmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.gear = input_msg->gear;
  }
}

void TwistGate::accel_cmd_callback(const autoware_msgs::AccelCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.accel_cmd.accel = input_msg->accel;
  }
}

void TwistGate::steer_cmd_callback(const autoware_msgs::SteerCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.steer_cmd.steer = input_msg->steer;
  }
}

void TwistGate::brake_cmd_callback(const autoware_msgs::BrakeCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.brake_cmd.brake = input_msg->brake;
  }
}

void TwistGate::lamp_cmd_callback(const autoware_msgs::LampCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.lamp_cmd.l = input_msg->l;
    twist_gate_msg_.lamp_cmd.r = input_msg->r;
  }
}

void TwistGate::ctrl_cmd_callback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.ctrl_cmd = input_msg->cmd;

    check_state();
  }
}

void TwistGate::state_callback(const std_msgs::StringConstPtr& input_msg)
{
  state_time_ = ros::Time::now();
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    // Set Parking Gear
    if (input_msg->data.find("WaitOrder") != std::string::npos)
    {
      twist_gate_msg_.gear = CMD_GEAR_P;
    }
    // Set Drive Gear
    else
    {
      twist_gate_msg_.gear = CMD_GEAR_D;
    }

    // get drive state
    if (input_msg->data.find("Drive\n") != std::string::npos &&
        input_msg->data.find("VehicleReady\n") != std::string::npos)
    {
      is_state_drive_ = true;
    }
    else
    {
      is_state_drive_ = false;
    }

    // reset emergency flags
    if (input_msg->data.find("VehicleReady") != std::string::npos)
    {
      emergency_stop_msg_.data = false;
    }
  }
}

void TwistGate::emergency_cmd_callback(const vehicle_cmd_msg_t::ConstPtr& input_msg)
{
  emergency_handling_time_ = ros::Time::now();
  emergency_handling_active_ = true;
  twist_gate_msg_ = *input_msg;
}

void TwistGate::timer_callback(const ros::TimerEvent& e)
{
  twist_gate_msg_.gear = 64;
  twist_gate_msg_.ctrl_cmd.steering_angle = -twist_gate_msg_.ctrl_cmd.steering_angle;

  if (twist_gate_msg_.ctrl_cmd.linear_acceleration > 0) {
    // twist_gate_msg_.ctrl_cmd.linear_acceleration /= 10.0;
    twist_gate_msg_.ctrl_cmd.linear_acceleration = std::min(twist_gate_msg_.ctrl_cmd.linear_acceleration, 0.5);

  } else {
    // twist_gate_msg_.ctrl_cmd.linear_acceleration *= 2.0;
    twist_gate_msg_.ctrl_cmd.linear_acceleration = std::max(twist_gate_msg_.ctrl_cmd.linear_acceleration * 1.0, -1.0);
    // twist_gate_msg_.ctrl_cmd.linear_acceleration = std::min(twist_gate_msg_.ctrl_cmd.linear_acceleration, -0.1);

  }


  tf::Vector3 v1(current_pose_.pose.position.x, current_pose_.pose.position.y, 0);
  tf::Vector3 v2(-105.1776, -322.5798, 0);

  // tf::Vector3 v3(-41.9457,-351.8343, 0);
  tf::Vector3 v3(-43.934,-351.8187, 0);

  double dist = tf::tfDistance(v1, v2);
  double dist2 = tf::tfDistance(v1, v3);


  if (dist < 12.0 && !obstacle)
  {
    if (obstacle_num.data > 0) obstacle = true;


    if (!obstacle) {
      twist_gate_msg_.ctrl_cmd.linear_acceleration = -1.0;
    }

  }

  if (dist2 < 8.0 && !obstacle2)
  {
    if (obstacle_num.data > 0) obstacle2 = true;

    if (!obstacle2) {
      twist_gate_msg_.ctrl_cmd.linear_acceleration = -1.0;

    }
  }



  ROS_INFO("%lf", dist);
  ROS_INFO("fajlasglj");



  vehicle_cmd_pub_.publish(twist_gate_msg_);
}

void TwistGate::config_callback(const autoware_config_msgs::ConfigTwistFilter& msg)
{
  use_decision_maker_ = msg.use_decision_maker;
}
