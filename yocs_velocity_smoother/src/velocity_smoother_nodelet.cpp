/*
 * velocity_smoother_nodelet.cpp
 *
 *  Created on: Oct 25, 2012
 *      Author: jorge
 */

/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   velocity_smoother_nodelet.cpp
 * @brief  Implementation for the command velocity smoother
 * @date   Dec 2, 2012
 * @author Jorge
 **/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "velocity_smoother/velocity_smoother_nodelet.h"

/*********************
** Inplementation
**********************/

VelSmoother::VelSmoother() { }

VelSmoother::~VelSmoother() { }

void VelSmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Estimate commands frequency; it can be very different depending on the publisher type
  double frequency = 1.0/(ros::Time::now() - last_cmd_time).toSec();
  double period = (ros::Time::now() - last_cmd_time).toSec();
  last_cmd_time = ros::Time::now();

  // Ensure we don't exceed the acceleration limits; for each dof, we calculate the
  // commanded velocity increment and the maximum allowed increment (i.e. acceleration)
  geometry_msgs::Twist cmd_vel = *msg;
  double cmd_vel_inc, max_vel_inc;
  //double period = 1.0/frequency;

  cmd_vel_inc = cmd_vel.linear.x - last_cmd_vel.linear.x;
  if (odometry_vel.linear.x*cmd_vel.linear.x < 0.0)
  {
    max_vel_inc = decel_lim_x*period;   // countermarch
    ROS_ERROR("countermarch  X");
  }
  else
  {
    max_vel_inc = ((cmd_vel_inc*cmd_vel.linear.x > 0.0)?accel_lim_x:decel_lim_x)*period;
  }
  if (std::abs(cmd_vel_inc) > max_vel_inc)
  {
    cmd_vel.linear.x = last_cmd_vel.linear.x + sign(cmd_vel_inc)*max_vel_inc;
  }

  cmd_vel_inc = cmd_vel.linear.y - last_cmd_vel.linear.y;
  if (odometry_vel.linear.y*cmd_vel.linear.y < 0.0)
  {
    max_vel_inc = decel_lim_y*period;   // countermarch
  }
  else
  {
    max_vel_inc = ((cmd_vel_inc*cmd_vel.linear.y > 0.0)?accel_lim_y:decel_lim_y)*period;
  }
  if (std::abs(cmd_vel_inc) > max_vel_inc)
  {
    cmd_vel.linear.y = last_cmd_vel.linear.y + sign(cmd_vel_inc)*max_vel_inc;
  }

  cmd_vel_inc = cmd_vel.angular.z - last_cmd_vel.angular.z;
  if (odometry_vel.angular.z*cmd_vel.angular.z < 0.0)
  {
    max_vel_inc = decel_lim_t*period;   // countermarch
  }
  else
  {
    max_vel_inc = ((cmd_vel_inc*cmd_vel.angular.z > 0.0)?accel_lim_t:decel_lim_t)*period;

  }
  if (std::abs(cmd_vel_inc) > max_vel_inc)
  {
    cmd_vel.angular.z = last_cmd_vel.angular.z + sign(cmd_vel_inc)*max_vel_inc;
  }

  lim_vel_pub.publish(cmd_vel);
  last_cmd_vel = cmd_vel;
}

void VelSmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometry_vel = msg->twist.twist;
}

/**
 * Initialise from a nodelet's private nodehandle.
 * @param nh : private nodehandle
 * @return bool : success or failure
 */
bool VelSmoother::init(ros::NodeHandle& nh)
{
  nh.getParam("accel_lim_x",  accel_lim_x);
  nh.getParam("accel_lim_y",  accel_lim_y);
  nh.getParam("accel_lim_t",  accel_lim_t);
  nh.getParam("decel_factor", decel_factor);

  // Deceleration can be more aggressive, if necessary
  decel_lim_x = decel_factor*accel_lim_x;
  decel_lim_y = decel_factor*accel_lim_y;
  decel_lim_t = decel_factor*accel_lim_t;

  // Publishers and subscribers
  cur_vel_sub = nh.subscribe("odometry",    1, &VelSmoother::odometryCB, this);
  raw_vel_sub = nh.subscribe("raw_cmd_vel", 1, &VelSmoother::velocityCB, this);
  lim_vel_pub = nh.advertise <geometry_msgs::Twist> ("smooth_cmd_vel", 1);

  ROS_INFO("Velocity smoother nodelet successfully initialized");

  return true;
}


/*********************
** Nodelet
**********************/

class VelSmootherNodelet : public nodelet::Nodelet
{
public:
  VelSmootherNodelet()  { }
  ~VelSmootherNodelet() { }

  virtual void onInit()
  {
    NODELET_DEBUG("Initialising nodelet...");

    vs_.reset(new VelSmoother);
    if (vs_->init(this->getPrivateNodeHandle()))
    {
      NODELET_DEBUG("Command velocity smoother nodelet initialised");
    }
    else
    {
      NODELET_ERROR("Command velocity smoother nodelet initialisation failed");
    }
  }

private:
  boost::shared_ptr<VelSmoother> vs_;
};

PLUGINLIB_DECLARE_CLASS(velocity_smoother, VelSmootherNodelet, VelSmootherNodelet, nodelet::Nodelet);
