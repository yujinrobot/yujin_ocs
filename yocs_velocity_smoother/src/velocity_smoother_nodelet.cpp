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

#include <ecl/threads/thread.hpp>

#include "velocity_smoother/velocity_smoother_nodelet.h"

#define PERIOD_RECORD_SIZE    5
#define ZERO_VEL_COMMAND      geometry_msgs::Twist();
#define IS_ZERO_VEOCITY(a)   ((a.linear.x == 0.0) && (a.linear.y == 0.0) && (a.angular.z == 0.0))


/*********************
** Implementation
**********************/

void VelocitySmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Estimate commands frequency; we do continuously as it can be very different depending on the
  // publisher type, and we don't want to impose extra constraints to keep this package flexible
  if (period_record.size() < PERIOD_RECORD_SIZE)
    period_record.push_back((ros::Time::now() - last_cb_time).toSec());
  else
    period_record[pr_next] = (ros::Time::now() - last_cb_time).toSec();

  pr_next++;
  pr_next %= period_record.size();
  last_cb_time = ros::Time::now();

  if (period_record.size() <= PERIOD_RECORD_SIZE/2)  // wait until we have an acceptable estimate
    return;

  cb_avg_time  = median(period_record);              // enough; recalculate with the latest input
  input_active = true;
  target_vel   = *msg;
}

void VelocitySmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometry_vel = msg->twist.twist;
}

void VelocitySmoother::spin()
{
  double period = 1.0/frequency;
  ros::Rate spin_rate(frequency);

  while (! shutdown_req && ros::ok())
  {
    if ((input_active == true) && ((ros::Time::now() - last_cb_time).toSec() > 3.0*cb_avg_time))
    {
      // Velocity input no active anymore; normally last command is a zero-velocity one, but reassure
      // this, just in case something went wrong with our input, or he just forgot good manners...
      input_active = false;
      if (IS_ZERO_VEOCITY(target_vel) == false)
      {
        ROS_WARN("Input got inactive letting us a non-zero target velocity (%f, %f, %f); zeroing...",
                 target_vel.linear.x, target_vel.linear.y, target_vel.angular.z);
        target_vel = ZERO_VEL_COMMAND;
      }
    }

    if ((input_active == true) &&
        (((ros::Time::now() - last_cb_time).toSec() > 3.0*cb_avg_time)     || // 3 missing msgs
         (std::abs(odometry_vel.linear.x  - last_cmd_vel.linear.x)  > 0.1) ||
         (std::abs(odometry_vel.linear.y  - last_cmd_vel.linear.y)  > 0.1) ||
         (std::abs(odometry_vel.angular.z - last_cmd_vel.angular.z) > 0.5)))
    {
      // If the publisher has been inactive for a while, or if odometry velocity has diverged
      // significatively from last_cmd_vel, we cannot trust the latter; relay on odometry instead
      // TODO: odometry thresholds are 진짜 arbitrary; should be proportional to the max v and w...
      ROS_DEBUG("Using odometry instead of last command: %f, %f, %f, %f",
                (ros::Time::now()      - last_cb_time).toSec(),
                odometry_vel.linear.x  - last_cmd_vel.linear.x,
                odometry_vel.linear.y  - last_cmd_vel.linear.y,
                odometry_vel.angular.z - last_cmd_vel.angular.z);
      last_cmd_vel = odometry_vel;
    }

    geometry_msgs::TwistPtr cmd_vel;

    if ((target_vel.linear.x  != last_cmd_vel.linear.x) ||
        (target_vel.linear.y  != last_cmd_vel.linear.y) ||
        (target_vel.angular.z != last_cmd_vel.angular.z))
    {
      // Try to reach target velocity but...
      cmd_vel.reset(new geometry_msgs::Twist(target_vel));

      // ...ensure we don't exceed the acceleration limits: for each dof, we calculate the
      // commanded velocity increment and the maximum allowed increment (i.e. acceleration)
      double cmd_vel_inc, max_vel_inc;

      cmd_vel_inc = target_vel.linear.x - last_cmd_vel.linear.x;
      if (odometry_vel.linear.x*target_vel.linear.x < 0.0)
      {
        max_vel_inc = decel_lim_x*period;   // countermarch
      }
      else
      {
        max_vel_inc = ((cmd_vel_inc*target_vel.linear.x > 0.0)?accel_lim_x:decel_lim_x)*period;
      }
      if (std::abs(cmd_vel_inc) > max_vel_inc)
      {
        cmd_vel->linear.x = last_cmd_vel.linear.x + sign(cmd_vel_inc)*max_vel_inc;
      }

      cmd_vel_inc = target_vel.linear.y - last_cmd_vel.linear.y;
      if (odometry_vel.linear.y*target_vel.linear.y < 0.0)
      {
        max_vel_inc = decel_lim_y*period;   // countermarch
      }
      else
      {
        max_vel_inc = ((cmd_vel_inc*target_vel.linear.y > 0.0)?accel_lim_y:decel_lim_y)*period;
      }
      if (std::abs(cmd_vel_inc) > max_vel_inc)
      {
        cmd_vel->linear.y = last_cmd_vel.linear.y + sign(cmd_vel_inc)*max_vel_inc;
      }

      cmd_vel_inc = target_vel.angular.z - last_cmd_vel.angular.z;
      if (odometry_vel.angular.z*target_vel.angular.z < 0.0)
      {
        max_vel_inc = decel_lim_th*period;  // countermarch
      }
      else
      {
        max_vel_inc = ((cmd_vel_inc*target_vel.angular.z > 0.0)?accel_lim_th:decel_lim_th)*period;

      }
      if (std::abs(cmd_vel_inc) > max_vel_inc)
      {
        cmd_vel->angular.z = last_cmd_vel.angular.z + sign(cmd_vel_inc)*max_vel_inc;
      }

      lim_vel_pub.publish(cmd_vel);
      last_cmd_vel = *cmd_vel;
    }
    else if (input_active == true)
    {
      // We already reached target velocity; just keep resending last command while input is active
      cmd_vel.reset(new geometry_msgs::Twist(last_cmd_vel));
      lim_vel_pub.publish(cmd_vel);
    }

    spin_rate.sleep();
  }
}

/**
 * Initialise from a nodelet's private nodehandle.
 * @param nh : private nodehandle
 * @return bool : success or failure
 */
bool VelocitySmoother::init(ros::NodeHandle& nh)
{
  // Optional parameters
  nh.param("frequency",    frequency,   20.0);
  nh.param("decel_factor", decel_factor, 1.0);

  // Mandatory parameters
  if ((nh.getParam("accel_lim_x",  accel_lim_x)  == false) ||
      (nh.getParam("accel_lim_y",  accel_lim_y)  == false) ||
      (nh.getParam("accel_lim_th", accel_lim_th) == false))
  {
    ROS_ERROR("Missing acceleration limit parameter(s)");
    return false;
  }

  // Deceleration can be more aggressive, if necessary
  decel_lim_x  = decel_factor*accel_lim_x;
  decel_lim_y  = decel_factor*accel_lim_y;
  decel_lim_th = decel_factor*accel_lim_th;

  // Publishers and subscribers
  cur_vel_sub = nh.subscribe("odometry",    1, &VelocitySmoother::odometryCB, this);
  raw_vel_sub = nh.subscribe("raw_cmd_vel", 1, &VelocitySmoother::velocityCB, this);
  lim_vel_pub = nh.advertise <geometry_msgs::Twist> ("smooth_cmd_vel", 1);

  ROS_INFO("Velocity smoother nodelet successfully initialized");

  return true;
}


/*********************
** Nodelet
**********************/

class VelocitySmootherNodelet : public nodelet::Nodelet
{
public:
  VelocitySmootherNodelet()  { }
  ~VelocitySmootherNodelet()
  {
    NODELET_DEBUG("Waiting for worker thread to finish...");
    vel_smoother_->shutdown();
    worker_thread_.join();
  }

  virtual void onInit()
  {
    NODELET_DEBUG("Initialising nodelet...");

    vel_smoother_.reset(new VelocitySmoother);
    if (vel_smoother_->init(this->getPrivateNodeHandle()))
    {
      NODELET_DEBUG("Command velocity smoother nodelet initialised");
      worker_thread_.start(&VelocitySmoother::spin, *vel_smoother_);
    }
    else
    {
      NODELET_ERROR("Command velocity smoother nodelet initialisation failed");
    }
  }

private:
  boost::shared_ptr<VelocitySmoother> vel_smoother_;
  ecl::Thread                        worker_thread_;
};

PLUGINLIB_DECLARE_CLASS(velocity_smoother, VelocitySmootherNodelet, VelocitySmootherNodelet, nodelet::Nodelet);
