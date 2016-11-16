/*
 * Copyright (c) 2013, Marcus Liebhardt, Yujin Robot.
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

#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <yocs_math_toolkit/geometry.hpp>

#include "yocs_diff_drive_pose_controller/diff_drive_pose_controller_ros.hpp"

namespace yocs
{

bool DiffDrivePoseControllerROS::init()
{
  enable_controller_subscriber_ = nh_.subscribe("enable", 10, &DiffDrivePoseControllerROS::enableCB, this);
  disable_controller_subscriber_ = nh_.subscribe("disable", 10, &DiffDrivePoseControllerROS::disableCB, this);
  control_velocity_subscriber_ = nh_.subscribe("control_max_vel", 10, &DiffDrivePoseControllerROS::controlMaxVelCB,
                                               this);
  command_velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("command_velocity", 10);
  pose_reached_publisher_ = nh_.advertise<std_msgs::Bool>("pose_reached", 10);

  // retrieve configuration parameters
  base_frame_name_ = "base_footprint";
  if (!nh_.getParam("base_frame_name", base_frame_name_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'base_frame_name' from parameter server! Using default '" << base_frame_name_ << "'. [" << name_ <<"]");
  }
  goal_frame_name_ = "base_goal_pose";
  if (!nh_.getParam("goal_frame_name", goal_frame_name_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'goal_frame_name' from parameter server! Using default '" << goal_frame_name_ << "'. [" << name_ <<"]");
  }

  if (!nh_.getParam("v_min", v_min_movement_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'v_min' from parameter server! Using default '" << v_min_movement_ << "'. [" << name_ <<"]");
  }
  v_min_ = v_min_movement_;
  if (!nh_.getParam("v_max", v_max_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'v_max' from parameter server! Using default '" << v_max_ << "'. [" << name_ <<"]");
  }
//  v_min_ = -v_max_; //if we also want to enable driving backwards
  if (!nh_.getParam("w_min", w_min_movement_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'w_min' from parameter server! Using default '" << w_min_movement_ << "'. [" << name_ <<"]");
  }
  w_max_ = M_PI / 4 * v_max_;
  if (!nh_.getParam("w_max", w_max_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'w_max' from parameter server! Using default '" << w_max_ << "'. [" << name_ <<"]");
  }
  w_min_ = -w_max_;

  if (!nh_.getParam("k_1", k_1_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'k_1' from parameter server! Using default '" << k_1_ << "'. [" << name_ <<"]");
  }
  if (!nh_.getParam("k_2", k_2_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'k_2' from parameter server! Using default '" << k_2_ << "'. [" << name_ <<"]");
  }
  if (!nh_.getParam("beta", beta_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'beta' from parameter server! Using default '" << beta_ << "'. [" << name_ <<"]");
  }
  if (!nh_.getParam("lambda", lambda_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'lambda' from parameter server! Using default '" << lambda_ << "'. [" << name_ <<"]");
  }
  if (!nh_.getParam("dist_thres", dist_thres_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'dist_thres' from parameter server! Using default '" << dist_thres_ << "'. [" << name_ <<"]");
  }
  if (!nh_.getParam("orient_thres", orient_thres_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'orient_thres' from parameter server! Using default '" << orient_thres_ << "'. [" << name_ <<"]");
  }
  dist_eps_ = dist_eps_ * 0.2;
  if (!nh_.getParam("dist_eps", dist_eps_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'dist_eps' from parameter server! Using default '" << dist_eps_ << "'. [" << name_ <<"]");
  }
  orient_eps_ = orient_thres_ * 0.2;
  if (!nh_.getParam("orient_eps", orient_eps_))
  {
    ROS_WARN_STREAM(
        "Couldn't retrieve parameter 'orient_eps' from parameter server! Using default '" << orient_eps_ << "'. [" << name_ <<"]");
  }

  ROS_DEBUG_STREAM("Controller initialised with the following parameters: [" << name_ <<"]");
  ROS_DEBUG_STREAM(
      "base_frame_name = " << base_frame_name_ <<", goal_frame_name = " << goal_frame_name_ << " [" << name_ <<"]");
  ROS_DEBUG_STREAM(
      "v_max = " << v_max_ <<", k_1 = " << k_1_ << ", k_2 = " << k_2_ << ", beta = " << beta_ << ", lambda = " << lambda_ << ", dist_thres = " << dist_thres_ << ", orient_thres = " << orient_thres_ <<" [" << name_ <<"]");

  reconfig_server_ = boost::shared_ptr<dynamic_reconfigure::Server<yocs_msgs::PoseControllerConfig> >(
                               new dynamic_reconfigure::Server<yocs_msgs::PoseControllerConfig>(nh_));

  ///dynamic reconfigure server callback type
  dynamic_reconfigure::Server<yocs_msgs::PoseControllerConfig>::CallbackType reconfig_callback_func;

  reconfig_callback_func = boost::bind(&DiffDrivePoseControllerROS::reconfigCB, this, _1, _2);

  reconfig_server_->setCallback(reconfig_callback_func);

  return true;
}

void DiffDrivePoseControllerROS::spinOnce()
{
  if (this->getState())
  {
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Controller spinning. [" << name_ <<"]");
    // determine pose difference in polar coordinates
    if (!getPoseDiff())
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Getting pose difference failed. Skipping control loop. [" << name_ <<"]");
      return;
    }
    // determine controller output (v, w) and check if goal is reached
    step();

    // set control output (v, w)
    setControlOutput();
    // Logging
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Current state: [" << name_ <<"]");
    ROS_DEBUG_STREAM_THROTTLE(1.0,
                              "r = " << r_ << ", theta = " << theta_ << ", delta = " << delta_ << " [" << name_ <<"]");
    ROS_DEBUG_STREAM_THROTTLE(1.0, "cur = " << cur_ << ", v = " << v_ << ", w = " << w_ << " [" << name_ <<"]");
  }
  else
  {
    ROS_DEBUG_STREAM_THROTTLE(3.0, "Controller is disabled. Idling ... [" << name_ <<"]");
  }
}

bool DiffDrivePoseControllerROS::getPoseDiff()
{
  // use tf to get information about the goal pose relative to the base
  try
  {
    tf_listener_.lookupTransform(base_frame_name_, goal_frame_name_, ros::Time(0), tf_goal_pose_rel_);
  }
  catch (tf::TransformException const &ex)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Couldn't get transform from base to goal pose! [" << name_ <<"]");
    ROS_DEBUG_STREAM_THROTTLE(1.0, "tf error: " << ex.what());
    return false;
  }

  // determine distance to goal
  double r = std::sqrt(
      std::pow(tf_goal_pose_rel_.getOrigin().getX(), 2) + std::pow(tf_goal_pose_rel_.getOrigin().getY(), 2));
  // determine orientation of r relative to the base frame
  double delta = std::atan2(-tf_goal_pose_rel_.getOrigin().getY(), tf_goal_pose_rel_.getOrigin().getX());

  // determine orientation of r relative to the goal frame
  // helper: theta = tf's orientation + delta
  double heading = mtk::wrapAngle(tf::getYaw(tf_goal_pose_rel_.getRotation()));
  double theta = heading + delta;

  setInput(r, delta, theta);

  return true;
}

void DiffDrivePoseControllerROS::onGoalReached()
{
  std_msgs::Bool bool_msg;
  bool_msg.data = true;
  pose_reached_publisher_.publish(bool_msg);
}

void DiffDrivePoseControllerROS::setControlOutput()
{
  geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist());
  if (!pose_reached_)
  {
    cmd_vel->linear.x = v_;
    cmd_vel->angular.z = w_;
  }
  command_velocity_publisher_.publish(cmd_vel);
}

void DiffDrivePoseControllerROS::controlMaxVelCB(const std_msgs::Float32ConstPtr msg)
{
  v_max_ = msg->data;
  //v_min_ = -v_max_; //if we also want to enable driving backwards
  ROS_INFO_STREAM("Maximum linear control velocity has been set to " << v_max_ << ". [" << name_ << "]");
}

void DiffDrivePoseControllerROS::enableCB(const std_msgs::StringConstPtr msg)
{
  if (!msg->data.empty())
  {
    goal_frame_name_ = msg->data;
  }

  if (this->enable())
  {
    ROS_INFO_STREAM("Controller has been enabled [" << name_ << "] with goal frame [" << goal_frame_name_ << "].");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled [" << name_ <<"], now tracking goal frame [" << goal_frame_name_ << "].");
  }
}

void DiffDrivePoseControllerROS::disableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->disable())
  {
    ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
}

void DiffDrivePoseControllerROS::reconfigCB(yocs_msgs::PoseControllerConfig &config, uint32_t level)
{
  controller_mutex_.lock();
  v_min_movement_ = config.v_min;
  v_max_ = config.v_max;
  w_max_ = config.w_max;
  w_min_ = config.w_min;
  k_1_ = config.k_1;
  k_2_ = config.k_2;
  beta_ = config.beta;
  lambda_ = config.lambda;
  dist_thres_ = config.dist_thres;
  orient_thres_ = config.orient_thres;
  dist_eps_ = config.dist_eps;
  orient_eps_ = config.orient_eps;
  controller_mutex_.unlock();
}

} // namespace yocs
