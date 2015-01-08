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

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef YOCS_CONTROLLER_HPP_
#define YOCS_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <cmath>
#include <string>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <yocs_controllers/default_controller.hpp>
#include <yocs_math_toolkit/geometry.hpp>

namespace yocs
{

/**
 * @brief A controller for driving a differential drive base to a pose goal
 *         or along a path specified by multiple poses.
 *
 * This controller implements a control law drives a differental drive base towards a planar pose goal,
 * i.e. 2D position (x,y) + 1D orientation (theta). It also allows path following by specifying multiple pose goals.
 * The control law contains a transition strategy, which insures that the base moves through each pose and transitions
 * smoothly to the next pose goal.
 *
 * This controller is an implementation of control law based on the following work:
 * @inproceedings{DBLP:conf/icra/ParkK11,
 *   author    = {Jong Jin Park and
 *                Benjamin Kuipers},
 *   title     = {A smooth control law for graceful motion of differential
 *                wheeled mobile robots in 2D environment},
 *   booktitle = {ICRA},
 *   year      = {2011},
 *   pages     = {4896-4902},
 *   ee        = {http://dx.doi.org/10.1109/ICRA.2011.5980167},
 *   crossref  = {DBLP:conf/icra/2011},
 *   bibsource = {DBLP, http://dblp.uni-trier.de}
 * }
 *
 * This controller can be enabled/disabled.
 */
class DiffDrivePoseController : public Controller
{
public:
  DiffDrivePoseController(ros::NodeHandle& nh, std::string& name) : Controller(),
                                                                       nh_(nh),
                                                                       name_(name){};
  virtual ~DiffDrivePoseController(){};

  /**
   * @brief Set-up necessary publishers/subscribers and variables
   * @return true, if successful
   */
  bool init();

  /**
   * @brief Calculates velocity commands to move the diff-drive base to the (next) pose goal.
   */
  void spinOnce();

private:
  /**
   * @brief Determines the pose difference in polar coordinates
   */
  bool getPoseDiff();
  /**
   * @brief Calculates the controller output based on the current pose difference
   */
  void getControlOutput();
  /**
   * @brief Sends out the new velocity commands for the left and right wheel based on the current controller output
   */
  void setControlOutput();

  /**
   * @brief Callback for updating the controller's maximum linear control velocity
   * @param msg maximum linear control velocity
   */
  void controlMaxVelCB(const std_msgs::Float32ConstPtr msg);

  /**
   * @brief Callback for enabling the controler
   * @param msg goal frame name
   */
  void enableCB(const std_msgs::StringConstPtr msg);

  /**
   * @brief Callback for disabling the controler
   * @param msg empty message
   */
  void disableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief Bounding range of velocity
   * @param v velocity
   * @param min minimum speed
   * @param max maximum speed
   * @return bounded velocity
   */
  double boundRange(double v, double min, double max);

  // basics
  ros::NodeHandle nh_;
  std::string name_;

  // interfaces
  /// subscriber for enabling the controller
  ros::Subscriber enable_controller_subscriber_;
  /// subscriber for disabling the controller
  ros::Subscriber disable_controller_subscriber_;
  /// subscriber for setting the controller's linear velocity
  ros::Subscriber control_velocity_subscriber_;
  /// publisher for sending out base velocity commands
  ros::Publisher command_velocity_publisher_;
  /// publishes the status of the goal pose tracking
  ros::Publisher pose_reached_publisher_;

  // variables and constants for the control law
  /// distance to pose goal [m]
  double r_;
  /// direction of the pose goal [rad]
  double theta_;
  /// current heading of the base [rad]
  double delta_;
  /// linear base velocity [m/s]
  double v_;
  /// minimum linear base velocity [m/s]
  double v_min_;
  /// maximum linear base velocity [m/s]
  double v_max_;
  /// angular base velocity [rad/s]
  double w_;
  /// minimum angular base velocity [rad/s]
  double w_min_;
  /// maximum angular base velocity [rad/s]
  double w_max_;
  /// path to goal curvature
  double cur_;
  /// constant factor determining the ratio of the rate of change in theta to the rate of change in r
  double k_1_;
  /// constant factor applied to the heading error feedback
  double k_2_;
  /**
   * constant factor for the curvature-based velocity rule
   * determines how fast the velocity drops when the curvature increases
   */
  double beta_;
  /**
   * constant factor for the curvature-based velocity rule
   * determines the sharpness of the curve: higher lambda -> bigger drop in short term, smaller in the long term
   */
  double lambda_;
  /// lower bound for the distance (v = 0)
  double dist_thres_;
  /// lower bound for the orientation (w = 0)
  double orient_thres_;
  /// True, if pose has been reached (v == 0, w == 0)
  bool pose_reached_;
  /// Error in distance above which pose is considered differen
  double dist_eps_;
  /// Error in orientation above which pose is considered differen
  double orient_eps_;

  /// tf used to get goal pose relative to the base pose
  tf::TransformListener tf_listener_;
  /// transform for the goal pose relative to the base pose
  tf::StampedTransform tf_goal_pose_rel_;
  /// frame name of the base
  std::string base_frame_name_;
  /// frame name of the goal (pose)
  std::string goal_frame_name_;
};

bool DiffDrivePoseController::init()
{
  enable_controller_subscriber_ = nh_.subscribe("enable", 10, &DiffDrivePoseController::enableCB, this);
  disable_controller_subscriber_ = nh_.subscribe("disable", 10, &DiffDrivePoseController::disableCB, this);
  control_velocity_subscriber_ = nh_.subscribe("control_max_vel", 10, &DiffDrivePoseController::controlMaxVelCB, this);
  command_velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("command_velocity", 10);
  pose_reached_publisher_ = nh_.advertise<std_msgs::Bool>("pose_reached", 10);

  // retrieve configuration parameters
  base_frame_name_ = "base_footprint";
  if(!nh_.getParam("base_frame_name", base_frame_name_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'base_frame_name' from parameter server! Using default '"
                    << base_frame_name_ << "'. [" << name_ <<"]");
  }
  goal_frame_name_ = "base_goal_pose";
  if(!nh_.getParam("goal_frame_name", goal_frame_name_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'goal_frame_name' from parameter server! Using default '"
                    << goal_frame_name_ << "'. [" << name_ <<"]");
  }
  v_ = 0.0;
  v_min_ = 0.01;
  if(!nh_.getParam("v_min", v_min_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'v_min' from parameter server! Using default '"
                    << v_min_ << "'. [" << name_ <<"]");
  }
  v_max_ = 0.5;
  if(!nh_.getParam("v_max", v_max_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'v_max' from parameter server! Using default '"
                    << v_max_ << "'. [" << name_ <<"]");
  }
  w_min_ = 0.01;
  if(!nh_.getParam("w_min", w_min_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'w_min' from parameter server! Using default '"
                    << w_min_ << "'. [" << name_ <<"]");
  }
  w_max_ = M_PI / 4 * v_max_;
  if(!nh_.getParam("w_max", w_max_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'w_max' from parameter server! Using default '"
                    << w_max_ << "'. [" << name_ <<"]");
  }
  k_1_ = 1.0;
  if(!nh_.getParam("k_1", k_1_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'k_1' from parameter server! Using default '"
                    << k_1_ << "'. [" << name_ <<"]");
  }
  k_2_ = 3.0;
  if(!nh_.getParam("k_2", k_2_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'k_2' from parameter server! Using default '"
                    << k_2_ << "'. [" << name_ <<"]");
  }
  beta_ = 0.4;
  if(!nh_.getParam("beta", beta_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'beta' from parameter server! Using default '"
                    << beta_ << "'. [" << name_ <<"]");
  }
  lambda_ = 2.0;
  if(!nh_.getParam("lambda", lambda_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'lambda' from parameter server! Using default '"
                    << lambda_ << "'. [" << name_ <<"]");
  }
  dist_thres_ = 0.01;
  if(!nh_.getParam("dist_thres", dist_thres_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'dist_thres' from parameter server! Using default '"
                    << dist_thres_ << "'. [" << name_ <<"]");
  }
  orient_thres_ = 0.02;
  if(!nh_.getParam("orient_thres", orient_thres_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'orient_thres' from parameter server! Using default '"
                    << orient_thres_ << "'. [" << name_ <<"]");
  }
  dist_eps_ = dist_eps_ * 0.2;
  if(!nh_.getParam("dist_eps", dist_eps_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'dist_eps' from parameter server! Using default '"
                    << dist_eps_ << "'. [" << name_ <<"]");
  }
  orient_eps_ = orient_thres_ * 0.2;
  if(!nh_.getParam("orient_eps", orient_eps_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'orient_eps' from parameter server! Using default '"
                    << orient_eps_ << "'. [" << name_ <<"]");
  }
  pose_reached_ = false;
  ROS_DEBUG_STREAM("Controller initialised with the following parameters: [" << name_ <<"]");
  ROS_DEBUG_STREAM("base_frame_name = " << base_frame_name_ <<", goal_frame_name = "
                   << goal_frame_name_ << " [" << name_ <<"]");
  ROS_DEBUG_STREAM("v_max = " << v_max_ <<", k_1 = " << k_1_ << ", k_2 = " << k_2_ << ", beta = " << beta_
                   << ", lambda = " << lambda_ << ", dist_thres = " << dist_thres_
                   << ", orient_thres = " << orient_thres_ <<" [" << name_ <<"]");
  return true;
};

void DiffDrivePoseController::spinOnce()
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
    // determine controller output (v, w)
    getControlOutput();
    // set control output (v, w)
    setControlOutput();
    // Logging
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Current state: [" << name_ <<"]");
    ROS_DEBUG_STREAM_THROTTLE(1.0, "r = " << r_ << ", theta = " << theta_ << ", delta = " << delta_
                                   << " [" << name_ <<"]");
    ROS_DEBUG_STREAM_THROTTLE(1.0, "cur = " << cur_ << ", v = " << v_ << ", w = " << w_ << " [" << name_ <<"]");
  }
  else
  {
    ROS_DEBUG_STREAM_THROTTLE(3.0, "Controller is disabled. Idling ... [" << name_ <<"]");
  }
};

bool DiffDrivePoseController::getPoseDiff()
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
  r_ = std::sqrt(std::pow(tf_goal_pose_rel_.getOrigin().getX(), 2)
                 + std::pow(tf_goal_pose_rel_.getOrigin().getY(), 2));
  // determine orientation of r relative to the base frame
  delta_ = std::atan2(-tf_goal_pose_rel_.getOrigin().getY(), tf_goal_pose_rel_.getOrigin().getX());
  
  // determine orientation of r relative to the goal frame
  // helper: theta = tf's orientation + delta
  double heading =  mtk::wrapAngle(tf::getYaw(tf_goal_pose_rel_.getRotation()));
  theta_ = heading + delta_;

  return true;
};

void DiffDrivePoseController::getControlOutput()
{
  double atan2_k1_tehta = std::atan2(-theta_, k_1_);

  cur_ = (-1 / r_) * (k_2_ * (delta_ - atan2_k1_tehta)
         + (1 + (k_1_ / (1 + std::pow((k_1_ * theta_), 2)))) * sin(delta_));
  v_ = v_max_ / (1 + beta_ * std::pow(std::abs(cur_), lambda_));

  v_ = boundRange(v_, v_min_, v_max_);

  w_ = cur_ * v_; // unbounded for now
  w_ = boundRange(w_, w_min_, w_max_);

  // pose reached thresholds
  if (r_ <= dist_thres_)
  {
    v_ = 0;
    if (std::abs(delta_ - theta_) <= orient_thres_)
    {
      w_ = 0;
    }
  }

  // check, if pose has been reached
  if ((r_ <= dist_thres_) && (std::abs(delta_ - theta_) <= orient_thres_))
  {
    if (!pose_reached_)
    {
      pose_reached_ = true;
      ROS_INFO_STREAM("Pose reached. [" << name_ <<"]");
      std_msgs::Bool bool_msg;
      bool_msg.data = true;
      pose_reached_publisher_.publish(bool_msg);
    }
  }
  else if ((r_ > (dist_thres_ + dist_eps_)) || (std::abs(delta_ - theta_) > (orient_thres_ + orient_eps_)))
  {
    if (pose_reached_)
    {
      pose_reached_ = false;
      ROS_INFO_STREAM("Tracking new goal pose. [" << name_ <<"]");
    }
  }
};

double DiffDrivePoseController::boundRange(double v, double min, double max)
{
  // bounds for v
  if (v < 0.0)
  {
    if (v > -min)
    {
      v = -min;
    }
    else if (v < -max)
    {
      v = -max;
    }
  }
  else
  {
    if (v < min)
    {
      v = min;
    }
    else if (v > max)
    {
      v = max;
    }
  }

  return v;
}

void DiffDrivePoseController::setControlOutput()
{
  geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist());
  if (!pose_reached_)
  {
    cmd_vel->linear.x = v_;
    cmd_vel->angular.z = w_;
  }
  command_velocity_publisher_.publish(cmd_vel);
};

void DiffDrivePoseController::controlMaxVelCB(const std_msgs::Float32ConstPtr msg)
{
  v_max_ = msg->data;
  ROS_INFO_STREAM("Maximum linear control velocity has been set to " << v_max_ << ". [" << name_ << "]");
};

void DiffDrivePoseController::enableCB(const std_msgs::StringConstPtr msg)
{
  if (this->enable())
  {
    goal_frame_name_ = msg->data;
    ROS_INFO_STREAM("Controller has been enabled. [" << name_ << "] with goal frame [" << goal_frame_name_ << "]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"] with Goal frame [" << goal_frame_name_ << "]");
  }
};

void DiffDrivePoseController::disableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->disable())
  {
    ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
};

} // namespace yocs

#endif /* YOCS_CONTROLLER_HPP_ */
