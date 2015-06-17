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

} // namespace yocs

#endif /* YOCS_CONTROLLER_HPP_ */
