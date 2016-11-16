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

#ifndef YOCS_DIFF_DRIVE_POSE_CONTROLLER_ROS_HPP_
#define YOCS_DIFF_DRIVE_POSE_CONTROLLER_ROS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <ros/ros.h>
#include <ecl/threads/mutex.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <yocs_msgs/PoseControllerConfig.h>

#include "yocs_diff_drive_pose_controller/diff_drive_pose_controller.hpp"

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
class DiffDrivePoseControllerROS : public DiffDrivePoseController
{
public:
  DiffDrivePoseControllerROS(ros::NodeHandle& nh, std::string& name) :
      DiffDrivePoseController(name, 0.5, M_PI / 4 * 0.5), nh_(nh)
  {
  }
  ;
  virtual ~DiffDrivePoseControllerROS()
  {
  }
  ;

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
   * @brief Publishes goal is reached message
   */
  virtual void onGoalReached();

  /**
   * @brief Calculates the controller output based on the current pose difference
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

  void reconfigCB(yocs_msgs::PoseControllerConfig &config, uint32_t level);

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

  /// tf used to get goal pose relative to the base pose
  tf::TransformListener tf_listener_;
  /// transform for the goal pose relative to the base pose
  tf::StampedTransform tf_goal_pose_rel_;
  /// frame name of the base
  std::string base_frame_name_;
  /// frame name of the goal (pose)
  std::string goal_frame_name_;

  ///dynamic reconfigure server
  boost::shared_ptr<dynamic_reconfigure::Server<yocs_msgs::PoseControllerConfig> > reconfig_server_;

};

} // namespace yocs

#endif /* YOCS_DIFF_DRIVE_POSE_CONTROLLER_ROS_HPP_ */
