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
** Includes
*****************************************************************************/
#include <string>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "yocs_diff_drive_pose_controller/diff_drive_pose_controller_ros.hpp"

namespace yocs
{

class DiffDrivePoseControllerNodelet : public nodelet::Nodelet
{
public:
  DiffDrivePoseControllerNodelet() : spin_rate_(20.0), shutdown_requested_(false) { };
  ~DiffDrivePoseControllerNodelet()
  {
    NODELET_DEBUG_STREAM("Waiting for update thread to finish. [" << name_ << "]");
    shutdown_requested_ = true;
    update_thread_.join();
  }

  void onInit()
  {
    ros::NodeHandle nh = this->getPrivateNodeHandle();
    // resolve node(let) name
    name_ = nh.getUnresolvedNamespace();
    int pos = name_.find_last_of('/');
    name_ = name_.substr(pos + 1);
    NODELET_INFO_STREAM("Initialising nodelet... [" << name_ << "]");
    double spin_rate_param = 20;
    if(nh.getParam("spin_rate", spin_rate_param))
    {
      ROS_DEBUG_STREAM("Controller will spin at " << spin_rate_param << " hz. [" << name_ <<"]");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't retrieve parameter 'spin_rate' from parameter server! Using default '"
                      << spin_rate_param << "'. [" << name_ <<"]");
    }
    spin_rate_ = ros::Rate(spin_rate_param);
    bool start_enabled = true;
    nh.getParam("start_enabled", start_enabled);
    controller_.reset(new DiffDrivePoseControllerROS(nh, name_));
    if (controller_->init())
    {
      if (start_enabled)
      {
        controller_->enable();
        ROS_INFO_STREAM("Controller will start enabled. [" << name_ <<"]");
      }
      else
      {
        controller_->disable();
        ROS_INFO_STREAM("Controller will start disabled. [" << name_ <<"]");
      }
      update_thread_.start(&DiffDrivePoseControllerNodelet::update, *this);
      NODELET_INFO_STREAM("Controller initialised. [" << name_ << "]");
    }
    else
    {
      NODELET_ERROR_STREAM("Couldn't initialise controller! Please restart. [" << name_ << "]");
    }
  }
private:
  void update()
  {
    while (!shutdown_requested_ && ros::ok())
    {
      controller_->spinOnce();
      spin_rate_.sleep();
    }
  }
  std::string name_;
  ros::Rate spin_rate_;
  boost::shared_ptr<DiffDrivePoseControllerROS> controller_;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
};

} // namespace yocs

PLUGINLIB_EXPORT_CLASS(yocs::DiffDrivePoseControllerNodelet, nodelet::Nodelet);
