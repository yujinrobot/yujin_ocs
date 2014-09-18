/**
* License: BSD
* https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
*/

#include <limits>
#include "yocs_safety_controller/safety_controller.hpp"

namespace yocs_safety_controller
{

void SafetyController::enableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->enable())
  {
    ROS_INFO_STREAM("Controller has been enabled. [" << name_ << "]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"]");
  }
}

void SafetyController::disableCB(const std_msgs::EmptyConstPtr msg)
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

void SafetyController::rangerCB(const sensor_msgs::RangeConstPtr msg)
{
  if (this->getState())
  {
    // readings from a fixed-distance ranger
    if (msg->range == -std::numeric_limits<double>::infinity())
    {
      cmd_vel_msg_.reset(new geometry_msgs::Twist());
      cmd_vel_msg_->linear.x = 0.0;
      cmd_vel_msg_->linear.y = 0.0;
      cmd_vel_msg_->linear.z = 0.0;
      cmd_vel_msg_->angular.x = 0.0;
      cmd_vel_msg_->angular.y = 0.0;
      cmd_vel_msg_->angular.z = 0.0;
      velocity_command_publisher_.publish(cmd_vel_msg_);
    }
  }
  return;
}

void SafetyController::spinOnce()
{
  return;
}

} // namespace yocs_safety_controller
