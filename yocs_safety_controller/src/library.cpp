/**
* License: BSD
* https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
*/

#include <limits>
#include "yocs_safety_controller/safety_controller.hpp"

namespace yocs_safety_controller
{

SafetyController::SafetyController(ros::NodeHandle& nh_priv, const std::string& name) :
    Controller(),
    nh_priv_(nh_priv),
    name_(name),
    obstacle_detected_(false),
    reverse_(false),
    reversing_(false),
    reversing_start_(ros::Time::now()),
    reversing_distance_(0.02),
    reversing_velocity_(0.02),
    reversing_duration_(0.0)
{};


bool SafetyController::init()
{
  if (!nh_priv_.getParam("reverse", reverse_))
  {
    ROS_WARN_STREAM("Safety Controller : no parameter on server for reverse, using default '"
                    << reverse_ << "'");
  }
  if (reverse_)
  {
    if (!nh_priv_.getParam("reversing_distance", reversing_distance_))
    {
      ROS_WARN_STREAM("Safety Controller : no parameter on server for reversing_distance, using default '"
                      << reversing_distance_ << "'");
    }
    if (!nh_priv_.getParam("reversing_velocity", reversing_velocity_))
    {
      ROS_WARN_STREAM("Safety Controller : no parameter on server for reversing_velocity, using default '"
                      << reversing_distance_ << "'");
    }
  }

  // calculate reversing duration
  reversing_duration_ = ros::Duration(reversing_distance_/reversing_velocity_);

  enable_controller_subscriber_ = nh_priv_.subscribe("enable", 10, &SafetyController::enableCB, this);
  disable_controller_subscriber_ = nh_priv_.subscribe("disable", 10, &SafetyController::disableCB, this);
  ranger_subscriber_ = nh_priv_.subscribe("rangers", 10, &SafetyController::rangerCB, this);
  velocity_command_publisher_ = nh_priv_.advertise< geometry_msgs::Twist >("cmd_vel", 10);
  return true;
};

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
    if (msg->range == -std::numeric_limits<double>::infinity()) // -Inf means obstacle detected
    {
      obstacle_detected_ = true;
    }
  }
  return;
}

void SafetyController::spinOnce()
{
  if (reversing_)
  {
    if ((ros::Time::now() - reversing_start_) > reversing_duration_)
    {
      reversing_ = false;
    }
    else
    {
      // reverse
      cmd_vel_msg_.reset(new geometry_msgs::Twist());
      cmd_vel_msg_->linear.x = -reversing_velocity_;
      cmd_vel_msg_->linear.y = 0.0;
      cmd_vel_msg_->linear.z = 0.0;
      cmd_vel_msg_->angular.x = 0.0;
      cmd_vel_msg_->angular.y = 0.0;
      cmd_vel_msg_->angular.z = 0.0;
      velocity_command_publisher_.publish(cmd_vel_msg_);

    }
  }
  else
  {
    if (obstacle_detected_)
    {
      // stop
      cmd_vel_msg_.reset(new geometry_msgs::Twist());
      cmd_vel_msg_->linear.x = 0.0;
      cmd_vel_msg_->linear.y = 0.0;
      cmd_vel_msg_->linear.z = 0.0;
      cmd_vel_msg_->angular.x = 0.0;
      cmd_vel_msg_->angular.y = 0.0;
      cmd_vel_msg_->angular.z = 0.0;
      velocity_command_publisher_.publish(cmd_vel_msg_);
      obstacle_detected_ = false;

      // change to reversing, if activated
      if (reverse_)
      {
        reversing_ = true;
        reversing_start_ = ros::Time::now();
      }
    }
  }
  return;
}

} // namespace yocs_safety_controller
