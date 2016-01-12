/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <ecl/exceptions.hpp>
#include <ecl/time.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"


namespace yocs_joyop
{

class JoyOp
{
public:
  JoyOp();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  // states
  int linear_, angular_, enabled_;
  // button ids
  int deadman_button_, enable_button_, disable_button_;
  double l_scale_, a_scale_, spin_freq_;
  ros::Publisher enable_pub_, disable_pub_, vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  // callback notifications (kept separate from current state)
  bool enable_pressed_, disable_pressed_, deadman_pressed_, zero_twist_published_;
  int wait_for_connection_; /**< Time to wait for enable/disable topics in seconds (-1 to not wait). **/
  ros::Timer timer_;

};

JoyOp::JoyOp():
  ph_("~"),
  linear_(1),
  angular_(0),
  deadman_button_(4),
  enable_button_(0),
  disable_button_(1),
  l_scale_(0.3),
  a_scale_(0.9),
  spin_freq_(10),
  wait_for_connection_(-1),
  enabled_(false),
  enable_pressed_(false),
  disable_pressed_(false),
  deadman_pressed_(false),
  zero_twist_published_(false)
{
  ph_.param("linear_axis", linear_, linear_);
  ph_.param("angular_axis", angular_, angular_);
  ph_.param("deadman_button", deadman_button_, deadman_button_);
  ph_.param("enable_button", enable_button_, enable_button_);
  ph_.param("disable_button", disable_button_, disable_button_);
  ph_.param("angular_scale", a_scale_, a_scale_);
  ph_.param("linear_scale", l_scale_, l_scale_);
  ph_.param("spin_frequency", spin_freq_, spin_freq_);
  ph_.param("wait_for_connection", wait_for_connection_, wait_for_connection_);

  enable_pub_ = ph_.advertise<std_msgs::String>("enable", 1, true);
  disable_pub_ = ph_.advertise<std_msgs::String>("disable", 1, true);
  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyOp::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(1/spin_freq_), boost::bind(&JoyOp::publish, this));

  /******************************************
   ** Wait for connection
   *******************************************/
  // this is only waiting for the enable/disable connection and if connected
  // it will send the 'all' message to enable the motors automatically
  if (wait_for_connection_ <= 0)
  {
    return;
  }
  ecl::MilliSleep millisleep;
  int count = 0;
  bool connected = false;
  while (!connected)
  {
    if ((enable_pub_.getNumSubscribers() > 0) &&
        (disable_pub_.getNumSubscribers() > 0))
    {
      connected = true;
      break;
    }
    if (count == 2*wait_for_connection_) // loop every 500ms
    {
      connected = false;
      break;
    }
    else
    {
      ROS_WARN_STREAM("JoyOp: Could not connect, trying again after 500ms...");
      try
      {
        millisleep(500);
      }
      catch (ecl::StandardException& e)
      {
        ROS_ERROR_STREAM("JoyOp: Waiting has been interrupted.");
        ROS_DEBUG_STREAM(e.what());
        return;
      }
      ++count;
    }
  }
  if (!connected)
  {
    ROS_ERROR("JoyOp: Could not connect.");
    ROS_ERROR("JoyOp: Check remappings for enable/disable topics.");
  }
  else
  {
    std_msgs::String msg;
    msg.data = "all";
    enable_pub_.publish(msg);
    ROS_INFO("JoyOp: connected.");
  }
}

void JoyOp::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_button_];
  enable_pressed_ = joy->buttons[enable_button_];
  disable_pressed_ = joy->buttons[disable_button_];
}

void JoyOp::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (enable_pressed_ && (!disable_pressed_))
  {
    if(!enabled_)
    {
      ROS_INFO_STREAM("JoyOp: Enabling motors.");
      std_msgs::String msg;
      msg.data = "all";
      enable_pub_.publish(msg);
      enabled_ = true;
    }
    else
    {
      ROS_WARN_STREAM("JoyOp: Motors have already been enabled.");
    }
  }
  else if (disable_pressed_)
  {
    if(enabled_)
    {
      ROS_INFO_STREAM("JoyOp: die, die, die (disabling motors).");
      std_msgs::String msg;
      msg.data = "all";
      enable_pub_.publish(msg);
      enabled_ = true;
    }
    else
    {
      ROS_WARN_STREAM("JoyOp: Motors have already been disabled.");
    }
    std_msgs::String msg;
    msg.data = "all";
    disable_pub_.publish(msg);
    enabled_ = false;
  }

  if (deadman_pressed_)
  {
    if (enabled_)
    {
      vel_pub_.publish(last_published_);
      zero_twist_published_=false;
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "JoyOp: Motor system disabled. Won't send velocity commands.");
    }
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

} // namespace yocs_joyop

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yocs_joyop");
  yocs_joyop::JoyOp joyop;

  ros::spin();
}
