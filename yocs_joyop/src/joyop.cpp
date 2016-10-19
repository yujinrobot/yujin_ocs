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
#include <dynamic_reconfigure/server.h>
#include <yocs_msgs/JoystickConfig.h>


namespace yocs_joyop
{

class JoyOp
{
public:
  JoyOp();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
  void reconfigCB(yocs_msgs::JoystickConfig &config, uint32_t level); 

  ros::NodeHandle ph_, nh_;

  // states
  int linear_axis_, angular_axis_, boost_axis_;
  // button ids
  int deadman_button_, enable_button_, disable_button_, brake_button_;
  double l_scale_, a_scale_, boost_scale_, spin_freq_;
  ros::Publisher enable_pub_, disable_pub_, vel_pub_, brake_vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  // callback notifications (kept separate from current state)
  bool enable_pressed_, disable_pressed_, deadman_pressed_, brake_pressed_, boost_active_, zero_twist_published_, enabled_;
  int wait_for_connection_; /**< Time to wait for enable/disable topics in seconds (-1 to not wait). **/
  ros::Timer timer_;
  
};

JoyOp::JoyOp():
  ph_("~"),
  linear_axis_(1),
  angular_axis_(0),
  boost_axis_(5),
  deadman_button_(4),
  enable_button_(0),
  disable_button_(1),
  brake_button_(5),
  l_scale_(0.3),
  a_scale_(0.9),
  boost_scale_(2.0),
  spin_freq_(10),
  wait_for_connection_(-1),
  enabled_(false),
  enable_pressed_(false),
  disable_pressed_(false),
  deadman_pressed_(false),
  brake_pressed_(false),
  boost_active_(false),
  zero_twist_published_(false)
{
  ph_.param("linear_axis", linear_axis_, linear_axis_);
  ph_.param("angular_axis", angular_axis_, angular_axis_);
  ph_.param("boost_axis", boost_axis_, boost_axis_);
  ph_.param("deadman_button", deadman_button_, deadman_button_);
  ph_.param("enable_button", enable_button_, enable_button_);
  ph_.param("disable_button", disable_button_, disable_button_);
  ph_.param("brake_button", brake_button_, brake_button_);
  ph_.param("boost_scale", boost_scale_, boost_scale_);
  ph_.param("spin_frequency", spin_freq_, spin_freq_);
  ph_.param("wait_for_connection", wait_for_connection_, wait_for_connection_);
  ph_.param("enable_on_start", enabled_, enabled_);

  enable_pub_ = ph_.advertise<std_msgs::String>("enable", 1, true);
  disable_pub_ = ph_.advertise<std_msgs::String>("disable", 1, true);
  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  brake_vel_pub_ = ph_.advertise<geometry_msgs::Twist>("brake_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyOp::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(1/spin_freq_), boost::bind(&JoyOp::publish, this));

  dynamic_reconfigure::Server<yocs_msgs::JoystickConfig> *             dynamic_reconfigure_server;
  dynamic_reconfigure::Server<yocs_msgs::JoystickConfig>::CallbackType dynamic_reconfigure_callback;
  
  dynamic_reconfigure_callback = boost::bind(&JoyOp::reconfigCB, this, _1, _2);
  dynamic_reconfigure_server = new dynamic_reconfigure::Server<yocs_msgs::JoystickConfig>(ph_);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);
  
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
        (disable_pub_.getNumSubscribers() > 0)) // brake is optional
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

    if(enabled_)
    {
      ROS_INFO_STREAM("JoyOp: Enabling motors.");
    }
  }
}

void JoyOp::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  if (!boost_active_ && (joy->axes[boost_axis_] == -1.0)) // fully pressed
  {
    boost_active_ = true;
    ROS_INFO_STREAM("JoyOp: Boost activated.");
  }
  else if (boost_active_ && (joy->axes[boost_axis_] == -1.0)) // fully pressed again
  {
    boost_active_ = false;
    ROS_INFO_STREAM("JoyOp: Boost deactivated.");
  }

  if (boost_active_)
  {
    vel.angular.z = a_scale_*joy->axes[angular_axis_]*boost_scale_;
    vel.linear.x = l_scale_*joy->axes[linear_axis_]*boost_scale_;
  }
  else
  {
    vel.angular.z = a_scale_*joy->axes[angular_axis_];
    vel.linear.x = l_scale_*joy->axes[linear_axis_];
  }
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_button_];
  enable_pressed_ = joy->buttons[enable_button_];
  disable_pressed_ = joy->buttons[disable_button_];
  if (!brake_pressed_ && joy->buttons[brake_button_])
  {
    ROS_INFO_STREAM("JoyOp: Brake activated.");
  }
  else if (brake_pressed_ && !joy->buttons[boost_axis_])
  {
    ROS_INFO_STREAM("JoyOp: Brake released.");
  }
  brake_pressed_ = joy->buttons[boost_axis_];
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

  if (brake_pressed_)
  {
    vel_pub_.publish(geometry_msgs::Twist());
    brake_vel_pub_.publish(geometry_msgs::Twist());
    zero_twist_published_=true;
  }
  else if (deadman_pressed_)
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
    brake_vel_pub_.publish(geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

void JoyOp::reconfigCB(yocs_msgs::JoystickConfig &config, uint32_t level) 
{
  ROS_INFO("JoyOp: Reconfigure linear : %f, angular : %f", config.linear_scale, config.angular_scale);
  l_scale_ = config.linear_scale;
  a_scale_ = config.angular_scale;
}

} // namespace yocs_joyop

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yocs_joyop");
  yocs_joyop::JoyOp joyop;

  ros::spin();
}
