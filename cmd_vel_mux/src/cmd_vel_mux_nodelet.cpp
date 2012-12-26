/**
 * @file /src/cmd_vel_mux_nodelet.cpp
 *
 * @brief  Implementation for the command velocity multiplexer
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/master/cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>
#include "cmd_vel_mux/cmd_vel_mux_nodelet.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

CmdVelMux::CmdVelMux()
{
}

CmdVelMux::~CmdVelMux()
{
}

void CmdVelMux::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx)
{
  // Reset timer for this source
  cmd_vel_sub[idx].timer.stop();
  cmd_vel_sub[idx].timer.start();

  cmd_vel_sub[idx].active = true;   // obviously his source is sending commands, so active

  // Give permit to publish to this source if it's the only active or is
  // already allowed or has higher priority that the currently allowed
  if ((cmd_vel_sub.allowed == VACANT) ||
      (cmd_vel_sub.allowed == idx)    ||
      (cmd_vel_sub[idx].priority > cmd_vel_sub[cmd_vel_sub.allowed].priority))
  {
    if (cmd_vel_sub.allowed != idx)
    {
      cmd_vel_sub.allowed = idx;

      // Notify the world that a new cmd_vel source took the control
      std_msgs::StringPtr acv_msg(new std_msgs::String);
      acv_msg->data = cmd_vel_sub[idx].name;
      allowed_sub_pub.publish(acv_msg);
    }

    mux_cmd_vel_pub.publish(msg);
  }
}

void CmdVelMux::timerCallback(const ros::TimerEvent& event, unsigned int idx)
{
  if (cmd_vel_sub.allowed == idx)
  {
    // No cmd_vel messages timeout happened to currently active source, so...
    cmd_vel_sub.allowed = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    std_msgs::StringPtr acv_msg(new std_msgs::String);
    acv_msg->data = "Vacant";
    allowed_sub_pub.publish(acv_msg);
  }

  cmd_vel_sub[idx].active = false;
}

/**
 * Initialise from a nodelet's private nodehandle.
 * @param nh : private nodehandle
 * @return bool : success or failure
 */
bool CmdVelMux::init(ros::NodeHandle& nh)
{
  /*********************
  ** Dynamic Reconfigure
  **********************/
  std::cout << "CmdVelMux: setting up the dynamic reconfiguration server" << std::endl;
  dynamic_reconfigure_cb = boost::bind(&CmdVelMux::reloadConfiguration, this, _1, _2);
  dynamic_reconfigure_server.setCallback(dynamic_reconfigure_cb);
  std::cout << "CmdVelMux: done" << std::endl;

  // Load subscribers configuration file
  std::string subscribers_cfg_file;
  nh.getParam("subscribers_cfg_file", subscribers_cfg_file);

  if (cmd_vel_sub.loadSubscribersCfg(subscribers_cfg_file) == false)
  {
    return false;
  }

  if (cmd_vel_sub.size() == 0)
  {
    ROS_WARN("No input cmd_vel configured; check subscribers configuration yaml file content");
  }

  // Publishers and subscribers
  for (unsigned int i = 0; i < cmd_vel_sub.size(); i++)
  {
    cmd_vel_sub[i].subs =
        nh.subscribe<geometry_msgs::Twist>(cmd_vel_sub[i].topic, 10, CmdVelFunctor(i, this));

    // Create (stopped by now) a one-shot timer for every subscriber
    cmd_vel_sub[i].timer =
        nh.createTimer(ros::Duration(cmd_vel_sub[i].timeout), TimerFunctor(i, this), true, false);

    ROS_DEBUG("Subscribed to %s on topic %s. pr: %d, to: %.2f",
              cmd_vel_sub[i].name.c_str(), cmd_vel_sub[i].topic.c_str(),
              cmd_vel_sub[i].priority, cmd_vel_sub[i].timeout);
  }

  mux_cmd_vel_pub = nh.advertise <geometry_msgs::Twist> ("mux_cmd_vel", 10);
  allowed_sub_pub = nh.advertise <std_msgs::String> ("allowed_cmd_vel", 1, true); // latched topic

  // Notify the world that by now nobody is publishing on cmd_vel; its vacant
  std_msgs::StringPtr acv_msg(new std_msgs::String);
  acv_msg->data = "Vacant";
  allowed_sub_pub.publish(acv_msg);

  ROS_INFO("Command velocity multiplexer successfully initialised");

  return true;
}


/*********************
** Nodelet
**********************/

class CmdVelMuxNodelet : public nodelet::Nodelet
{
public:
  CmdVelMuxNodelet()  { }
  ~CmdVelMuxNodelet() { }

  virtual void onInit()
  {
    NODELET_DEBUG("Initialising nodelet...");

    cvm_.reset(new CmdVelMux);
    if (cvm_->init(this->getPrivateNodeHandle()))
    {
      NODELET_DEBUG("Command velocity multiplexer nodelet initialised");
    }
    else
    {
      NODELET_ERROR("Command velocity multiplexer nodelet initialisation failed");
    }
  }

private:
  boost::shared_ptr<CmdVelMux> cvm_;
};

} // namespace cmd_vel_mux

PLUGINLIB_EXPORT_CLASS(cmd_vel_mux::CmdVelMuxNodelet, nodelet::Nodelet);
