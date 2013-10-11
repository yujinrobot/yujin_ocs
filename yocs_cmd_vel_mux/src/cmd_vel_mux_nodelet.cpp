/**
 * @file /src/cmd_vel_mux_nodelet.cpp
 *
 * @brief  Implementation for the command velocity multiplexer
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>

#include "yocs_cmd_vel_mux/cmd_vel_mux_nodelet.hpp"
#include "yocs_cmd_vel_mux/exceptions.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_cmd_vel_mux {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void CmdVelMuxNodelet::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx)
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
      active_subscriber.publish(acv_msg);
    }

    mux_cmd_vel_pub.publish(msg);
  }
}

void CmdVelMuxNodelet::timerCallback(const ros::TimerEvent& event, unsigned int idx)
{
  if (cmd_vel_sub.allowed == idx)
  {
    // No cmd_vel messages timeout happened to currently active source, so...
    cmd_vel_sub.allowed = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    std_msgs::StringPtr acv_msg(new std_msgs::String);
    acv_msg->data = "idle";
    active_subscriber.publish(acv_msg);
  }

  cmd_vel_sub[idx].active = false;
}

void CmdVelMuxNodelet::onInit()
{
  ros::NodeHandle &nh = this->getPrivateNodeHandle();

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure_cb = boost::bind(&CmdVelMuxNodelet::reloadConfiguration, this, _1, _2);
  dynamic_reconfigure_server = new dynamic_reconfigure::Server<yocs_cmd_vel_mux::reloadConfig>(nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_cb);

  active_subscriber = nh.advertise <std_msgs::String> ("active", 1, true); // latched topic

  // Notify the world that by now nobody is publishing on cmd_vel yet
  std_msgs::StringPtr active_msg(new std_msgs::String);
  active_msg->data = "idle";
  active_subscriber.publish(active_msg);

  // could use a call to reloadConfiguration here, but it seems to automatically call it once with defaults anyway.
  NODELET_DEBUG("CmdVelMux : successfully initialised");
}

void CmdVelMuxNodelet::reloadConfiguration(yocs_cmd_vel_mux::reloadConfig &config, uint32_t unused_level)
{
  std::string yaml_cfg_file;
  ros::NodeHandle &nh = this->getPrivateNodeHandle();
  if( config.yaml_cfg_file == "" )
  {
    // typically fired on startup, so look for a parameter to set a default
    nh.getParam("yaml_cfg_file", yaml_cfg_file);
  }
  else
  {
    yaml_cfg_file = config.yaml_cfg_file;
  }

  /*********************
  ** Yaml File Parsing
  **********************/
  std::ifstream ifs(yaml_cfg_file.c_str(), std::ifstream::in);
  if (ifs.good() == false)
  {
    NODELET_ERROR_STREAM("CmdVelMux : configuration file not found [" << yaml_cfg_file << "]");
    return;
  }
  // probably need to bring the try catches back here
  YAML::Parser parser(ifs);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  /*********************
  ** Output Publisher
  **********************/
  std::string output_name("output");
  const YAML::Node *node = doc.FindValue("publisher");
  if ( node != NULL ) {
    *node >> output_name;
  }
  mux_cmd_vel_pub = nh.advertise <geometry_msgs::Twist> (output_name, 10);

  /*********************
  ** Input Subscribers
  **********************/
  try {
    cmd_vel_sub.configure(doc["subscribers"]);
  }
  catch(EmptyCfgException& e) {
    NODELET_WARN("CmdVelMux : yaml configured zero subscribers, check yaml content.");
  }
  catch(YamlException& e) {
    NODELET_ERROR_STREAM("CmdVelMux : yaml parsing problem [" << std::string(e.what()) + "]");
  }

  // Publishers and subscribers
  for (unsigned int i = 0; i < cmd_vel_sub.size(); i++)
  {
    cmd_vel_sub[i].subs =
        nh.subscribe<geometry_msgs::Twist>(cmd_vel_sub[i].topic, 10, CmdVelFunctor(i, this));

    // Create (stopped by now) a one-shot timer for every subscriber
    cmd_vel_sub[i].timer =
        nh.createTimer(ros::Duration(cmd_vel_sub[i].timeout), TimerFunctor(i, this), true, false);

    NODELET_DEBUG("CmdVelMux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
              cmd_vel_sub[i].name.c_str(), cmd_vel_sub[i].topic.c_str(),
              cmd_vel_sub[i].priority, cmd_vel_sub[i].timeout);
  }

  NODELET_INFO_STREAM("CmdVelMux : (re)configured [" << yaml_cfg_file << "]");
  ifs.close();
}

} // namespace yocs_cmd_vel_mux

PLUGINLIB_EXPORT_CLASS(yocs_cmd_vel_mux::CmdVelMuxNodelet, nodelet::Nodelet);
