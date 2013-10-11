/**
 * @file /include/yocs_cmd_vel_mux/cmd_vel_mux_nodelet.hpp
 *
 * @brief Structure for the yocs_cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef YUJIN_OCS_CMD_VEL_MUX_HPP_
#define YUJIN_OCS_CMD_VEL_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include "yocs_cmd_vel_mux/reloadConfig.h"
#include "yocs_cmd_vel_mux/cmd_vel_subscribers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_cmd_vel_mux {

/*****************************************************************************
 ** CmdVelMux
 *****************************************************************************/

class CmdVelMuxNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

  CmdVelMuxNodelet()
  {
    dynamic_reconfigure_server = NULL;
  }

  ~CmdVelMuxNodelet()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

private:
  CmdVelSubscribers cmd_vel_sub; /**< Pool of cmd_vel topics subscribers */
  ros::Publisher mux_cmd_vel_pub; /**< Multiplexed command velocity topic */
  ros::Publisher active_subscriber; /**< Currently allowed cmd_vel subscriber */

  void timerCallback(const ros::TimerEvent& event, unsigned int idx);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx);

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<yocs_cmd_vel_mux::reloadConfig> * dynamic_reconfigure_server;
  dynamic_reconfigure::Server<yocs_cmd_vel_mux::reloadConfig>::CallbackType dynamic_reconfigure_cb;
  void reloadConfiguration(yocs_cmd_vel_mux::reloadConfig &config, uint32_t unused_level);

  /*********************
   ** Private Classes
   **********************/
  // Functor assigned to each incoming velocity topic to bind it to cmd_vel callback
  class CmdVelFunctor
  {
  private:
    unsigned int idx;
    CmdVelMuxNodelet* node;

  public:
    CmdVelFunctor(unsigned int idx, CmdVelMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const geometry_msgs::Twist::ConstPtr& msg)
    {
      node->cmdVelCallback(msg, idx);
    }
  };

  // Functor assigned to each velocity messages source to bind it to timer callback
  class TimerFunctor
  {
  private:
    unsigned int idx;
    CmdVelMuxNodelet* node;

  public:
    TimerFunctor(unsigned int idx, CmdVelMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const ros::TimerEvent& event)
    {
      node->timerCallback(event, idx);
    }
  };
};

} // namespace yocs_cmd_vel_mux

#endif /* YUJIN_OCS_CMD_VEL_MUX_HPP_ */
