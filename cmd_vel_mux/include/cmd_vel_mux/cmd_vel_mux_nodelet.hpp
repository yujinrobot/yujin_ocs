/**
 * @file /include/cmd_vel_mux/cmd_vel_mux_nodelet.hpp
 *
 * @brief Structure for the cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/master/cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef CMD_VEL_MUX_HPP_
#define CMD_VEL_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include "cmd_vel_subscribers.hpp"
#include <dynamic_reconfigure/server.h>
#include <cmd_vel_mux/reloadConfig.h>

/*****************************************************************************
 ** CmdVelMux
 *****************************************************************************/

class CmdVelMux
{
public:

  CmdVelMux();
  ~CmdVelMux();

  bool init(ros::NodeHandle& nh);

private:
  CmdVelSubscribers cmd_vel_sub; /**< Pool of cmd_vel topics subscribers */

  ros::Publisher mux_cmd_vel_pub; /**< Multiplexed command velocity topic */
  ros::Publisher allowed_sub_pub; /**< Currently allowed cmd_vel subscriber */

  void timerCallback(const ros::TimerEvent& event, unsigned int idx);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx);

  /*********************
   ** Private Classes
   **********************/
  // Functor assigned to each incoming velocity topic to bind it to cmd_vel callback
  class CmdVelFunctor
  {
  private:
    unsigned int idx;
    CmdVelMux* node;

  public:
    CmdVelFunctor(unsigned int idx, CmdVelMux* node) :
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
    CmdVelMux* node;

  public:
    TimerFunctor(unsigned int idx, CmdVelMux* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const ros::TimerEvent& event)
    {
      node->timerCallback(event, idx);
    }
  };
};

#endif /* CMD_VEL_MUX_HPP_ */
