/*
 * cmd_vel_mux.h
 *
 *  Created on: Nov 30, 2012
 *      Author: jorge
 */

#ifndef CMD_VEL_MUX_H_
#define CMD_VEL_MUX_H_


#include <ros/ros.h>

#include "cmd_vel_subscribers.h"


class CmdVelMux
{
public:

  CmdVelMux();
  ~CmdVelMux();

  bool init(ros::NodeHandle& nh);

private:
  CmdVelSubscribers  cmd_vel_sub;  /**< Pool of cmd_vel topics subscribers */

  ros::Publisher mux_cmd_vel_pub;  /**< Multiplexed command velocity topic */
  ros::Publisher act_cmd_vel_pub;  /**< Currently active cmd_vel publisher */

  void timerCallback(const ros::TimerEvent& event, unsigned int idx);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx);

  // Functor assigned to each incoming velocity topic to bind it to cmd_vel callback
  class CmdVelFunctor
  {
  private:
    unsigned int idx;
    CmdVelMux*  node;

  public:
    CmdVelFunctor(unsigned int idx, CmdVelMux* node) :
        idx(idx), node(node) { }

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
    CmdVelMux*  node;

  public:
    TimerFunctor(unsigned int idx, CmdVelMux* node) :
        idx(idx), node(node) { }

    void operator()(const ros::TimerEvent& event)
    {
      node->timerCallback(event, idx);
    }
  };
};

#endif /* CMD_VEL_MUX_H_ */
