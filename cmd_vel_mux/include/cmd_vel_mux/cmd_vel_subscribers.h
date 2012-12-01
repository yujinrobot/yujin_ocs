/*
 * cmd_vel_subscribers.hpp
 *
 *  Created on: Oct 31, 2012
 *      Author: jorge
 */

#ifndef CMD_VEL_SUBSCRIBERS_H_
#define CMD_VEL_SUBSCRIBERS_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <yaml-cpp/yaml.h>

#define VACANT  std::numeric_limits<unsigned int>::max()


/**
 * Pool of cmd_vel topics subscribers
 */
class CmdVelSubscribers
{
public:

  /**
   * Inner class describing an individual subscriber to a cmd_vel topic
   */
  class CmdVelSubs
  {
  public:
    unsigned int           idx;          /**< Index; assigned according to the order on YAML file */
    std::string            name;         /**< Descriptive name */
    ros::Subscriber        subs;         /**< The subscriber itself */
    std::string            topic;        /**< The name of the topic */
    ros::Timer             timer;        /**< No incoming messages timeout */
    double                 timeout;      /**< Timer's timeout, in seconds  */
    unsigned int           priority;     /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
    std::string            short_desc;   /**< Short description (optional) */
    bool                   active;       /**< Whether this source is active */

    CmdVelSubs(unsigned int idx) : idx(idx), active(false) {};

    void operator << (const YAML::Node& node);
  };

  unsigned int allowed;

  CmdVelSubscribers() : allowed(VACANT) { }
  ~CmdVelSubscribers() { }

  std::vector<CmdVelSubs>::size_type size() { return list.size(); };

  CmdVelSubs& operator [] (unsigned int idx) { return list[idx]; };

  bool loadSubscribersCfg(std::string path);

private:
  std::vector<CmdVelSubs> list;
};

#endif /* CMD_VEL_SUBSCRIBERS_H_ */
