/*
 *  basic_move_controller.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <ros/ros.h>
#include <tf/tf.h>

#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "yocs_navigator/default_params.h"

#ifndef _YOCS_BASIC_MOVE_CONTROLLER_HPP_
#define _YOCS_BASIC_MOVE_CONTROLLER_HPP_

namespace yocs_navigator {
class BasicMoveController {
  public:
    BasicMoveController(ros::NodeHandle& n);
    BasicMoveController(ros::NodeHandle& n, const std::string& twist_topic, const std::string& odometry_topic);
    virtual ~BasicMoveController();

    void init();

    // linear, angualr, time
    void moveAt(double v, double w, double t);

    void slowForward(); 
    void slowBackward();
    void turnClockwise();
    void turnCounterClockwise();
    void stop();

    void forward(double distance);
    void backward(double distance);
    void turn(double angle);

    void turn2(double angle);
    void spinClockwise();
    void spinCounterClockwise();
  protected:
    void processOdometry(const nav_msgs::Odometry::ConstPtr& msg);

  private:
    ros::NodeHandle nh_;
    ros::Publisher  pub_cmd_vel_;
    ros::Subscriber sub_odom_;

    nav_msgs::Odometry odometry_;
    std::string cmd_vel_topic_;
    std::string odometry_topic_;
};
}
#endif
