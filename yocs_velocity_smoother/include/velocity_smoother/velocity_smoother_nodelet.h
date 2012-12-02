/*
 * velocity_smoother_nodelet.h
 *
 *  Created on: Dec 2, 2012
 *      Author: jorge
 */

#ifndef VELOCITY_SMOOTHER_H_
#define VELOCITY_SMOOTHER_H_


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


class VelSmoother
{
public:
  VelSmoother();
  ~VelSmoother();

  bool init(ros::NodeHandle& nh);

private:
  double accel_lim_x, decel_lim_x;
  double accel_lim_y, decel_lim_y;
  double accel_lim_t, decel_lim_t;
  double decel_factor;

  geometry_msgs::Twist odometry_vel;
  geometry_msgs::Twist last_cmd_vel;
  ros::Time           last_cmd_time;

  ros::Subscriber cur_vel_sub;  /**< Current velocity from odometry */
  ros::Subscriber raw_vel_sub;  /**< Incoming raw velocity commands */
  ros::Publisher  lim_vel_pub;  /**< Outgoing smoothed velocity commands */

  void velocityCB(const geometry_msgs::Twist::ConstPtr& msg);
  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; }
};

#endif /* VELOCITY_SMOOTHER_H_ */
