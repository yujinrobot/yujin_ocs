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
  VelSmoother() : period_record(1, 0.1), pr_next(1) { };
  ~VelSmoother() { };

  bool init(ros::NodeHandle& nh);

private:
  double accel_lim_x, decel_lim_x;
  double accel_lim_y, decel_lim_y;
  double accel_lim_t, decel_lim_t;
  double decel_factor;

  geometry_msgs::Twist odometry_vel;
  geometry_msgs::Twist last_cmd_vel;
  ros::Time           last_cmd_time;
  std::vector<double> period_record; /**< Historic of latest periods between velocity commands */
  unsigned int              pr_next; /**< Next position to fill in the periods record buffer */

  ros::Subscriber cur_vel_sub;  /**< Current velocity from odometry */
  ros::Subscriber raw_vel_sub;  /**< Incoming raw velocity commands */
  ros::Publisher  lim_vel_pub;  /**< Outgoing smoothed velocity commands */

  void velocityCB(const geometry_msgs::Twist::ConstPtr& msg);
  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };

  double median(std::vector<double>& values) {
    // Return the median element of an doubles vector
    nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  };
};

#endif /* VELOCITY_SMOOTHER_H_ */
