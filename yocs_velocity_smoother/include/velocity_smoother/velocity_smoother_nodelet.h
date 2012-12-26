/**
 * @file /include/velocity_smoother/velocity_smoother_nodelet.hpp
 *
 * @brief Velocity smoother interface
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/master/velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef YUJIN_OCS_VELOCITY_SMOOTHER_HPP_
#define YUJIN_OCS_VELOCITY_SMOOTHER_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace velocity_smoother {

/*****************************************************************************
** VelocitySmoother
*****************************************************************************/

class VelocitySmoother
{
public:
  VelocitySmoother() : shutdown_req(false), input_active(false), pr_next(0) { };
  ~VelocitySmoother() { };

  bool init(ros::NodeHandle& nh);
  void spin();
  void shutdown() { shutdown_req = true; };

private:
  double speed_lim_v, accel_lim_v, decel_lim_v;
  double speed_lim_w, accel_lim_w, decel_lim_w;
  double decel_factor;

  double frequency;

  geometry_msgs::Twist odometry_vel;
  geometry_msgs::Twist last_cmd_vel;
  geometry_msgs::Twist   target_vel;

  bool                 shutdown_req; /**< Shutdown requested by nodelet; kill worker thread */
  bool                 input_active;
  double                cb_avg_time;
  ros::Time            last_cb_time;
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

} // namespace velocity_smoother

#endif /* YUJIN_OCS_VELOCITY_SMOOTHER_HPP_ */
