/**
 * @file /yocs_navi_toolkit/src/lib/odometry_helper.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <tf/transform_datatypes.h>
#include "../../include/yocs_navi_toolkit/odometry_helper.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_navi_toolkit {

/*****************************************************************************
** Implementation
*****************************************************************************/

OdometryHelper::OdometryHelper(const std::string& odometry_topic_name)
{
  ros::NodeHandle nodehandle;
  odometry_subscriber_ = nodehandle.subscribe<nav_msgs::Odometry>(odometry_topic_name, 1, boost::bind(&OdometryHelper::odometryCallback, this, _1));
}

OdometryHelper::~OdometryHelper() {}

void OdometryHelper::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  odometry_ = *msg;
}

nav_msgs::Odometry OdometryHelper::odometry()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  nav_msgs::Odometry msg = odometry_;
  return msg;
}

void OdometryHelper::position(Eigen::Vector3f& position) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  position << odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z;
}

void OdometryHelper::yaw(float& angle) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  angle = tf::getYaw(odometry_.pose.pose.orientation);
}

void OdometryHelper::velocities(std::pair<float, float>& mobile_twist_velocities)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  mobile_twist_velocities.first = odometry_.twist.twist.linear.x;
  mobile_twist_velocities.second = odometry_.twist.twist.angular.z;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace yocs_navi_toolkit
