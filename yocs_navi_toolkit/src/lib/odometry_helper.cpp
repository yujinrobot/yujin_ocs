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

  if(!odometry_)
  {
    odometry_ = std::unique_ptr<nav_msgs::Odometry>(new nav_msgs::Odometry());
  }

  *odometry_ = *msg;
}

bool OdometryHelper::initialized()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if(odometry_)
  {
    return true;
  }

  return false;
}

std::shared_ptr<nav_msgs::Odometry> OdometryHelper::odometry()
{
  if(!initialized())
  {
    return std::shared_ptr<nav_msgs::Odometry>();
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  std::shared_ptr<nav_msgs::Odometry> msg = std::make_shared<nav_msgs::Odometry>(*odometry_);
  return msg;
}

bool OdometryHelper::position(Eigen::Vector3f& position) {
  if(!initialized())
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  position << odometry_->pose.pose.position.x, odometry_->pose.pose.position.y, odometry_->pose.pose.position.z;

  return true;
}

bool OdometryHelper::yaw(float& angle) {
  if(!initialized())
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  angle = tf::getYaw(odometry_->pose.pose.orientation);

  return true;
}

bool OdometryHelper::velocities(std::pair<float, float>& mobile_twist_velocities)
{
  if(!initialized())
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  mobile_twist_velocities.first = odometry_->twist.twist.linear.x;
  mobile_twist_velocities.second = odometry_->twist.twist.angular.z;

  return true;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace yocs_navi_toolkit
