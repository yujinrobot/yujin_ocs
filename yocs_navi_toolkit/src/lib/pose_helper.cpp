/**
 * @file /yocs_navi_toolkit/src/lib/pose_helper.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/linear_algebra.hpp>
#include <tf/transform_datatypes.h>
#include "../../include/yocs_navi_toolkit/pose_helper.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_navi_toolkit {

/*****************************************************************************
** Implementation
*****************************************************************************/

PoseHelper::PoseHelper(const std::string& pose_topic_name)
{
  ros::NodeHandle nodehandle;
  pose_subscriber_ = nodehandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_name, 1, boost::bind(&PoseHelper::poseCallback, this, _1));
}

PoseHelper::~PoseHelper() {}

void PoseHelper::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  pose_ = *msg;
}

geometry_msgs::PoseWithCovarianceStamped PoseHelper::pose()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  geometry_msgs::PoseWithCovarianceStamped msg = pose_;
  return msg;
}

void PoseHelper::yaw(float& angle) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  angle = tf::getYaw(pose_.pose.pose.orientation);
}

void PoseHelper::position(Eigen::Vector3f& position)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  position << pose_.pose.pose.position.x, pose_.pose.pose.position.y, pose_.pose.pose.position.z;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace yocs_navi_toolkit
