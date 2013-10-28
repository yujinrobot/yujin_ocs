/*
 * common.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include "yocs_math_toolkit/common.hpp"


namespace mtk
{


void tf2pose(const tf::Transform& tf, geometry_msgs::Pose& pose)
{
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  tf::quaternionTFToMsg(tf.getRotation(), pose.orientation);
}

void tf2pose(const tf::StampedTransform& tf, geometry_msgs::PoseStamped& pose)
{
  pose.header.stamp    = tf.stamp_;
  pose.header.frame_id = tf.frame_id_;
  tf2pose(tf, pose.pose);
}

void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf)
{
  tf.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  tf.setRotation(q);
}

void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf)
{
  tf.stamp_    = pose.header.stamp;
  tf.frame_id_ = pose.header.frame_id;
  pose2tf(pose.pose, tf);
}

char ___buffer___[256];

const char* point2str(const geometry_msgs::Point& point)
{
  sprintf(___buffer___, "%.2f, %.2f, %.2f", point.x, point.y, point.z);
  return (const char*)___buffer___;
}

const char* pose2str(const geometry_msgs::Pose& pose)
{
  sprintf(___buffer___, "%.2f, %.2f, %.2f", pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
  return (const char*)___buffer___;
}

const char* pose2str(const geometry_msgs::PoseStamped& pose)
{
  return pose2str(pose.pose);
}

} /* namespace mtk */
