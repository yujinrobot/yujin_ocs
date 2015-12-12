/*
 * common.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include "../../include/yocs_math_toolkit/common.hpp"
#include "../../include/yocs_math_toolkit/geometry.hpp"


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
  sprintf(___buffer___, "%.2f, %.2f, %.2f", pose.position.x, pose.position.y, yaw(pose));
  return (const char*)___buffer___;
}

const char* pose2str(const geometry_msgs::PoseStamped& pose)
{
  sprintf(___buffer___, "%.2f, %.2f, %.2f", pose.pose.position.x, pose.pose.position.y, yaw(pose));
  return (const char*)___buffer___;
}

std::string vector2str3D(const geometry_msgs::Vector3& vector)
{
  sprintf(___buffer___, "%.2f, %.2f, %.2f", vector.x, vector.y, vector.z);
  return std::string(___buffer___);
}

std::string vector2str3D(const geometry_msgs::Vector3Stamped& vector)
{
  return vector2str3D(vector.vector);
}

std::string point2str2D(const geometry_msgs::Point& point)
{
  sprintf(___buffer___, "%.2f, %.2f", point.x, point.y);
  return std::string(___buffer___);
}

std::string point2str2D(const geometry_msgs::PointStamped& point)
{
  return point2str2D(point.point);
}

std::string point2str3D(const geometry_msgs::Point& point)
{
  sprintf(___buffer___, "%.2f, %.2f, %.2f", point.x, point.y, point.z);
  return std::string(___buffer___);
}

std::string point2str3D(const geometry_msgs::PointStamped& point)
{
  return point2str3D(point.point);
}

std::string pose2str2D(const geometry_msgs::Pose& pose)
{
  sprintf(___buffer___, "%.2f, %.2f, %.2f", pose.position.x, pose.position.y, yaw(pose));
  return std::string(___buffer___);
}

std::string pose2str2D(const geometry_msgs::PoseStamped& pose)
{
  return pose2str2D(pose.pose);
}

std::string pose2str3D(const geometry_msgs::Pose& pose)
{
  sprintf(___buffer___, "%.2f, %.2f, %.2f,  %.2f, %.2f, %.2f",
          pose.position.x, pose.position.y, pose.position.z, roll(pose), pitch(pose), yaw(pose));
  return std::string(___buffer___);
}

std::string pose2str3D(const geometry_msgs::PoseStamped& pose)
{
  return pose2str3D(pose.pose);
}

} /* namespace mtk */
