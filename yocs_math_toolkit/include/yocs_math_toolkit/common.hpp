/*
 * common.hpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#ifndef COMMON_HPP_
#define COMMON_HPP_

// deprecation macros
#ifdef __GNUC__
#define MTK_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define MTK_DEPRECATED __declspec(deprecated)
#elif defined(__clang__)
#define MTK_DEPRECATED  __attribute__((deprecated("MTK: Use of this method is deprecated")))
#else
#define MTK_DEPRECATED /* Nothing */
#endif


#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


namespace mtk
{

// Template functions

template <typename T> std::string nb2str(T x)
{
  return static_cast<std::ostringstream*>( &(std::ostringstream() << x) )->str();
}

template <typename T> T median(const std::vector<T>& v) {
  // Return the median element of a vector
  std::vector<T> c = v;  // clone, as nth_element will short half of the vector
  std::nth_element(c.begin(), c.begin() + c.size()/2, c.end());
  return c[c.size()/2];
};

template <typename T> T mean(const std::vector<T>& v)
{
  T acc = 0;
  for (unsigned int i = 0; i < v.size(); i++)
    acc += v[i];
  return acc/v.size();
};

template <typename T> T variance(const std::vector<T>& v)
{
  T acc = 0;
  T avg = mean(v);
  for (unsigned int i = 0; i < v.size(); i++)
    acc += std::pow(v[i] - avg, 2);
  return acc/(v.size() - 1);
};

template <typename T> T std_dev(const std::vector<T>& v)
{
  return std::sqrt(variance(v));
}


template <typename T> T sign(T x)
{
  return x > 0.0 ? +1.0 : x < 0.0 ? -1.0 : 0.0;
}

// Other functions

void tf2pose(const tf::Transform& tf, geometry_msgs::Pose& pose);
void tf2pose(const tf::StampedTransform& tf, geometry_msgs::PoseStamped& pose);

void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf);
void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf);

std::string point2str2D(const geometry_msgs::Point& point);
std::string point2str2D(const geometry_msgs::PointStamped& point);
std::string point2str3D(const geometry_msgs::Point& point);
std::string point2str3D(const geometry_msgs::PointStamped& point);
std::string pose2str2D(const geometry_msgs::Pose& pose);
std::string pose2str2D(const geometry_msgs::PoseStamped& pose);
std::string pose2str3D(const geometry_msgs::Pose& pose);
std::string pose2str3D(const geometry_msgs::PoseStamped& pose);

// Deprecated, as you cannot call more than once in a single sentence; use the xxx2str2D/3D cousins instead
MTK_DEPRECATED const char* point2str(const geometry_msgs::Point& point);
MTK_DEPRECATED const char* pose2str(const geometry_msgs::Pose& pose);
MTK_DEPRECATED const char* pose2str(const geometry_msgs::PoseStamped& pose);


} /* namespace mtk */

#endif /* COMMON_HPP_ */
