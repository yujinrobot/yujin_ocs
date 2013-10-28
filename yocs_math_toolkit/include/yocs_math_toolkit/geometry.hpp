/*
 * common.hpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_


#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


namespace mtk
{

double wrapAngle(double a);

double roll(const tf::Transform& tf);
double roll(geometry_msgs::Pose pose);
double roll(geometry_msgs::PoseStamped pose);

double pitch(const tf::Transform& tf);
double pitch(geometry_msgs::Pose pose);
double pitch(geometry_msgs::PoseStamped pose);

double distance2D(double ax, double ay, double bx, double by);
double distance2D(geometry_msgs::Point a, geometry_msgs::Point b = geometry_msgs::Point());
double distance2D(geometry_msgs::Pose a, geometry_msgs::Pose b = geometry_msgs::Pose());
double distance2D(const tf::Vector3& a, const tf::Vector3& b = tf::Vector3());
double distance2D(const tf::Transform& a, const tf::Transform& b = tf::Transform());

double distance3D(double ax, double ay, double az, double bx, double by, double bz);
double distance3D(geometry_msgs::Point a, geometry_msgs::Point b = geometry_msgs::Point());
double distance3D(geometry_msgs::Pose a, geometry_msgs::Pose b = geometry_msgs::Pose());
double distance3D(const tf::Vector3& a, const tf::Vector3& b);
double distance3D(const tf::Transform& a, const tf::Transform& b);

double heading(geometry_msgs::Point a, geometry_msgs::Point b = geometry_msgs::Point());
double heading(geometry_msgs::Pose a, geometry_msgs::Pose b = geometry_msgs::Pose());
double heading(const tf::Vector3& a, const tf::Vector3& b);
double heading(const tf::Transform& a, const tf::Transform& b);

double minAngle(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b);
double minAngle(geometry_msgs::Pose a, geometry_msgs::Pose b);
double minAngle(const tf::Quaternion& a, const tf::Quaternion& b);
double minAngle(const tf::Transform& a, const tf::Transform& b);

double pointSegmentDistance(double px, double py, double s1x, double s1y, double s2x, double s2y);

bool raySegmentIntersection(double r1x, double r1y, double r2x, double r2y,
                            double s1x, double s1y, double s2x, double s2y,
                            double& ix, double& iy, double& distance);
bool rayCircleIntersection(double rx, double ry, double cx, double cy, double radius,
                           double& ix, double& iy, double& distance);

} /* namespace mtk */

#endif /* GEOMETRY_HPP_ */
