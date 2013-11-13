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

/**
 * Normalize an angle between -π and +π
 * @param a Unnormalized angle
 * @return Normalized angle
 */
double wrapAngle(double a);

/**
 * Shortcut to take the roll of a transform/pose
 * @param tf/pose
 * @return Transform/pose's roll
 */
double roll(const tf::Transform& tf);
double roll(geometry_msgs::Pose pose);
double roll(geometry_msgs::PoseStamped pose);

/**
 * Shortcut to take the pitch of a transform/pose
 * @param tf/pose
 * @return Transform/pose's pitch
 */
double pitch(const tf::Transform& tf);
double pitch(geometry_msgs::Pose pose);
double pitch(geometry_msgs::PoseStamped pose);

/**
 * Euclidean distance between 2D points; z coordinate is ignored
 * @param a point a
 * @param b point b
 * @return Distance
 */
double distance2D(double ax, double ay, double bx, double by);
double distance2D(geometry_msgs::Point a, geometry_msgs::Point b = geometry_msgs::Point());
double distance2D(geometry_msgs::Pose a, geometry_msgs::Pose b = geometry_msgs::Pose());
double distance2D(const tf::Vector3& a, const tf::Vector3& b = tf::Vector3());
double distance2D(const tf::Transform& a, const tf::Transform& b = tf::Transform());

/**
 * Euclidean distance between 3D points
 * @param a point a
 * @param b point b
 * @return Distance
 */
double distance3D(double ax, double ay, double az, double bx, double by, double bz);
double distance3D(geometry_msgs::Point a, geometry_msgs::Point b = geometry_msgs::Point());
double distance3D(geometry_msgs::Pose a, geometry_msgs::Pose b = geometry_msgs::Pose());
double distance3D(const tf::Vector3& a, const tf::Vector3& b);
double distance3D(const tf::Transform& a, const tf::Transform& b);

/**
 * Heading angle from point a to point b
 * @param a point a
 * @param b point b
 * @return Heading angle
 */
double heading(geometry_msgs::Point a, geometry_msgs::Point b = geometry_msgs::Point());
double heading(geometry_msgs::Pose a, geometry_msgs::Pose b = geometry_msgs::Pose());
double heading(const tf::Vector3& a, const tf::Vector3& b);
double heading(const tf::Transform& a, const tf::Transform& b);

/**
 * Minimum angle between quaternions
 * @param a quaternion a
 * @param b quaternion b
 * @return Minimum angle
 */
double minAngle(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b);
double minAngle(geometry_msgs::Pose a, geometry_msgs::Pose b);
double minAngle(const tf::Quaternion& a, const tf::Quaternion& b);
double minAngle(const tf::Transform& a, const tf::Transform& b);

/**
 * Compares frame ids ignoring the leading /, as it is frequently omitted.
 * @param frame_a
 * @param frame_b
 * @return true if frame ids match regardless leading /
 */
bool sameFrame(const std::string& frame_a, const std::string& frame_b);
bool sameFrame(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b);

/**
 * Minimum distance from a point to a line segment
 * @param px point x-coordinate
 * @param py point y-coordinate
 * @param s1x segment's point 1 x-coordinate
 * @param s1y segment's point 1 y-coordinate
 * @param s2x segment's point 2 x-coordinate
 * @param s2y segment's point 2 y-coordinate
 * @return distance
 */
double pointSegmentDistance(double px, double py, double s1x, double s1y, double s2x, double s2y);

/**
 * Do a finite length ray intersects a line segment?
 * @param r1x rays's start point x-coordinate
 * @param r1y rays's start point y-coordinate
 * @param r2x rays's end point x-coordinate
 * @param r2y rays's end point y-coordinate
 * @param s1x segment's point 1 x-coordinate
 * @param s1y segment's point 1 y-coordinate
 * @param s2x segment's point 2 x-coordinate
 * @param s2y segment's point 2 y-coordinate
 * @param ix  intersection point (if any) x-coordinate
 * @param iy  intersection point (if any) y-coordinate
 * @param distance Distance from rays's start to intersection point
 * @return True if ray intersects the segment
 */
bool raySegmentIntersection(double r1x, double r1y, double r2x, double r2y,
                            double s1x, double s1y, double s2x, double s2y,
                            double& ix, double& iy, double& distance);

/**
 * Do a zero-centered, finite length ray intersects a circle?
 * @param rx rays's end point x-coordinate
 * @param ry rays's end point y-coordinate
 * @param cx circle's center x-coordinate
 * @param cy circle's center y-coordinate
 * @param radius circle's radius
 * @param ix closest intersection point (if any) x-coordinate
 * @param iy closest intersection point (if any) y-coordinate
 * @param distance Distance from rays's start (zero) to intersection point
 * @return True if ray intersects the circle
 */
bool rayCircleIntersection(double rx, double ry, double cx, double cy, double radius,
                           double& ix, double& iy, double& distance);

} /* namespace mtk */

#endif /* GEOMETRY_HPP_ */
