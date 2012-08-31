
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * Library to improve ar tag estimate based on depth data
 *
 * \author Bhaskara Marthi
 */

#include <ros/ros.h>
#include <ar_track_alvar/kinect_filtering.h>
#include <tf/transform_datatypes.h>

namespace ar_track_alvar
{

namespace gm=geometry_msgs;

using std::vector;
using std::cerr;
using std::endl;
using std::ostream;

// Distance threshold for plane fitting: how far are points
// allowed to be off the plane?
const double distance_threshold_ = 0.005;

PlaneFitResult fitPlane (ARCloud::ConstPtr cloud)
{
  PlaneFitResult res;
  pcl::PointIndices::Ptr inliers=boost::make_shared<pcl::PointIndices>();

  pcl::SACSegmentation<ARPoint> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold_);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, res.coeffs);

  pcl::ExtractIndices<ARPoint> extracter;
  extracter.setInputCloud(cloud);
  extracter.setIndices(inliers);
  extracter.setNegative(false);
  extracter.filter(*res.inliers);
  
  return res;
}

ARCloud::Ptr filterCloud (const ARCloud& cloud, const vector<cv::Point>& pixels)
{
  ARCloud::Ptr out(new ARCloud());
  //ROS_INFO("  Filtering %zu pixels", pixels.size());
  //for (const cv::Point& p : pixels)
  for(size_t i=0; i<pixels.size(); i++)
  {
    const cv::Point& p = pixels[i];
    const ARPoint& pt = cloud(p.x, p.y);
    if (isnan(pt.x) || isnan(pt.y) || isnan(pt.z))
      ROS_INFO("    Skipping (%.4f, %.4f, %.4f)", pt.x, pt.y, pt.z);
    else
      out->points.push_back(pt);
  }
  return out;
}

gm::Point centroid (const ARCloud& points)
{
  gm::Point sum;
  sum.x = 0;
  sum.y = 0;
  sum.z = 0;
  //for (const Point& p : points)
  for(size_t i=0; i<points.size(); i++)
  {
    sum.x += points[i].x;
    sum.y += points[i].y;
    sum.z += points[i].z;
  }
  
  gm::Point center;
  const size_t n = points.size();
  center.x = sum.x/n;
  center.y = sum.y/n;
  center.z = sum.z/n;
  return center;
}

// Helper function to construct a geometry_msgs::Quaternion
inline
gm::Quaternion makeQuaternion (double x, double y, double z, double w)
{
  gm::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

// Extract and normalize plane coefficients
void getCoeffs (const pcl::ModelCoefficients& coeffs, double* a, double* b,
                double* c, double* d)
{
  ROS_ASSERT(coeffs.values.size()==4);
  const double s = coeffs.values[0]*coeffs.values[0] +
    coeffs.values[1]*coeffs.values[1] + coeffs.values[2]*coeffs.values[2];
  ROS_ASSERT(fabs(s)>1e-6);
  *a = coeffs.values[0]/s;
  *b = coeffs.values[1]/s;
  *c = coeffs.values[2]/s;
  *d = coeffs.values[3]/s;
}

// Project point onto plane
btVector3 project (const ARPoint& p, const double a, const double b,
                   const double c, const double d)
{
  const double t = a*p.x + b*p.y + c*p.z + d;
  return btVector3(p.x-t*a, p.y-t*b, p.z-t*c);
}

ostream& operator<< (ostream& str, const btMatrix3x3& m)
{
  str << "[" << m[0][0] << ", " << m[0][1] << ", " << m[0][2] << "; "
      << m[1][0] << ", " << m[1][1] << ", " << m[1][2] << "; "
      << m[2][0] << ", " << m[2][1] << ", " << m[2][2] << "]";
  return str;
}

ostream& operator<< (ostream& str, const btQuaternion& q)
{
  str << "[(" << q.x() << ", " << q.y() << ", " << q.z() <<
    "), " << q.w() << "]";
  return str;
}

ostream& operator<< (ostream& str, const btVector3& v)
{
  str << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
  return str;
}

btMatrix3x3 extractFrame (const pcl::ModelCoefficients& coeffs,
                                const ARPoint& p1, const ARPoint& p2)
{
  // Get plane coeffs and project p1 and p2
  double a=0, b=0, c=0, d=0;
  getCoeffs(coeffs, &a, &b, &c, &d);
  const btVector3 q1 = project(p1, a, b, c, d);
  const btVector3 q2 = project(p2, a, b, c, d);
  
  // Make sure q2 and q1 aren't the same so things are well-defined
  ROS_ASSERT((q2-q1).length()>1e-3);
  
  // (inverse) matrix with the given properties
  const btVector3 v = (q2-q1).normalized();
  const btVector3 n(a, b, c);
  const btVector3 w = -v.cross(n); 
  btMatrix3x3 m(v[0], v[1], v[2], w[0], w[1], w[2], n[0], n[1], n[2]);
  btMatrix3x3 m2 = m.inverse();

  cerr << "Frame is " << m2 << endl;

  return m2;
}


btQuaternion getQuaternion (const btMatrix3x3& m)
{
  ROS_ASSERT_MSG(m.determinant()>0, "Matrix had determinant %.2f",
                 m.determinant());
  btScalar y=0, p=0, r=0;
  m.getEulerZYX(y, p, r);
  btQuaternion q;
  q.setEulerZYX(y, p, r);
  btMatrix3x3 m2;
  m2.setRotation(q);
  ROS_INFO_STREAM("(y, p, r) are " << y << ", " << p << ", " << r <<
                  " and quaternion is " << q << " and frame is " << m2);
  return q;
}


gm::Quaternion extractOrientation (const pcl::ModelCoefficients& coeffs,
                                   const ARPoint& p1, const ARPoint& p2)
{
  btMatrix3x3 m = extractFrame(coeffs, p1, p2);
  btQuaternion q = getQuaternion(m);
  gm::Quaternion q_ros;
  tf::quaternionTFToMsg(q, q_ros);
  return q_ros;
}

} // namespace
