
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
 * Test node to demonstrate improving ar tag estimate based on depth data
 *
 * \author Bhaskara Marthi
 */

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <boost/lexical_cast.hpp>

using std::cerr;

namespace ar_track_alvar
{

namespace gm=geometry_msgs;

using std::vector;
using boost::make_shared;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> Cloud;

// Pixel coordinates in an image
struct Pixel
{
  unsigned r, c;
  Pixel (unsigned r=0, unsigned c=0) : r(r), c(c) {}
};

// Result of plane fit: inliers and the plane equation
struct PlaneFitResult
{
  PlaneFitResult () : inliers(make_shared<Cloud>()) {}
  Cloud::Ptr inliers;
  pcl::ModelCoefficients coeffs;
};
  

// ROS node state
class Node
{
public:

  Node (const vector<Pixel>& pixels);
  
  void cloudCB (const Cloud::ConstPtr& cloud);

private:
  
  PlaneFitResult fitPlane (Cloud::ConstPtr cloud) const;
  
  ros::NodeHandle nh_;
  
  // Distance threshold for plane fitting: how far are points
  // allowed to be off the plane?
  double distance_threshold_;

  // Pixel coordinates in depth image
  vector<Pixel> pixels_;
  
  ros::Subscriber cloud_sub_;
  
  ros::Publisher pose_pub_;

};


// Constructor
Node::Node (const vector<Pixel>& pixels) :
  distance_threshold_(0.005), pixels_(pixels),
  cloud_sub_(nh_.subscribe("cloud_in", 1, &Node::cloudCB, this)),
  pose_pub_(nh_.advertise<gm::PoseStamped>("origin", 10))
{}

// Wrapper for pcl plane fit
PlaneFitResult Node::fitPlane (Cloud::ConstPtr cloud) const
{
  PlaneFitResult res;
  pcl::PointIndices::Ptr inliers=boost::make_shared<pcl::PointIndices>();

  pcl::SACSegmentation<Point> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold_);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, res.coeffs);

  pcl::ExtractIndices<Point> extracter;
  extracter.setInputCloud(cloud);
  extracter.setIndices(inliers);
  extracter.setNegative(false);
  extracter.filter(*res.inliers);
  
  return res;
}

// Select out a subset of a cloud corresponding to a set of pixel coordinates
Cloud::Ptr filterCloud (const Cloud& cloud, const vector<Pixel>& pixels)
{
  Cloud::Ptr out(new Cloud());
  ROS_INFO("  Filtering out %zu pixels", pixels.size());
  for (const Pixel& p : pixels)
  {
    const Point& pt = cloud(p.r, p.c);
    if (isnan(pt.x) || isnan(pt.y) || isnan(pt.z))
      ROS_INFO("    Skipping (%.4f, %.4f, %.4f)", pt.x, pt.y, pt.z);
    else
      out->points.push_back(cloud(p.r, p.c));
  }
  return out;
}

// Return the centroid (mean) of a point cloud
gm::Point centroid (const Cloud& points)
{
  gm::Point sum;
  sum.x = 0;
  sum.y = 0;
  sum.z = 0;
  for (const Point& p : points)
  {
    sum.x += p.x;
    sum.y += p.y;
    sum.z += p.z;
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

// Given the plane coefficients, produce a Quaternion that corresponds to a 
// coordinate frame whose x axis lies along the normal to that plane.  Note that
// the quaternion has a degree of freedom which is resolved arbitrarily.
gm::Quaternion extractNormal (const pcl::ModelCoefficients& plane_coeffs)
{
  ROS_ASSERT(plane_coeffs.values.size()==4);

  // Normalize normal vector
  const double x0 = plane_coeffs.values[0];
  const double y0 = plane_coeffs.values[1];
  const double z0 = plane_coeffs.values[2];
  const double s = sqrt(x0*x0 + y0*y0 + z0*z0);
  const double x = x0/s;
  const double y = y0/s;
  const double z = z0/s;
  
  // Deal with degenerate case where normal is along x axis
  if (fabs(y)<1e-3 && fabs(z)<1e-3)
    return makeQuaternion(0, 0, 0, 1);

  // Find rotation axis that's perpendicular to normal and to x axis, and 
  // use this along with angle to x-axis to construct the quaternion
  const double ny = -z;
  const double nz = y;
  const double theta = acos(x);
  return makeQuaternion(0, ny*sin(theta/2), nz*sin(theta/2), cos(theta/2));
}


// Main callback: call fitPlane and publish result
void Node::cloudCB (const Cloud::ConstPtr& cloud)
{
  ROS_INFO("In callback");
  Cloud::Ptr selected_points = filterCloud(*cloud, pixels_);
  ROS_INFO("  Selected out %zu points", selected_points->size());
  PlaneFitResult res = fitPlane(selected_points);
  gm::PoseStamped pose;
  pose.header.stamp = cloud->header.stamp;
  pose.header.frame_id = cloud->header.frame_id;
  pose.pose.position = centroid(*res.inliers);
  pose.pose.orientation = extractNormal(res.coeffs);
  pose_pub_.publish(pose);
  ROS_INFO("Callback completed");
};



} // namespace

int main (int argc, char** argv)
{
  namespace ar=ar_track_alvar;
  ros::init(argc, argv, "kinect_filtering");
  
  // Parse command line arguments
  if (argc!=5)
  {
    cerr << "Usage: " << argv[0] << " R1 R2 C1 C2\n";
    return 1;
  }
  const int r0 = boost::lexical_cast<int>(argv[1]);
  const int c0 = boost::lexical_cast<int>(argv[2]);
  const int r1 = boost::lexical_cast<int>(argv[3]);
  const int c1 = boost::lexical_cast<int>(argv[4]);
  std::vector<ar::Pixel> pixels;

  for (int r=r0; r<=r1; r++)
    for (int c=c0; c<=c1; c++)
      pixels.push_back(ar::Pixel(r, c));
  
  ar_track_alvar::Node node(pixels);
  ros::spin();
  return 0;
}
