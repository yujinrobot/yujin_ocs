/*
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_ar_pair_tracking/tracking.hpp"
#include <std_msgs/String.h>

namespace yocs
{

ARPairTracking::ARPairTracking() {
  std::cout << "ARPairTracking constructor" << std::endl;
  init(); 
}

ARPairTracking::~ARPairTracking() {}

bool ARPairTracking::init()
{
  ros::NodeHandle pnh("~");
  pnh.param("publish_transforms", publish_transforms, ARPairTrackingDefaultParams::PUBLISH_TRANSFORMS);
  pnh.param("global_frame", global_frame_, ARPairTrackingDefaultParams::GLOBAL_FRAME); 
  pnh.param("marker_frame", marker_frame_, ARPairTrackingDefaultParams::MARKER_FRAME);
  pnh.param("base_frame", base_frame_, ARPairTrackingDefaultParams::BASE_FRAME);

  /*********************
  ** Publishers
  **********************/
  pub_initial_pose_ = pnh.advertise<geometry_msgs::PoseWithCovarianceStamped>(ARPairTrackingDefaultParams::PUB_INITIAL_POSE, 1);
  pub_relative_target_pose_ = pnh.advertise<geometry_msgs::PoseStamped>(ARPairTrackingDefaultParams::PUB_RELATIVE_TARGET_POSE, 1);
  pub_spotted_markers_ = pnh.advertise<std_msgs::String>(ARPairTrackingDefaultParams::PUB_SPOTTED_MARKERS, 1);
  sub_update_ar_pairs_ = pnh.subscribe<yocs_msgs::ARPairList>(ARPairTrackingDefaultParams::SUB_UPDATE_AR_PAIRS, 1,  &ARPairTracking::updateARPairsCB, this);
  
  ar_pairs_.clear();
  return true;
}

void ARPairTracking::addPair(const yocs_msgs::ARPair& pair)
{
  ar_pairs_.push_back(pair);
}

void ARPairTracking::updateARPairsCB(const yocs_msgs::ARPairList::ConstPtr& msg)
{
  unsigned int i;
  ROS_INFO("AR Pair Tracking : New Pairs Received");

  ar_pairs_.clear();

  for(i =0; i < msg->pairs.size(); i++)
  {
    ar_pairs_.push_back(msg->pairs[i]);
  }
}

void ARPairTracking::customCB(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const std::vector<TrackedMarker> &tracked_markers)
{
  ar_track_alvar_msgs::AlvarMarker left;
  ar_track_alvar_msgs::AlvarMarker right;
  bool both_spotted;
  unsigned int i;

  // More than one global Marker Pairs should not be detected in one scene
  for(i = 0; i < ar_pairs_.size(); i++)
  {
    both_spotted = spotMarkerPair(spotted_markers, ar_pairs_[i], left, right);

    // TODO: use confidence to incorporate covariance to global poses
    if(both_spotted) {
      computeRelativeRobotPose(ar_pairs_[i], tracked_markers, left, right);
    }
  }
}

bool ARPairTracking::spotMarkerPair(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const yocs_msgs::ARPair& pair, ar_track_alvar_msgs::AlvarMarker& left, ar_track_alvar_msgs::AlvarMarker& right)
{
  bool left_spotted = included(pair.left_id, spotted_markers, &left);
  bool right_spotted = included(pair.right_id, spotted_markers, &right);

  // should use a custom message instead of strings (e.g. bool left_spotted, bool right_spotted)
  std_msgs::String spotted;
  if(left_spotted && !right_spotted) {
    spotted.data = "left";
  } else if(!left_spotted && right_spotted) {
      spotted.data = "right";
  } else if(left_spotted && right_spotted) {
      spotted.data = "both";
  } else {
    spotted.data = "none";
  }
  pub_spotted_markers_.publish(spotted);

  return left_spotted && right_spotted;
}

void ARPairTracking::computeRelativeRobotPose(const yocs_msgs::ARPair& spotted_pair, const std::vector<TrackedMarker>& tracked_markers, const ar_track_alvar_msgs::AlvarMarker& left, const ar_track_alvar_msgs::AlvarMarker& right)
{
  double baseline = spotted_pair.baseline;
  double target_pose_offset = spotted_pair.target_offset;
  std::string target_pose_frame = spotted_pair.target_frame;
  double left_x = left.pose.pose.position.x;
  double left_z = left.pose.pose.position.z;
  double right_x = right.pose.pose.position.x;
  double right_z = right.pose.pose.position.z;

  double left_d = std::sqrt(left_x*left_x + left_z*left_z);
  double right_d = std::sqrt(right_x*right_x + right_z*right_z);
  double b = baseline/2 + (left_d*left_d-right_d*right_d)/(2*baseline);
  double a = std::sqrt(left_d*left_d - b*b);
  ROS_DEBUG_STREAM("AR Pairing Tracker : computing robot-marker relative pose");
  ROS_DEBUG_STREAM("AR Pairing Tracker :   left : [" << left_x << "," << left_z << "," << left_d << "]");
  ROS_DEBUG_STREAM("AR Pairing Tracker :   right: [" << right_x << "," << right_z << "," << right_d <<  "]");
  ROS_DEBUG_STREAM("AR Pairing Tracker :   1: " << baseline/2);
  ROS_DEBUG_STREAM("AR Pairing Tracker :   2: " << (left_d*left_d-right_d*right_d)/(2*baseline));
  ROS_DEBUG_STREAM("AR Pairing Tracker :   a=" << a << " b=" << b);

  // angle between the robot and the first marker
  double alpha = atan2(left_x, left_z);
  double alpha_degrees = alpha * (180.0) / M_PI;

  // alpah + beta is angle between the robot and the second marker
  double beta = atan2(right_x, right_z);
  double beta_degrees = beta * (180.0) / M_PI;

  // theta is the angle between the wall and the perpendicular in front of the robot
  double theta = atan2((left_z - right_z), (right_x - left_x));
  double theta_degrees = theta * (180.0) / M_PI;

  double target_x = left_x + (right_x - left_x) / 2 - target_pose_offset * sin(theta);
  double target_z = left_z + (right_z - left_z) / 2 - target_pose_offset * cos(theta);
  double target_heading = atan2(target_x, target_z);
  double target_heading_degrees = target_heading * 180.0 / M_PI;

  ROS_DEBUG_STREAM("AR Pairing Tracker :      alpha=" << alpha_degrees << "degrees");
  ROS_DEBUG_STREAM("AR Pairing Tracker :       beta=" << beta_degrees << "degrees");
  ROS_DEBUG_STREAM("AR Pairing Tracker :      theta=" << theta_degrees << "degrees");
  ROS_DEBUG_STREAM("AR Pairing Tracker : t_[x,z,h]=[" << target_x << "," << target_z << "," << target_heading_degrees << "deg]");

  // target_pose -> camera
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = marker_frame_;
  pose.pose.position.x = target_x;
  pose.pose.position.y = 0; 
  pose.pose.position.z = target_z;

  tf::Quaternion quat; 
  quat.setEuler(theta,0,0);

  pose.pose.orientation.x = quat.getX(); 
  pose.pose.orientation.y = quat.getY(); 
  pose.pose.orientation.z = quat.getZ(); 
  pose.pose.orientation.w = quat.getW(); 

  try
  {
    // get and set ar_link -> target_pose
    tf::StampedTransform tf_ar_target_pose;
    tf_listener_.lookupTransform(target_pose_frame, global_frame_, ros::Time(0), tf_ar_target_pose);
    tf_internal_.setTransform(tf_ar_target_pose);

    // set target_pose -> camera
    tf::StampedTransform tf_ar_camera;
    tf_ar_camera.child_frame_id_ = target_pose_frame;
    mtk::pose2tf(pose, tf_ar_camera);  // pose frame_id is set above (camera_rgb_optical_frame)
    tf_ar_camera.stamp_ = ros::Time::now();
    tf_internal_.setTransform(tf_ar_camera);

    // get and set camera -> base_footprint
    tf::StampedTransform tf_camera_base_footprint;
    tf_listener_.lookupTransform(base_frame_, marker_frame_, ros::Time(0), tf_camera_base_footprint);
    tf_internal_.setTransform(tf_camera_base_footprint);

    // get and publish ar_link -> base_footprint
    tf::StampedTransform tf_ar_base_footprint;
    tf_internal_.lookupTransform(global_frame_, base_frame_, ros::Time(0), tf_ar_base_footprint);
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pwcs(new geometry_msgs::PoseWithCovarianceStamped);

    pwcs->header.stamp = tf_ar_base_footprint.stamp_;
    pwcs->header.frame_id = global_frame_;

    geometry_msgs::PoseStamped ps;
    mtk::tf2pose(tf_ar_base_footprint, ps);
    pwcs->pose.pose = ps.pose;
    pwcs->pose.covariance[6*0+0] = 0.2 * 0.2;
    pwcs->pose.covariance[6*1+1] = 0.2 * 0.2;
    pwcs->pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

    // publish robot pose to nav watch dog
    pub_initial_pose_.publish(pwcs);
    pub_relative_target_pose_.publish(pose);

    // only for use in standalone mode with a 3d sensor (no robot).
    if(publish_transforms) {
      tf_brcaster_.sendTransform(tf_ar_base_footprint);
    }
  }
  catch (tf::TransformException const &ex)
  {
    ROS_WARN_STREAM("TF error: " << ex.what());
  }
}

} // yocs namespace
