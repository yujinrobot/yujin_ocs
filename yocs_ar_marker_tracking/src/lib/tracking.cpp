/*
 * ar_marker_processor.hpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee
 */

#include "yocs_ar_marker_tracking/tracking.hpp"

namespace yocs 
{

ARMarkerTracking::ARMarkerTracking() {
  init(); 
}

ARMarkerTracking::~ARMarkerTracking() {}

bool ARMarkerTracking::init()
{
  ros::NodeHandle nh, pnh("~");
  
  // Parameters
  pnh.param("max_valid_d_inc",    max_valid_d_inc_,        ARMarkerTrackingDefaultParams::MAX_VALID_D_INC);
  pnh.param("max_valid_h_inc",    max_valid_h_inc_,        ARMarkerTrackingDefaultParams::MAX_VALID_H_INC);
  pnh.param("max_tracking_time",  max_tracking_time_,      ARMarkerTrackingDefaultParams::MAX_TRACKING_TIME);
  pnh.param("min_penalized_dist", min_penalized_dist_,     ARMarkerTrackingDefaultParams::MIN_PENALIZED_DIST);
  pnh.param("max_reliable_dist",  max_reliable_dist_,      ARMarkerTrackingDefaultParams::MAX_RELIABLE_DIST);
  pnh.param("min_penalized_head", min_penalized_head_,     ARMarkerTrackingDefaultParams::MIN_PENALIZED_HEAD);
  pnh.param("max_reliable_head",  max_reliable_head_,      ARMarkerTrackingDefaultParams::MAX_RELIABLE_HEAD);

  if (nh.getParam("ar_track_alvar/max_frequency", ar_tracker_freq_) == false)
  {
    ar_tracker_freq_ = ARMarkerTrackingDefaultParams::AR_TRACKER_FREQ;
    ROS_WARN("Cannot get AR tracker frequency; using default value (%f)", ar_tracker_freq_);
    ROS_WARN("Confidence evaluation can get compromised if this is not the right value!");
  }

  sub_ar_markers_ = nh.subscribe("ar_track_alvar/ar_pose_marker", 1, &ARMarkerTracking::arPoseMarkersCB, this);

  // There are 18 different markers
  tracked_markers_.resize(ARMarkerTrackingDefaultParams::MARKERS_COUNT);

  return true;
}

void ARMarkerTracking::arPoseMarkersCB(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  // TODO: use confidence to incorporate covariance to global poses
  std::stringstream ss;
  for (unsigned int i = 0; i < msg->markers.size(); i++) {
    ss << " " << msg->markers[i].id;
  }
  ss << " ";
  if(msg->markers.size() > 0) {
    ROS_DEBUG_STREAM("AR Marker Tracking : received markers [" << ss.str() << "]");
  }

  // Maintain markers
  maintainTrackedMarkers(msg, tracked_markers_);
  spotted_markers_ = *msg;
  customCB(spotted_markers_, tracked_markers_);
}

void ARMarkerTracking::maintainTrackedMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg,std::vector<TrackedMarker>& tracked_markers)
{
  // Make thresholds relative to the tracking frequency (as it can be dynamically changed)
  int obs_list_max_size  = (int)round(max_tracking_time_*ar_tracker_freq_);
  double max_valid_d_inc = max_valid_d_inc_ / ar_tracker_freq_;
  double max_valid_h_inc = max_valid_h_inc_ / ar_tracker_freq_;

  for (unsigned int i = 0; i < msg->markers.size(); i++)
  {
    if (msg->markers[i].id >= tracked_markers.size())
    {
      // A recognition error from Alvar markers tracker
      ROS_WARN("Discarding AR marker with unrecognized id (%d)", msg->markers[i].id);
      continue;
    }
    //ROS_INFO("Maintaining marker id = %d",msg->markers[i].id);

    TrackedMarker& marker = tracked_markers[msg->markers[i].id];

    // Confidence evaluation
    maintainTrackedMarker(marker, msg->markers[i], obs_list_max_size, max_valid_d_inc, max_valid_h_inc);
  }
}


void ARMarkerTracking::maintainTrackedMarker(TrackedMarker& marker,const ar_track_alvar_msgs::AlvarMarker& msgMarker, const int obs_list_max_size, const double max_valid_d_inc, const double max_valid_h_inc)
{
  marker.distance = mtk::distance3D(msgMarker.pose.pose);
  marker.distance2d = mtk::distance2D(msgMarker.pose.pose.position.x, msgMarker.pose.pose.position.z, 0,0);
  marker.heading  = tf::getYaw(msgMarker.pose.pose.orientation) + M_PI/2.0;

  int position = 0;
  ros::Time now = ros::Time::now();
  geometry_msgs::PoseStamped prev = msgMarker.pose;
  for (ObsList::iterator it = marker.obs_list_.begin(); it != marker.obs_list_.end(); ++it)
  {
    double age = (now - it->header.stamp).toSec();
    if (age > ((position + 1)/ar_tracker_freq_) + 1.5)
    {
      int s0 = marker.obs_list_.size();
      marker.obs_list_.erase(it, marker.obs_list_.end());
      int s1 = marker.obs_list_.size();
      ROS_DEBUG("%d observations discarded (first one with position %d in the list) for being %f seconds old ( > %f)", s0 - s1, position, age, ((position + 1)/ar_tracker_freq_) + 1.5);
      break;
    }

    if ((mtk::distance3D(prev.pose, it->pose) > max_valid_d_inc) || (std::abs(mtk::minAngle(prev.pose, it->pose)) > max_valid_h_inc))
    {
      // Incoherent observation; stop going over the list and use current position value to fill confidence values
      ROS_DEBUG("%d  BREAK at %d   %f  %f     %f   %f        %f", msgMarker.id, position, mtk::distance3D(prev.pose, it->pose),  mtk::minAngle(prev.pose, it->pose), max_valid_d_inc, max_valid_h_inc, ar_tracker_freq_);
      break;
    }

    prev = *it;
    position++;
  }

  marker.conf_distance = marker.distance <= min_penalized_dist_ ? 1.0
                       : marker.distance >= max_reliable_dist_  ? 0.0
                       : 1.0 - std::pow((marker.distance - min_penalized_dist_) / (max_reliable_dist_ - min_penalized_dist_), 2);
  marker.conf_heading  = std::abs(marker.heading) <= min_penalized_head_ ? 1.0
                       : std::abs(marker.heading) >= max_reliable_head_  ? 0.0
                       : 1.0 - std::pow((std::abs(marker.heading) - min_penalized_head_) / (max_reliable_head_ - min_penalized_head_), 2);
  marker.stability     = marker.obs_list_.size() == 0 ? 0.0 : std::sqrt((double)position/(double)marker.obs_list_.size());
  marker.persistence   = std::sqrt((double)marker.obs_list_.size() / (double)obs_list_max_size);
  marker.confidence    = marker.conf_distance * marker.stability * marker.persistence; // * marker.conf_heading I don't understand how conf_heading works. it always <=-1.4 or 1.4 >= disable it until becomes necessary
  marker.obs_list_.insert(marker.obs_list_.begin(), msgMarker.pose);
  marker.obs_list_.begin()->header = msgMarker.header;  // bloody alvar tracker doesn't fill pose's header
  if (marker.obs_list_.size() > obs_list_max_size)
    marker.obs_list_.pop_back();

  ROS_DEBUG_STREAM(msgMarker.id << "\n" << marker);
}

void ARMarkerTracking::spin()
{
  ros::spin();
//  // Broadcasts the spotted ar markers tf
//  if (tf_broadcast_freq_ > 0.0)
//  {
//    ros::Rate rate(tf_broadcast_freq_);
//
//    while(ros::ok())
//    {
//      rate.sleep();
//      ros::spinOnce();
//    }
//  }
//  else {
//    while(ros::ok())
//    {
//      ros::Duration(0.5).sleep();
//      ros::spinOnce();
//    }
//  }
}

} // yocs 
