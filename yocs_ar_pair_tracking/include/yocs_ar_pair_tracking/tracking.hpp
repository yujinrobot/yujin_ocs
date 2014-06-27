/*
 * ar_marker_processor.hpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee          
 *
 */

#ifndef AR_PAIR_TRACKING_HPP_
#define AR_PAIR_TRACKING_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <yocs_ar_marker_tracking/tracking.hpp>
#include <yocs_msgs/ARPair.h>
#include <yocs_msgs/ARPairList.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs
{

namespace ARPairTrackingDefaultParams
{
  const int    AR_PAIR_LEFT_ID            = 3;
  const int    AR_PAIR_RIGHT_ID           = 0;
  const double AR_PAIR_BASELINE           = 0.26;
  const double AR_PAIR_TARGET_POSE_OFFSET = 0.40;

  const bool   PUBLISH_TRANSFORMS = true;
  const std::string PUB_RELATIVE_TARGET_POSE = "relative_target_pose";
  const std::string PUB_INITIAL_POSE         = "initial_pose";
  const std::string PUB_SPOTTED_MARKERS      = "spotted_markers";
  const std::string SUB_UPDATE_AR_PAIRS      = "update_ar_pairs";
  const std::string GLOBAL_FRAME             = "ar_global";
  const std::string MARKER_FRAME             = "camera_rgb_optical_frame";
  const std::string BASE_FRAME               = "base_footprint";
  const std::string TARGET_POSE_FRAME        = "target_pose";
}

/*****************************************************************************
** Classes
*****************************************************************************/

class ARPairTracking : public ARMarkerTracking
{
  public:

    ARPairTracking();
    virtual ~ARPairTracking();

    bool init();
    void addPair(const yocs_msgs::ARPair& p);

  protected:
    // raw list of ar markers from ar_alvar_track pkg 
    void customCB(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const std::vector<TrackedMarker> &tracked_markers);
      bool spotMarkerPair(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const yocs_msgs::ARPair& pair, ar_track_alvar_msgs::AlvarMarker& left, ar_track_alvar_msgs::AlvarMarker& right);
      void computeRelativeRobotPose(const yocs_msgs::ARPair& spotted_pair, const std::vector<TrackedMarker>& tracked_markers, const ar_track_alvar_msgs::AlvarMarker& left, const ar_track_alvar_msgs::AlvarMarker& right);

    void updateARPairsCB(const yocs_msgs::ARPairList::ConstPtr& msg);

  private:
    // Confidence evaluation attributes

    ros::Publisher pub_relative_target_pose_, pub_initial_pose_, pub_spotted_markers_;
    ros::Subscriber sub_update_ar_pairs_;

    tf::Transformer          tf_internal_;
    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_brcaster_;
//    double                   tf_broadcast_freq_;  /**< Allows enabling tf broadcasting; mostly for debug */

    std::vector<yocs_msgs::ARPair> ar_pairs_;
    bool   publish_transforms;

    std::string global_frame_;
    std::string marker_frame_;
    std::string base_frame_;
    std::string target_pose_frame_;
};

} /* namespace yocs */

#endif /* AR_MARKERS_HPP_ */
