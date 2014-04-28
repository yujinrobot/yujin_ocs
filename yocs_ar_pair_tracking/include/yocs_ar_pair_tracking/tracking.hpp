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
#include <ar_track_alvar/AlvarMarkers.h>
#include <yocs_ar_marker_tracking/tracking.hpp>

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

  protected:
    // raw list of ar markers from ar_alvar_track pkg 
    void customCB(const ar_track_alvar::AlvarMarkers& spotted_markers, const std::vector<TrackedMarker> &tracked_markers);
    void computeRelativeRobotPose(const ar_track_alvar::AlvarMarkers& spotted_markers, const std::vector<TrackedMarker>& tracked_markers);

  private:
    // Confidence evaluation attributes

    ros::Publisher pub_relative_target_pose_, pub_initial_pose_, pub_spotted_markers_;

    tf::Transformer          tf_internal_;
    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_brcaster_;
//    double                   tf_broadcast_freq_;  /**< Allows enabling tf broadcasting; mostly for debug */

    int    ar_pair_left_id_;
    int    ar_pair_right_id_;
    double ar_pair_baseline_;
    double ar_pair_target_pose_offset_;
    bool   publish_transforms;
};

} /* namespace yocs */

#endif /* AR_MARKERS_HPP_ */
