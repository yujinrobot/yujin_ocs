/*
 * ar_marker_processor
 *   main.cpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee
 */

#include "yocs_ar_pair_tracking/tracking.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_marker_processor");
  yocs::ARPairTracking tracking;

  ros::NodeHandle pnh("~");

  int left_id, right_id;
  double baseline, target_offset;
  yocs_msgs::ARPair p;
  
  // Parameters
  pnh.param("ar_pair_left_id",  left_id,  yocs::ARPairTrackingDefaultParams::AR_PAIR_LEFT_ID);
  pnh.param("ar_pair_right_id", right_id, yocs::ARPairTrackingDefaultParams::AR_PAIR_RIGHT_ID);
  pnh.param("ar_pair_baseline", baseline, yocs::ARPairTrackingDefaultParams::AR_PAIR_BASELINE);
  pnh.param("ar_pair_target_pose_offset", target_offset, yocs::ARPairTrackingDefaultParams::AR_PAIR_TARGET_POSE_OFFSET);

  p.left_id = left_id;
  p.right_id = right_id;
  p.baseline = baseline;
  p.target_offset = target_offset;
  p.target_frame = "global_marker_3_target";

  tracking.addPair(p);
  
  ROS_INFO("AR Pair Tracking : initialised");
  tracking.spin();

  return 0;
}
