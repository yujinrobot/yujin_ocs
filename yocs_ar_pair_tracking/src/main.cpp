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
  ROS_INFO("AR Pair Tracking : initialised");
  tracking.spin();

  return 0;
}
