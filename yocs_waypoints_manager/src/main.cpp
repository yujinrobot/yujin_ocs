/*
   Way point Manager

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */


#include "yocs_waypoints_manager/waypoints_manager.hpp"

int main(int argc, char** argv)
{
  ros::NodeHandle priv_n;

  yocs::WaypointManager* wm;

  wm = new yocs::WaypointManager;

  ROS_INFO("Waypoint Manager : Initialized");

  wm->spin();

  delete wm;

  return 0;
}

