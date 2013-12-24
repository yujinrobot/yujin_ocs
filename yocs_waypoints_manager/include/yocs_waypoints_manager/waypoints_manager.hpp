/*
   Way point Manager

   inspired by yocs_waypoints_navi

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */


#ifndef _YOCS_WAYPOINT_MANAGER_HPP_
#define _YOCS_WAYPOINT_MANAGER_HPP_ 

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <yocs_msgs/Waypoint.h>

namespace yocs {
  class WaypointManager {
    public:
      WaypointManager(ros::NodeHandle& n);
      ~WaypointManager();

      void spin();
    protected:
    private:
      ros::NodeHandle nh;
      std::string filename;
      std::vector
  };
}

#endif // _YOCS_WAYPOINT_MANAGER_HPP_
