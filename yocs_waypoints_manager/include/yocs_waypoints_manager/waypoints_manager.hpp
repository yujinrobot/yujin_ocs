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
#include <yocs_msgs/WaypointListService.h>
#include <yocs_msgs/WaypointList.h>
#include <visualization_msgs/MarkerArray.h>

/*
   It simply parses waypoints in yaml file and provides latch topic

   Future improvements
    - Utilise database for persistent waypoint storing
    - add/remove waypoints via srv or pub/sub
 */

namespace yocs {
  class WaypointManager {
    public:
      WaypointManager(ros::NodeHandle& n, yocs_msgs::WaypointList& wp);
      ~WaypointManager();

      void spin();
    protected:
      void init();

      void generateVizmarkers(const yocs_msgs::WaypointList& wp, visualization_msgs::MarkerArray& wp_viz);
      bool processWaypointsService(yocs_msgs::WaypointListService::Request& request, yocs_msgs::WaypointListService::Response& response);

    private:
      bool initialized_;
      ros::NodeHandle nh_;
      ros::Publisher waypoints_pub_;
      ros::Publisher waypoints_viz_pub_;
      ros::ServiceServer waypoints_srv_;

      yocs_msgs::WaypointList waypoints_;
      visualization_msgs::MarkerArray waypoints_viz_;
  };
}

#endif // _YOCS_WAYPOINT_MANAGER_HPP_
