/*
   Way point Manager

   inspired by yocs_waypoints_navi

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */

#include "yocs_waypoint_manager/waypoint_manager.hpp"

namespace yocs {
  WaypointManager::WaypointManager(ros::NodeHandle& n, yocs_msgs::WaypointList& wp) : nh_(n) 
  { 

    // setup pub
    waypoints_pub_ = nh_.advertise<yocs_msgs::WaypointList>("waypoints", 5, true);
    waypoints_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoints_viz", 5, true); 
    // setup srv server
    waypoints_srv_ = nh_.advertiseService("request_waypoints", &WaypointManager::processWaypointsService, this);

    waypoints_ = wp;
    generateVizmarkers(waypoints_, waypoints_viz_);
  }


  WaypointManager::~WaypointManager() {}

  bool WaypointManager::processWaypointsService(yocs_msgs::WaypointListService::Request& request, yocs_msgs::WaypointListService::Response& response)
  {
    ROS_INFO("Waypoint Manager : Received request");
    if(!initialized_) // return false if node is not initialized with points
    {
      response.success = false;
    }
    else {
      response.waypoints = this->waypoints_;
      response.success = true;
    }
    return true;
  }

  void WaypointManager::generateVizmarkers(const yocs_msgs::WaypointList& wp, visualization_msgs::MarkerArray& wp_viz)
  {
  }

  void WaypointManager::spin() {
    waypoints_pub_.publish(waypoints_);
    waypoints_viz_pub_.publish(waypoints_viz_);
    initialized_ = true;
    ros::spin();
  }
}
