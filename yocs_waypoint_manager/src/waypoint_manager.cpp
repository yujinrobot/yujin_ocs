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

    marker_index_ = 1000;
    label_index_ = 2000;
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

  void WaypointManager::generateVizmarkers(const yocs_msgs::WaypointList& wps, visualization_msgs::MarkerArray& wp_viz)
  {
    wp_viz.markers.clear();

    unsigned int i;
    
    for(i = 0; i < wps.waypoints.size(); i++)
    {
      visualization_msgs::Marker marker;
      visualization_msgs::Marker label;

      createArrowMarker(i, wps.waypoints[i], marker);
      createLabelMarker(i, wps.waypoints[i], label);

      wp_viz.markers.push_back(marker);
      wp_viz.markers.push_back(label);
    }
  }

  void WaypointManager::createArrowMarker(const int i,const yocs_msgs::Waypoint& wp, visualization_msgs::Marker& marker)
  {
    marker.header.frame_id = wp.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoints";
    marker.id = i + marker_index_;
    marker.pose = wp.pose;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.r = 1.0f;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
  }

  void WaypointManager::createLabelMarker(const int i,const yocs_msgs::Waypoint& wp, visualization_msgs::Marker& marker)
  {
    marker.header.frame_id = wp.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoints_label";
    marker.id = i + label_index_;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.pose = wp.pose;
    marker.pose.position.z = marker.pose.position.z + marker.scale.z / 2.0 + 0.05;  // just above the marker
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = wp.name;
  }

  void WaypointManager::spin() {
    waypoints_pub_.publish(waypoints_);
    waypoints_viz_pub_.publish(waypoints_viz_);
    initialized_ = true;
    ros::spin();
  }
}
