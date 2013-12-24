/*
   Waypoint yaml parser

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */

#ifndef _YOCS_WAYPOINT_MANAGER_YAML_PARSER_HPP_
#define _YOCS_WAYPOINT_MANAGER_YAML_PARSER_HPP_

#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <yocs_msgs/WaypointList.h>

namespace yocs {
  bool loadWaypointListFromYaml(const std::string& filename,yocs_msgs::WaypointList& wps);
  void getYamlNode(const std::string& filename, YAML::Node& node); 
  void parseWaypoints(const YAML::Node& node, yocs_msgs::WaypointList& wps); 
}

#endif
