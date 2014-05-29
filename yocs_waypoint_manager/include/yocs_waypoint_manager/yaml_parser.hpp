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

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

#include <yocs_msgs/WaypointList.h>

namespace yocs {
  bool loadWaypointListFromYaml(const std::string& filename,yocs_msgs::WaypointList& wps);
  void getYamlNode(const std::string& filename, YAML::Node& node); 
  void parseWaypoints(const YAML::Node& node, yocs_msgs::WaypointList& wps); 
}

#endif
