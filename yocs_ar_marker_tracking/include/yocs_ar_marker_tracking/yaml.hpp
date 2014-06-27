/*
   AR marker yaml parser

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */

#ifndef _YAML_PARSER_AR_ALVAR_HPP_
#define _YAML_PARSER_AR_ALVAR_HPP__

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

#include <ar_track_alvar_msgs/AlvarMarkers.h>

namespace yocs {
  bool loadAlvarMarkersFromYaml(const std::string& filename,ar_track_alvar_msgs::AlvarMarkers& ams);
  void getYamlNode(const std::string& filename, YAML::Node& node); 
  void parseMarkers(const YAML::Node& node, ar_track_alvar_msgs::AlvarMarkers& ams); 
}

#endif
