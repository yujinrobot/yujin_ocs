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
#include <ar_track_alvar/AlvarMarkers.h>

namespace yocs {
  bool loadAlvarMarkersFromYaml(const std::string& filename,ar_track_alvar::AlvarMarkers& ams);
  void getYamlNode(const std::string& filename, YAML::Node& node); 
  void parseMarkers(const YAML::Node& node, ar_track_alvar::AlvarMarkers& ams); 
}

#endif
