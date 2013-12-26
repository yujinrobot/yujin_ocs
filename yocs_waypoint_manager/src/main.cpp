/*
   Way point Manager

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */

#include "yocs_waypoint_manager/waypoint_manager.hpp"
#include "yocs_waypoint_manager/yaml_parser.hpp"
#include <yocs_msgs/WaypointList.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_manager");
  ros::NodeHandle priv_n("~");
  ros::NodeHandle n;
  yocs::WaypointManager* wm;
  yocs_msgs::WaypointList wps;
  std::string filename;
  
  if(!priv_n.getParam("filename", filename)) {
    ROS_ERROR("Waypoint Manager : filename argument is not set");
    return -1;
  }

  if(!yocs::loadWaypointListFromYaml(filename, wps)) {
    ROS_ERROR("Waypoint Manager : Failed to parse yaml[%s]",filename.c_str());
    return -1;
  }

  wm = new yocs::WaypointManager(n, wps);

  ROS_INFO("Waypoint Manager : Initialized");
  wm->spin();
  ROS_INFO("Waypoint Manager : Bye Bye");

  delete wm;

  return 0;
}

