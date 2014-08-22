/*
   Way point Manager

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */

#include "yocs_waypoint_provider/waypoint_provider.hpp"
#include "yocs_waypoint_provider/yaml_parser.hpp"
#include <yocs_msgs/WaypointList.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_provider");
  ros::NodeHandle priv_n("~");
  ros::NodeHandle n;
  yocs::WaypointProvider* wm;
  yocs_msgs::WaypointList wps;
  yocs_msgs::TrajectoryList trajs;
  std::string filename;

  if(!priv_n.getParam("filename", filename)) {
    ROS_ERROR("Waypoint Provider : filename argument is not set");
    return -1;
  }

  if(!yocs::loadWaypointsAndTrajectoriesFromYaml(filename, wps, trajs))
  {
    ROS_ERROR("Waypoint Provider : Failed to parse yaml[%s]",filename.c_str());
    return -1;
  }

  wm = new yocs::WaypointProvider(n, wps, trajs);

  ROS_INFO("Waypoint Provider : Initialized");
  wm->spin();
  ROS_INFO("Waypoint Provider : Bye Bye");

  delete wm;

  return 0;
}

