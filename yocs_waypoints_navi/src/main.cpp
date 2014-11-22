#include "yocs_waypoints_navi/waypoints_navi.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoints_navi");

  yocs::WaypointsGoalNode eg;
  if (eg.init() == false)
  {
    ROS_ERROR("%s initialization failed", ros::this_node::getName().c_str());
    return -1;
  }
  ROS_INFO("%s initialized", ros::this_node::getName().c_str());
  eg.spin();
  return 0;
}
