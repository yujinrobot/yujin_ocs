/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include "yocs_navigator/semantic_navigator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "semantic_navigator");
  ros::NodeHandle n("");

  boost::shared_ptr<yocs_navigator::SemanticNavigator> navigator(new yocs_navigator::SemanticNavigator(n));

  if (navigator->init())
  {
    navigator->spin();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't initialise the navigator.");
    return -1;
  }

  navigator->loginfo("Bye Bye");

  return 0;
}
