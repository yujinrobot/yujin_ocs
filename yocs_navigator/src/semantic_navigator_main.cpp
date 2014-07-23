/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <ros/ros.h>
#include "yocs_navigator/semantic_navigator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "semantic_navigator");
  ros::NodeHandle n("");

  yocs::SemanticNavigator* navigator;

  navigator = new yocs::SemanticNavigator(n);
  navigator->spin();
  navigator->loginfo("Bye Bye");

  delete navigator;

  return 0;
}
