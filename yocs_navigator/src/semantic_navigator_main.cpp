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

  yocs_navigator::SemanticNavigator* navigator;

  navigator = new yocs_navigator::SemanticNavigator(n);
  navigator->spin();
  navigator->loginfo("Bye Bye");

  delete navigator;

  return 0;
}
