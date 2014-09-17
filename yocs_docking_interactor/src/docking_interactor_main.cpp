/*
 *  docking_interactor_main.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <ros/ros.h>
#include "yocs_docking_interactor/docking_interactor.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "docking_interactor");
  ros::NodeHandle n("");

  yocs_docking_interactor::DockingInteractor* interactor;

  interactor =  new yocs_docking_interactor::DockingInteractor(n);
  interactor->loginfo("Initialized");
  interactor->spin();
  interactor->loginfo("ByeBye");

  delete interactor;

  return 0;
}
