/*  
 *  logs.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/docking_interactor.hpp"

namespace yocs_docking_interactor {

void DockingInteractor::loginfo(const std::string& msg)
{
  ROS_INFO_STREAM(ros::this_node::getName() << " : " << msg);
}

void DockingInteractor::logwarn(const std::string& msg)
{
  ROS_WARN_STREAM(ros::this_node::getName() << " : " << msg);
}

void DockingInteractor::sendMovebaseGoal(const geometry_msgs::PoseStamped& pose)
{                                                                                              
  // TODO This is a horrible workaround for a problem I cannot solve: send a new goal          
  // when the previous one has been cancelled return immediately with succeeded state          
  int times_sent = 0;                                                                          
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = pose;
  goal.target_pose.header.stamp = ros::Time::now();
  

  do                                                                                           
  {                                                                                            
    ac_move_base_->sendGoal(goal);
    times_sent++;
  } while ((ac_move_base_->waitForResult(ros::Duration(0.1)) == true) &&
           (ac_move_base_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED));

  goal_pose_pub_.publish(pose);

  if (times_sent > 1)
    ROS_DEBUG("Again the strange case of instantaneous goals... (goal sent %d times)", times_sent);
}
}
