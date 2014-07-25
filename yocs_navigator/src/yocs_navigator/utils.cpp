/*
 *  navigation_handler.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_navigator/semantic_navigator.hpp"

namespace yocs {
bool SemanticNavigator::getGoalLocationTable(const std::string location,yocs_msgs::Table& table)
{
  unsigned int i;
  for(i = 0; i < tablelist_.tables.size(); i ++)
  {
    yocs_msgs::Table t = tablelist_.tables[i];
    if(!location.compare(t.name)) // if matches
    {
      table = t;
      return true;
    }
  }
  return false;
}

void SemanticNavigator::terminateNavigation(bool success, const std::string message) 
{
  yocs_msgs::NavigateToResult result;

  result.success = success;
  result.message = message;
  result.distance = distance_to_goal_;

  loginfo(message);

  as_navi_.setSucceeded(result);

  return;
}

void SemanticNavigator::feedbackNavigation(const int status, const double distance, const std::string message)
{
  yocs_msgs::NavigateToFeedback feedback;
  feedback.status = status;
  feedback.distance = distance;
  feedback.message = message;
  loginfo(message);
  
  as_navi_.publishFeedback(feedback);
}

bool SemanticNavigator::cancelMoveBaseGoal()
{
  double timeout = 2.0;
  ac_move_base_.cancelAllGoals();
  if (ac_move_base_.waitForResult(ros::Duration(timeout)) == false)
  {
    logwarn("Failed to cancel move_base goal...");
    return false;
  }

  loginfo("move_base goal has cancelled");
  return true;
}


void SemanticNavigator::loginfo(const std::string& msg)
{
  ROS_INFO_STREAM_NAMED(ros::this_node::getName(), msg);
}

void SemanticNavigator::logwarn(const std::string& msg)
{
  ROS_WARN_STREAM_NAMED(ros::this_node::getName(), msg);
}

}
