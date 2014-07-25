/*
 *  navigation_handler.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */


#include "yocs_navigator/semantic_navigator.hpp"

namespace yocs {
void SemanticNavigator::processNavigation(yocs_msgs::NavigateToGoal::ConstPtr goal)
{
  std::string location = goal->location;
  int approach_type = goal->approach_type;
  unsigned int num_retry = goal->num_retry;
  double distance = goal->distance;
  double timeout = goal->timeout;

  yocs_msgs::Table table;
  bool result;

  result = getGoalLocationTable(location, table);
  if(!result) // if it fails to find the requested table
  {
    terminateNavigation(false, "Failed to find the requested table");
    return;
  }

  switch(approach_type) {
    case yocs_msgs::NavigateToGoal::APPROACH_NEAR:
      loginfo("Approach Type : NEAR");
      goNear(table, distance, num_retry, timeout);
      break;
    case yocs_msgs::NavigateToGoal::APPROACH_ON:
      loginfo("Approach Type : ON");
      goOn(table, distance, num_retry, timeout);
      break;
    default:
      terminateNavigation(false, "Invalid Approach Type");
      break;
  }
}

void SemanticNavigator::goOn(const yocs_msgs::Table table, const double distance, const int num_retry, const double timeout)
{
  geometry_msgs::PoseStamped target;
  target.pose = table.pose.pose.pose;
  target.header = table.pose.header;

  if(mtk::sameFrame(target.header.frame_id, global_frame_) == false)
  {
    terminateNavigation(false, "Target is not in global frame");
  }

  double distance_to_target = std::numeric_limits<double>::infinity();
  int result = 1; // 0 : fail 1: none 2 : success
  int attempt = 0;
  
  while(attempt < num_retry)
  {
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = target;
    ac_move_base_.sendGoal(mb_goal, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleDoneCallback(), 
                                    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(), 
                                    boost::bind(&SemanticNavigator::processMoveBaseFeedback, this, _1, target));

    // while wait for result...
    //   feed the target robot distance
    //   terminate when...
    //      timed out
    //      success
    //      failure
  }
}

void SemanticNavigator::goNear(const yocs_msgs::Table table, const double distance, const int num_retry, const double timeout)
{
}

void SemanticNavigator::processMoveBaseFeedback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback, const geometry_msgs::PoseStamped& target_pose)
{
  geometry_msgs::PoseStamped robot_pose = feedback->base_position;

  // compute distance
  double distance = mtk::distance2D(robot_pose.pose.position.x, robot_pose.pose.position.y, target_pose.pose.position.x, target_pose.pose.position.y);
  int status = yocs_msgs::NavigateToFeedback::STATUS_INPROGRESS;

  feedbackNavigation(status, distance, "IN PROGRESS");
}
}
