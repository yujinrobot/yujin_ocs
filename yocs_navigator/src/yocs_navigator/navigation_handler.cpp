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

  if(mtk::sameFrame(table.pose.header.frame_id, global_frame_) == false)
  {
    terminateNavigation(false, "Target is not in global frame");
    return;
  }
  clearCostmaps();

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

void SemanticNavigator::goOn(const yocs_msgs::Table table, const double in_distance, const int num_retry, const double timeout)
{
  // Note that in_distance variable is not used yet.. for On 
  geometry_msgs::PoseStamped target;
  target.pose = table.pose.pose.pose;
  target.header = table.pose.header;

  int attempt = 0;
  int move_base_result;
  int navi_result;
  bool final_result;
  std::string message;
  move_base_msgs::MoveBaseGoal mb_goal;
  
  ros::Time started_time = ros::Time::now();

  while(attempt < num_retry && ros::ok())
  {

    move_base_result = NAVI_UNKNOWN;
    mb_goal.target_pose = target;
    ac_move_base_.sendGoal(mb_goal, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleDoneCallback(), 
                                    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(), 
                                    boost::bind(&SemanticNavigator::processMoveBaseFeedback, this, _1, target));
    
    waitForMoveBase(move_base_result, started_time, timeout);
    determineNavigationState(navi_result, move_base_result, ac_move_base_.getState());

    if(navi_result == NAVI_TIMEOUT)
    {
      cancelMoveBaseGoal();
      final_result = false;
      message = "Navigation Timed out";
      break;
    }
    else if(navi_result == NAVI_SUCCESS)
    { 
      final_result = true;
      message = "SUCCESS!";
      break;
    }
    else if(navi_result == NAVI_FAILED)
    {
      final_result = false;
      message = "Navigation Failed";
      break;
    }
    else if(navi_result == NAVI_UNKNOWN)
    {
      final_result = false;
      message = "Navigation has got unknown error....";     
      break;
    }
    else if(navi_result == NAVI_RETRY)
    {
      std::stringstream ss;
      attempt++;
      ss << "Reattempt to naviate.. " << attempt;

      ros::Time current_time = ros::Time::now();
      double diff = (current_time - started_time).toSec();
      double remain_time = timeout - diff; 

      feedbackNavigation(yocs_msgs::NavigateToFeedback::STATUS_RETRY, distance_to_goal_, remain_time, ss.str()); 
      clearCostmaps();
    }
    else {
      final_result = false;
      message = "Something got wrong... What is going on";
      break;
    }
  }

  terminateNavigation(final_result, message);
}

void SemanticNavigator::waitForMoveBase(int& move_base_result, const ros::Time& start_time, const double timeout)
{
  int result = NAVI_UNKNOWN;
  while(ros::ok() && ac_move_base_.waitForResult(ros::Duration(0.5)) == false)
  {
    ros::Time current_time = ros::Time::now();
    // timed out. Navigation Failed..
    double diff = (current_time - start_time).toSec();
    double remain_time = timeout - diff; 
    //ROS_INFO("Diff = %.4f,  timeout = %.4f",diff, timeout);
    if(diff > timeout)
    {
      result = NAVI_TIMEOUT;
      break;
    }

    if(as_navi_.isPreemptRequested())
    {
      cancelMoveBaseGoal();
    }

    feedbackNavigation(yocs_msgs::NavigateToFeedback::STATUS_INPROGRESS, distance_to_goal_, remain_time, "In Progress");
  }
  
//  if(result != NAVI_TIMEOUT) 
//    move_base_result = ac_move_base_.getState();

  ROS_INFO("Movebase : %d", result);
}

void SemanticNavigator::determineNavigationState(int& navi_result, const int move_base_result, const actionlib::SimpleClientGoalState  move_base_state)
{
  int result;
    
  if(move_base_result == NAVI_TIMEOUT)
  {
    result = NAVI_TIMEOUT;
  }
  else {
    actionlib::SimpleClientGoalState state = ac_move_base_.getState();

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      loginfo("Arrived the destination");
      result = NAVI_SUCCESS;
    }
    else if(state == actionlib::SimpleClientGoalState::ABORTED)
    {
      loginfo("movebase Aborted");
      result = NAVI_RETRY;
    }
    else if(state == actionlib::SimpleClientGoalState::REJECTED)
    {
      loginfo("movebase rejected");
      result = NAVI_FAILED;
    }
    else if(state == actionlib::SimpleClientGoalState::PREEMPTED) 
    {
      loginfo("movebase preempted");
      result = NAVI_FAILED;
    }
    else if(state == actionlib::SimpleClientGoalState::LOST)
    {
      loginfo("robot Lost");
      result = NAVI_FAILED;
    }
    else {
      std::stringstream message; 
      message << "Move base unknown result : " << move_base_result;
      loginfo(message.str());
      result = NAVI_UNKNOWN;
    }
  }

  ROS_INFO("Navi : %d", result);
  navi_result = result;
}

void SemanticNavigator::goNear(const yocs_msgs::Table table, const double distance, const int num_retry, const double timeout)
{
  geometry_msgs::PoseStamped target;
  target.pose = table.pose.pose.pose;
  target.header = table.pose.header;

  int attempt = 0;
  int move_base_result;
  int navi_result;
  bool final_result;
  std::string message;
  move_base_msgs::MoveBaseGoal mb_goal;

  ros::Time started_time = ros::Time::now();
  
  while(attempt < num_retry && ros::ok())
  {
  }
}

void SemanticNavigator::processMoveBaseFeedback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback, const geometry_msgs::PoseStamped& target_pose)
{
  geometry_msgs::PoseStamped robot_pose = feedback->base_position;

  // compute distance
  double distance = mtk::distance2D(robot_pose.pose.position.x, robot_pose.pose.position.y, target_pose.pose.position.x, target_pose.pose.position.y);
  distance_to_goal_ = distance;
}
}
