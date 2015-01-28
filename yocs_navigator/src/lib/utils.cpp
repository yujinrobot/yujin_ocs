/*
 *  navigation_handler.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_navigator/semantic_navigator.hpp"

namespace yocs_navigator {
bool SemanticNavigator::getGoalLocation(const std::string location,yocs_msgs::Waypoint& waypoint)
{
  unsigned int i;
  for(i = 0; i < waypointlist_.waypoints.size(); i ++)
  {
    yocs_msgs::Waypoint w = waypointlist_.waypoints[i];
    if(!location.compare(w.name)) // if matches
    {
      waypoint = w;
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

  navigation_in_progress_ = false;
  as_navi_.setSucceeded(result);

  return;
}

void SemanticNavigator::feedbackNavigation(const int status, const double distance, const double remain_time, const std::string message)
{
  yocs_msgs::NavigateToFeedback feedback;
  feedback.status = status;
  feedback.distance = distance;
  feedback.remain_time = remain_time;
  feedback.message = message;
  //loginfo(message);
  
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

bool SemanticNavigator::clearCostmaps()
{
  ros::Time t0 = ros::Time::now();

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>(SemanticNavigatorDefaultParam::CLEAR_COSTMAP);
  std_srvs::Empty srv;

  if (client.call(srv) == true)
  {
    ROS_INFO("Successfully cleared costmaps (%f seconds)", (ros::Time::now() - t0).toSec());
    return true;
  }
  else
  {
    ROS_ERROR("Failed to clear costmaps (%f seconds)", (ros::Time::now() - t0).toSec());
    return false;
  }
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

void SemanticNavigator::nextState(bool& retry,bool& final_result,std::string& message, const int navi_result,const ros::Time started_time)
{
  if(navi_result == NAVI_TIMEOUT)
  {
    cancelMoveBaseGoal();

    retry = false;
    final_result = false;
    message = "Navigation Timed out";
  }
  else if(navi_result == NAVI_SUCCESS)
  { 
    retry = false;
    final_result = true;
    message = "SUCCESS!";
  }
  else if(navi_result == NAVI_FAILED)
  {
    retry = false;
    final_result = false;
    message = "Navigation Failed";
  }
  else if(navi_result == NAVI_UNKNOWN)
  {
    retry = false;
    final_result = false;
    message = "Navigation has got unknown error....";     
  }
  else if(navi_result == NAVI_RETRY)
  {
    retry = true;
    final_result = false;
    message = "Retry...";
  }
  else {
    retry = false;
    final_result = false;
    message = "Something got wrong... What is going on";
  }

  return;
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
  
  ROS_INFO("Movebase : %d", result);
}

void SemanticNavigator::loginfo(const std::string& msg)
{
  ROS_INFO_STREAM(ros::this_node::getName() << " : " << msg);
}

void SemanticNavigator::logwarn(const std::string& msg)
{
  ROS_WARN_STREAM(ros::this_node::getName() << " : " <<  msg);
}
}
