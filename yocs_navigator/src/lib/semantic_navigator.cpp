/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_navigator/semantic_navigator.hpp"

namespace yocs_navigator {

SemanticNavigator::SemanticNavigator(ros::NodeHandle& n) 
: nh_(n), basic_move_(n),
  as_navi_(nh_, SemanticNavigatorDefaultParam::AS_NAVI, false),
  ac_move_base_(nh_, SemanticNavigatorDefaultParam::AC_MOVE_BASE, true)
{
  sub_waypointlist_topic_= SemanticNavigatorDefaultParam::SUB_WAYPOINTLIST;
}

SemanticNavigator::SemanticNavigator(ros::NodeHandle& n, const std::string& as_navigator_topic, const std::string& sub_waypointlist_topic)
: nh_(n), basic_move_(n),
  as_navi_(as_navigator_topic, false),
  ac_move_base_(SemanticNavigatorDefaultParam::AC_MOVE_BASE, true)
{
  sub_waypointlist_topic_= sub_waypointlist_topic; 
}

SemanticNavigator::~SemanticNavigator()
{
}

bool SemanticNavigator::init()
{
  ros::NodeHandle pnh("~");

  pnh.param("global_frame", global_frame_, std::string("map"));

  distance_to_goal_ = 0.0f;
  waypoint_received_ = false;
  navigation_in_progress_ = false;

  loginfo("Wait for move_base");
  ac_move_base_.waitForServer();

  loginfo("Wait for waypoint lists"); 
  sub_waypointlist_ = nh_.subscribe(sub_waypointlist_topic_, 1, &SemanticNavigator::processWaypointList, this); 
  while(ros::ok() && !waypoint_received_) {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }
  
  loginfo("Initialized");
  as_navi_.registerGoalCallback(boost::bind(&SemanticNavigator::processNavigateToGoal, this));
  as_navi_.registerPreemptCallback(boost::bind(&SemanticNavigator::processPreemptNavigateTo, this));
  as_navi_.start();

  return true;
}

void SemanticNavigator::processWaypointList(const yocs_msgs::WaypointList::ConstPtr& msg)
{
  waypointlist_ = *msg;
  waypoint_received_ = true;
}

void SemanticNavigator::processNavigateToGoal()
{
  if(navigation_in_progress_)
  {
    as_navi_.acceptNewGoal(); 
    terminateNavigation(false, "Navigation under progress yet.. Ignoring");
    return;
  }
 
  navigation_in_progress_ = true;
  order_process_thread_ = boost::thread(&SemanticNavigator::processNavigation, this, as_navi_.acceptNewGoal());
}

void SemanticNavigator::processPreemptNavigateTo()
{
  logwarn("Navigation Preemption Requested");
  as_navi_.setPreempted();
}

void SemanticNavigator::spin()
{
  ros::Rate r(2);

  init();
  
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}

}
