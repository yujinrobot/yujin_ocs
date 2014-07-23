/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_navigator/semantic_navigator.hpp"

namespace yocs {

SemanticNavigator::SemanticNavigator(ros::NodeHandle& n) 
: nh_(n), basic_move_(n),
  as_navigator_topic_(SemanticNavigatorDefaultParam::AS_NAVI),
  sub_tablelist_topic_(SemanticNavigatorDefaultParam::SUB_TABLELIST),
  as_navi_(nh_, as_navigator_topic_, false),
  ac_move_base_(nh_, SemanticNavigatorDefaultParam::AC_MOVE_BASE, true)
{
}

SemanticNavigator::SemanticNavigator(ros::NodeHandle& n, const std::string& as_navigator_topic, const std::string& sub_tablelist_topic) 
: nh_(n), basic_move_(n),
  as_navigator_topic_(as_navigator_topic),
  sub_tablelist_topic_(sub_tablelist_topic),
  as_navi_(as_navigator_topic, false),
  ac_move_base_(SemanticNavigatorDefaultParam::AC_MOVE_BASE, true)
{
}

SemanticNavigator::~SemanticNavigator()
{
}

bool SemanticNavigator::init()
{
  table_received_ = false;

  loginfo("Wait for move_base");
  ac_move_base_.waitForServer();

  loginfo("Wait for table lists"); 
  sub_tablelist_ = nh_.subscribe(sub_tablelist_topic_, 1, &SemanticNavigator::processTableList, this); 
  while(ros::ok() && !table_received_) {
    ros::Duration(0.5).sleep();
  }
  
  loginfo("Initialized");
  as_navi_.registerGoalCallback(boost::bind(&SemanticNavigator::processNavigateTo, this));
  as_navi_.registerPreemptCallback(boost::bind(&SemanticNavigator::processPreemptNavigateTo, this));
  as_navi_.start();

  return true;
}

void SemanticNavigator::processTableList(const yocs_msgs::TableList::ConstPtr& msg)
{
  tablelist_ = *msg;
  table_received_ = true;
}

void SemanticNavigator::processNavigateTo()
{
}

void SemanticNavigator::processPreemptNavigateTo()
{
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

void SemanticNavigator::loginfo(const std::string& msg)
{
  ROS_INFO_STREAM_NAMED(ros::this_node::getName(), msg);
}
}
