/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_navigator/semantic_navigator.hpp"

namespace yocs_navigator
{

SemanticNavigator::SemanticNavigator(ros::NodeHandle& n) :
    nh_(n),
    basic_move_(n),
    r_(2),
    apprs_received_(false),
    wps_received_(false)
{
}

SemanticNavigator::~SemanticNavigator()
{
}

bool SemanticNavigator::init()
{
  ros::NodeHandle pnh("~");

  pnh.param("global_frame", global_frame_, std::string("map")); // TODO: check, if used (Marcus)
  std::string move_base_name, navigator_name;
  pnh.param("move_base_server_name", move_base_name, std::string("move_base"));
  pnh.param("navigator_server_name", navigator_name, std::string("navigator"));

  double rate;
  pnh.param("spin_rate", rate, 2.0);
  r_ = ros::Rate(rate);

  distance_to_goal_ = 0.0f;
  navigation_in_progress_ = false;

  // reset goal way points
  target_wps_.clear();
  target_wps_it_ = target_wps_.end();

  loginfo("Waiting for move base ...");
  ac_move_base_.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(nh_, move_base_name, true));
  ac_move_base_->waitForServer();
  loginfo("Connected to move base.");

  loginfo("Waiting for way points, trajectories and approaches ...");
  approaches_sub_ = nh_.subscribe("approaches", 1, &SemanticNavigator::approachesCB, this);
  trajectories_sub_ = nh_.subscribe("trajectories", 1, &SemanticNavigator::trajectoriesCB, this);
  waypoints_sub_ = nh_.subscribe("waypoints", 1, &SemanticNavigator::waypointsCB, this);

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    if (wps_received_ && apprs_received_) // trajectories are optional
    {
      break;
    }
  }

  as_navi_.reset(new actionlib::SimpleActionServer<yocs_msgs::NavigateToAction>(nh_, navigator_name, false));
  as_navi_->registerGoalCallback(boost::bind(&SemanticNavigator::processNavigateToGoal, this));
  as_navi_->registerPreemptCallback(boost::bind(&SemanticNavigator::processPreemptNavigateTo, this));
  as_navi_->start();

  loginfo("Initialized");

  return true;
}

void SemanticNavigator::approachesCB(const yocs_msgs::ApproachList::ConstPtr& apprs)
{
  apprs_ = *apprs;
  std::ostringstream oss;
  oss << "Received " << apprs_.approaches.size() << " approaches";
  loginfo(oss.str());
  apprs_received_ = true;
}

void SemanticNavigator::trajectoriesCB(const yocs_msgs::TrajectoryList::ConstPtr& trajs)
{
  trajs_ = *trajs;
  std::ostringstream oss;
  oss << "Received " << trajs_.trajectories.size() << " trajectories.";
  loginfo(oss.str());
}

void SemanticNavigator::waypointsCB(const yocs_msgs::WaypointList::ConstPtr& wps)
{
  wps_ = *wps;
  std::ostringstream oss;
  oss << "Received " << wps_.waypoints.size() << " way points.";
  wps_received_ = true;
}

void SemanticNavigator::processNavigateToGoal()
{
  if (navigation_in_progress_)
  {
    as_navi_->acceptNewGoal();
    terminateNavigation(false, "Navigation under progress yet.. Ignoring");
    return;
  }

  navigation_in_progress_ = true;
  order_process_thread_ = boost::thread(&SemanticNavigator::processNavigation, this, as_navi_->acceptNewGoal());
}

void SemanticNavigator::processPreemptNavigateTo()
{
  logwarn("Navigation Preemption Requested");
  as_navi_->setPreempted();
}

void SemanticNavigator::spin()
{
  while (ros::ok())
  {
    ros::spinOnce();
    r_.sleep();
  }
}

} // namespace
