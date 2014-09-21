/*  
 *  docking_interactor.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/docking_interactor.hpp"

namespace yocs_docking_interactor {

DockingInteractor::DockingInteractor(ros::NodeHandle& n) 
: nh_(n),
  bmc_(nh_),
  as_command_(nh_, DockingInteractorDefaultParam::AS_COMMAND, false),
  ac_move_base_(nh_, DockingInteractorDefaultParam::AC_MOVE_BASE, true)
{
}

DockingInteractor::DockingInteractor(ros::NodeHandle& n, const std::string as_command_topic) 
: nh_(n),
  bmc_(nh_),
  as_command_(as_command_topic, false),
  ac_move_base_(nh_, DockingInteractorDefaultParam::AC_MOVE_BASE, true)
{
}

DockingInteractor::~DockingInteractor()
{
}

bool DockingInteractor::init()
{
  // variables
  command_in_progress_ = false;
  global_marker_received_ = false;
  
  // ar track alvar tracker handler
  srv_tracker_params_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(DockingInteractorDefaultParam::AR_TRACKER_SET_PARAM);
  tracker_enabled_ = false;

  // global marker subscriber
  loginfo("Wait for global marker list");
  sub_global_markers_ = nh_.subscribe(DockingInteractorDefaultParam::SUB_GLOBAL_MARKERS, 1, &DockingInteractor::processGlobalMarkers, this); 
  while(ros::ok() && !global_marker_received_)
  {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  // move base
  loginfo("Wait for movebase");
  ac_move_base_.waitForServer();



  loginfo("Initialised");
  as_command_.registerGoalCallback(boost::bind(&DockingInteractor::processGoalCommand, this));
  as_command_.registerPreemptCallback(boost::bind(&DockingInteractor::processPreemptCommand, this));
  as_command_.start();

  return true;
}

void DockingInteractor::spin()
{
  ros::Rate r(10);

  init();

  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}

void DockingInteractor::processGoalCommand()
{
  if(command_in_progress_)
  {
    as_command_.acceptNewGoal();
    terminateCommand(false, "Command under progress yet.. Ignoring new command");
    return;
  }

  command_in_progress_ = true;
  command_process_thread_ = boost::thread(&DockingInteractor::processCommand, this, as_command_.acceptNewGoal());
}

void DockingInteractor::processPreemptCommand()
{
  logwarn("Command Preemption Requested");
  as_command_.setPreempted();
}

void DockingInteractor::processGlobalMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  global_markers_ = *msg;

  for(unsigned int i=0; i < global_markers_.markers.size(); i++)
  {
    ar_track_alvar_msgs::AlvarMarker m = global_markers_.markers[i];
    m.id = m.id - 3;
    global_markers_.markers.push_back(m);
  }

  std::stringstream ss;
  ss << global_markers_.markers.size() << "global marker pose(s) received";
  loginfo(ss.str());

  global_marker_received_ = true;
}

void DockingInteractor::processArMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
}
}
