/*  
 *  command_handles:.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/docking_interactor.hpp"

namespace yocs_docking_interactor {
void DockingInteractor::processCommand(yocs_msgs::DockingInteractorGoal::ConstPtr goal)
{
  int command = goal->command;

  switch(command) {
    case yocs_msgs::DockingInteractorGoal::WAKE_UP:
      wakeUp(goal->distance);
      break;
    case yocs_msgs::DockingInteractorGoal::REGISTER_DOCK_IN_GLOBAL_FRAME:
      registerDockMarker();
      break;
    case yocs_msgs::DockingInteractorGoal::RETURN_TO_DOCK:
      returnToDock();
      break;
    default:
      terminateCommand(false,"Unknown command");
      break;
  }
}

void DockingInteractor::wakeUp(double distance)
{
  // enable tracker
  loginfo("Waking up! Slowly moving back...");
  if (docking_ar_tracker_->enableTracker() == false)
  {
    terminateCommand(false,"Unable to start AR markers tracker; aborting wake up!");
  }
 
  // Move back until we detect the AR marker identifying this robot's docking station
  bool timeout = false;
  ros::Time t0 = ros::Time::now();
  ar_track_alvar_msgs::AlvarMarkers spotted_markers;

// until
// it sees docking ar marker
}

void DockingInteractor::registerDockMarker()
{
}

void DockingInteractor::returnToDock()
{
}



void DockingInteractor::terminateCommand(bool success, const std::string message) 
{ 
  yocs_msgs::DockingInteractorResult result;                   

  result.success = success;                             
  result.message = message;
  
  command_in_progress_ = false;
  as_command_.setSucceeded(result);                        
  return;
}
}
