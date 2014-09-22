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
  docking_ar_tracker_->reset();
  if (docking_ar_tracker_->enableTracker() == false)
  {
    terminateCommand(false,"Unable to start AR markers tracker; aborting wake up!");
  }
 
  bmc_.backward(distance);

  bool success = docking_ar_tracker_->setClosestAsDockingMarker();

  if(success)
    terminateCommand(true, "Wake up!");
  else
    terminateCommand(false, "Failed to spot docking makrer");
}

void DockingInteractor::registerDockMarker()
{
  loginfo("Registering Docking Marker on global frame");
  std::string message;
  bool success;

  success = docking_ar_tracker_->registerDockingOnGlobalFrame(global_frame_, message);

  if(success)
    terminateCommand(true, message);
  else
    terminateCommand(false,message);
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
