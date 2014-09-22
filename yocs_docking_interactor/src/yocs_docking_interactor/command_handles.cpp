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

  success = docking_ar_tracker_->registerDockingOnGlobalFrame(global_frame_, base_frame_, message);

  terminateCommand(success, message);
}

void DockingInteractor::returnToDock()
{
  loginfo("Return to dock");
  std::string message;
  bool success;

  // global navigation to dock

  // local navigation to dock

  // auto dock
  success = callAutoDock(message);
  terminateCommand(success, message);
}

bool DockingInteractor::callAutoDock(std::string& message)
{
  kobuki_msgs::AutoDockingGoal goal;
  ac_auto_dock_.sendGoal(goal);

  bool retry = false;
  ros::Time t0 = ros::Time::now(); 
  while(ac_auto_dock_.waitForResult(ros::Duration(2.0)) == false)
  {
    if ((ros::Time::now() - t0).toSec() < auto_dock_timeout_)
    {
      ROS_DEBUG_THROTTLE(5.0, "Auto-dock action state: %s (%.2f seconds elapsed)",
                         ac_auto_dock_.getState().toString().c_str(), (ros::Time::now() - t0).toSec());
    }
    else if ((ros::Time::now() - t0).toSec() < auto_dock_timeout_*1.5)
    {
      if (retry == false)
      {
        // Go back a bit and retry for an extra half timeout period  TODO really poor-man recovery...
        bmc_.backward(0.20);
        ROS_WARN("Cannot auto-dock after %.2f seconds; current state is %s. Going backward to retry...",
                 (ros::Time::now() - t0).toSec(), ac_auto_dock_.getState().toString().c_str());
        retry = true;
      }
      else
      {
        ROS_DEBUG_THROTTLE(5.0, "Auto-dock action state: %s (%.2f seconds elapsed)",
                           ac_auto_dock_.getState().toString().c_str(), (ros::Time::now() - t0).toSec());
      }
    }
    else
    {
      ROS_WARN("Cannot auto-dock after %.2f seconds; current state is %s. Aborting...",
               (ros::Time::now() - t0).toSec(), ac_auto_dock_.getState().toString().c_str());
      break;
    }
  }
  if (ac_auto_dock_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    message ="Successfully docked...  zzz...   zzz...";
    return true;
  }
  else
  {
    std::stringstream ss;
    ss << "Go to docking base failed: " << ac_auto_dock_.getState().toString();
    message = ss.str();
    return false; 
  }
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
