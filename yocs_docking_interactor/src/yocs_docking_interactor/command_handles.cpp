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
  bool success;
  // enable tracker
  loginfo("Waking up! Slowly moving back...");
  docking_ar_tracker_->reset();
  if (docking_ar_tracker_->enableTracker() == false)
  {
    terminateCommand(false,"Unable to start AR markers tracker; aborting wake up!");
  }
 
  bmc_->backward(distance);
  success = docking_ar_tracker_->setClosestAsDockingMarker();

  docking_ar_tracker_->disableTracker();

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

  if(docking_ar_tracker_->isDockRegistered() == false)
  {
    terminateCommand(false, "Dock Marker is not registered... Cannot go back to nest");
    return;
  }

  // navigation to dock
  docking_ar_tracker_->enableTracker(); // Keep an eye open to detect the marker as we approach it
  success = moveToDockFront(message);
  docking_ar_tracker_->disableTracker();

  // auto dock
  if(success)
    success = callAutoDock(message);

  terminateCommand(success, message);
}

bool DockingInteractor::callAutoDock(std::string& message)
{
  // Auto docking action
  // It tries twice to return to the docking station.
  // it returns false when it failed to dock.
  kobuki_msgs::AutoDockingGoal goal;
  ac_auto_dock_->sendGoal(goal);

  bool retry = false;
  ros::Time t0 = ros::Time::now(); 
  ros::Time t1 = t0;
  while(ac_auto_dock_->waitForResult(ros::Duration(2.0)) == false)
  {
    if ((ros::Time::now() - t1).toSec() < auto_dock_timeout_)
    {
      if (retry == false)
      {
        // Go back a bit and retry for an extra half timeout period  TODO really poor-man recovery...
        bmc_->backward(0.20);
    
        std::stringstream ss;
        ss << "Cannot auto-dock after " << std::setprecision(3) << (ros::Time::now() - t1).toSec() << " seconds; current state is "<< ac_auto_dock_->getState().toString() << ". Going backward to retry...";
        sendFeedback(yocs_msgs::DockingInteractorFeedback::WARN,ss.str()); 
        retry = true;
        t1 = ros::Time::now();
      }
    }
    else
    {
      ROS_WARN("Cannot auto-dock after %.2f seconds; current state is %s. Aborting...",
               (ros::Time::now() - t0).toSec(), ac_auto_dock_->getState().toString().c_str());
      break;
    }
  }
  if (ac_auto_dock_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    message ="Successfully docked...  zzz...   zzz...";
    return true;
  }
  else
  {
    cancelAllGoals(*ac_auto_dock_);
    std::stringstream ss;
    ss << "Go to docking base failed: " << ac_auto_dock_->getState().toString();
    message = ss.str();
    return false; 
  }
}

bool DockingInteractor::moveToDockFront(std::string& message)
{
  bool dock_infront = false;
  DockingState state = START_GLOBAL_DOCKING;

  ros::Time t0;

  while(!dock_infront) {
    switch(state) { 
      case START_GLOBAL_DOCKING:
        state = startGlobalNaviToDock();
        t0 = ros::Time::now();
        break;
      case GLOBAL_DOCKING:
        state = underGlobalNavigation();
        break;
      case CANCEL_GLOBAL_NAVIGATION_AND_START_MARKER_DOCKING:
        state = startSpottedMarkerBasedNaivgation();
        break;
      case MARKER_DOCKING:
        state = underSpottedMarkerBasedNavigation(); 
        break;
      case TERMINATE_GLOBAL_DOCKING:
      case TERMINATE_MARKER_DOCKING:
      case TERMINATE_ON_ERROR:
        dock_infront = true;
      default:
        break;
    }
  }

  if(state == TERMINATE_ON_ERROR)
  {
    message = "Failed to return back to docking station..";  
    return false;
  }

  // no matter it finishes in global docking or marker docking. It is success
  return true;
}

void DockingInteractor::terminateCommand(const bool success, const std::string message) 
{ 
  yocs_msgs::DockingInteractorResult result;                   

  result.success = success;                             
  result.message = message;
  
  command_in_progress_ = false;
  as_command_->setSucceeded(result);                        
  return;
}

void DockingInteractor::sendFeedback(const int level, const std::string message)
{
  yocs_msgs::DockingInteractorFeedback feedback;

  feedback.level = level ;
  feedback.message = message;
  as_command_->publishFeedback(feedback);
}
}
