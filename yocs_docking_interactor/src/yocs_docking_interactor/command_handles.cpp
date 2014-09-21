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
      wakeUp();
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

void DockingInteractor::wakeUp()
{
// move backward
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
