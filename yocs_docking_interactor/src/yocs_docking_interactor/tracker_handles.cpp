/*  
 *  docking_interactor.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/docking_interactor.hpp"

namespace yocs_docking_interactor {

void DockingInteractor::enableTracker() {
  if(callTrackerService(true))
    tracker_enabled_ = true;
  else
    tracker_enabled_ = false;
}

void DockingInteractor::disableTracker() {
  if(callTrackerService(false))
    tracker_enabled_ = false;
  else
    tracker_enabled_ = true;
}

bool DockingInteractor::callTrackerService(bool value)
{
  ros::Time t0 = ros::Time::now();
  dynamic_reconfigure::Reconfigure srv;
  srv.request.config.bools.resize(1);
  srv.request.config.bools[0].name = "enabled";
  srv.request.config.bools[0].value = value;

  if (tracker_params_srv_.call(srv))
  {
    std::stringstream ss;
    float time = (ros::Time::now() - t0).toSec();
    ss << "AR markers tracker enabled (" << time << " seconds)";
    loginfo(ss.str());

    return true;
  }
  else
  {
    std::stringstream ss;
    float time = (ros::Time::now() - t0).toSec();
    ss << "Failed to enable AR markers tracker (" << time << " seconds)";
    loginfo(ss.str());
    return false;
  }
}
}
