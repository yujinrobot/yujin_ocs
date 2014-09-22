/*  
 *  docking_interactor.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/ar_tracker.hpp"

namespace yocs_docking_interactor {

bool DockingARTracker::enableTracker() {
  if(callTrackerService(true))
    tracker_enabled_ = true;
  else
    tracker_enabled_ = false;

  return tracker_enabled_;
}

bool DockingARTracker::disableTracker() {
  if(callTrackerService(false))
    tracker_enabled_ = false;
  else
    tracker_enabled_ = true;
  return tracker_enabled_;
}

bool DockingARTracker::callTrackerService(bool value)
{
/*
  Note from Jorge
  TODO do not use by now because:
    - most times takes really long; I cannot find a pattern on timings
    - with -no kinect version of the tracker is not necessary anymore
    - a couple of times crashed and arduino gateway cpu usage spiked to 90%  NO IDEA WHY!!!!
    * Update:  retest with service call;  probably solves the last issue
  TODO should I call waitForServer? here? on init? mollaio...
 */
  ros::Time t0 = ros::Time::now();
  dynamic_reconfigure::Reconfigure srv;
  srv.request.config.bools.resize(1);
  srv.request.config.bools[0].name = "enabled";
  srv.request.config.bools[0].value = value;

  if (srv_tracker_params_.call(srv))
  {
    /*
    std::stringstream ss;
    float time = (ros::Time::now() - t0).toSec();
    ss << "AR markers tracker enabled (" << time << " seconds)";
    loginfo(ss.str());
    */
    return true;
  }
  else
  {
    /*
    std::stringstream ss;
    float time = (ros::Time::now() - t0).toSec();
    ss << "Failed to enable AR markers tracker (" << time << " seconds)";
    loginfo(ss.str());
    */
    return false;
  }
}
}
