/*  
 *  docking_ar_tracker.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/ar_tracker.hpp"

namespace yocs_docking_interactor {

DockingARTracker::DockingARTracker(ros::NodeHandle& n): nh_(n) {
  init();
}

DockingARTracker::~DockingARTracker() {
}



bool DockingARTracker::init()
{
  // ar track alvar tracker handler
  srv_tracker_params_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(DockingARTrackerDefaultParam::AR_TRACKER_SET_PARAM);
  tracker_enabled_ = false;


  global_marker_received_ = false;
  sub_global_markers_ = nh_.subscribe(DockingARTrackerDefaultParam::SUB_GLOBAL_MARKERS, 1, &DockingARTracker::processGlobalMarkers, this); 
}

bool DockingARTracker::isReady()
{
  return global_marker_received_;
}

void DockingARTracker::processGlobalMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  global_markers_ = *msg;

  for(unsigned int i=0; i < global_markers_.markers.size(); i++)
  {
    ar_track_alvar_msgs::AlvarMarker m = global_markers_.markers[i];
    m.id = m.id - 3;
    global_markers_.markers.push_back(m);
  }

  global_marker_received_ = true;
}


void DockingARTracker::customCB(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const std::vector<yocs::TrackedMarker> &tracked_markers)
{
}
}
