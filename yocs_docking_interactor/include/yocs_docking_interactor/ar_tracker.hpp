/*
 *  docking_ar_tracker.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
*/

#ifndef _YOCS_DOCKING_AR_TRACKER_HPP_
#define _YOCS_DOCKING_AR_TRACKER_HPP__

#include <ros/ros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include "yocs_docking_interactor/default_params.h"
#include "yocs_ar_marker_tracking/tracking.hpp"

namespace yocs_docking_interactor {
class DockingARTracker : public yocs::ARMarkerTracking
{
  public:
    DockingARTracker(ros::NodeHandle& n);
    ~DockingARTracker();

    bool init();
    bool isReady();
    bool enableTracker();
    bool disableTracker();

  protected:
    void customCB(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const std::vector<yocs::TrackedMarker> &tracked_markers);
    void processGlobalMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    bool callTrackerService(bool value);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_global_markers_;
    ros::ServiceClient srv_tracker_params_;

    ar_track_alvar_msgs::AlvarMarkers global_markers_;

    bool global_marker_received_;
    bool tracker_enabled_;
};
}
#endif
