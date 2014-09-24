/*
 *  docking_ar_tracker.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
*/

#ifndef _YOCS_DOCKING_AR_TRACKER_HPP_
#define _YOCS_DOCKING_AR_TRACKER_HPP__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
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
    bool reset();
    bool isReady();
    bool isDockRegistered();
    bool enableTracker();
    bool disableTracker();
    bool setClosestAsDockingMarker();
    bool registerDockingOnGlobalFrame(const std::string global_frame, const std::string base_frame, std::string& message);
    void getRobotDockPose(geometry_msgs::PoseStamped& pose);
    bool isDockMarkerSpotted(geometry_msgs::PoseStamped& pose, std::string& message);
    void getRobotPose(const std::string& global_frame, const std::string& base_frame, geometry_msgs::PoseStamped& pose);
    void getDockPoseInGlobal(const std::string& global_frame, const geometry_msgs::PoseStamped before, geometry_msgs::PoseStamped& pose);

  protected:
    void customCB(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const std::vector<yocs::TrackedMarker> &tracked_markers);
    void processGlobalMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    bool callTrackerService(bool value);


  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_global_markers_;
    ros::ServiceClient srv_tracker_params_;
  
    tf::TransformListener tf_listener_;

    ar_track_alvar_msgs::AlvarMarkers global_markers_;
    ar_track_alvar_msgs::AlvarMarker closest_;
    ar_track_alvar_msgs::AlvarMarker docking_marker_in_robot_frame_;
    ar_track_alvar_msgs::AlvarMarker docking_marker_in_global_frame_;
    geometry_msgs::PoseStamped robot_dock_pose_;

    double min_confidence_;
    bool global_marker_received_;
    bool tracker_enabled_;
    bool dock_marker_registered_;
};
}
#endif
