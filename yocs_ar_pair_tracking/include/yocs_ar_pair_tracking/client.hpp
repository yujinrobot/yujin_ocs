

#ifndef _YOCS_AR_PAIR_TRACKING_CLIENT_HPP_
#define _YOCS_AR_PAIR_TRACKING_CLIENT_HPP_

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <yocs_msgs/ARPairList.h>
#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>


namespace yocs {

class ARPairTrackingClient {
  public:
    ARPairTrackingClient();
    ~ARPairTrackingClient();
    void globalMarkersCB(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void broadcastMarkersTF();
  protected:
    void createMirrorMarkers();
    void createTargets();
    void notifyARPairTracker();
    void publishMarkerTFs(const std::string prefix, const ar_track_alvar_msgs::AlvarMarkers& markers);
    void publishTargetTFs(const std::string prefix, const ar_track_alvar_msgs::AlvarMarkers& markers);
  private:
    ar_track_alvar_msgs::AlvarMarkers global_markers_, global_markers_mirrors_;
    ros::Subscriber sub_global_marker_;
    ros::Publisher  pub_update_ar_pair_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_brcaster_;
    double tf_broadcast_freq_;

    std::string global_frame_;
    std::string global_marker_prefix_;
    std::string target_frame_postfix_;
    double baseline_;
    double target_offset_;
};
}

#endif
