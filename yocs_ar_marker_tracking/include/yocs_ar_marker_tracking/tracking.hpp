/*
 * ar_marker_processor.hpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee          
 *
 */

#ifndef AR_MARKER_PROCESSOR_HPP_
#define AR_MARKER_PROCESSOR_HPP_

#include <ros/ros.h>

#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

namespace yocs
{
typedef std::list<geometry_msgs::PoseStamped> ObsList;

namespace ARMarkerTrackingDefaultParams {
  const double      MAX_VALID_D_INC        = 0.8;
  const double      MAX_VALID_H_INC        = 4.0;
  const double      MAX_TRACKING_TIME      = 2.0;
  const double      MIN_PENALIZED_DIST     = 1.4;
  const double      MAX_RELIABLE_DIST      = 2.8;
  const double      MIN_PENALIZED_HEAD     = 1.0;
  const double      MAX_RELIABLE_HEAD      = 1.4;

  const double      AR_TRACKER_FREQ        = 10.0;

  const uint32_t    MARKERS_COUNT          = 32;

  const std::string SUB_AR_MARKERS = "ar_track_alvar_msgs/ar_pose_marker"; 
  const std::string PUB_ROBOT_POSE_AR = "robot_pose_ar";
}

class TrackedMarker
{
  public:
    TrackedMarker()
    {
      distance      = 0.0;
      distance2d    = 0.0;
      heading       = 0.0;
      confidence    = 0.0;
      conf_distance = 0.0;
      conf_heading  = 0.0;
      persistence   = 0.0;
      stability     = 0.0;
    }
                           
    ~TrackedMarker()
    {
      obs_list_.clear();
    }

    // be careful with these - calling lastObservation on an empty list causes undefined behaviour.
    unsigned int numberOfObservations() const { return obs_list_.size(); }
    const geometry_msgs::PoseStamped& lastObservation() const { return obs_list_.back(); }
                           
    ObsList obs_list_;
    double distance;
    double distance2d;
    double heading;
    double confidence;
    double conf_distance;
    double conf_heading;
    double persistence;
    double stability;


    /*********************
    ** Streaming
    **********************/
    /**
     * Insertion operator for sending a formatted string of this object to an output stream.
     * @param ostream : the output stream.
     * @param marker : the marker to be inserted.
     * @return OutputStream : continue streaming with the updated output stream.
     */
    template <typename OutputStream>
    friend OutputStream& operator<<(OutputStream &ostream , const TrackedMarker& marker);
};

template <typename OutputStream>
OutputStream& operator<<(OutputStream &ostream , const TrackedMarker& marker) {

    if ( marker.numberOfObservations() > 0 ) {
      // be careful - this is not thread safe.
      const geometry_msgs::PoseStamped& last_observation = marker.lastObservation();
      ostream << "  Observation\n";
      ostream << "    Timestamp: " << last_observation.header.stamp.toNSec() << "\n";
      const geometry_msgs::Pose& pose = last_observation.pose;
      ostream << "    x, y, z  : [" << pose.position.x << "," << pose.position.y << "," << pose.position.z << "]\n";
    }
    ostream << "  Distance   : " << marker.distance << "\n";
    ostream << "  Distance2d : " << marker.distance2d << "\n";
    ostream << "  Heading    : " << marker.heading << "\n";
    ostream << "  Confidence : " << marker.confidence << "\n";
    ostream << "  Conf Dist  : " << marker.conf_distance<< "\n";
    ostream << "  Conf Head  : " << marker.conf_heading<< "\n";
    ostream << "  Persistence: " << marker.persistence << "\n";
    ostream << "  Stability  : " << marker.stability << "\n";
    ostream.flush();
    return ostream;
}


/*****************************************************************************
** Tracking Classes
*****************************************************************************/

class ARMarkerTracking
{
  public:

    ARMarkerTracking();
    virtual ~ARMarkerTracking();

    bool init();
    void spin();


  protected:

    // raw list of ar markers from ar_alvar_track pkg
    void arPoseMarkersCB(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    virtual void customCB(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const std::vector<TrackedMarker> &tracked_markers) {}
    void maintainTrackedMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg,std::vector<TrackedMarker>& tracked_markers);
    void maintainTrackedMarker(TrackedMarker& marker,const ar_track_alvar_msgs::AlvarMarker& msgMarker, const int obs_list_max_size, const double max_valid_d_inc, const double max_valid_h_inc);

    /////////////////// utility functions
    /**
     * Return spotted markers satisfying the constraints specified by the parameters
     * @param younger_than    Elapsed time between now and markers timestamp must be below this limit.
     * @param min_confidence
     * @param exclude_globals
     * @param spotted_markers
     * @return
     */
    bool spotted(double younger_than, double min_confidence, ar_track_alvar_msgs::AlvarMarkers& spotted_markers);
    bool closest(double younger_than, double min_confidence, ar_track_alvar_msgs::AlvarMarker& closest_marker);
    bool spotted(double younger_than, const ar_track_alvar_msgs::AlvarMarkers& including, const ar_track_alvar_msgs::AlvarMarkers& excluding, ar_track_alvar_msgs::AlvarMarkers& spotted_markers);
    bool closest(const ar_track_alvar_msgs::AlvarMarkers& including, const ar_track_alvar_msgs::AlvarMarkers& excluding, ar_track_alvar_msgs::AlvarMarker& closest_marker);

    bool spotted(double younger_than, double min_confidence, ar_track_alvar_msgs::AlvarMarkers& excluding, ar_track_alvar_msgs::AlvarMarkers& spotted);
    bool closest(double younger_than, double min_confidence, ar_track_alvar_msgs::AlvarMarkers& excluding, ar_track_alvar_msgs::AlvarMarker& closest_marker);

    // check if the given id ar_marker is in the list. If yes, return the full ar marker data
    bool included(const uint32_t id, const ar_track_alvar_msgs::AlvarMarkers& v, ar_track_alvar_msgs::AlvarMarker* e = NULL);

    // Check if the given id is in the list of ar markers
    bool excluded(const uint32_t id, const ar_track_alvar_msgs::AlvarMarkers& v);

    // Confidence evaluation attributes
    double min_penalized_dist_;
    double max_reliable_dist_;
    double min_penalized_head_;
    double max_reliable_head_;
    double max_tracking_time_;  /**< Maximum time tacking a marker to ensure that it's a stable observation */
    double max_valid_d_inc_;    /**< Maximum valid distance increment per second to consider stable tracking */
    double max_valid_h_inc_;    /**< Maximum valid heading increment per second to consider stable tracking */
    double ar_tracker_freq_;    /**< AR tracker frequency; unless changed with setTrackerFreq, it must be the
                                     same value configured on ar_track_alvar node */

    std::vector<TrackedMarker> tracked_markers_;
    ar_track_alvar_msgs::AlvarMarkers spotted_markers_;
    ros::Subscriber    sub_ar_markers_;

//    tf::Transformer          tf_internal_;
//    tf::TransformListener    tf_listener_;
//    tf::TransformBroadcaster tf_brcaster_;
//    double                   tf_broadcast_freq_;  /**< Allows enabling tf broadcasting; mostly for debug */
};

} /* namespace yocs */

#endif /* AR_MARKERS_HPP_ */
