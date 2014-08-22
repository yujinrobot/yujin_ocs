/*
   Way point Provider

   highly inspired by yocs_waypoint_navi written by Jorge Santos

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */


#ifndef _YOCS_WAYPOINT_PROVIDER_HPP_
#define _YOCS_WAYPOINT_PROVIDER_HPP_

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <yocs_msgs/Trajectory.h>
#include <yocs_msgs/TrajectoryList.h>
#include <yocs_msgs/WaypointListService.h>
#include <yocs_msgs/WaypointList.h>

/*
   It simply parses waypoints in yaml file and provides latch topic

   Future improvements
    - Utilise database for persistent waypoint storing
    - add/remove waypoints via srv or pub/sub
 */

namespace yocs {
  class WaypointProvider {
    public:
      WaypointProvider(ros::NodeHandle& n, yocs_msgs::WaypointList& wps, yocs_msgs::TrajectoryList& trajs);
      ~WaypointProvider();

      void spin();
    protected:
      void init();
      bool processWaypointsService(yocs_msgs::WaypointListService::Request& request,
                                   yocs_msgs::WaypointListService::Response& response);
      void generateWaypointMarkers(const yocs_msgs::WaypointList& wps, visualization_msgs::MarkerArray& wp_markers);
      void generateTrajectoryMarkers(const yocs_msgs::TrajectoryList& trajs,
                                     visualization_msgs::MarkerArray& traj_markers);
      void createMarkerArrow(const int i, const yocs_msgs::Waypoint& wp, visualization_msgs::Marker& marker);
      void createMarkerLineStrip(const int i, const yocs_msgs::Trajectory& traj, visualization_msgs::Marker& marker);
      void createMarkerLabel(const std::string frame_id,
                             const int id,
                             const std::string ns,
                             const std::string wp_name,
                             const geometry_msgs::Pose wp_pose,
                             visualization_msgs::Marker& marker);

    private:
      bool initialized_;
      ros::NodeHandle nh_;
      ros::Publisher waypoints_pub_, trajectories_pub_;
      ros::Publisher waypoints_marker_pub_, trajectory_marker_pub_;
      ros::ServiceServer waypoints_srv_;

      yocs_msgs::WaypointList waypoints_;
      yocs_msgs::TrajectoryList trajectories_;
      visualization_msgs::MarkerArray waypoint_markers_, trajectory_markers_;

      int marker_index_;
      int label_index_;
  };
}

#endif // _YOCS_WAYPOINT_PROVIDER_HPP_
