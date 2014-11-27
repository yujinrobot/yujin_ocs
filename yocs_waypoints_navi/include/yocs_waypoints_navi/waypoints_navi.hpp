/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#ifndef YOCS_WAYPOINT_NAVI_HPP_
#define YOCS_WAYPOINT_NAVI_HPP_

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>
#include <yocs_msgs/NavigationControl.h>
#include <yocs_msgs/NavigationControlStatus.h>
#include <yocs_msgs/TrajectoryList.h>
#include <yocs_msgs/WaypointList.h>


namespace yocs
{

/*
 * TODO
 *  * think about how to best visualise the waypoint(s)/trajectory(ies) which are being executed
 *  * add RViz interface to yocs_waypoint_provider
 */

class WaypointsGoalNode
{
public:
  WaypointsGoalNode();
  ~WaypointsGoalNode();

  bool init();

  void waypointsCB(const yocs_msgs::WaypointList::ConstPtr& wps);

  void trajectoriesCB(const yocs_msgs::TrajectoryList::ConstPtr& trajs);

  void navCtrlCB(const yocs_msgs::NavigationControl::ConstPtr& nav_ctrl);

  void spin();

private:
  const geometry_msgs::PoseStamped NOWHERE;

  enum { NONE = 0,
         GOAL,
         LOOP
       } mode_;

  enum { IDLE = 0,
         START,
         ACTIVE,
         COMPLETED
       } state_;

  double      frequency_;
  double      close_enough_;
  double      goal_timeout_;
  std::string robot_frame_;
  std::string world_frame_;

  std::vector<geometry_msgs::PoseStamped>           waypoints_;
  std::vector<geometry_msgs::PoseStamped>::iterator waypoints_it_;

  geometry_msgs::PoseStamped goal_;

  yocs_msgs::WaypointList wp_list_;
  yocs_msgs::TrajectoryList traj_list_;

  tf::TransformListener tf_listener_;
  ros::Subscriber    waypoints_sub_;
  ros::Subscriber    trajectories_sub_;

  ros::Subscriber nav_ctrl_sub_;
  ros::Publisher  status_pub_;
  bool idle_status_update_sent_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;

  bool cancelAllGoals(double timeout = 2.0);

  void resetWaypoints();

  bool equals(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b);

  bool equals(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

  void publishStatusUpdate(const uint8_t& status);
};

} // namespace yocs

#endif /* YOCS_WAYPOINT_NAVI_HPP_ */
