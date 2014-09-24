
#include "yocs_docking_interactor/docking_interactor.hpp"


namespace yocs_docking_interactor {

DockingState DockingInteractor::startGlobalNaviToDock()
{
  geometry_msgs::PoseStamped pose;
  docking_ar_tracker_->getRobotDockPose(pose);
  sendMovebaseGoal(pose);
  return GLOBAL_DOCKING;
}

DockingState DockingInteractor::underGlobalNavigation()
{
  DockingState state = GLOBAL_DOCKING;
  double distance;
  std::string message;
  geometry_msgs::PoseStamped robot_pose;
  geometry_msgs::PoseStamped dock_pose;

  if(ac_move_base_->waitForResult(ros::Duration(0.5)) == false) {
    if(docking_ar_tracker_->isDockMarkerSpotted(dock_pose, message))
    {
      docking_ar_tracker_->getRobotPose(global_frame_, base_frame_, robot_pose);
      // 1. check distance
      distance = mtk::distance2D(robot_pose.pose, dock_pose.pose);
                                                                                                                                                                             
      //if it is less than relay distance, switch to local navigation 
      if(distance <= relay_on_marker_distance_) 
      {
        state = CANCEL_GLOBAL_NAVIGATION_AND_START_MARKER_DOCKING;
      }
      else {
        std::stringstream ss;
        ss << "Retunring to dock. It spotted Docking Marker but too far.[Global Navigation] Dist[" << distance << "] State[" << ac_move_base_->getState().toString() << "]";
        sendFeedback(yocs_msgs::DockingInteractorFeedback::INFO, ss.str());
      }
    }
    else {
      // send feedback?
      std::stringstream ss;
      ss << "Retunring to dock[Global Navigation] State[" << ac_move_base_->getState().toString() << "]";
      sendFeedback(yocs_msgs::DockingInteractorFeedback::INFO, ss.str());
    }
  }
  else {
    if(ac_move_base_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      // We cannot see the marker; that's normal if we are trying the odometry origin fallback solution (see
      // no-parameters dockInBase method). If not, probably we have a problem; switch on auto-docking anyway
      state = TERMINATE_GLOBAL_DOCKING;
    }
    else {
      state = TERMINATE_ON_ERROR;
    }
  }

  return state;
}

DockingState DockingInteractor::startSpottedMarkerBasedNaivgation()
{
  DockingState state = CANCEL_GLOBAL_NAVIGATION_AND_START_MARKER_DOCKING;
  std::string message;
  geometry_msgs::PoseStamped dock_pose;
  geometry_msgs::PoseStamped robot_target_pose;
  bool spotted;

  spotted = docking_ar_tracker_->isDockMarkerSpotted(dock_pose, message);

  if(spotted) // switch to spotted marker based navigation
  { 

    getRobotTargetPose(dock_pose, robot_target_pose);
    robot_target_pose.header.frame_id = dock_pose.header.frame_id;
    robot_target_pose.header.stamp = ros::Time::now();

    cancelAllGoals(*ac_move_base_); // before sending new goal, cancel old goal.
    sendMovebaseGoal(robot_target_pose);

    sendFeedback(yocs_msgs::DockingInteractorFeedback::INFO, "Spotted Marker based navigation has started");
    state = MARKER_DOCKING;
  }
  else 
  {
    sendFeedback(yocs_msgs::DockingInteractorFeedback::WARN, "Something went wrong.. it has lost docking marker while swithcing to marker based navigation. Back to global navigation");
    state = GLOBAL_DOCKING;
  }

  return state;
}

void DockingInteractor::getRobotTargetPose(const geometry_msgs::PoseStamped& dock_pose, geometry_msgs::PoseStamped& robot_target_pose)
{
  geometry_msgs::PoseStamped pose;
  tf::StampedTransform dock_pose_tf;

  mtk::pose2tf(dock_pose, dock_pose_tf);
  if(mtk::roll(dock_pose_tf) < -1.0)
  {
    tf::Transform flip(tf::createQuaternionFromRPY(0.0, 0.0, M_PI));
    dock_pose_tf *= flip;
  }

  // Compensate the vertical alignment of markers and put at ground level to adopt navistack goals format
  tf::Transform goal_gb(tf::createQuaternionFromYaw(tf::getYaw(dock_pose_tf.getRotation()) - M_PI/2.0),
  tf::Vector3(dock_pose_tf.getOrigin().x(), dock_pose_tf.getOrigin().y(), 0.0));

  // Project a pose relative to map frame in front of the docking base and heading to it
  // As before, half turn and translate to put goal at some distance in front of the marker
  tf::Transform in_front(tf::createQuaternionFromYaw(M_PI), tf::Vector3(relay_on_marker_distance_, 0.0, 0.0));
  goal_gb *= in_front;

  mtk::tf2pose(goal_gb, pose.pose); 

  robot_target_pose.pose = pose.pose;
}

DockingState DockingInteractor::underSpottedMarkerBasedNavigation()
{
  DockingState state;

  if(ac_move_base_->waitForResult(ros::Duration(0.5)) == true)
  {
    if(ac_move_base_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      state = TERMINATE_MARKER_DOCKING;
    }
    else {
      state = TERMINATE_ON_ERROR;
    }
  }
  return state;
}
}
