/*
 *  docking_interactor.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */


#ifndef _YOCS_DOCKING_INTERACTOR_HPP_
#define _YOCS_DOCKING_INTERACTOR_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <yocs_msgs/DockingInteractorAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include "yocs_math_toolkit/geometry.hpp"

#include "yocs_docking_interactor/default_params.h"
#include "yocs_docking_interactor/ar_tracker.hpp"
#include "yocs_navigator/basic_move_controller.hpp"

namespace yocs_docking_interactor {
  
typedef actionlib::SimpleActionServer<yocs_msgs::DockingInteractorAction> DockingInteractorActionServer;
typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> KobukiAutoDockActionClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

typedef enum { START_GLOBAL_DOCKING, GLOBAL_DOCKING, CANCEL_GLOBAL_NAVIGATION_AND_START_MARKER_DOCKING, MARKER_DOCKING, TERMINATE_GLOBAL_DOCKING, TERMINATE_MARKER_DOCKING, TERMINATE_ON_ERROR} DockingState;

class DockingInteractor {
  public:
    DockingInteractor(ros::NodeHandle& n);
    DockingInteractor(ros::NodeHandle& n, const std::string as_command_topic);
    virtual ~DockingInteractor();

    bool init();
    void spin();
    void loginfo(const std::string& msg);
    void logwarn(const std::string& msg);

  protected:
    void processGoalCommand();
      void processCommand(yocs_msgs::DockingInteractorGoal::ConstPtr goal);
        void wakeUp(double distance);
        void registerDockMarker();
        void returnToDock();
          bool moveToDockFront(std::string& message);
          bool callAutoDock(std::string& message);
    void processPreemptCommand();

    void terminateCommand(const bool success, const std::string message); 
    void sendFeedback(const int level, const std::string message);

    void sendMovebaseGoal(const geometry_msgs::PoseStamped& pose);
    
    DockingState startGlobalNaviToDock();
    DockingState underGlobalNavigation();
    DockingState startSpottedMarkerBasedNaivgation();
      void getRobotTargetPose(const geometry_msgs::PoseStamped& dock_pose, geometry_msgs::PoseStamped& robot_target_pose);
    DockingState underSpottedMarkerBasedNavigation();
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pose_pub_;
    boost::shared_ptr<DockingARTracker> docking_ar_tracker_;
    boost::shared_ptr<DockingInteractorActionServer> as_command_;
    boost::shared_ptr<KobukiAutoDockActionClient> ac_auto_dock_;
    boost::shared_ptr<MoveBaseActionClient> ac_move_base_;
    boost::shared_ptr<yocs_navigator::BasicMoveController> bmc_;

    boost::thread command_process_thread_; 
    
    std::string as_command_topic_;

    bool command_in_progress_;
    double auto_dock_timeout_;
    double relay_on_marker_distance_;
    std::string global_frame_;
    std::string base_frame_;
};

template <typename T>
bool cancelAllGoals(actionlib::SimpleActionClient<T> & action_client, double timeout=2.0)
{
  actionlib::SimpleClientGoalState goal_state = action_client.getState();
  if ((goal_state != actionlib::SimpleClientGoalState::ACTIVE) &&
      (goal_state != actionlib::SimpleClientGoalState::PENDING) &&
      (goal_state != actionlib::SimpleClientGoalState::RECALLED) &&
      (goal_state != actionlib::SimpleClientGoalState::PREEMPTED))
  {
    // We cannot cancel a REJECTED, ABORTED, SUCCEEDED or LOST goal
//    ROS_WARN("Cannot cancel %s goal, as it has %s state!", acName(action_client), goal_state.toString().c_str());
    return true;
  }
                                                                                               
//  ROS_INFO("Canceling %s goal with %s state...", acName(action_client), goal_state.toString().c_str());
  action_client.cancelAllGoals();                                                              
  if (action_client.waitForResult(ros::Duration(timeout)) == false)                            
  {
//    ROS_WARN("Cancel %s goal didn't finish after %.2f seconds: %s", acName(action_client), timeout, goal_state.toString().c_str());
    return false;
  }

//  ROS_INFO("Cancel %s goal succeed. New state is %s", acName(action_client), goal_state.toString().c_str());
  return true;
}
}
#endif
