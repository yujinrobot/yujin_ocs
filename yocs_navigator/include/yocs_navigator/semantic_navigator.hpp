/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <yocs_msgs/NavigateToAction.h>
#include <yocs_msgs/WaypointList.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

#include "yocs_navigator/default_params.h"
#include "yocs_navigator/basic_move_controller.hpp"

#ifndef _YOCS_SEMANTIC_NAVIGATOR_HPP_
#define _YOCS_SEMANTIC_NAVIGATOR_HPP_

namespace yocs_navigator {

class SemanticNavigator {
  public:
    SemanticNavigator(ros::NodeHandle& n);
    SemanticNavigator(ros::NodeHandle& n, const std::string& as_navigator_topic, const std::string& sub_waypointlist_topic);
    virtual ~SemanticNavigator();
    bool init();
    void spin();
    void loginfo(const std::string& msg);
    void logwarn(const std::string& msg);
  protected:
    void processWaypointList(const yocs_msgs::WaypointList::ConstPtr& msg);
    void processNavigateToGoal();
    void processPreemptNavigateTo();

    void processNavigation(yocs_msgs::NavigateToGoal::ConstPtr goal);

    void terminateNavigation(bool success, const std::string message);
    void feedbackNavigation(const int status, const double distance, const double remain_time, const std::string message);
    bool getGoalLocation(const std::string location, yocs_msgs::Waypoint& waypoint);

    void goOn(const yocs_msgs::Waypoint waypoint, const double in_distance, const int num_retry, const double timeout);
      void waitForMoveBase(int& move_base_result, const ros::Time& start_time, const double timeout);
      void determineNavigationState(int& navi_result, const int move_base_result, const actionlib::SimpleClientGoalState  move_base_state);
      void nextState(bool& retry,bool& final_result,std::string& message, const int navi_result, const ros::Time started_time);
    void goNear(const yocs_msgs::Waypoint waypoint, const double in_distance, const int num_retry, const double timeout);

    void processMoveBaseFeedback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback, const geometry_msgs::PoseStamped& target);


    bool cancelMoveBaseGoal();

    bool clearCostmaps();
  

  private:
    ros::NodeHandle nh_;

    BasicMoveController basic_move_;
    ros::Subscriber                                               sub_waypointlist_;
    actionlib::SimpleActionServer<yocs_msgs::NavigateToAction>    as_navi_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base_;

    std::string sub_waypointlist_topic_;
    std::string global_frame_;

    yocs_msgs::WaypointList waypointlist_;
    double distance_to_goal_;
    bool waypoint_received_;
    bool navigation_in_progress_;
    boost::thread order_process_thread_;

    static const int NAVI_IN_PROGRESS =14;
    static const int NAVI_SUCCESS     =15;
    static const int NAVI_RETRY       =16;
    static const int NAVI_FAILED      =17;
    static const int NAVI_TIMEOUT     =18;
    static const int NAVI_UNKNOWN     =19;
};
}

#endif
