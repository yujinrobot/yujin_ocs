/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <yocs_msgs/ApproachList.h>
#include <yocs_msgs/NavigateToAction.h>
#include <yocs_msgs/TrajectoryList.h>
#include <yocs_msgs/WaypointList.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "yocs_navigator/default_params.h"
#include "yocs_navigator/basic_move_controller.hpp"

#ifndef _YOCS_SEMANTIC_NAVIGATOR_HPP_
#define _YOCS_SEMANTIC_NAVIGATOR_HPP_

namespace yocs_navigator
{

class SemanticNavigator
{
  public:
    SemanticNavigator(ros::NodeHandle& n);
    virtual ~SemanticNavigator();
    bool init();
    void spin();
    void loginfo(const std::string& msg);
    void logwarn(const std::string& msg);

  private:
    void processNavigateToGoal();
    void processPreemptNavigateTo();

    void processNavigation(yocs_msgs::NavigateToGoal::ConstPtr goal);

    void terminateNavigation(bool success, const std::string message);
    void feedbackNavigation(const int status, const double distance, const double remain_time, const std::string message);
    bool findTarget(const std::string& target_name,
                    const yocs_msgs::WaypointList& stored_wps,
                    const yocs_msgs::TrajectoryList& stored_trajs,
                    std::vector<geometry_msgs::PoseStamped>& target_wps,
                    std::vector<geometry_msgs::PoseStamped>::iterator& target_wps_it);

    void goOn(const geometry_msgs::PoseStamped& target,
              const double& in_distance,
              const int& num_retry,
              const double& timeout);
    void goNear(const geometry_msgs::PoseStamped& target,
                const double& in_distance,
                const int& num_retry,
                const double& timeout);

    void waitForMoveBase(int& move_base_result, const ros::Time& start_time, const double timeout);
    void determineNavigationState(int& navi_result, const int move_base_result, const actionlib::SimpleClientGoalState  move_base_state);
    void nextState(bool& retry,bool& final_result,std::string& message, const int navi_result, const ros::Time started_time);

    void processMoveBaseFeedback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback, const geometry_msgs::PoseStamped& target);

    bool cancelMoveBaseGoal();

    bool clearCostmaps();

    void approachesCB(const yocs_msgs::ApproachList::ConstPtr& apprs);
    void trajectoriesCB(const yocs_msgs::TrajectoryList::ConstPtr& trajs);
    void waypointsCB(const yocs_msgs::WaypointList::ConstPtr& wps);

    ros::NodeHandle nh_;
    ros::Rate r_;

//    BasicMoveController basic_move_;
    ros::Subscriber waypoints_sub_, trajectories_sub_, approaches_sub_;
    boost::shared_ptr<actionlib::SimpleActionServer<yocs_msgs::NavigateToAction> > as_navi_;
    boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > ac_move_base_;

    std::string global_frame_;

    /**
     * List of approaches
     */
    yocs_msgs::ApproachList apprs_;
    /**
     * List of trajectories
     */
    yocs_msgs::TrajectoryList trajs_;
    /**
     * List of way points
     */
    yocs_msgs::WaypointList wps_;
    /**
     * List of way points to reach the goal
     */
    std::vector<geometry_msgs::PoseStamped> target_wps_;
    /**
     * Iterator for the target way points
     */
    std::vector<geometry_msgs::PoseStamped>::iterator target_wps_it_;
    /**
     * Distance to goal
     */
    double distance_to_goal_;
    /**
     * Flag indicating the receipt of a list of approaches
     */
    bool apprs_received_;
    /**
     * Flag indicating the receipt of a list of way points
     */
    bool wps_received_;
    bool navigation_in_progress_;
    boost::thread order_process_thread_;

    static const int NAVI_IN_PROGRESS =14;
    static const int NAVI_SUCCESS     =15;
    static const int NAVI_RETRY       =16;
    static const int NAVI_FAILED      =17;
    static const int NAVI_TIMEOUT     =18;
    static const int NAVI_UNKNOWN     =19;
};

} // namespace

#endif // _YOCS_SEMANTIC_NAVIGATOR_HPP_
