/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <yocs_msgs/NavigateToAction.h>
#include <yocs_msgs/TableList.h>
#include <geometry_msgs/PoseStamped.h>

#include "yocs_navigator/default_params.h"
#include "yocs_navigator/basic_move_controller.hpp"

#ifndef _YOCS_SEMANTIC_NAVIGATOR_HPP_
#define _YOCS_SEMANTIC_NAVIGATOR_HPP_

namespace yocs {

class SemanticNavigator {
  public:
    SemanticNavigator(ros::NodeHandle& n);
    SemanticNavigator(ros::NodeHandle& n, const std::string& as_navigator_topic, const std::string& sub_tablelist_topic);
    virtual ~SemanticNavigator();
    bool init();
    void spin();
    void loginfo(const std::string& msg);
    void logwarn(const std::string& msg);
  protected:
    void processTableList(const yocs_msgs::TableList::ConstPtr& msg);
    void processNavigateToGoal();
    void processPreemptNavigateTo();

    void processNavigation(yocs_msgs::NavigateToGoal::ConstPtr goal);

    void terminateNavigation(bool success, const std::string message);
    void feedbackNavigation(const int status, const double distance, const std::string message);
    bool getGoalLocationTable(const std::string location, yocs_msgs::Table& table);

    void goOn(const yocs_msgs::Table table, const double distance, const int num_retry, const double timeout);
    void goNear(const yocs_msgs::Table table, const double distance, const int num_retry, const double timeout);

    void processMoveBaseFeedback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback, const geometry_msgs::PoseStamped& target);

  private:
    ros::NodeHandle nh_;

    BasicMoveController basic_move_;
    ros::Subscriber                                               sub_tablelist_;
    actionlib::SimpleActionServer<yocs_msgs::NavigateToAction>    as_navi_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base_;

    std::string as_navigator_topic_;
    std::string sub_tablelist_topic_;
    std::string global_frame_;

    yocs_msgs::TableList tablelist_;
    double distance_to_goal_;
    bool table_received_;
    bool navigation_in_progress_;
    boost::thread order_process_thread_;
    
};
}

#endif
