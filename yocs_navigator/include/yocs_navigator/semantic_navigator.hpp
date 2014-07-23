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
  protected:
    void processTableList(const yocs_msgs::TableList::ConstPtr& msg);
    void processNavigateTo();
    void processPreemptNavigateTo();
  private:
    ros::NodeHandle nh_;

    BasicMoveController basic_move_;
    ros::Subscriber                                               sub_tablelist_;
    actionlib::SimpleActionServer<yocs_msgs::NavigateToAction>   as_navi_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base_;

    std::string as_navigator_topic_;
    std::string sub_tablelist_topic_;

    yocs_msgs::TableList tablelist_;
    bool table_received_;
};
}

#endif
