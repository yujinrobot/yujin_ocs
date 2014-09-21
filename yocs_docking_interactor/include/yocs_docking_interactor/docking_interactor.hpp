/*
 *  docking_interactor.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */


#ifndef _YOCS_DOCKING_INTERACTOR_HPP_
#define _YOCS_DOCKING_INTERACTOR_HPP_


#include <ros/ros.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <yocs_msgs/DockingInteractorAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "yocs_docking_interactor/default_params.h"

namespace yocs_docking_interactor {
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
    void enableTracker();
    void disableTracker();
      bool callTrackerService(bool value);

    void processGoalCommand();
      void processCommand(yocs_msgs::DockingInteractorGoal::ConstPtr goal);
    void processPreemptCommand();

    void terminateCommand(bool success, const std::string message); 
  
  private:
    ros::NodeHandle nh_;
    ros::ServiceClient tracker_params_srv_;
    actionlib::SimpleActionServer<yocs_msgs::DockingInteractorAction> as_command_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base_;
  
    bool command_in_progress_;
    bool tracker_enabled_;

    boost::thread command_process_thread_; 

};
}
#endif
