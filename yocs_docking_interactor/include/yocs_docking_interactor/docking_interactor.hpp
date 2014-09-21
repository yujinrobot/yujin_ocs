/*
 *  docking_interactor.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */


#ifndef _YOCS_DOCKING_INTERACTOR_HPP_
#define _YOCS_DOCKING_INTERACTOR_HPP_


#include <ros/ros.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/Reconfigure.h>


namespace yocs_docking_interactor {
class DockingInteractor {
  public:
    DockingInteractor(ros::NodeHandle& n);
    virtual ~DockingInteractor();

    bool init();
    void spin();
    void loginfo(const std::string& msg);
    void logwarn(const std::string& msg);

  protected:
    void enableTracker();
    void disableTracker();
      bool callTrackerService(bool value);
  
  private:
    ros::NodeHandle nh_;
    ros::ServiceClient tracker_params_srv_;
    
    bool tracker_enabled_;
};
}
#endif
