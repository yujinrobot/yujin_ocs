/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "yocs_navigator/default_params.h"
#include "yocs_navigator/basic_move_controller.hpp"

namespace yocs {

class SemanticNavigator {
  public:
    SemanticNavigator(ros::NodeHandle& n);
    virtual ~SemanticNavigator();
    bool init();
    void spin();
    
    void loginfo(std::string& msg);

//  protected:
  private:
    ros::NodeHandle nh_;
    bool initialized_;

    BasicMoveController basic_move_;
};
}
