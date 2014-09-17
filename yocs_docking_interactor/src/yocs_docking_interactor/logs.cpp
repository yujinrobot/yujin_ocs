/*  
 *  logs.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/docking_interactor.hpp"

namespace yocs_docking_interactor {

void DockingInteractor::loginfo(const std::string& msg)
{
  ROS_INFO_STREAM_NAMED(ros::this_node::getName(), msg);
}

void DockingInteractor::logwarn(const std::string& msg)
{
  ROS_WARN_STREAM_NAMED(ros::this_node::getName(), msg);
}

}
