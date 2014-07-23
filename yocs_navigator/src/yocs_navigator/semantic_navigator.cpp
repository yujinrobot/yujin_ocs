/*
 *  semantic_navigator.hpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_navigator/semantic_navigator.hpp"

namespace yocs {

SemanticNavigator::SemanticNavigator(ros::NodeHandle& n) : nh_(n), basic_move_(n)
{
}

SemanticNavigator::~SemanticNavigator()
{
}

bool SemanticNavigator::init()
{
}

void SemanticNavigator::spin()
{
  ros::Rate r(2);

  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}

}
