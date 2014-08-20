/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_keyop/keyop.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "yocs_keyop");
  yocs_keyop::KeyOp keyop;
  if (keyop.init())
  {
    keyop.spin();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't initialise KeyOp!");
  }

  ROS_INFO_STREAM("Program exiting");
  return 0;
}
