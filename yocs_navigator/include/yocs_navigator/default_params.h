/*
  Default parameters for navigator 

  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#ifndef _YOCS_NAVIGATOR_DEFAULT_PARAMS_H_
#define _YOCS_NAVIGATOR_DEFAULT_PARAMS_H_

namespace yocs_navigator {
  namespace BasicMoveControllerDefaultParam {
    const std::string PUB_CMD_VEL   = "cmd_vel";
    const std::string SUB_ODOM      = "odom";
  }

  namespace SemanticNavigatorDefaultParam {
    const std::string AS_NAVI       = "navigator";
    const std::string AC_MOVE_BASE  = "move_base";
    const std::string SUB_WAYPOINTLIST = "waypointlist";
    const std::string CLEAR_COSTMAP = "move_base/clear_costmaps";
  }
}
#endif 
