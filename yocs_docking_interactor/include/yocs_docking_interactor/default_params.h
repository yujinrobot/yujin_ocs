/*
  Default parameters for docking interactor 

  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#ifndef _YOCS_DOCKING_INTERACTOR_DEFAULT_PARAMS_H_
#define _YOCS_DOCKING_INTERACTOR_DEFAULT_PARAMS_H_

namespace yocs_docking_interactor {
  namespace DockingInteractorDefaultParam {
    const std::string AS_COMMAND = "docking_interactor";
    const std::string AC_MOVE_BASE = "move_base";
    const std::string AC_AUTO_DOCK = "dock_drive_action";
  }

  namespace DockingARTrackerDefaultParam {
    const std::string AR_TRACKER_SET_PARAM = "ar_track_alvar/set_parameters"; 
    const std::string SUB_GLOBAL_MARKERS = "global_markers";
  }
}
  

#endif
