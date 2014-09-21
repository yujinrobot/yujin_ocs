/*  
 *  docking_interactor.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/docking_interactor.hpp"

namespace yocs_docking_interactor {

DockingInteractor::DockingInteractor(ros::NodeHandle& n) : nh_(n)
{
}

DockingInteractor::~DockingInteractor()
{
}

bool DockingInteractor::init()
{
  tracker_params_srv_  = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("ar_track_alvar/set_parameters");
  tracker_enabled_ = false;

  return true;
}

void DockingInteractor::spin()
{
  ros::Rate r(10);

  init();

  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}
}
