/*  
 *  docking_interactor.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/docking_interactor.hpp"

namespace yocs_docking_interactor {

DockingInteractor::DockingInteractor(ros::NodeHandle& n) 
: nh_(n)
{
  as_command_topic_ = DockingInteractorDefaultParam::AS_COMMAND;
}

DockingInteractor::DockingInteractor(ros::NodeHandle& n, const std::string as_command_topic) 
: nh_(n)
{
  as_command_topic_ = as_command_topic;
}

DockingInteractor::~DockingInteractor()
{
}

bool DockingInteractor::init()
{
  ros::NodeHandle pnh("~");
  pnh.param("global_frame", global_frame_, std::string("map"));
  pnh.param("base_frame", base_frame_, std::string("base_footprint"));
  pnh.param("auto_dock_timeout", auto_dock_timeout_, 90.0);
  pnh.param("relay_on_marker_distance", relay_on_marker_distance_, 1.0);

  // variables
  command_in_progress_ = false;
  
  // global marker subscriber
  loginfo("Wait for docking ar tracker");
  docking_ar_tracker_.reset(new DockingARTracker(nh_));
  while(ros::ok() && docking_ar_tracker_->isReady()) {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  goal_pose_pub_ = nh_.advertise <geometry_msgs::PoseStamped> ("goal_pose", 1, true);

  // basic move controller
  loginfo("instantiate basic move controller");
  bmc_.reset(new yocs_navigator::BasicMoveController(nh_));

  // audo dock
  loginfo("Wait for auto dock");
  ac_auto_dock_.reset(new KobukiAutoDockActionClient(nh_, DockingInteractorDefaultParam::AC_AUTO_DOCK, true));
  ac_auto_dock_->waitForServer();

  // move base
  loginfo("Wait for movebase");
  ac_move_base_.reset(new MoveBaseActionClient(nh_, DockingInteractorDefaultParam::AC_MOVE_BASE, true)); 
  ac_move_base_->waitForServer();

  loginfo("Initialised");
  as_command_.reset(new DockingInteractorActionServer(nh_, as_command_topic_, false));
  as_command_->registerGoalCallback(boost::bind(&DockingInteractor::processGoalCommand, this));
  as_command_->registerPreemptCallback(boost::bind(&DockingInteractor::processPreemptCommand, this));
  as_command_->start();

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

void DockingInteractor::processGoalCommand()
{
  if(command_in_progress_)
  {
    as_command_->acceptNewGoal();
    terminateCommand(false, "Command under progress yet.. Ignoring new command");
    return;
  }

  command_in_progress_ = true;
  command_process_thread_ = boost::thread(&DockingInteractor::processCommand, this, as_command_->acceptNewGoal());
}

void DockingInteractor::processPreemptCommand()
{
  logwarn("Command Preemption Requested");
  as_command_->setPreempted();
}

}
