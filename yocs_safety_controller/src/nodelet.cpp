/**
* License: BSD
* https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
*/

#include <ecl/threads/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "yocs_safety_controller/safety_controller.hpp"


namespace yocs_safety_controller
{

class SafetyControllerNodelet : public nodelet::Nodelet
{
public:
  SafetyControllerNodelet() : shutdown_requested_(false) { };
  ~SafetyControllerNodelet()
  {
    NODELET_DEBUG_STREAM("Waiting for update thread to finish.");
    shutdown_requested_ = true;
    update_thread_.join();
  }
  virtual void onInit()
  {
    ros::NodeHandle nh = this->getPrivateNodeHandle();
    // resolve node(let) name
    std::string name = nh.getUnresolvedNamespace();
    int pos = name.find_last_of('/');
    name = name.substr(pos + 1);
    NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
    controller_.reset(new SafetyController(nh, name));
    if (controller_->init())
    {
      NODELET_INFO_STREAM("Safety controller initialised. Spinning up update thread ... [" << name << "]");
      update_thread_.start(&SafetyControllerNodelet::update, *this);
      NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
    }
    else
    {
      NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
    }
  }
private:
  void update()
  {
    ros::Rate spin_rate(10);
    controller_->enable(); // enable the controller when loading the nodelet
    while (! shutdown_requested_ && ros::ok())
    {
      controller_->spinOnce();
      spin_rate.sleep();
    }
  }

  boost::shared_ptr<SafetyController> controller_;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
};

} // namespace yocs_safety_controller

PLUGINLIB_EXPORT_CLASS(yocs_safety_controller::SafetyControllerNodelet,
                       nodelet::Nodelet);
