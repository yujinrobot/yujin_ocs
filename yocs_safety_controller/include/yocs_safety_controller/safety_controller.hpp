/**
* License: BSD
* https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
*/

#ifndef YOCS_SAFETY_CONTROLLER_HPP_
#define YOCS_SAFETY_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Range.h>
#include <yocs_controllers/default_controller.hpp>

namespace yocs_safety_controller
{

/**
 * @ brief Keeps track of ranger readings to stop your robot, if necessary
 *
 * The SafetyController keeps track of ranger readings. If obstacles get too close, your robot is stopped.
 * You can also activate the move backward option, if you like your robot to reverse a bit after stopping.
 *
 * This controller can be enabled/disabled.
 */
class YOCSSafetyController : public yocs::Controller
{
public:
  YOCSSafetyController(ros::NodeHandle& nh, std::string& name) : Controller(),
                                                                 nh_(nh),
                                                                 name_(name){};
  ~YOCSSafetyController(){};

  /**
   * Set-up necessary publishers/subscribers and variables
   * @return true, if successful
   */
  bool init()
  {
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &YOCSSafetyController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &YOCSSafetyController::disableCB, this);
    ranger_subscriber_ = nh_.subscribe("rangers", 10, &YOCSSafetyController::rangerCB, this);
    velocity_command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel", 10);
    return true;
  };

  /**
   * @ brief Checks safety states and publishes velocity commands when necessary
   */
  void spinOnce();

private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_, ranger_subscriber_;
  ros::Publisher controller_state_publisher_, velocity_command_publisher_;
  geometry_msgs::TwistPtr cmd_vel_msg_; // velocity command

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void enableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief ROS logging output for disabling the controller
   * @param msg incoming topic message
   */
  void disableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief Keeps track of rangers
   * @param msg incoming topic message
   */
  void rangerCB(const sensor_msgs::RangeConstPtr msg);
};

} // namespace yocs_safety_controller

#endif // YOCS_SAFETY_CONTROLLER_HPP_
