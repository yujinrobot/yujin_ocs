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
 * Note: Currently only supports fixed-distance rangers.
 *
 * TODO: Implement logic to handle variable-distance rangers.
 *
 * This controller can be enabled/disabled.
 */
class SafetyController : public yocs::Controller
{
public:
  SafetyController(ros::NodeHandle& nh_priv, const std::string& name);
  ~SafetyController(){};

  /**
   * Set-up necessary publishers/subscribers and variables
   * @return true, if successful
   */
  bool init();

  /**
   * @ brief Checks safety states and publishes velocity commands when necessary
   */
  void spinOnce();


protected:
  ros::NodeHandle nh_priv_;
  std::string name_;
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_, ranger_subscriber_;
  ros::Publisher controller_state_publisher_, velocity_command_publisher_;
  geometry_msgs::TwistPtr cmd_vel_msg_; // velocity command

  /**
   * Indicates whether an obstacle has been detected
   */
  bool obstacle_detected_;
  /**
   * Indicates whether to reverse after an obstacle has been detected
   */
  bool reverse_;
  /**
   * Reversing state
   */
  bool reversing_;
  /**
   * the time robot started reversing
   */
  ros::Time reversing_start_;
  /**
   * the total distance the robot will reverse in [m](configurable)
   */
  double reversing_distance_;
  /**
   * how fast the robot should move, when reversing in [m/s] (configurable)
   */
  double reversing_velocity_;
  /**
   * max. duration the robot will reverse in [s]
   * automatically calculated based on reversing_distance_ and reversing_speed_
   */
  ros::Duration reversing_duration_;

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
