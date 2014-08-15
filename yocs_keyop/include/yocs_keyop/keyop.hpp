/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#ifndef YOCS_KEYOP_KEYOP_HPP_
#define YOCS_KEYOP_KEYOP_HPP_

#include <ros/ros.h>
#include <termios.h> // for keyboard input
#include <ecl/threads.hpp>
#include <geometry_msgs/Twist.h>  // for velocity commands
#include <geometry_msgs/TwistStamped.h>  // for velocity commands


namespace yocs_keyop
{
/**
 * @brief Keyboard remote control for your robot
 *
 */
class KeyOp
{
public:
  /*********************
   ** C&D
   **********************/
  KeyOp();
  ~KeyOp();
  bool init();

  /*********************
   ** Runtime
   **********************/
  void spin();

private:
  ros::Publisher velocity_publisher_, enable_motors_publisher_, disable_motors_publisher_;
  bool last_zero_vel_sent_;
  bool accept_incoming_;
  bool power_status_;
  bool wait_for_connection_;
  geometry_msgs::TwistPtr cmd_;
  geometry_msgs::TwistStampedPtr cmd_stamped_;
  double linear_vel_step_, linear_vel_max_;
  double angular_vel_step_, angular_vel_max_;
  std::string name_;

  /*********************
   ** Commands
   **********************/
  void enable();
  void disable();
  void incrementLinearVelocity();
  void decrementLinearVelocity();
  void incrementAngularVelocity();
  void decrementAngularVelocity();
  void resetVelocity();

  /*********************
   ** Keylogging
   **********************/

  void keyboardInputLoop();
  void processKeyboardInput(char c);
  void restoreTerminal();
  bool quit_requested_;
  int key_file_descriptor_;
  struct termios original_terminal_state_;
  ecl::Thread thread_;
};

} // namespace yocs_keyop

#endif /* YOCS_KEYOP_KEYOP_HPP_ */
