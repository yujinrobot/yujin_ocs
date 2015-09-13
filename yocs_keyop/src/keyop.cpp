/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include <ecl/exceptions.hpp>
#include <ecl/time.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "yocs_keyop/keyop.hpp"

#define KEYCODE_RIGHT 67 // 0x43
#define KEYCODE_LEFT  68 // 0x44
#define KEYCODE_UP    65 // 0x41
#define KEYCODE_DOWN  66 // 0x42
#define KEYCODE_SPACE 32 // 0x20


namespace yocs_keyop
{

KeyOp::KeyOp() : last_zero_vel_sent_(true), // avoid zero-vel messages from the beginning
                 accept_incoming_(true),
                 power_status_(false),
                 wait_for_connection_(true),
                 cmd_(new geometry_msgs::Twist()),
                 cmd_stamped_(new geometry_msgs::TwistStamped()),
                 linear_vel_step_(0.1),
                 linear_vel_max_(3.4),
                 angular_vel_step_(0.02),
                 angular_vel_max_(1.2),
                 quit_requested_(false),
                 key_file_descriptor_(0)
{
  tcgetattr(key_file_descriptor_, &original_terminal_state_); // get terminal properties
}

KeyOp::~KeyOp()
{
  tcsetattr(key_file_descriptor_, TCSANOW, &original_terminal_state_);
}

/**
 * @brief Initialises the node.
 */
bool KeyOp::init()
{
  ros::NodeHandle nh("~");

  name_ = nh.getUnresolvedNamespace();

  /*********************
   ** Parameters
   **********************/
  nh.getParam("linear_vel_step", linear_vel_step_);
  nh.getParam("linear_vel_max", linear_vel_max_);
  nh.getParam("angular_vel_step", angular_vel_step_);
  nh.getParam("angular_vel_max", angular_vel_max_);
  nh.getParam("wait_for_connection", wait_for_connection_);

  ROS_INFO_STREAM("KeyOp : using linear  vel step [" << linear_vel_step_ << "].");
  ROS_INFO_STREAM("KeyOp : using linear  vel max  [" << linear_vel_max_ << "].");
  ROS_INFO_STREAM("KeyOp : using angular vel step [" << angular_vel_step_ << "].");
  ROS_INFO_STREAM("KeyOp : using angular vel max  [" << angular_vel_max_ << "].");

  /*********************
   ** Publishers
   **********************/
  velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  enable_motors_publisher_ = nh.advertise<std_msgs::String>("enable_motors", 1);
  disable_motors_publisher_ = nh.advertise<std_msgs::String>("disable_motors", 1);

  /*********************
   ** Velocities
   **********************/
  cmd_->linear.x = 0.0;
  cmd_->linear.y = 0.0;
  cmd_->linear.z = 0.0;
  cmd_->angular.x = 0.0;
  cmd_->angular.y = 0.0;
  cmd_->angular.z = 0.0;

  /*********************
   ** Wait for connection
   **********************/
  bool connected = false;
  if (!wait_for_connection_)
  {
    connected = true;
  }
  ecl::MilliSleep millisleep;
  int count = 0;
  while (!connected)
  {
    if ((enable_motors_publisher_.getNumSubscribers() > 0) &&
        (disable_motors_publisher_.getNumSubscribers() > 0))
    {
      connected = true;
      break;
    }
    if (count == 6)
    {
      connected = false;
      break;
    }
    else
    {
      ROS_WARN_STREAM("KeyOp: Could not connect, trying again after 500ms...");
      try
      {
        millisleep(500);
      }
      catch (ecl::StandardException& e)
      {
        ROS_ERROR_STREAM("Waiting has been interrupted.");
        ROS_DEBUG_STREAM(e.what());
        return false;
      }
      ++count;
    }
  }
  if (!connected)
  {
    ROS_ERROR("KeyOp: Could not connect.");
    ROS_ERROR("KeyOp: Check remappings for enable/disable topics.");
  }
  else
  {
    std_msgs::String msg;
    msg.data = "all";
    enable_motors_publisher_.publish(msg);
    ROS_INFO("KeyOp: connected.");
    power_status_ = true;
  }

  // start keyboard input thread
  thread_.start(&KeyOp::keyboardInputLoop, *this);
  return true;
}

/*****************************************************************************
 ** Implementation [Spin]
 *****************************************************************************/

/**
 * @brief Worker thread loop; sends current velocity command at a fixed rate.
 *
 * It also process ros functions as well as aborting when requested.
 */
void KeyOp::spin()
{
  ros::Rate loop_rate(10);

  while (!quit_requested_ && ros::ok())
  {
    // Avoid spamming robot with continuous zero-velocity messages
    if ((cmd_->linear.x  != 0.0) || (cmd_->linear.y  != 0.0) || (cmd_->linear.z  != 0.0) ||
        (cmd_->angular.x != 0.0) || (cmd_->angular.y != 0.0) || (cmd_->angular.z != 0.0))
    {
      velocity_publisher_.publish(cmd_);
      last_zero_vel_sent_ = false;
    }
    else if (last_zero_vel_sent_ == false)
    {
      velocity_publisher_.publish(cmd_);
      last_zero_vel_sent_ = true;
    }
    accept_incoming_ = true;
    ros::spinOnce();
    loop_rate.sleep();
  }
  if (quit_requested_)
  { // ros node is still ok, send a disable command
    disable();
  }
  else
  {
    // just in case we got here not via a keyboard quit request
    quit_requested_ = true;
    thread_.cancel();
  }
  thread_.join();
}

/*****************************************************************************
 ** Implementation [Keyboard]
 *****************************************************************************/

/**
 * @brief The worker thread function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */
void KeyOp::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state_, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor_, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Forward/back arrows : linear velocity incr/decr.");
  puts("Right/left arrows : angular velocity incr/decr.");
  puts("Spacebar : reset linear/angular velocities.");
  puts("d : disable motors.");
  puts("e : enable motors.");
  puts("q : quit.");
  char c;
  while (!quit_requested_)
  {
    if (read(key_file_descriptor_, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    processKeyboardInput(c);
  }
}

/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void KeyOp::processKeyboardInput(char c)
{
  /*
   * Arrow keys are a bit special, they are escape characters - meaning they
   * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
   * ignore the esc-[ and just parse the last one. So long as we avoid using
   * the last one for its actual purpose (e.g. left arrow corresponds to
   * esc-[-D) we can keep the parsing simple.
   */
  switch (c)
  {
    case KEYCODE_LEFT:
    {
      incrementAngularVelocity();
      break;
    }
    case KEYCODE_RIGHT:
    {
      decrementAngularVelocity();
      break;
    }
    case KEYCODE_UP:
    {
      incrementLinearVelocity();
      break;
    }
    case KEYCODE_DOWN:
    {
      decrementLinearVelocity();
      break;
    }
    case KEYCODE_SPACE:
    {
      resetVelocity();
      break;
    }
    case 'q':
    {
      quit_requested_ = true;
      break;
    }
    case 'd':
    {
      disable();
      break;
    }
    case 'e':
    {
      enable();
      break;
    }
    default:
    {
      break;
    }
  }
}

/*****************************************************************************
 ** Implementation [Commands]
 *****************************************************************************/
/**
 * @brief Disables commands to the navigation system.
 *
 * This does the following things:
 *
 * - Disables power to the navigation motors (via device_manager).
 * @param msg
 */
void KeyOp::disable()
{
  accept_incoming_ = false;
  cmd_->linear.x = 0.0;
  cmd_->angular.z = 0.0;
  velocity_publisher_.publish(cmd_);

  if (power_status_)
  {
    ROS_INFO_STREAM("KeyOp: die, die, die (disabling power to the device's motor system).");
    std_msgs::String msg;
    msg.data = "all";
    disable_motors_publisher_.publish(msg);
    power_status_ = false;
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: Motor system has already been powered down.");
  }
}

/**
 * @brief Reset/re-enable the navigation system.
 *
 * - resets the command velocities.
 * - reenable power if not enabled.
 */
void KeyOp::enable()
{
  accept_incoming_ = false;
  cmd_->linear.x = 0.0;
  cmd_->angular.z = 0.0;
  velocity_publisher_.publish(cmd_);

  if (!power_status_)
  {
    ROS_INFO_STREAM("KeyOp: Enabling power to the device subsystem.");
    std_msgs::String msg;
    msg.data = "all";
    enable_motors_publisher_.publish(msg);
    power_status_ = true;
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: Device has already been powered up.");
  }
}

/**
 * @brief If not already maxxed, increment the command velocities..
 */
void KeyOp::incrementLinearVelocity()
{
  if (power_status_)
  {
    if (cmd_->linear.x <= linear_vel_max_)
    {
      cmd_->linear.x += linear_vel_step_;
    }
    ROS_INFO_STREAM("KeyOp: linear  velocity incremented [" << cmd_->linear.x << "|" << cmd_->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already minned, decrement the linear velocities..
 */
void KeyOp::decrementLinearVelocity()
{
  if (power_status_)
  {
    if (cmd_->linear.x >= -linear_vel_max_)
    {
      cmd_->linear.x -= linear_vel_step_;
    }
    ROS_INFO_STREAM("KeyOp: linear  velocity decremented [" << cmd_->linear.x << "|" << cmd_->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already maxxed, increment the angular velocities..
 */
void KeyOp::incrementAngularVelocity()
{
  if (power_status_)
  {
    if (cmd_->angular.z <= angular_vel_max_)
    {
      cmd_->angular.z += angular_vel_step_;
    }
    ROS_INFO_STREAM("KeyOp: angular velocity incremented [" << cmd_->linear.x << "|" << cmd_->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already mined, decrement the angular velocities..
 */
void KeyOp::decrementAngularVelocity()
{
  if (power_status_)
  {
    if (cmd_->angular.z >= -angular_vel_max_)
    {
      cmd_->angular.z -= angular_vel_step_;
    }
    ROS_INFO_STREAM("KeyOp: angular velocity decremented [" << cmd_->linear.x << "|" << cmd_->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void KeyOp::resetVelocity()
{
  if (power_status_)
  {
    cmd_->angular.z = 0.0;
    cmd_->linear.x = 0.0;
    ROS_INFO_STREAM("KeyOp: reset linear/angular velocities.");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

} // namespace yocs_keyop
