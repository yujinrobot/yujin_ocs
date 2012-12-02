/*
 * velocity_smoother_nodelet.cpp
 *
 *  Created on: Oct 25, 2012
 *      Author: jorge
 */

#include <ros/ros.h>

#include "velocity_smoother/velocity_smoother_nodelet.h"

/*********************
** Inplementation
**********************/

VelSmoother::VelSmoother() { }

VelSmoother::~VelSmoother() { }

void VelSmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Estimate commands frequency; it can be very different depending on the publisher type
  double frequency = 1.0/(ros::Time::now() - last_cmd_time).toSec();
  double period = (ros::Time::now() - last_cmd_time).toSec();
  ROS_ERROR("%f", frequency);

  // Ensure we don't exceed the acceleration limits; for each dof, we calculate the
  // commanded velocity increment and the maximum allowed increment (i.e. acceleration)
  double cmd_vel_inc, max_vel_inc;
  //double period = 1.0/frequency;

  cmd_vel_inc = cmd_vel.linear.x - last_cmd_vel.linear.x;
  if (cur_vel.linear.x*cmd_vel.linear.x < 0.0)
  {
    max_vel_inc = decel_lim_x*period;   // countermarch
  }
  else
  {
    max_vel_inc = ((cmd_vel_inc*cmd_vel.linear.x > 0.0)?accel_lim_x:decel_lim_x)*period;
  }
  if (std::abs(cmd_vel_inc) > max_vel_inc)
  {
    cmd_vel.linear.x = last_cmd_vel.linear.x + sign(cmd_vel_inc)*max_vel_inc;
  }

  cmd_vel_inc = cmd_vel.linear.y - last_cmd_vel.linear.y;
  if (cur_vel.linear.y*cmd_vel.linear.y < 0.0)
  {
    max_vel_inc = decel_lim_y*period;   // countermarch
  }
  else
  {
    max_vel_inc = ((cmd_vel_inc*cmd_vel.linear.y > 0.0)?accel_lim_y:decel_lim_y)*period;
  }
  if (abs(cmd_vel_inc) > max_vel_inc)
  {
    cmd_vel.linear.y = last_cmd_vel.linear.y + sign(cmd_vel_inc)*max_vel_inc;
  }

  cmd_vel_inc = cmd_vel.angular.z - last_cmd_vel.angular.z;
  if (cur_vel.angular.z*cmd_vel.angular.z < 0.0)
  {
    max_vel_inc = decel_lim_t*period;   // countermarch
  }
  else
  {
    max_vel_inc = ((cmd_vel_inc*cmd_vel.angular.z > 0.0)?accel_lim_t:decel_lim_t)*period;
  }
  if (abs(cmd_vel_inc) > max_vel_inc)
  {
    cmd_vel.angular.z = last_cmd_vel.angular.z + sign(cmd_vel_inc)*max_vel_inc;
  }

  lim_vel_pub.publish(cmd_vel);

  last_cmd_vel = cmd_vel;
}

/**
 * Initialise from a nodelet's private nodehandle.
 * @param nh : private nodehandle
 * @return bool : success or failure
 */
bool VelSmoother::init(ros::NodeHandle& nh)
{
  nh.getParam("accel_lim_x",  accel_lim_x);
  nh.getParam("accel_lim_y",  accel_lim_y);
  nh.getParam("accel_lim_t",  accel_lim_t);
  nh.getParam("decel_factor", decel_factor);

  // Deceleration can be more aggressive, if necessary
  decel_lim_x = decel_factor*accel_lim_x;
  decel_lim_y = decel_factor*accel_lim_y;
  decel_lim_t = decel_factor*accel_lim_t;

  // Publishers and subscribers
  cur_vel_sub = nh.subscribe("odometry",    1, &VelSmoother::odometryCB, this);
  raw_vel_sub = nh.subscribe("raw_cmd_vel", 1, &VelSmoother::velocityCB, this);
  lim_vel_pub = nh.advertise <geometry_msgs::Twist> ("smooth_cmd_vel", 1);

  ROS_INFO("Velocity smoother nodelet successfully initialized");

  return true;
}


/*********************
** Nodelet
**********************/

class VelSmootherNodelet : public nodelet::Nodelet
{
public:
  VelSmootherNodelet()  { }
  ~VelSmootherNodelet() { }

  virtual void onInit()
  {
    NODELET_DEBUG("Initialising nodelet...");

    vs_.reset(new VelSmoother);
    if (vs_->init(this->getPrivateNodeHandle()))
    {
      NODELET_DEBUG("Command velocity smoother nodelet initialised");
    }
    else
    {
      NODELET_ERROR("Command velocity smoother nodelet initialisation failed");
    }
  }

private:
  boost::shared_ptr<VelSmoother> vs_;
};

PLUGINLIB_DECLARE_CLASS(velocity_smoother, VelSmootherNodelet, VelSmootherNodelet, nodelet::Nodelet);


// old version replaced by smoothStop, which reuse boundSpeed; probably will be discarded
bool VelSmoother::accelerateTo___2(const geometry_msgs::Twist& target_vel)
{
  // Accelerate to target velocity without violating the same limits that local planner uses
  double period = 0.1; // 10 Hz, same as stage odometry rate, but we can use an arbitrary value

  // Estimate how many steps we will need to accelerate, so we exit even if something goes wrong
  int estim_steps =
      ceil(_max(std::abs(target_vel.linear.x  - last_cmd_vel.linear.x)  / (accel_lim_x*period),
                std::abs(target_vel.linear.y  - last_cmd_vel.linear.y)  / (accel_lim_y*period),
                std::abs(target_vel.angular.z - last_cmd_vel.angular.z) / (accel_lim_t*period)));

  if (estim_steps == 0)
    // Don't need to accelerate? could be a warning...
    return true;

  ROS_INFO("Accelerating from %.2f, %.2f, %.2f to %.2f, %.2f, %.2f in %d steps at %.2f, %.2f, %.2f",
            last_cmd_vel.linear.x, last_cmd_vel.linear.y, last_cmd_vel.angular.z,
            target_vel.linear.x, target_vel.linear.y, target_vel.angular.z, estim_steps,
            accel_lim_x, accel_lim_y, accel_lim_t);

  // Ensure we don't exceed the acceleration limits; for each dof, we calculate the
  // commanded velocity increment and the maximum allowed increment (i.e. acceleration)
  accelerating = true;

  geometry_msgs::Twist cmd_vel;

  for (int i = 0; i < (int)estim_steps; i++) {
/*
    // Verify that the user is not moving the joystick while in manual mode
    // This can provoke a sharp motion, but... user rules
    if ((joystick_input == true) ) {//&& (mode == MODE_MANUAL)) {
      ROS_INFO("Acceleration aborted, as user is issuing joystick commands");
      accelerating = false;
      is not concurrent, so this don't make sense; rethink... break loop   return true;
    }

    // Verify that nav stack is not issuing commands while in automatic mode
    if ((navstack_input == true) && (mode == MODE_AUTO)) {
      ROS_INFO("Acceleration aborted, as nav stack issuing speed commands");
      accelerating = false;
      return true;
    }*/



    if (std::abs(target_vel.linear.x - last_cmd_vel.linear.x) <= accel_lim_x*period)
      cmd_vel.linear.x = target_vel.linear.x;
    else
      cmd_vel.linear.x = last_cmd_vel.linear.x
                       + sign(target_vel.linear.x - last_cmd_vel.linear.x)*accel_lim_x*period;

    if (std::abs(target_vel.linear.y - last_cmd_vel.linear.y) <= accel_lim_y*period)
      cmd_vel.linear.y = target_vel.linear.y;
    else
      cmd_vel.linear.y = last_cmd_vel.linear.y
                       + sign(target_vel.linear.y - last_cmd_vel.linear.y)*accel_lim_y*period;

    if (std::abs(target_vel.angular.z - last_cmd_vel.angular.z) <= accel_lim_t*period)
      cmd_vel.angular.z = target_vel.angular.z;
    else
      cmd_vel.angular.z = last_cmd_vel.angular.z
                        + sign(target_vel.angular.z - last_cmd_vel.angular.z)*accel_lim_t*period;

    mng_cmd_vel_pub.publish(cmd_vel);

    last_cmd_vel = cmd_vel;

    ROS_DEBUG("Step %d >>> SET SPEED TO %f, %f, %f m/s", i, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    if ((std::abs(cmd_vel.linear.x  - target_vel.linear.x)  <= DBL_EPSILON) &&
        (std::abs(cmd_vel.linear.y  - target_vel.linear.y)  <= DBL_EPSILON) &&
        (std::abs(cmd_vel.angular.z - target_vel.angular.z) <= DBL_EPSILON)) {
      // Acceleration completed
      accelerating = false;
      return true;
    }

    usleep(period*1000000);
  }

  // Something went wrong; just send the desired speed
  ROS_WARN("Accelerate to %.2f, %.2f, %.2f failed after %d steps; forcing target speed",
           target_vel.linear.x, target_vel.linear.y, target_vel.angular.z, estim_steps);

  mng_cmd_vel_pub.publish(target_vel);

  last_cmd_vel = target_vel;

  accelerating = false;
  return false;
}
