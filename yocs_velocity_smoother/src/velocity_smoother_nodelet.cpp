/**
 * @file /src/velocity_smoother_nodelet.cpp
 *
 * @brief Velocity smoother implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/master/velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <ecl/threads/thread.hpp>

#include "velocity_smoother/velocity_smoother_nodelet.h"

/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5
#define ZERO_VEL_COMMAND      geometry_msgs::Twist();
#define IS_ZERO_VEOCITY(a)   ((a.linear.x == 0.0) && (a.angular.z == 0.0))

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace velocity_smoother {

/*********************
** Implementation
**********************/

void VelocitySmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Estimate commands frequency; we do continuously as it can be very different depending on the
  // publisher type, and we don't want to impose extra constraints to keep this package flexible
  if (period_record.size() < PERIOD_RECORD_SIZE)
  {
    period_record.push_back((ros::Time::now() - last_cb_time).toSec());
  }
  else
  {
    period_record[pr_next] = (ros::Time::now() - last_cb_time).toSec();
  }

  pr_next++;
  pr_next %= period_record.size();
  last_cb_time = ros::Time::now();

  if (period_record.size() <= PERIOD_RECORD_SIZE/2)
  {
    // wait until we have some values; make a reasonable assumption meanwhile
    cb_avg_time = 0.05;
  }
  else
  {
    // enough; recalculate with the latest input
    cb_avg_time = median(period_record);
  }

  input_active = true;

  // Bound speed with the maximum values
  target_vel.linear.x  =
      msg->linear.x  > 0.0 ? std::min(msg->linear.x,  speed_lim_v) : std::max(msg->linear.x,  -speed_lim_v);
  target_vel.angular.z =
      msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w) : std::max(msg->angular.z, -speed_lim_w);
}

void VelocitySmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometry_vel = msg->twist.twist;
}

void VelocitySmoother::spin()
{
  double period = 1.0/frequency;
  ros::Rate spin_rate(frequency);

  while (! shutdown_req && ros::ok())
  {
    if ((input_active == true) &&
        ((ros::Time::now() - last_cb_time).toSec() > std::min(3.0*cb_avg_time, 0.5)))
    {
      // Velocity input no active anymore; normally last command is a zero-velocity one, but reassure
      // this, just in case something went wrong with our input, or he just forgot good manners...
      // Issue #2, extra check in case cb_avg_time is very bit, for example with several atomic commands
      input_active = false;
      if (IS_ZERO_VEOCITY(target_vel) == false)
      {
        ROS_WARN("Input got inactive letting us a non-zero target velocity (%f, %f); zeroing...",
                 target_vel.linear.x, target_vel.angular.z);
        target_vel = ZERO_VEL_COMMAND;
      }
    }

    if ((input_active == true) &&
        (((ros::Time::now() - last_cb_time).toSec() > 3.0*cb_avg_time)     || // 3 missing msgs
         (std::abs(odometry_vel.linear.x  - last_cmd_vel.linear.x)  > 0.2) ||
         (std::abs(odometry_vel.angular.z - last_cmd_vel.angular.z) > 2.0)))
    {
      // If the publisher has been inactive for a while, or if odometry velocity has diverged
      // significatively from last_cmd_vel, we cannot trust the latter; relay on odometry instead
      // TODO: odometry thresholds are 진짜 arbitrary; should be proportional to the max v and w...
      // The one for angular velocity is very big because is it's less necessary (for example the
      // reactive controller will never make the robot spin) and because the gyro has a 15 ms delay
      ROS_DEBUG("Using odometry instead of last command: %f, %f, %f",
		(ros::Time::now()      - last_cb_time).toSec(),
                odometry_vel.linear.x  - last_cmd_vel.linear.x,
                odometry_vel.angular.z - last_cmd_vel.angular.z);
      last_cmd_vel = odometry_vel;
    }

    geometry_msgs::TwistPtr cmd_vel;

    if ((target_vel.linear.x  != last_cmd_vel.linear.x) ||
        (target_vel.angular.z != last_cmd_vel.angular.z))
    {
      // Try to reach target velocity but...
      cmd_vel.reset(new geometry_msgs::Twist(target_vel));

      // ...ensure we don't exceed the acceleration limits: calculate raw to limited
      // velocity ratios and apply the highest one to both linear and angular speeds
      double cmd_vel_inc, max_vel_inc, raw_lim_ratio_v = 1.0, raw_lim_ratio_w = 1.0;

      cmd_vel_inc = target_vel.linear.x - last_cmd_vel.linear.x;
      if (odometry_vel.linear.x*target_vel.linear.x < 0.0)
      {
        max_vel_inc = decel_lim_v*period;   // countermarch
      }
      else
      {
        max_vel_inc = ((cmd_vel_inc*target_vel.linear.x > 0.0)?accel_lim_v:decel_lim_v)*period;
      }
      if (std::abs(cmd_vel_inc) > max_vel_inc)
      {
        raw_lim_ratio_v =
            std::max(1.0, target_vel.linear.x/(last_cmd_vel.linear.x + sign(cmd_vel_inc)*max_vel_inc));
      }

      cmd_vel_inc = target_vel.angular.z - last_cmd_vel.angular.z;
      if (odometry_vel.angular.z*target_vel.angular.z < 0.0)
      {
        max_vel_inc = decel_lim_w*period;  // countermarch
      }
      else
      {
        max_vel_inc = ((cmd_vel_inc*target_vel.angular.z > 0.0)?accel_lim_w:decel_lim_w)*period;

      }
      if (std::abs(cmd_vel_inc) > max_vel_inc)
      {
        raw_lim_ratio_w =
            std::max(1.0, target_vel.angular.z/(last_cmd_vel.angular.z + sign(cmd_vel_inc)*max_vel_inc));
      }

      cmd_vel->linear.x  = target_vel.linear.x  / std::max(raw_lim_ratio_v, raw_lim_ratio_w);
      cmd_vel->angular.z = target_vel.angular.z / std::max(raw_lim_ratio_v, raw_lim_ratio_w);

      lim_vel_pub.publish(cmd_vel);
      last_cmd_vel = *cmd_vel;
    }
    else if (input_active == true)
    {
      // We already reached target velocity; just keep resending last command while input is active
      cmd_vel.reset(new geometry_msgs::Twist(last_cmd_vel));
      lim_vel_pub.publish(cmd_vel);
    }

    spin_rate.sleep();
  }
}

/**
 * Initialise from a nodelet's private nodehandle.
 * @param nh : private nodehandle
 * @return bool : success or failure
 */
bool VelocitySmoother::init(ros::NodeHandle& nh)
{
  // Optional parameters
  nh.param("frequency",    frequency,   20.0);
  nh.param("decel_factor", decel_factor, 1.0);

  // Mandatory parameters
  if ((nh.getParam("speed_lim_v", speed_lim_v) == false) ||
      (nh.getParam("speed_lim_w", speed_lim_w) == false))
  {
    ROS_ERROR("Missing velocity limit parameter(s)");
    return false;
  }

  if ((nh.getParam("accel_lim_v", accel_lim_v) == false) ||
      (nh.getParam("accel_lim_w", accel_lim_w) == false))
  {
    ROS_ERROR("Missing acceleration limit parameter(s)");
    return false;
  }

  // Deceleration can be more aggressive, if necessary
  decel_lim_v = decel_factor*accel_lim_v;
  decel_lim_w = decel_factor*accel_lim_w;

  // Publishers and subscribers
  cur_vel_sub = nh.subscribe("odometry",    1, &VelocitySmoother::odometryCB, this);
  raw_vel_sub = nh.subscribe("raw_cmd_vel", 1, &VelocitySmoother::velocityCB, this);
  lim_vel_pub = nh.advertise <geometry_msgs::Twist> ("smooth_cmd_vel", 1);

  ROS_INFO("Velocity smoother nodelet successfully initialized");

  return true;
}


/*********************
** Nodelet
**********************/

class VelocitySmootherNodelet : public nodelet::Nodelet
{
public:
  VelocitySmootherNodelet()  { }
  ~VelocitySmootherNodelet()
  {
    NODELET_DEBUG("Waiting for worker thread to finish...");
    vel_smoother_->shutdown();
    worker_thread_.join();
  }

  virtual void onInit()
  {
    NODELET_DEBUG("Initialising nodelet...");

    vel_smoother_.reset(new VelocitySmoother);
    if (vel_smoother_->init(this->getPrivateNodeHandle()))
    {
      NODELET_DEBUG("Command velocity smoother nodelet initialised");
      worker_thread_.start(&VelocitySmoother::spin, *vel_smoother_);
    }
    else
    {
      NODELET_ERROR("Command velocity smoother nodelet initialisation failed");
    }
  }

private:
  boost::shared_ptr<VelocitySmoother> vel_smoother_;
  ecl::Thread                        worker_thread_;
};

} // namespace velocity_smoother

PLUGINLIB_EXPORT_CLASS(velocity_smoother::VelocitySmootherNodelet, nodelet::Nodelet);
