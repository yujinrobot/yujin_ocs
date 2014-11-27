/*
  Basic Move Controller - Provides simple robot behavior such as go forward backward

  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */
#include "yocs_navigator/basic_move_controller.hpp" 

namespace yocs_navigator {

BasicMoveController::BasicMoveController(ros::NodeHandle& n) 
: nh_(n), cmd_vel_topic_(BasicMoveControllerDefaultParam::PUB_CMD_VEL), odometry_topic_(BasicMoveControllerDefaultParam::SUB_ODOM)
{
  init();
}

BasicMoveController::BasicMoveController(ros::NodeHandle& n, const std::string& cmd_vel_topic, const std::string& odometry_topic) 
: nh_(n), cmd_vel_topic_(cmd_vel_topic), odometry_topic_(odometry_topic)
{
  init();
}

BasicMoveController::~BasicMoveController()
{
}

void BasicMoveController::init() 
{
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 5);
  sub_odom_    = nh_.subscribe(odometry_topic_, 1, &BasicMoveController::processOdometry, this);
}

void BasicMoveController::processOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{   
  odometry_ = *msg;
}   

void BasicMoveController::slowForward()
{
  moveAt( 0.1,  0.0,  0.1);
}

void BasicMoveController::slowBackward()
{
  moveAt(-0.1,  0.0,  0.1);
}

void BasicMoveController::turnClockwise()
{
  moveAt( 0.0, -0.5,  0.1);
}

void BasicMoveController::turnCounterClockwise()
{
  moveAt( 0.0,  0.5,  0.1);
}

void BasicMoveController::stop()
{
  moveAt( 0.0,  0.0,  0.0);
}

void BasicMoveController::moveAt(double v, double w, double t = 0.0)
{
  geometry_msgs::Twist vel;
  vel.linear.x  = v;
  vel.angular.z = w;
  pub_cmd_vel_.publish(vel);
  ros::Duration(t).sleep();
}

void BasicMoveController::forward(double distance)
{
  geometry_msgs::Point pos0 = odometry_.pose.pose.position;
  while (mtk::distance2D(pos0, odometry_.pose.pose.position) < distance)
  {
    slowForward();
  }
}

void BasicMoveController::backward(double distance)
{
  geometry_msgs::Point pos0 = odometry_.pose.pose.position;
  while(mtk::distance2D(pos0, odometry_.pose.pose.position) < distance)
  {
    slowBackward();
  }
}

void BasicMoveController::turn(double angle)
{
  double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);
  double yaw1 = mtk::wrapAngle(yaw0 + angle);

  ROS_DEBUG("%f  %f  %f", angle, yaw0, yaw1);
  while (std::abs(mtk::wrapAngle(yaw1 - tf::getYaw(odometry_.pose.pose.orientation))) > 0.05)
    moveAt(0.0, mtk::sign(angle)*0.5, 0.05);
}

// aaaagh...  TODO make a decent version checking sign change and turns over 2pi! I have no time now
void BasicMoveController::turn2(double angle)
{
  double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);
  double yaw1 = mtk::wrapAngle(yaw0 + angle);
  double sign = mtk::sign(yaw1 - yaw0);

  ROS_DEBUG("%f  %f  %f", angle, yaw0, yaw1);
  while (mtk::sign(mtk::wrapAngle(tf::getYaw(odometry_.pose.pose.orientation) - yaw1)) == sign)
  {
    moveAt(0.0, mtk::sign(angle)*0.5, 0.05);
  }
}

void BasicMoveController::spinClockwise()
{
  // WARN; note that this requires that callbacks keep running!
  double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);

  for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) <= yaw0; ++i)
    turnClockwise();

  for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) >  yaw0; ++i)
    turnClockwise();
}

void BasicMoveController::spinCounterClockwise()
{
  // WARN; note that this requires that callbacks keep running!
  double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);

  for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) >= yaw0; ++i)
    turnCounterClockwise();

  for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) <  yaw0; ++i)
    turnCounterClockwise();
}

} // namespace yocs
