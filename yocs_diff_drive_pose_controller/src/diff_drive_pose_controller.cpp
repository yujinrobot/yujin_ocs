#include "yocs_diff_drive_pose_controller/diff_drive_pose_controller.hpp"

#include <cmath>
#include <ros/console.h>
#include <yocs_math_toolkit/geometry.hpp>

namespace yocs
{

DiffDrivePoseController::DiffDrivePoseController(std::string name, double v_max, double w_max, double dist_thres,
                                                 double orient_thres, double dist_eps, double orient_eps,
                                                 double orientation_gain, double k_1, double k_2, double beta,
                                                 double lambda, double v_min, double v_min_movement,
                                                 double w_min_movement)
{
  name_ = name;

  r_ = 0.0;
  v_ = 0.0;
  w_ = 0.0;
  delta_ = 0.0;
  theta_ = 0.0;
  cur_ = 0.0;
  pose_reached_ = false;

  v_min_movement_ = v_min_movement;
  w_min_movement_ = w_min_movement;
  setCurrentLimits(v_min, -w_max, v_max, w_max); //set the limits so they include everything for now

  dist_thres_ = dist_thres;
  orient_thres_ = orient_thres;
  dist_eps_ = dist_eps;
  orient_eps_ = orient_eps;

  orientation_gain_ = orientation_gain;
  k_1_ = k_1;
  k_2_ = k_2;
  beta_ = beta;
  lambda_ = lambda;
}

void DiffDrivePoseController::setInput(double distance_to_goal, double delta, double theta)
{
  r_ = distance_to_goal;
  delta_ = mtk::wrapAngle(delta);
  theta_ = mtk::wrapAngle(theta);
}

void DiffDrivePoseController::setCurrentLimits(double v_min, double w_min, double v_max, double w_max)
{
  v_min_ = v_min;
  w_min_ = w_min;
  v_max_ = v_max;
  w_max_ = w_max;
}

bool DiffDrivePoseController::step()
{
  calculateControls();
  return pose_reached_;
}

void DiffDrivePoseController::calculateControls()
{
  controller_mutex_.lock();
  double angle_diff = mtk::wrapAngle(theta_ - delta_);

  if (r_ > dist_thres_)
  {
    controlPose();
  }
  else
  {
    //reached goal position, so just adjust orientation
    v_ = 0.0;
    controlOrientation(angle_diff);
  }

  // check, if pose has been reached
  if ((r_ <= dist_thres_) && (std::abs(angle_diff) <= orient_thres_))
  {
    if (!pose_reached_)
    {
      pose_reached_ = true;
      if ( verbose_ ) {
        ROS_INFO_STREAM("Pose reached. [" << name_ <<"]");
      }
      onGoalReached();
    }
  }
  else if ((r_ > (dist_thres_ + dist_eps_)) || (std::abs(angle_diff) > (orient_thres_ + orient_eps_)))
  {
    if (pose_reached_)
    {
      pose_reached_ = false;
      if ( verbose_ ) {
        ROS_INFO_STREAM("Tracking new goal pose. [" << name_ <<"]");
      }
    }
  }
  controller_mutex_.unlock();
}

void DiffDrivePoseController::controlPose()
{
  double atan2_k1_theta = std::atan2(-k_1_*theta_, 1.0);
  cur_ = (-1 / r_)
      * (k_2_ * (delta_ - atan2_k1_theta) + (1 + (k_1_ / (1 + std::pow((k_1_ * theta_), 2)))) * sin(delta_));

  v_ = v_max_ / (1 + beta_ * std::pow(std::abs(cur_), lambda_));
  v_ = enforceMinVelocity(v_, v_min_movement_);
  v_ = enforceMinMax(v_, v_min_, v_max_);

  //calculate needed angular velocity for given curvature
  w_ = cur_ * v_;
  //enforce limits on angular velocity
  w_ = enforceMinVelocity(w_, w_min_movement_);
  w_ = enforceMinMax(w_, w_min_, w_max_);

  //adjust linear velocity to bounded angular velocity
  v_ = w_ / cur_;
  //enforce limits on linear velocity
  v_ = enforceMinVelocity(v_, v_min_movement_);
  v_ = enforceMinMax(v_, v_min_, v_max_);

//  ROS_WARN_STREAM("r_: " << r_ << " dist_thres_: " << dist_thres_ << ", delta_-theta_: " << delta_ - theta_ << ", orient_thres_" << orient_thres_);
}

void DiffDrivePoseController::controlOrientation(double angle_difference)
{
  w_ = orientation_gain_ * (angle_difference);

  //enforce limits on angular velocity
  w_ = enforceMinVelocity(w_, w_min_movement_);
  w_ = enforceMinMax(w_, w_min_, w_max_);
}

double DiffDrivePoseController::enforceMinMax(double& value, double min, double max)
{
  return std::min(std::max(value, min), max);
}

double DiffDrivePoseController::enforceMinVelocity(double value, double min)
{
  if (value < 0.0)
  {
    if (value > -min)
    {
      value = -min;
    }
  }
  else
  {
    if (value < min)
    {
      value = min;
    }
  }

  return value;
}

void DiffDrivePoseController::getControlOutput(double& v, double& w)
{
  v = v_;
  w = w_;
}

} /* end namespace */
