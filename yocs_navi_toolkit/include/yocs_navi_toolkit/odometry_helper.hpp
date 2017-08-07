/**
 * @file /yocs_navi_toolkit/include/yocs_navi_toolkit/odometry_helper.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef yocs_navi_toolkit_ODOMETRY_HELPER_HPP_
#define yocs_navi_toolkit_ODOMETRY_HELPER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/linear_algebra.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <utility>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_navi_toolkit {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class OdometryHelper
{
public:
  OdometryHelper(const std::string& odometry_topic_name);
  virtual ~OdometryHelper();

  /**
   * @brief Returns if initialised (odometry available)
   */
  virtual bool initialized();

  /********************
  ** Setters
  ********************/
  virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /********************
  ** Getters
  ********************/
  // convenience getters in various formats.

  /**
   * @brief 3d position of the robot in Eigen format.
   * @return true if successful (helper got odometry)
   */
  virtual bool position(Eigen::Vector3f& position);

  /**
   * @brief Heading angle for mobile robot 2d use cases.
   *
   * @param angle : in radians
   * @return true if successful (helper got odometry)
   */
  virtual bool yaw(float& angle);

  /**
   * @brief Mobile robot velocities in a 2d use case.
   *
   * @param mobile_robot_velocities : linear translational velocity, v and angular rate, w
   * @return true if successful (helper got odometry)
   */
  virtual bool velocities(std::pair<float, float>& mobile_robot_velocities);

  virtual std::shared_ptr<nav_msgs::Odometry> odometry();

protected:
  OdometryHelper() // for mocking
  {
  }

  ros::Subscriber odometry_subscriber_;
  std::mutex data_mutex_;
  std::unique_ptr<nav_msgs::Odometry> odometry_;
};

/*****************************************************************************
** Typedefs
*****************************************************************************/

typedef std::shared_ptr<OdometryHelper> OdometryHelperPtr;

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace yocs_navi_toolkit

#endif /* yocs_navi_toolkit_ODOMETRY_HELPER_HPP_ */
