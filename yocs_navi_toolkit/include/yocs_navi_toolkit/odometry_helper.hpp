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

  /********************
  ** Setters
  ********************/
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /********************
  ** Getters
  ********************/
  std::pair<float, float> velocities();
  nav_msgs::Odometry odometry();

protected:
  ros::Subscriber odometry_subscriber_;
  std::mutex data_mutex_;
  nav_msgs::Odometry odometry_;
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
