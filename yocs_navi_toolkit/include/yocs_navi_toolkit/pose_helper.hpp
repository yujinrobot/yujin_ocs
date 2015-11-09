/**
 * @file /yocs_navi_toolkit/include/yocs_navi_toolkit/pose_helper.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef yocs_navi_toolkit_POSE_HELPER_HPP_
#define yocs_navi_toolkit_POSE_HELPER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/linear_algebra.hpp>
#include <memory>
#include <mutex>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_navi_toolkit {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class PoseHelper
{
public:
  PoseHelper(const std::string& pose_topic_name);
  virtual ~PoseHelper();

  /********************
  ** Setters
  ********************/
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  /********************
  ** Getters
  ********************/
  // getters that return the data in various convenience formats

  /**
   * @brief 3d position of the robot in eigen format.
   */
  void position(Eigen::Vector3f& position);

  /**
   * @brief Heading angle for mobile robot 2d use cases.
   *
   * @param angle : in radians
   */
  void yaw(float& angle);

  geometry_msgs::PoseWithCovarianceStamped pose();

protected:
  ros::Subscriber pose_subscriber_;
  std::mutex data_mutex_;
  geometry_msgs::PoseWithCovarianceStamped pose_;
};

/*****************************************************************************
** Typedefs
*****************************************************************************/

typedef std::shared_ptr<PoseHelper> PoseHelperPtr;

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace yocs_navi_toolkit

#endif /* yocs_navi_toolkit_ODOMETRY_HELPER_HPP_ */
