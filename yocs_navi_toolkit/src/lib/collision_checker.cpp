/**
 * @file /yocs_navi_toolkit/src/lib/collision_checker.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/yocs_navi_toolkit/collision_checker.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_navi_toolkit {

/*****************************************************************************
** Implementation
*****************************************************************************/

CollisionChecker::CollisionChecker(costmap_2d::Costmap2DROS* costmap_ros)
: costmap_ros_(costmap_ros)
, costmap_model_(*costmap_ros->getCostmap())
{
}

bool CollisionChecker::inCollision(const float& x, const float& y, const float& yaw) {
  ///TODO make own fast implementation
  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
  double cost = costmap_model_.footprintCost(x, y, yaw, footprint);
  // cost >= 0 implies not in collision
  // cost <  0 implies collision
  return (cost < 0 );
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace yocs_navi_toolkit
