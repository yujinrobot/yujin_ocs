/**
 * @file /yocs_navi_toolkit/src/lib/collision_checker.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/yocs_navi_toolkit/collision_checker.hpp"
#include <costmap_2d/footprint.h>

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
, footprint_(costmap_ros_->getRobotFootprint())
{
  // don't need to compute the radii, but if we precompute here, it saves
  // the footprintCost() function from repeatedly computing them internally
  costmap_2d::calculateMinAndMaxDistances(footprint_, inscribed_radius_, circumscribed_radius_);
}

bool CollisionChecker::inCollision(const float& x, const float& y, const float& yaw) {
  double cost = costmap_model_.footprintCost(x, y, yaw, footprint_, inscribed_radius_, circumscribed_radius_);
  // cost >= 0 implies not in collision
  // cost <  0 implies collision
  return (cost < 0 );
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace yocs_navi_toolkit
