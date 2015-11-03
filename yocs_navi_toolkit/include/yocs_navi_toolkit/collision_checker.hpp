/**
 * @file /yocs_navi_toolkit/include/yocs_navi_toolkit/collision_checker.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef yocs_navi_toolkit_COLLISION_CHECKER_HPP_
#define yocs_navi_toolkit_COLLISION_CHECKER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <memory>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_navi_toolkit {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief Simple single-state collision checker.
 *
 * This may/should merge with the collision checker in the crrt local
 * planner, however that is currently more complicated and entangled with
 * the concept of a trajectory state.
 */
class CollisionChecker {
public:
  CollisionChecker(costmap_2d::Costmap2DROS* costmap_ros);
  bool inCollision(const float& x, const float& y, const float& yaw);

private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  base_local_planner::CostmapModel costmap_model_;
};

typedef std::shared_ptr<CollisionChecker> CollisionCheckerPtr;

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace yocs_navi_toolkit

#endif /* yocs_navi_toolkit_COLLISION_CHECKER_HPP_ */
