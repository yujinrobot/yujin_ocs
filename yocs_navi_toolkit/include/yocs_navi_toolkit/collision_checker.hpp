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
 * Assumptions:
 *  - footprint, inscribed and circumscribed_radii don't change, compute on construction.
 *
 * This may/should merge with the collision checker in the crrt local
 * planner, however that is currently more complicated and entangled with
 * the concept of a trajectory state.
 */
class CollisionChecker {
public:
  /**
   * @brief Initialise the checker with a costmap.
   *
   * @param costmap_ros : usually either the local or global costmap
   */
  CollisionChecker(costmap_2d::Costmap2DROS* costmap_ros);
  /**
   * @brief Check for collision along the boundaries of the footprint.
   *
   * Keep in mind that it is only checking along the boundaries. This means
   * that you can get false positives if you just carelessly look for collisions
   * at any convenient point on the costmap as the boundaries might be fine, but
   * there is an unlooked for collision inside the boundary of the footprint.
   */
  bool inCollision(const float& x, const float& y, const float& yaw);

private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  base_local_planner::CostmapModel costmap_model_;
  std::vector<geometry_msgs::Point> footprint_;
  double inscribed_radius_, circumscribed_radius_; // properties of the footprint.
};

typedef std::shared_ptr<CollisionChecker> CollisionCheckerPtr;

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace yocs_navi_toolkit

#endif /* yocs_navi_toolkit_COLLISION_CHECKER_HPP_ */
