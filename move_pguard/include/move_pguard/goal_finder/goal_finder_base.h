#ifndef __GOAL_FINDER_BASE_H__
#define __GOAL_FINDER_BASE_H__

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace move_pguard
{
namespace goal_finder
{
/**
 * Base class for all goal finder algorithm
 */
class GoalFinderBase
{
public:
  virtual ~GoalFinderBase() = default; /**< Virtual destructor for inheritance */

  /**
   * Find a goal that is not on a blocked cell.
   *
   * NOTE: The goal is not garanteed to be reachable.
   *
   * @param costmap The costmap to get the costs from
   * @param start The start pose
   * @param goal The goal pose
   * @return A goal that is not on a blocked cell.
   */
  virtual geometry_msgs::PoseStamped operator()(const costmap_2d::Costmap2D& costmap,
                                                const geometry_msgs::PoseStamped& start,
                                                const geometry_msgs::PoseStamped& goal) = 0;

protected:
  /**
   * Predicate that tests if the cell is suitable for a goal
   *
   * Ensure that the cell is on the map and not an obstacle
   *
   * @param x The X map coordinate
   * @param y The Y map coordinate
   * @param costmap The costmap to get the costs from
   * @return `true` if the cell is suitable; otherwise `false`
   */
  bool isSuitable(int x, int y, const costmap_2d::Costmap2D& costmap) const;
};
}  // namespace goal_finder
}  // namespace move_pguard
#endif  // __GOAL_FINDER_BASE_H__
