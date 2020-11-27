#ifndef __HV_FINDER_H__
#define __HV_FINDER_H__

#include "move_pguard/goal_finder/goal_finder_base.h"
namespace move_pguard
{
namespace goal_finder
{
/**
 * Horizontal or vertical goal finder.
 */
class HVFinder : public GoalFinderBase
{
public:
  /**
   * Find a suitable goal horizontally or vertically aligned with the initial goal
   *
   * @param costmap The costmap to get the costs from
   * @param start The start pose
   * @param goal The goal pose
   * @return a suitable goal horizontally or vertically aligned with the initial goal
   */
  virtual geometry_msgs::PoseStamped operator()(const costmap_2d::Costmap2D& costmap,
                                                const geometry_msgs::PoseStamped& start,
                                                const geometry_msgs::PoseStamped& goal) override;
};
}  // namespace goal_finder
}  // namespace move_pguard
#endif  // __HV_FINDER_H__
