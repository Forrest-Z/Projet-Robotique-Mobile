#ifndef __RAYCAST_FINDER_H__
#define __RAYCAST_FINDER_H__

#include "move_pguard/goal_finder/goal_finder_base.h"

namespace move_pguard
{
namespace goal_finder
{
/**
 * Find a suitable goal through one raycast
 */
class RaycastFinder : public GoalFinderBase
{
public:
/**
 * Find a suitable goal by raycasting from the goal to the start pose.
 *
 * @param costmap The costmap to get the costs from
 * @param start The start pose
 * @param goal The goal pose
 * @return The first valid pose encountered by the raycast
 */
  virtual geometry_msgs::PoseStamped operator()(const costmap_2d::Costmap2D& costmap,
                                                const geometry_msgs::PoseStamped& start,
                                                const geometry_msgs::PoseStamped& goal) override;
};

}  // namespace goal_finder
}  // namespace move_pguard
#endif  // __RAYCAST_FINDER_H__
