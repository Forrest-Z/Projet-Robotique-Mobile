#include "move_pguard/goal_finder/raycast_finder.h"

#include "move_pguard/bresenham_line.h"

namespace move_pguard
{
namespace goal_finder
{
geometry_msgs::PoseStamped RaycastFinder::operator()(const costmap_2d::Costmap2D& costmap,
                                                     const geometry_msgs::PoseStamped& start,
                                                     const geometry_msgs::PoseStamped& goal)
{
  Pose2D goal_m;
  costmap.worldToMapNoBounds(goal.pose.position.x, goal.pose.position.y, goal_m.x, goal_m.y);
  geometry_msgs::PoseStamped suitable_goal = goal;
  if (!isSuitable(goal_m.x, goal_m.y, costmap))
  {
    // The initial goal is not reachable; find a new one
    Pose2D start_m;
    costmap.worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, start_m.x, start_m.y);
    // Raycast from the goal to the start
    BresenhamLine direct_path(goal_m, start_m);
    for (Pose2D pose : direct_path)
    {
      if (isSuitable(pose.x, pose.y, costmap))
      {
        // Found a suitable cell; take it as the new goal
        costmap.mapToWorld(pose.x, pose.y, suitable_goal.pose.position.x, suitable_goal.pose.position.y);
        break;
      }
    }
  }
  return suitable_goal;
}
}  // namespace goal_finder
}  // namespace move_pguard
