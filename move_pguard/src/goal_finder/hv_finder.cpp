#include "move_pguard/goal_finder/hv_finder.h"

namespace move_pguard
{
namespace goal_finder
{
geometry_msgs::PoseStamped HVFinder::operator()(const costmap_2d::Costmap2D& costmap,
                                                const geometry_msgs::PoseStamped&,
                                                const geometry_msgs::PoseStamped& goal)
{
  geometry_msgs::PoseStamped suitable_goal = goal;
  int goal_mx, goal_my;
  costmap.worldToMapEnforceBounds(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my);
  if (isSuitable(goal_mx, goal_my, costmap))
    return goal; // The initial goal was suitable; return it

  if (costmap.getCost(goal_mx, goal_my) != 0)
  {
    int size_x = costmap.getSizeInCellsX();
    int delta_x;
    for (int dx = 1; dx < size_x; ++dx)
    {
      if (isSuitable(goal_mx + dx, goal_my, costmap))
      {
        delta_x = dx; // Found a suitable goal along X+ axis; stop here
        break;
      }
      if (isSuitable(goal_mx - dx, goal_my, costmap))
      {
        delta_x = -dx; // Found a suitable goal along X- axis; stop here
        break;
      }
    }
    int size_y = costmap.getSizeInCellsY();
    int delta_y;
    for (int dy = 1; dy < size_y; ++dy)
    {
      if (isSuitable(goal_mx, goal_my + dy, costmap))
      {
        delta_y = dy; // Found a suitable goal along Y+ axis; stop here
        break;
      }
      if (isSuitable(goal_mx, goal_my - dy, costmap))
      {
        delta_y = -dy; // Found a suitable goal along Y- axis; stop here
        break;
      }
    }
    // Select the nearest goal from the original one
    if (std::abs(delta_x) < std::abs(delta_y))
      goal_mx += delta_x;
    else
      goal_my += delta_y;
  }
  costmap.mapToWorld(goal_mx, goal_my, suitable_goal.pose.position.x, suitable_goal.pose.position.y);
  return suitable_goal;
}
}  // namespace goal_finder
}  // namespace move_pguard
