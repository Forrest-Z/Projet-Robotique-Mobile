#include "move_pguard/goal_finder/goal_finder_base.h"

namespace move_pguard
{
namespace goal_finder
{
bool GoalFinderBase::isSuitable(int x, int y, const costmap_2d::Costmap2D& costmap) const
{
  return 0 <= x && (unsigned int)x < costmap.getSizeInCellsX() &&  //
         0 <= y && (unsigned int)y < costmap.getSizeInCellsY() &&  //
         costmap.getCost(x, y) < 128;
}
}  // namespace goal_finder
}  // namespace move_pguard
