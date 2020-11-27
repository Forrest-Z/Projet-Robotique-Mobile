#ifndef __A_STAR_PLANNER_H__
#define __A_STAR_PLANNER_H__

#include "move_pguard/global/dijkstra_planner.h"

namespace move_pguard
{
namespace global
{
/**
 * The A* planner implements the A* algorithm to find the optimal path on the
 * graph.
 */
class AStarPlanner : public DijkstraPlanner
{
public:
  AStarPlanner(); /**< Construct a new AStarPlanner */

  virtual ~AStarPlanner(); /**< Virtual destructor for inheritance */

protected:
  /**
   * Visit an edge
   *
   * For all the neighbors of the edge, the method checks if going from
   * the start edge to the neighbor is shorter through this edge. If so,
   * the path is updated and the neighbor are added to the heap of edges
   * to visit (or their priority is updated if the heap already contained
   * it).
   *
   * @param edge_i The edge to visit
   * @param goal_i The goal of the path
   */
  virtual void visit(index_t edge_i, index_t goal_i) override;
};
}  // namespace global
}  // namespace move_pguard
#endif  // __A_STAR_PLANNER_H__
