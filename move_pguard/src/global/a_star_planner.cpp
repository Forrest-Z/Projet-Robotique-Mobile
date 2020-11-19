#include "move_pguard/global/a_star_planner.h"

#include <stdexcept>

namespace cm = costmap_2d;

namespace move_pguard
{
namespace global
{
AStarPlanner::AStarPlanner() : DijkstraPlanner()
{
}

AStarPlanner::~AStarPlanner()
{
}

void AStarPlanner::visit(index_t edge_i, index_t goal_i)
{
  Edge& edge = edges_.at(edge_i);
  std::vector<index_t> neighbors = listNeighbors(edge_i);
  for (index_t neighbor_i : neighbors)
  {
    Edge& neighbor = edges_.at(neighbor_i);
    if (!neighbor.is_open)
      continue; // The edge is closed, explore the next one
    float potential_cost = edge.cost + distance(edge_i, neighbor_i);
    if (potential_cost < neighbor.cost)
    {
      // Traveling through the edge is shorter to reach this neighbor!
      // Updating the prodecessor and the costs of this neighbor
      neighbor.predecessor = edge_i;
      neighbor.cost = potential_cost;
      float heuristic_path_cost = potential_cost + distance(edge_i, goal_i);
      if (edges_to_visit_.contains(neighbor_i))
        edges_to_visit_.changePriority(neighbor_i, heuristic_path_cost);
      else
        edges_to_visit_.insert(neighbor_i, heuristic_path_cost);
    }
  }
}
}  // namespace global
}  // namespace move_pguard
