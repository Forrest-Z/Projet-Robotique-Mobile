#include "move_pguard/global/theta_star_planner.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <stdexcept>

#include "move_pguard/bresenham_line.h"

namespace cm = costmap_2d;

namespace move_pguard
{
namespace global
{
const float ThetaStarPlanner::COST_GAIN = 1.0F;

ThetaStarPlanner::ThetaStarPlanner() : AStarPlanner()
{
}

ThetaStarPlanner::~ThetaStarPlanner()
{
}

bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
  // Get the position of the start
  getCostmap().worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, start_mx_, start_my_);
  // Get the orientation of the start
  tf2::Quaternion q_orientation;
  tf2::fromMsg(start.pose.orientation, q_orientation);
  tf2::Matrix3x3 m_orientation(q_orientation);
  double roll, pitch, yaw;
  m_orientation.getEulerYPR(yaw, pitch, roll);
  start_orientation_ = yaw;
  // Make the actual plan
  return AStarPlanner::makePlan(start, goal, plan);
}

void ThetaStarPlanner::visit(index_t edge_i, index_t goal_i)
{
  Edge& edge = edges_.at(edge_i);
  unsigned char* char_map = getCostmap().getCharMap();
  std::vector<index_t> neighbors = listNeighbors(edge_i);
  index_t parent_i = edge.predecessor;
  float line_cost;
  float potential_cost;

  for (index_t neighbor_i : neighbors)
  {
    Edge& neighbor = edges_.at(neighbor_i);
    if (!neighbor.is_open)
      continue;  // The edge is closed, explore the next one
    if (parent_i != NO_PREDECESSOR && hasLineOfSight(parent_i, neighbor_i, line_cost))
    {
      // A straight line from the parent to the neighbor is possible
      Edge& parent = edges_.at(parent_i);
      potential_cost =
          COST_GAIN * line_cost + orientationPenalty(parent_i) + parent.cost + distance(parent_i, neighbor_i);
      if (potential_cost < neighbor.cost)
        // The straight line is a cheaper path
        setPredecessor(neighbor_i, parent_i, goal_i, potential_cost);
    }
    else
    {
      potential_cost =
          COST_GAIN * char_map[edge_i] + orientationPenalty(edge_i) + edge.cost + distance(edge_i, neighbor_i);
      if (potential_cost < neighbor.cost)
        // Reaching the neighbor from the edge is cheaper
        setPredecessor(neighbor_i, edge_i, goal_i, potential_cost);
    }
  }
}

void ThetaStarPlanner::setPredecessor(index_t edge_i, index_t parent_i, index_t goal_i, float actual_cost)
{
  Edge& edge = edges_.at(edge_i);
  edge.cost = actual_cost;
  edge.predecessor = parent_i;
  float heuristic_cost = actual_cost + distance(edge_i, goal_i);

  if (edges_to_visit_.contains(edge_i))
    edges_to_visit_.changePriority(edge_i, heuristic_cost);  // Update the cost of the edge
  else
    edges_to_visit_.insert(edge_i, heuristic_cost);  // Add the edge to the heap
}

bool ThetaStarPlanner::hasLineOfSight(index_t edge_i, index_t at_edge_i, float& cost) const
{
  // Convert the indexes to cell coordinates
  const cm::Costmap2D& costmap = getCostmap();
  index_t edge_mx, edge_my;
  costmap.indexToCells(edge_i, edge_mx, edge_my);
  index_t at_edge_mx, at_edge_my;
  costmap.indexToCells(at_edge_i, at_edge_mx, at_edge_my);

  return hasLineOfSight(edge_mx, edge_my, at_edge_mx, at_edge_my, cost);
}

bool ThetaStarPlanner::hasLineOfSight(index_t x0, index_t y0, index_t x1, index_t y1, float& cost_out) const
{
  const cm::Costmap2D& costmap = getCostmap();
  BresenhamLine raycast(Pose2D{ .x = (int)x0, .y = (int)y0 }, Pose2D{ .x = (int)x1, .y = (int)y1 });
  auto itr_end = raycast.end();
  unsigned int line_cost = 0;

  for (auto itr = raycast.begin(); itr != itr_end; ++itr)
  {
    Pose2D pose = *itr;
    // Check the intermediate cell and its neighbor
    unsigned char cell_cost = costmap.getCost(pose.x, pose.y);
    line_cost += cell_cost;
    if (cell_cost >= 16)
      return false;
    if (raycast.isSteep())
    {
      int x = pose.x + raycast.getStepX();
      if (!isOutOfBound(costmap, x, pose.y) && costmap.getCost(x, pose.y) >= 16)
        return false;
    }
    else
    {
      int y = pose.y + raycast.getStepY();
      if (!isOutOfBound(costmap, pose.x, y) && costmap.getCost(pose.x, y) >= 16)
        return false;
    }
  }
  cost_out = line_cost;
  return true;
}

float ThetaStarPlanner::orientationPenalty(index_t edge_i)
{
  // Convert the index to cell coordinates
  cm::Costmap2D& costmap = getCostmap();
  index_t mx, my;
  costmap.indexToCells(edge_i, mx, my);

  return orientationPenalty(mx, my);
}

float ThetaStarPlanner::orientationPenalty(index_t mx, index_t my)
{
  int dx = (int)mx - start_mx_;
  int dy = (int)my - start_my_;
  float angle = std::atan2(dy, dx);
  float intensity = (1. - std::cos(angle)) / 2.; // aka. cos^2(angle / 2)
  return 50.0F * intensity;
}
}  // namespace global
}  // namespace move_pguard
