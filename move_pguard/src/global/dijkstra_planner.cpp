#include "move_pguard/global/dijkstra_planner.h"

#include <algorithm>
#include <cmath>
#include <list>
#include <mutex>
#include <stdexcept>

namespace cm = costmap_2d;

namespace move_pguard
{
namespace global
{
const index_t DijkstraPlanner::NO_PREDECESSOR = -1U;

const float DijkstraPlanner::INFINITE_COST = INFINITY;

DijkstraPlanner::DijkstraPlanner() : costmap_ros_(nullptr)
{
}

DijkstraPlanner::~DijkstraPlanner()
{
}

void DijkstraPlanner::initialize(std::string, costmap_2d::Costmap2DROS* costmap_ros)
{
  // Save the pointer to the costmap
  if (costmap_ros == nullptr)
    throw std::invalid_argument("DijkstraPlanner::initialize: costmap_ros is null.");
  costmap_ros_ = costmap_ros;
  // Measure the costmap
  cm::Costmap2D& costmap = getCostmap();
  index_t size_mx = costmap.getSizeInCellsX();
  index_t size_my = costmap.getSizeInCellsY();
  index_t cell_quantity = size_mx * size_my;
  // Preallocate the data structures
  edges_.resize(cell_quantity);
}

bool DijkstraPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*getCostmap().getMutex());  // Prevent map updates
  if (costmap_ros_ == nullptr)
    throw std::runtime_error("DijkstraPlanner::makePlan: Planner not initialized.");
  // Reinitialize the data structures
  edges_to_visit_.clear();
  std::fill(edges_.begin(), edges_.end(),
            Edge{ .predecessor = NO_PREDECESSOR, .cost = INFINITE_COST, .is_open = true });
  // Set the cost of the start cell
  index_t start_i = poseToIndex(start);
  edges_to_visit_.insert(start_i, 0.0F);
  edges_.at(start_i).cost = 0.0F;

  index_t goal_i = poseToIndex(goal);
  // Iterates at most over all the edges
  for (size_t iteration = 0; iteration < edges_.size() && edges_to_visit_.size() != 0; iteration++)
  {
    // Retrieve the nearest unvisited edge
    float current_edge_cost;
    index_t current_edge_i = edges_to_visit_.top(current_edge_cost);
    edges_to_visit_.pop();

    if (current_edge_i == goal_i)
      // We have a path to the goal; build it and return
      return buildPath(start, goal, plan);

    visit(current_edge_i, goal_i);
    edges_.at(current_edge_i).is_open = false;  // We are done with this edge
  }
  return false;
}

costmap_2d::Costmap2D& DijkstraPlanner::getCostmap()
{
  if (costmap_ros_ == nullptr)
    throw std::runtime_error("DijkstraPlanner::getCostmap2D: Not initialized (aka costmap_ros_ is null).");
  return *costmap_ros_->getCostmap();
}

const costmap_2d::Costmap2D& DijkstraPlanner::getCostmap() const
{
  if (costmap_ros_ == nullptr)
    throw std::runtime_error("DijkstraPlanner::getCostmap2D: Not initialized (aka costmap_ros_ is null).");
  return *costmap_ros_->getCostmap();
}

index_t DijkstraPlanner::poseToIndex(const geometry_msgs::PoseStamped& pose) const
{
  const cm::Costmap2D& costmap = getCostmap();
  // Convert to cell coordinates
  int mx, my;
  costmap.worldToMapEnforceBounds(pose.pose.position.x, pose.pose.position.y, mx, my);
  // Convert cell coordinates to the index
  return costmap.getIndex((index_t)mx, (index_t)my);
}

bool DijkstraPlanner::buildPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan) const
{
  index_t start_i = poseToIndex(start);
  index_t current_i = poseToIndex(goal);
  // Build the path as indexes
  std::list<index_t> path_index;
  if (start_i == current_i)
    // The goal is the starting point; stay in place
    path_index.push_front(current_i);
  else
  {
    for (; current_i != start_i; current_i = edges_.at(current_i).predecessor)
    {
      if (current_i == NO_PREDECESSOR)
      {
        ROS_WARN_STREAM("The path does not reach the robot's pose");
        return false;
      }
      path_index.push_front(current_i);
    }
  }
  path_index.push_front(start_i);
  // Convert the indexes to PoseStamped
  plan.clear();
  plan.reserve(path_index.size());
  geometry_msgs::PoseStamped current_pose = start;
  for (index_t current_i : path_index)
  {
    indexToPoint(current_i, current_pose.pose.position);
    plan.push_back(current_pose);
  }
  // Set the orientation to the next subgoal
  for (unsigned int i = 0; i + 1 < plan.size(); i++)
  {
    geometry_msgs::PoseStamped& pose = plan.at(i);
    geometry_msgs::PoseStamped& next_pose = plan.at(i + 1);
    float angle =
        std::atan2(next_pose.pose.position.y - pose.pose.position.y, next_pose.pose.position.x - pose.pose.position.x);
    next_pose.pose.orientation.w = std::cos(angle / 2);
    next_pose.pose.orientation.z = std::sin(angle / 2);
  }
  // Add a straight line to the goal if it is off the map
  index_t goal_mx, goal_my;
  if (!getCostmap().worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
  {
    geometry_msgs::PoseStamped& pose = plan.back();
    geometry_msgs::PoseStamped final_pose = goal;
    float angle = std::atan2(final_pose.pose.position.y - pose.pose.position.y,
                             final_pose.pose.position.x - pose.pose.position.x);
    final_pose.pose.orientation.w = std::cos(angle / 2);
    final_pose.pose.orientation.z = std::sin(angle / 2);
    plan.push_back(final_pose);
  }
  return true;
}

void DijkstraPlanner::indexToPoint(index_t index, geometry_msgs::Point& point) const
{
  const cm::Costmap2D& costmap = getCostmap();
  // Convert to cell coordinates
  index_t mx, my;
  costmap.indexToCells(index, mx, my);
  // Convert cell coordinates to world coordinates
  costmap.mapToWorld(mx, my, point.x, point.y);
  point.z = 0.0;
}

std::vector<index_t> DijkstraPlanner::listNeighbors(index_t edge_i) const
{
  const cm::Costmap2D& costmap = getCostmap();
  long size_mx = costmap.getSizeInCellsX();
  long size_my = costmap.getSizeInCellsY();
  // Prepapres the list of neighbors
  std::vector<index_t> neighbors;
  neighbors.reserve(8);
  // Convert the index of the edge to its position
  index_t edge_mx, edge_my;
  costmap.indexToCells(edge_i, edge_mx, edge_my);
  // Iterates in a 3 by 3 square centered on the edge and ensure the
  // coordinates are on the costmap
  for (long neighbor_mx = (long)edge_mx - 1; neighbor_mx <= (long)edge_mx + 1; neighbor_mx++)
  {
    if (neighbor_mx < 0 || neighbor_mx >= size_mx)
      continue;  // The column is out of bound; next one
    for (long neighbor_my = (long)edge_my - 1; neighbor_my <= (long)edge_my + 1; neighbor_my++)
    {
      if (neighbor_my < 0 || neighbor_my >= size_my)
        continue;  // The cell is out of bound; next one
      index_t neighbor_i = costmap.getIndex(neighbor_mx, neighbor_my);
      // The cell is a neighbor if it is free and not the edge
      if (costmap.getCost(neighbor_mx, neighbor_my) < 128 && neighbor_i != edge_i)
        neighbors.push_back(neighbor_i);
    }
  }
  return neighbors;
}

float DijkstraPlanner::distance(index_t e1_i, index_t e2_i) const
{
  const cm::Costmap2D& costmap = getCostmap();
  index_t e1_mx, e1_my;
  costmap.indexToCells(e1_i, e1_mx, e1_my);
  index_t e2_mx, e2_my;
  costmap.indexToCells(e2_i, e2_mx, e2_my);
  // Relative position of the goal from the edge
  float pose_mx = (float)e2_mx - (float)e1_mx;
  float pose_my = (float)e2_my - (float)e1_my;
  // Length of the vector
  return std::sqrt(pose_mx * pose_mx + pose_my * pose_my);
}

void DijkstraPlanner::visit(index_t edge_i, index_t)
{
  Edge& edge = edges_.at(edge_i);
  std::vector<index_t> neighbors = listNeighbors(edge_i);
  for (index_t neighbor_i : neighbors)
  {
    Edge& neighbor = edges_.at(neighbor_i);
    if (!neighbor.is_open)
      continue;  // The edge is closed, explore the next one
    float potential_cost = edge.cost + distance(edge_i, neighbor_i);
    if (potential_cost < neighbor.cost)
    {
      // Traveling through the edge is shorter to reach the neighbor!
      // Updating the predecessor and the cost of the neighbor
      neighbor.predecessor = edge_i;
      neighbor.cost = potential_cost;
      if (edges_to_visit_.contains(neighbor_i))
        edges_to_visit_.changePriority(neighbor_i, potential_cost);
      else
        edges_to_visit_.insert(neighbor_i, potential_cost);
    }
  }
}
}  // namespace global
}  // namespace move_pguard
