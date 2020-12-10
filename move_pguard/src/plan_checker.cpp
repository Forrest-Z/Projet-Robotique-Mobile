#include "move_pguard/plan_checker.h"

#include <costmap_2d/cost_values.h>
#include <ros/console.h>

#include <cmath>
#include <stdexcept>

#include "move_pguard/bresenham_line.h"
#include "move_pguard/utils.h"
namespace move_pguard
{
PlanChecker::PlanChecker(std::vector<geometry_msgs::PoseStamped>* plan) : plan_(plan)
{
  if (plan == nullptr)
    throw std::invalid_argument("PlanChecker::PlanChecker: Null pointer provided!");
}

float PlanChecker::length() const
{
  if (plan_->size() < 2)
    return 0.0F;
  float path_length = 0.0F;
  geometry_msgs::PoseStamped from = plan_->front();
  auto itr_end = plan_->end();
  // Iterates from the second waypoint to the end
  for (auto itr = ++plan_->begin(); itr != itr_end; ++itr)
  {
    // Sum the length of the segment and prepare the next one
    path_length += distance(from, *itr);
    from = *itr;
  }
  return path_length;
}

bool PlanChecker::isObstructed(const costmap_2d::Costmap2D& costmap, std::function<bool(unsigned char)> criteria) const
{
  geometry_msgs::PoseStamped from = plan_->front();
  auto itr_end = plan_->end();
  for (auto itr = ++plan_->begin(); itr != itr_end; itr++)
  {
    if (isObstructed(from, *itr, costmap, criteria))
      return true;
  }
  return false;
}

bool PlanChecker::isObstructed(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to,
                               const costmap_2d::Costmap2D& costmap, std::function<bool(unsigned char)> criteria) const
{
  // Convert to map coordinates
  int from_mx, from_my;
  costmap.worldToMapNoBounds(from.pose.position.x, from.pose.position.y, from_mx, from_my);
  int to_mx, to_my;
  costmap.worldToMapNoBounds(to.pose.position.x, to.pose.position.y, to_mx, to_my);

  unsigned int size_x = costmap.getSizeInCellsX();
  unsigned int size_y = costmap.getSizeInCellsY();
  // Raycast between the two waypoint
  BresenhamLine raycast(Pose2D{ .x = from_mx, .y = from_my }, Pose2D{ .x = to_mx, .y = to_my });
  for (Pose2D pose : raycast)
  {
    unsigned char cost = costmap_2d::NO_INFORMATION;
    if (!isOutOfBound(costmap, pose.x, pose.y))
      cost = costmap.getCost(pose.x, pose.y);
    if (criteria(cost))
    {
      ROS_WARN_STREAM("(" << pose.x << ", " << pose.y << "): " << (int)cost);
      return true;
    }
  }
  return false;
}
}  // namespace move_pguard
