#ifndef __PLAN_CHECKER_H__
#define __PLAN_CHECKER_H__

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

#include <functional>
#include <vector>

namespace move_pguard
{
/**
 * Plan checker extract information from a plan
 */
class PlanChecker
{
public:
  /**
   * Construct a new Plan Checker
   *
   * @param plan A pointer to the plan to check
   * @throw std::invalid_argument When the pointer is `null`
   */
  PlanChecker(std::vector<geometry_msgs::PoseStamped>* plan);

  /**
   * Measure the length of the path
   *
   * @return the length of the path (in m)
   */
  float length() const;

  /**
   * Predicate that test if the plan is obstructed
   *
   * @param costmap The costmap to get the occupancy from
   * @param criterion The criterion on the occupancy
   * @return `true` if no cell on the path fills in the criterion; otherwise `false`
   */
  bool isObstructed(const costmap_2d::Costmap2D& costmap, std::function<bool(unsigned char)> criterion) const;

private:
  /**
   * Predicate that test if a segment of the plan is obstructed
   *
   * @param from The beginning of the segment
   * @param to The end of the segment
   * @param costmap The costmap to get the occupancy from
   * @param criterion The criterion on the occupancy
   * @return `true` if no cell on the segment fills in the criterion; otherwise `false`
   */
  bool isObstructed(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to,
                    const costmap_2d::Costmap2D& costmap, std::function<bool(unsigned char)> criterion) const;

  std::vector<geometry_msgs::PoseStamped>* plan_; /**< A pointer to the plan to check */
};
}  // namespace move_pguard
#endif  // __PLAN_CHECKER_H__
