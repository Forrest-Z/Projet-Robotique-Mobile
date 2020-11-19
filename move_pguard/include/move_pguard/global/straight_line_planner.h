#ifndef __STRAIGHT_LINE_PLANNER_H__
#define __STRAIGHT_LINE_PLANNER_H__

#include <nav_core/base_global_planner.h>

namespace move_pguard
{
namespace global
{
/**
 * The straight line planner provide simplistic plan as a line without caring
 * about the obstacle. Therefor it is very fast.
 */
class StraightLinePlanner : public nav_core::BaseGlobalPlanner
{
public:
  StraightLinePlanner(); /**< Construct a new Straight Line Planner. */

  virtual ~StraightLinePlanner(); /**< Virtual destructor for inheritance */

  /**
   * Draws a straight line from the start to the goal.
   *
   * Every waypoint points toward the next one. The last waypoint has the same
   * orientation as the previous one.
   *
   * @param start The start pose
   * @param goal The goal pose
   * @param plan *[out]* The plan from the start to the goal
   * @return `true` on success otherwise `false`
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan) override;

  /**
   * Initialize the planner.
   *
   * Store the costmap.
   *
   * @param name The name of this planner
   * @param costmap_ros The costmap used by this planner
   * @throw std::invalid_argument When costmap_ros is null
   */
  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  private:
    bool is_initialized_; /**< Flag used to ensure at least one initialization */
  protected:
    costmap_2d::Costmap2DROS* costmap_ros_; /**< The costmap to use. */
};
}  // namespace global
}  // namespace move_pguard
#endif  // __STRAIGHT_LINE_PLANNER_H__
