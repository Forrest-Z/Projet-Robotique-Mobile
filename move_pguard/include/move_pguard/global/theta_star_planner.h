#ifndef __THETA_STAR_PLANNER_H__
#define __THETA_STAR_PLANNER_H__

#include <vector>

#include "move_pguard/global/a_star_planner.h"
#include "move_pguard/heap.h"

namespace move_pguard
{
namespace global
{
/**
 * Theta* planner uses the Theta* algorithm to plan an any-angle optimal path.
 */
class ThetaStarPlanner : public AStarPlanner
{
public:
  /** Gain to multiplying the cost of cell (in m/cost) */
  static const float COST_GAIN;

  ThetaStarPlanner(); /**< Construct a new ThetaStarPlanner */

  virtual ~ThetaStarPlanner(); /**< Virtual destructor for inheritance */

  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

protected:
  void visit(index_t edge_i, index_t goal_i) override;

  /**
   * Set the predecessor of an edge
   *
   * @param edge_i The edge to set the predecessor
   * @param parent_i The predecessor of the edge
   * @param goal_i The goal to reach
   * @param actual_cost The cost of the path at the edge with the new
   * predecessor
   */
  void setPredecessor(index_t edge_i, index_t parent_i, index_t goal_i, float actual_cost);

  /**
   * Predicat for line of sight between two edge
   *
   * There is a line of sight iff all the edges between these two are FREE. The
   * line is constructed thanks to the Bresenham's line algorithm.
   *
   * @param edge_i The first edge
   * @param at_edge_i The possibly visible edge
   * @return `true` if there is only FREE edges in the line of sight otherwise
   * `false`.
   */
  bool hasLineOfSight(index_t edge_i, index_t at_edge_i, float& cost) const;

  /**
   * Predicate that test if there is a line of sight between two edge
   *
   * There is a line of sight iff all the edges between these two are FREE. The
   * line is constructed thanks to the Bresenham's line algorithm.
   *
   * @param x0 The X coordinate of the edge
   * @param y0 The Y coordinate of the edge
   * @param x1 The X coordinate of the possibly visible edge
   * @param y1 The Y coordinate of the possibly visible edge
   * @return `true` if there is only FREE edges in the line of sight otherwise
   * `false`.
   */
  bool hasLineOfSight(index_t x0, index_t y0, index_t x1, index_t y1, float& cost) const;

  float start_orientation_; /**< The orienation of the start pose */
  int start_mx_;            /**< The X coordinate of the start pose */
  int start_my_;            /**< The Y coordinate of the start pose */
};
}  // namespace global
}  // namespace move_pguard
#endif  // __THETA_STAR_PLANNER_H__
