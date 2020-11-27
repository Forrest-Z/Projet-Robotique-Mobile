#ifndef __DIJKSTRA_PLANNER_H__
#define __DIJKSTRA_PLANNER_H__

#include <nav_core/base_global_planner.h>

#include <vector>

#include "move_pguard/edge.h"
#include "move_pguard/heap.h"
#include "move_pguard/utils.h"

namespace move_pguard
{
namespace global
{
  /**
   * The Dijkstra planner uses the Dijkstra algorithm to find the optimal path on
   * the graph.
   */
class DijkstraPlanner : public nav_core::BaseGlobalPlanner
{
public:
  static const index_t NO_PREDECESSOR; /**< Value for the predecessor indicating there is none. */
  static const float INFINITE_COST;    /**< Value for unknown cost. */

  DijkstraPlanner(); /**< Construct a new DijkstraPlanner.*/

  virtual ~DijkstraPlanner(); /**< Virtual destructor for inheritance */

  /**
   * Get the Costmap2D.
   *
   * @return the costmap used by this planner
   */
  costmap_2d::Costmap2D& getCostmap();

  /** @overload const costmap_2d::Costmap2D& getCostmap() const */
  const costmap_2d::Costmap2D& getCostmap() const;

  /**
   * Convert a pose (in world) to an edge in the graph.
   *
   * NOTE: MUST have been initialized
   *
   * @param pose The pose to convert into an edge
   * @return The index of the edge corresponding to the pose
   *
   * @throw std::runtime_error When called before initialization
   */
  index_t poseToIndex(const geometry_msgs::PoseStamped& pose) const;

  /**
   * Convert an edge from its index into a point in the world.
   *
   * @param index The index of the edge to convert
   * @param point A Point to store the converted edge
   */
  void indexToPoint(index_t index, geometry_msgs::Point& point) const;

  /**
   * Compute the distance (in cells) between two edges.
   *
   * @param e1_i The index of the first edge
   * @param e2_i The index of the second edge
   * @return The distance (in cells) between the two edges
   */
  virtual float distance(index_t e1_i, index_t e2_i) const;

  /**
   * List the neighbors of the given edge.
   *
   * NOTE: The neighbors can be absent from the heap
   *
   * @param edge_i The edge which you want to find its neighbor
   * @return The list of neighbor of the edge
   */
  virtual std::vector<index_t> listNeighbors(index_t edge_i) const;

  /**
   * Builds a path.
   *
   * The path is read backward from the list of predecessors then flipped
   * and turned into pose. The pose are in the same frame and timestamp as
   * the start pose.
   *
   * @param start The initial position
   * @param goal The position to reach
   * @param plan A vector to store the path
   * @return `true` if a valid path has been found; otherwise `false`.
   */
  virtual bool buildPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                         std::vector<geometry_msgs::PoseStamped>& plan) const;

  /**
   * Initialize the planner.
   *
   * Store the pointer to the costmap et preallocate memory to work with this
   * costmap.
   *
   * @param name The name of this planner
   * @param costmap_ros A pointer to the costmap to use
   * @throw std::invalid_argument When the pointer to the costmap is null
   */
  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  /**
   * Make a plan to reach the goal from the start.
   *
   * @param start The start pose
   * @param goal The goal pose
   * @param plan *[out]* The plan from the start to the goal
   * @return `true` if a valid plan was found; otherwise `false`
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan) override;

protected:
  /**
   * Visit an edge.
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
  virtual void visit(index_t edge_i, index_t goal_i);

  Heap<index_t, float> edges_to_visit_;   /**< Heap of the edges to visit */
  std::vector<Edge> edges_;               /**< List of all the edges */
  costmap_2d::Costmap2DROS* costmap_ros_; /**< The global costmap */
};
}  // namespace global
}  // namespace move_pguard
#endif  // __DIJKSTRA_PLANNER_H__
