#ifndef __MOVE_PGUARD__
#define __MOVE_PGUARD__

#include <actionlib/server/simple_action_server.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include "move_pguard/GoToAction.h"

namespace move_pguard
{
using GlobalPlannerS = std::shared_ptr<nav_core::BaseGlobalPlanner>;
using LocalPlannerS = std::shared_ptr<nav_core::BaseLocalPlanner>;

using GoToActionServer = actionlib::SimpleActionServer<GoToAction>;

class MovePGuard
{
public:
  /**
   * Construct a new Move PGuard
   *
   * @param name The name of the instance
   * @param buffer A pointer to a transform listener
   * @param global_costmap A pointer to the global costmap to use
   * @param local_costmap A pointer to the local costmap to use
   * @throw std::invalid_argument When a pointer is `null`
   */
  MovePGuard(std::string name, tf::TransformListener* buffer, costmap_2d::Costmap2DROS* global_costmap,
             costmap_2d::Costmap2DROS* local_costmap);

  ~MovePGuard();

private:
  /**
   * Plan a global path to the goal
   *
   * Ensure the goal is suitable (aka. not in an obstacle) and find another one
   * if needed. Then call the global planner and store the goal and the path.
   *
   * @param goal The goal request
   * @param path *[out]* The path to the goal
   * @return `true` if the valid plan was found; otherwise `false`
   */
  bool planGlobalPath(const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path);

  /**
   * Callback used when a goal has been received
   *
   * Resend the goal to the action goal topic
   *
   * @param goal The requested goal
   */
  void goalCallback(const geometry_msgs::PoseStamped& goal);

  /**
   * Callback used by the action server to move the pguard
   *
   * Find a global plan and update it only when it is no longer valid. Every
   * tick, compute the velocity command to follow the plan. The action is
   * aborteed when a failure occur either from the global or the local planner.
   * The plan and the velocity command are published to their respective topic.
   *
   * @param goal The goal of the action
   */
  void executeCallback(const GoToGoalConstPtr& goal);

  /**
   * Publish a zero velocity command to stop the PGuard
   */
  void publishZeroVelocity();

  /**
   * Request the velocity command from the local planner and publish the command
   *
   * @throw std::runtime_error When a valid velocity command has not been found
   */
  void control();

  /**
   * Update the global plan at most every second
   *
   * The plan is checked for obstruction. If it is obstructed then a new plan is
   * requested to the planner. The path is then published and stored. And a
   * feedback is sent to the action client.
   *
   * @param is_forced Flag to bypass all the verifications; a new plan is
   *                  garanteed to be requested
   */
  void updateGlobalPlan(bool is_forced = false);

  GlobalPlannerS global_planner_; /**< A shared pointer to the global planner */
  LocalPlannerS local_planner_;   /**< A shared pointer to the local planner */

  costmap_2d::Costmap2DROS* global_costmap_ros_; /**< A pointer to the costmap used by the global planner */
  costmap_2d::Costmap2DROS* local_costmap_ros_;  /**< A pointer to the costmap used by the local planner */

  ros::NodeHandle private_handle_; /**< A node handle used by every subscriber, publisher and action server */

  ros::Subscriber goal_sub_;       /**< A subscriber to the goal topic */
  ros::Publisher path_pub_;        /**< A publisher for the global plan */
  ros::Publisher cmd_vel_pub_;     /**< A publisher for the velocity command */
  ros::Publisher action_goal_pub_; /**< A publisher for the action goal */
  ros::Publisher cycle_pub_;        /**< A publisher for the duration of a cycle */

  geometry_msgs::PoseStamped goal_;  /**< The goal pose */
  geometry_msgs::PoseStamped start_; /**< The start pose */

  std::vector<geometry_msgs::PoseStamped> global_plan_; /**< The global plan followed by the local planner */

  GoToActionServer action_server_; /**< The action server of the GoTo action */
  GoToResult action_result_;       /**< A dummy result of the GoTo action */
  GoToFeedback action_feedback_;   /**< A feedback of the GoTo action */
};
}  // namespace move_pguard
#endif  // __MOVE_PGUARD__
