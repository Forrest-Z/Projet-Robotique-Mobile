#ifndef __BASIC_LOCAL_PLANNER_H__
#define __BASIC_LOCAL_PLANNER_H__

#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>

#include "move_pguard/local/a_local_planner.h"

namespace move_pguard
{
namespace local
{
/**
 * Local planner which uses a theta* algorithm to find a local path and a
 * proportional corrector to follow this subplan.
 */
class PathFollower : public nav_core::BaseLocalPlanner
{
public:
  PathFollower(); /**< Construct a new Basic Local Planner */

  virtual ~PathFollower()
  {
  }

  /**
   * Compute a velocity command to send to the base
   *
   * @param cmd_vel *[out]* The velocity command
   * @return `true` if a valid velocity command was found; otherwise `false`
   */
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  /**
   * Predicate that tests if the goal is reached
   *
   * @return `true` if the goal is reached; otherwise `false`
   */
  virtual bool isGoalReached() override;

  /**
   * Set the plan that this local planner is following
   *
   * @param plan The plan to pass to the local planner
   * @return `true` if the plan was updated successfully; otherwise `false`
   */
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override
  {
    plan_ = plan;
  }

  /**
   * Initialize  this local planner
   *
   * @param name The name of this local planner
   * @param tf A pointer to a transform listener
   * @param costmap_ros The costmap to use for assigning costs to local plans
   */
  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

protected:
  /**
   * Predicate that tests if the subgoal is reached
   *
   * @return `true` if the subgoal is reached; otherwise `false`
   */
  bool isGoalReached(const geometry_msgs::PoseStamped& goal);

  void findNearestPoint(const tf2::Vector3& begin, const tf2::Vector3& end, double& d, double& theta_e);

  void findNearestSegment(tf2::Vector3& begin, tf2::Vector3& end);

  void getRobotPose(tf2::Vector3& position, double& theta);

  tf2::Vector3 computeTargetPosition(const tf2::Vector3& robot, double orientation);

  costmap_2d::Costmap2DROS* costmap_ros_; /**< The local costmap */

  std::vector<geometry_msgs::PoseStamped> plan_; /**< The global plan to follow */
  geometry_msgs::PoseStamped subgoal_;           /**< The next pose to reach within the global plan */

  double angular_velocity_max_; /**< The maximum angular velocity (in rad/s) */
  double max_dist_from_goal_;
  double speed_;
  double l1_;
};
}  // namespace local
}  // namespace move_pguard

#endif  // __BASIC_LOCAL_PLANNER_H__