#ifndef __BASIC_LOCAL_PLANNER_H__
#define __BASIC_LOCAL_PLANNER_H__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "move_pguard/global/theta_star_planner.h"
#include "move_pguard/local/a_local_planner.h"

namespace move_pguard
{
namespace local
{
  /**
   * Local planner which uses a theta* algorithm to find a local path and a
   * proportional corrector to follow this subplan.
   */
class BasicLocalPlanner : public ALocalPlanner
{
public:
  BasicLocalPlanner(); /**< Construct a new Basic Local Planner */

  ~BasicLocalPlanner();

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
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

  /**
   * Initialize  this local planner
   *
   * @param name The name of this local planner
   * @param tf A pointer to a transform listener
   * @param costmap_ros The costmap to use for assigning costs to local plans
   */
  virtual void initialize(std::string name,  tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

protected:
  /**
   * Predicate that tests if the subgoal is reached
   *
   * @return `true` if the subgoal is reached; otherwise `false`
   */
  bool isSubgoalReached();

  /**
   * Select a subgoal
   *
   * Find the nearest waypoint, in the plan, to the robot. Select the child of
   * this waypoint as the new subgoal.
   *
   * @return `true` if a subgoal has been found; otherwise `false`
   */
  bool selectSubgoal();

  /**
   * Estimate the duration of the trip to the subgoal
   *
   * @return A estimation of the duration of the trip to the subgoal or ros::Duration() when
   * the speed is 0 m/s.
   */
  ros::Duration estimateRemainingTime();

  /**
   * Callback used by the subscriber when a laser scan measurement is received
   *
   * Compute the safety gauge from the distance to the nearest obstacle.
   *
   * @param msg The laser scan message
   */
  void laserScanCallback(const sensor_msgs::LaserScan& msg);

  virtual void configCallback(LocalPlannerConfig& config, unsigned int level) override;

  std::vector<geometry_msgs::PoseStamped> global_plan_; /**< The global plan to follow */
  geometry_msgs::PoseStamped subgoal_;                  /**< The next pose to reach within the global plan */

  double velocity_max_;  /**< The maximum linear velocity (in m/s) */
  double velocity_gain_; /**< The gain of a proportional corrector (in s^-1) */

  double angular_velocity_max_;  /**< The maximum angular velocity (in rad/s) */
  double angular_velocity_gain_; /**< The gain of a proportianal corrector (in s^-1) */
  double angular_slowdown_gain_; /**< The gain of the slowdown factor based on the angular error (in rad^-1) */

  /** Threshold of the squared minimal distance between the robot and its target (in m^2) */
  double min_goal_distance2_;

  global::ThetaStarPlanner path_planner_; /**< The path planner used locally to avoid collisions */
  costmap_2d::Costmap2DROS* costmap_ros_; /**< The local costmap */
  ros::Duration duration_threshold_;      /**< A travel duration threshold to target the next subgoal */
  ros::Publisher path_pub_;               /**< Publisher of the local path */
  ros::Subscriber lidar_sub_;             /**< Subscriber to the LiDAR*/
  float safety_;                          /**< A safety gauge (in [0, 1]) */
  float safe_radius_;                     /**< The safe distance from any obstacle (in m) */
  float critical_radius_;                 /**< The critical distance from any obstacle (in m) */
};
}  // namespace local
}  // namespace move_pguard

#endif  // __BASIC_LOCAL_PLANNER_H__
