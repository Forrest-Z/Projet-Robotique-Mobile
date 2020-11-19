#include "move_pguard/local/basic_local_planner.h"

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "move_pguard/goal_finder/hv_finder.h"
#include "move_pguard/utils.h"

namespace move_pguard
{
namespace local
{
BasicLocalPlanner::BasicLocalPlanner()
  : global_plan_()
  , subgoal_()
  , velocity_max_(0.5F) //1.4F)
  , velocity_gain_(1.2F) //2.0F)
  , angular_velocity_max_(0.5F) //0.7F)
  , angular_velocity_gain_(1.6F) //2.0F)
  , angular_slowdown_gain_(0.8 / M_PI)
  , min_goal_distance2_(1.0F)
  , path_planner_()
  , costmap_ros_(nullptr)
  , duration_threshold_(2.0)
  , safe_radius_(5.0F)
  , critical_radius_(0.6F)
{
  ros::NodeHandle n("~");
  path_pub_ = n.advertise<nav_msgs::Path>("local/path", 1);
  std::string lidar_topic = n.param<std::string>("lidar_topic", "/laserfront/scan");
  lidar_sub_ = n.subscribe(lidar_topic, 1, &BasicLocalPlanner::laserScanCallback, this);
}

BasicLocalPlanner::~BasicLocalPlanner()
{
}

void BasicLocalPlanner::initialize(std::string name, tf::TransformListener*, costmap_2d::Costmap2DROS* costmap_ros)
{
  costmap_ros_ = costmap_ros;
  path_planner_.initialize(name + "/planner", costmap_ros);
  initializeConfigServer(name);
}

void BasicLocalPlanner::configCallback(LocalPlannerConfig& config, unsigned int)
{
  critical_radius_ = config.critical_radius;
  safe_radius_ = config.safe_radius;
  velocity_max_ = config.max_linear_velocity;
  angular_velocity_max_ = config.max_angular_velocity;
  velocity_gain_ = config.velocity_gain;
  angular_velocity_gain_ = config.angular_velocity_gain;
  angular_slowdown_gain_ = config.angular_slowdown_gain;

  // Ensure the critical radius is lower than the safe radius
  if (safe_radius_ < critical_radius_)
  {
    safe_radius_ = critical_radius_;
    config.safe_radius = critical_radius_;
  }

  ROS_INFO_STREAM("New configuration: {Crit radius: " << critical_radius_ <<  //
                  ", Safe radius: " << safe_radius_ <<                        //
                  ", Max linear velocity: " << velocity_max_ <<               //
                  ", Max angular velocity: " << angular_velocity_max_ << '}');
}

bool BasicLocalPlanner::isSubgoalReached()
{
  if (global_plan_.size() > 2)
  {
    // In the middle of the path. Go to the next point if enough time has passed.
    ros::Duration remaining_time = estimateRemainingTime();
    if (!remaining_time.isZero() && remaining_time < duration_threshold_)
      return true;
  }

  geometry_msgs::PoseStamped robot_pose = getRobotPose(*costmap_ros_);
  tf2::Vector3 p_robot;
  tf2::fromMsg(robot_pose.pose.position, p_robot);

  tf2::Vector3 p_subgoal;
  tf2::fromMsg(subgoal_.pose.position, p_subgoal);

  tf2::Vector3 p_relative_subgoal = p_subgoal - p_robot;
  return p_relative_subgoal.length2() < 1;
}

bool BasicLocalPlanner::selectSubgoal()
{
  geometry_msgs::PoseStamped robot_pose = getRobotPose(*costmap_ros_);
  // Find the nearest pose to the subgoal from the global plan
  unsigned int i_max = global_plan_.size();
  unsigned int pose_index;
  double min_distance(INFINITY);
  for (unsigned int i = 0; i < i_max; ++i)
  {
    double dist = distance(robot_pose, global_plan_.at(i));
    if (dist < min_distance)
    {
      pose_index = i;
      min_distance = dist;
    }
  }

  if (pose_index + 1 < i_max)
  {
    // We have not reached the end; go to next pose
    subgoal_ = global_plan_.at(pose_index + 1);
    return true;
  }
  // Reached the end
  return false;
}

bool BasicLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  global_plan_ = plan;
  return selectSubgoal();
}

bool BasicLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // Plan a path locally to the subgoal
  std::vector<geometry_msgs::PoseStamped> local_plan;
  geometry_msgs::PoseStamped robot_pose = getRobotPose(*costmap_ros_);
  // Make the goal suitable
  goal_finder::HVFinder goal_finder;
  subgoal_ = goal_finder(*costmap_ros_->getCostmap(), robot_pose, subgoal_);
  if (!path_planner_.makePlan(robot_pose, subgoal_, local_plan))
  {
    ROS_WARN_STREAM("Cannot find valid local path");
    return false;
  }
  nav_msgs::Path p;
  p.header = robot_pose.header;
  p.poses = local_plan;
  path_pub_.publish(p);
  if (isSubgoalReached() && !selectSubgoal())
  {
    // Subgoal reached and cannot find another subgoal. Either the goal
    // is reached, or there is a failure
    if (isGoalReached())
    {
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Cannot find next subgoal");
      return false;
    }
  }
  // Follow the local path
  if (local_plan.size() > 1)
  {
    geometry_msgs::PoseStamped& target_pose = local_plan.at(1);
    // Compute the angular velocity
    float angle_to_target = angle(robot_pose, target_pose);
    cmd_vel.angular.z = angular_velocity_gain_ * angle_to_target;
    cmd_vel.angular.z = std::min(cmd_vel.angular.z, angular_velocity_max_);
    cmd_vel.angular.z = std::max(cmd_vel.angular.z, -angular_velocity_max_);
    // Compute the linear velocity
    cmd_vel.linear.x = velocity_gain_ * distance(robot_pose, target_pose);
    cmd_vel.linear.x = std::min(cmd_vel.linear.x, velocity_max_);
    cmd_vel.linear.x *= std::max(1.0 - angular_slowdown_gain_ * std::abs(angle_to_target), 0.0);
    cmd_vel.linear.x *= safety_;
    ROS_WARN_STREAM_COND(safety_ < 1e-2, "Obstacle critically near, stopping the robot");
    return true;
  }
  return false;
}

bool BasicLocalPlanner::isGoalReached()
{
  if (global_plan_.size() == 0)
    // There is no goal. Consider the goal as reached
    return true;

  geometry_msgs::PoseStamped robot_pose = getRobotPose(*costmap_ros_);
  const geometry_msgs::PoseStamped& goal_pose = global_plan_.back();
  return distance(robot_pose, goal_pose) < min_goal_distance2_;
}

ros::Duration BasicLocalPlanner::estimateRemainingTime()
{
  if (velocity_max_ < 1e-5)
    return ros::Duration(); // Too slow, return invalid duration
  geometry_msgs::PoseStamped robot_pose = getRobotPose(*costmap_ros_);
  ros::Duration remaining_time(distance(subgoal_, robot_pose) / velocity_max_ / 0.95);
  return remaining_time;
}

void BasicLocalPlanner::laserScanCallback(const sensor_msgs::LaserScan& msg)
{
  // Compute the safety gauge
  //float nearest_obstacle = *std::min_element(msg.ranges.begin(), msg.ranges.end());
  float angle = msg.angle_min;
  float angle_offset = (msg.angle_max + msg.angle_min) / 2.0;
  float nearest_obstacle = msg.ranges.front() * (2 / M_PI * std::abs(angle - angle_offset) + 1);
  for (unsigned int i = 1; i < msg.ranges.size(); ++i)
  {
    angle += msg.angle_increment;
    float distance = msg.ranges.at(i) * (2 / M_PI * std::abs(angle - angle_offset) + 1);
    if (distance < nearest_obstacle)
      nearest_obstacle = distance;
  }
  safety_ = (nearest_obstacle - critical_radius_) / (safe_radius_ - critical_radius_);
  safety_ = std::max(safety_, 0.0F);
  safety_ = std::min(safety_, 1.0F);
}
}  // namespace local
}  // namespace move_pguard
