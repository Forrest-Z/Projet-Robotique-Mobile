#include "move_pguard/move_pguard.h"

#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>

#include <functional>
#include <stdexcept>

#include "move_pguard/bresenham_line.h"
#include "move_pguard/global/theta_star_planner.h"
#include "move_pguard/goal_finder/hv_finder.h"
// #include "move_pguard/local/basic_local_planner.h"
#include "move_pguard/local/path_follower.h"
#include "move_pguard/plan_checker.h"
#include "move_pguard/utils.h"

namespace cm = costmap_2d;

namespace move_pguard
{
MovePGuard::MovePGuard(std::string name, tf2_ros::Buffer* buffer, cm::Costmap2DROS* global_costmap_ros,
                       cm::Costmap2DROS* local_costmap_ros)
  : global_planner_(nullptr)
  , local_planner_(nullptr)
  , global_costmap_ros_(global_costmap_ros)
  , local_costmap_ros_(local_costmap_ros)
  , private_handle_("~")
  , goal_()
  , action_server_(private_handle_, "go_to", boost::bind(&MovePGuard::executeCallback, this, _1), false)
{
  if (global_costmap_ros_ == nullptr)
    throw std::invalid_argument("MovePGuard::MovePGuard: Null global costmap");
  if (local_costmap_ros_ == nullptr)
    throw std::invalid_argument("MovePGuard::MovePGuard: Null local costmap");
  // Prepare the planners
  global_planner_ = GlobalPlannerS(new global::ThetaStarPlanner());
  global_planner_->initialize(name + "/global", global_costmap_ros_);
  ROS_INFO_STREAM("Global planner: OK");
  // local_planner_ = LocalPlannerS(new local::BasicLocalPlanner());
  local_planner_ = LocalPlannerS(new local::PathFollower());
  local_planner_->initialize(name + "/local", buffer, local_costmap_ros_);
  ROS_INFO_STREAM("Local planner: OK");

  // Subscribe to the goal topic
  std::string goal_topic = private_handle_.param<std::string>("goal", "/move_base_simple/goal");
  goal_sub_ = private_handle_.subscribe(goal_topic, 5, &MovePGuard::goalCallback, this);
  // Advertise the GoTo action's goal topic
  action_goal_pub_ = private_handle_.advertise<GoToActionGoal>("go_to/goal", 5);
  // Advertise the path topic
  std::string path_topic = private_handle_.param<std::string>("path", "path");
  path_pub_ = private_handle_.advertise<nav_msgs::Path>(path_topic, 1);
  goal_.header.stamp = ros::Time();
  // Advertise the command velocity topic
  std::string cmd_vel_topic = private_handle_.param<std::string>("cmd_vel", "/cmd_vel");
  cmd_vel_pub_ = private_handle_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 5, true);

  cycle_pub_ = private_handle_.advertise<std_msgs::Float32>("cycle", 5);
  action_server_.start();
}

MovePGuard::~MovePGuard()
{
}

bool MovePGuard::planGlobalPath(const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path)
{
  geometry_msgs::PoseStamped start_pose = getRobotPose(*global_costmap_ros_);
  // Find a suitable goal
  goal_finder::HVFinder goal_finder;
  geometry_msgs::PoseStamped suitable_goal = goal_finder(*global_costmap_ros_->getCostmap(), start_pose, goal);
  // Find the path
  path.header.frame_id = goal.header.frame_id;
  path.header.stamp = goal.header.stamp;
  bool success = global_planner_->makePlan(start_pose, suitable_goal, path.poses);
  // Update the clock and the time
  goal_ = goal;
  start_ = start_pose;
  return success;
}

void MovePGuard::goalCallback(const geometry_msgs::PoseStamped& goal_pose)
{
  // Send the goal to the action server
  GoToActionGoal action_goal;
  action_goal.goal.target_pose = goal_pose;
  action_goal.header.stamp = ros::Time::now();
  action_goal_pub_.publish(action_goal);
}

void MovePGuard::executeCallback(const GoToGoalConstPtr& goal)
{
  goal_ = goal->target_pose;
  try
  {
    updateGlobalPlan(true);  // Try to make a first plan
  }
  catch (const std::runtime_error& exception)
  {
    action_server_.setAborted(action_result_, "Failed to find a first valid plan");
  }
  ros::Rate rate(15);
  ros::Duration expected_cycle_time = rate.expectedCycleTime();
  while (private_handle_.ok() && action_server_.isActive())
  {
    if (action_server_.isPreemptRequested())
    {
      if (action_server_.isNewGoalAvailable())
      {
        // Change the goal to the new one
        GoToGoalConstPtr new_goal = action_server_.acceptNewGoal();
        goal_ = new_goal->target_pose;
        try
        {
          updateGlobalPlan(true);
        }
        catch (const std::runtime_error& exception)
        {
          action_server_.setAborted(action_result_, "Failed to find a first valid plan");
          break;
        }
      }
      else
      {
        action_server_.setPreempted(action_result_, "Goal preempted");
        break;
      }
    }
    // Check if the goal is reached
    if (local_planner_->isGoalReached())
    {
      ROS_INFO_STREAM("Goal reached!");
      publishZeroVelocity();
      action_server_.setSucceeded();
      break;
    }
    try
    {
      updateGlobalPlan();
      control();
    }
    catch (const std::runtime_error& exception)
    {
      ROS_ERROR_STREAM(exception.what());
      action_server_.setAborted(action_result_, exception.what());
      break;
    }
    // Send some feedback
    action_feedback_.distance = distance(getRobotPose(*global_costmap_ros_), goal_);
    action_feedback_.description = "Moving...";
    action_server_.publishFeedback(action_feedback_);
    rate.sleep();
    // Cycle duration feedback
    ros::Duration cycle_time = rate.cycleTime();
    std_msgs::Float32 msg;
    msg.data = cycle_time.toSec();
    cycle_pub_.publish(msg);
    if (cycle_time > expected_cycle_time)
    {
      ROS_WARN_STREAM("[GoTo] The loop took too long: " << cycle_time  //
                                                        << " (expected " << expected_cycle_time << ')');
    }
  }
  if (!private_handle_.ok())
  {
    // When the node is killed, we stop the robot and we abort
    ROS_ERROR_STREAM("The node has been killed");
    action_server_.setAborted(action_result_, "The node has been killed");
  }
  // For any reason the action has ended, we stop the robot
  publishZeroVelocity();
}

void MovePGuard::publishZeroVelocity()
{
  geometry_msgs::Twist msg;
  msg.angular.x = 0.0F;
  msg.angular.y = 0.0F;
  msg.angular.z = 0.0F;
  msg.linear.x = 0.0F;
  msg.linear.y = 0.0F;
  msg.linear.z = 0.0F;
  cmd_vel_pub_.publish(msg);
}

void MovePGuard::control()
{
  // Compute the velocity command. On failure: stop the robot & abort
  geometry_msgs::Twist cmd_vel;
  bool is_valid_vel = local_planner_->computeVelocityCommands(cmd_vel);
  if (is_valid_vel)
    cmd_vel_pub_.publish(cmd_vel);
  else
    throw std::runtime_error("Failed to find a valid velocity command");
}

void MovePGuard::updateGlobalPlan(bool is_forced)
{
  // Update the global path every 1 s (approximately) or if there is a
  // new goal
  ros::Time now = ros::Time::now();
  if (is_forced || now - start_.header.stamp >= ros::Duration(1, 0))
  {
    // Check the distance to the origin or the obstruction of the path
    PlanChecker checker(&global_plan_);
    cm::Costmap2D& costmap = *global_costmap_ros_->getCostmap();
    std::function<bool(unsigned char)> criteria = [](unsigned char cost) {
      return cost >= 240 && cost != cm::NO_INFORMATION;
    };
    if (is_forced)// || checker.isObstructed(costmap, criteria))
    {
      // Make the plan and store it
      ROS_INFO_STREAM("Updating the global path");
      nav_msgs::Path path;
      bool is_valid_path = planGlobalPath(goal_, path);
      if (!is_valid_path)
        throw std::runtime_error("Failed to find a valid plan");
      global_plan_ = path.poses;
      // Send some feedback
      local_planner_->setPlan(path.poses);
      path_pub_.publish(path);
      action_feedback_.description = "Global plan updated";
      action_server_.publishFeedback(action_feedback_);
    }
  }
}
}  // namespace move_pguard
