#include "move_pguard/local/path_follower.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

namespace move_pguard
{
namespace local
{
PathFollower::PathFollower() : max_dist_from_goal_(0.05), speed_(1), l1_(0.1)
{
}

void PathFollower::initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS* costmap_ros)
{
  costmap_ros_ = costmap_ros;
}

bool PathFollower::isGoalReached(const geometry_msgs::PoseStamped& goal)
{
}

bool PathFollower::isGoalReached()
{
  if (plan_.empty())
    return true;

  tf2::Vector3 p_robot;
  double th_robot;
  getRobotPose(p_robot, th_robot);
  tf2::Vector3 target = computeTargetPosition(p_robot, th_robot);

  tf2::Vector3 p_goal;
  tf2::fromMsg(plan_.at(plan_.size() - 1).pose.position, p_goal);

  return target.distance(p_goal) < max_dist_from_goal_;
}

void PathFollower::findNearestSegment(tf2::Vector3& begin, tf2::Vector3& end)
{
  tf2::Vector3 robot_pos;
  double robot_orientation;
  getRobotPose(robot_pos, robot_orientation);

  size_t nearest_waypoint_index(0);
  double nearest_waypoint_distance2(1e10);
  tf2::Vector3 position;
  for (size_t i = 0; i < plan_.size(); ++i)
  {
    tf2::fromMsg(plan_.at(i).pose.position, position);
    double distance2 = robot_pos.distance2(position);
    if (distance2 < nearest_waypoint_distance2)
    {
      nearest_waypoint_index = i;
      nearest_waypoint_distance2 = distance2;
      begin = position;
    }
  }
  size_t segment_end_index;
  if (nearest_waypoint_index == 0)
    segment_end_index = 1;
  else if (nearest_waypoint_index == plan_.size() - 1)
    segment_end_index = nearest_waypoint_index - 1;
  else
  {
    tf2::Vector3 next_waypoint, previous_waypoint;
    tf2::fromMsg(plan_.at(nearest_waypoint_index + 1).pose.position, next_waypoint);
    tf2::fromMsg(plan_.at(nearest_waypoint_index - 1).pose.position, previous_waypoint);
    if (robot_pos.distance2(next_waypoint) < robot_pos.distance2(previous_waypoint))
      segment_end_index = nearest_waypoint_index + 1;
    else
      segment_end_index = nearest_waypoint_index - 1;
  }
  tf2::fromMsg(plan_.at(segment_end_index).pose.position, end);
}

void PathFollower::findNearestPoint(const tf2::Vector3& begin, const tf2::Vector3& end, double& d, double& theta_e)
{
  tf2::Vector3 robot_position;
  double robot_orientation;
  getRobotPose(robot_position, robot_orientation);
  tf2::Vector3 target = computeTargetPosition(robot_position, robot_orientation);

  tf2::Vector3 segment = end - begin;
  tf2::Vector3 point = begin + (target - begin).dot(segment) * segment.normalized();

  double theta_seg = std::atan2(segment.y(), segment.x());
  theta_e = robot_orientation - theta_seg;
  d = target.distance(point);
}

bool PathFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  tf2::Vector3 segment_b, segment_e;
  findNearestSegment(segment_b, segment_e);
  double d, theta_e;
  findNearestPoint(segment_b, segment_e, d, theta_e);

  double C_theta_e = std::cos(theta_e);
  double gain = std::abs(C_theta_e);
  double angular_speed = speed_ * (-std::tan(theta_e) / l1_ - gain * d / C_theta_e);
}

void PathFollower::getRobotPose(tf2::Vector3& position, double& theta)
{
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  tf2::fromMsg(robot_pose.pose.position, position);

  tf2::Quaternion quaternion;
  tf2::fromMsg(robot_pose.pose.orientation, quaternion);
  tf2::Matrix3x3 rotation(quaternion);

  double roll, pitch;
  rotation.getEulerYPR(theta, pitch, roll);
}

tf2::Vector3 PathFollower::computeTargetPosition(const tf2::Vector3& robot, double orientation)
{
  return robot + tf2::Vector3(l1_ * std::cos(orientation), l1_ * std::sin(orientation), 0);
}
}  // namespace local
}  // namespace move_pguard
