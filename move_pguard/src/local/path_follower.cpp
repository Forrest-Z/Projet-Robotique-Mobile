#include "move_pguard/local/path_follower.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

#include "move_pguard/utils.h"

namespace move_pguard
{
namespace local
{
PathFollower::PathFollower()
  : max_dist_from_goal_(0.05), max_angle_from_goal_(0.1), speed_(1), l1_(0.5), angular_velocity_max_(M_PI), kp_(10)
{
  ros::NodeHandle n("~");
  path_pub_ = n.advertise<nav_msgs::Path>("local/path", 1, true);
  target_pub_ = n.advertise<geometry_msgs::PoseStamped>("local/target", 1, true);
  proj_pub_ = n.advertise<geometry_msgs::PoseStamped>("local/proj", 1, true);
}

void PathFollower::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  costmap_ros_ = costmap_ros;
  tf_ = tf;
}

bool PathFollower::isGoalReached()
{
  return state_ == DONE;
}

bool PathFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  plan_ = plan;
  state_ = ROTATING;
  next_wp_index_ = 0;
  tf2::fromMsg(plan_.at(0).pose.position, segment_[0]);
  tf2::fromMsg(plan_.at(1).pose.position, segment_[1]);
  tf2::Vector3 vertex = segment_[1] - segment_[0];
  desired_orientation_ = std::atan2(vertex.y(), vertex.x());

  path_.header = plan_.at(0).header;
  path_.poses.clear();
  path_.poses.push_back(plan_.at(0));
  path_.poses.push_back(plan_.at(1));
  path_pub_.publish(path_);
}

bool PathFollower::updateSegment()
{
  bool is_done = false;
  tf2::Vector3 target = computeTargetPosition();
  geometry_msgs::PoseStamped target_msgs;
  target_msgs.header.frame_id = costmap_ros_->getGlobalFrameID();
  target_msgs.header.stamp = ros::Time::now();
  target_msgs.pose.position.x = target.x();
  target_msgs.pose.position.y = target.y();
  target_pub_.publish(target_msgs);
  tf2::Vector3 vertex = segment_[1] - segment_[0];

  if (vertex.dot(target - segment_[0]) > vertex.length2())  // The target is beyond the vertex
  {
    if (next_wp_index_ + 1 < plan_.size())
    {
      segment_[0] = segment_[1];
      next_wp_index_++;
      tf2::fromMsg(plan_.at(next_wp_index_).pose.position, segment_[1]);
      geometry_msgs::PoseStamped a = path_.poses.at(1);
      path_.poses.clear();
      path_.poses.push_back(a);
      path_.poses.push_back(plan_.at(next_wp_index_));
      path_pub_.publish(path_);
    }
    else
    {
      is_done = true;
    }
  }
  return is_done;
}

void PathFollower::findNearestPoint(const tf2::Vector3& begin, const tf2::Vector3& end, double& d, double& theta_e)
{
  tf2::Vector3 robot_position;
  double robot_orientation;
  getRobotPose(robot_position, robot_orientation);
  tf2::Vector3 target = computeTargetPosition();

  tf2::Vector3 vertex = end - begin;
  tf2::Vector3 tangent = vertex.normalized();
  tf2::Vector3 normal;
  normal.setX(-tangent.y());
  normal.setY(tangent.x());
  tf2::Vector3 point = begin + std::max((target - begin).dot(tangent), 0.0) * tangent;

  double theta_seg = std::atan2(vertex.y(), vertex.x());
  theta_e = robot_orientation - theta_seg;
  d = normal.dot(target - point);

  geometry_msgs::PoseStamped proj;
  proj.header.frame_id = costmap_ros_->getGlobalFrameID();
  proj.header.stamp = ros::Time::now();
  proj.pose.position.x = point.x();
  proj.pose.position.y = point.y();
  proj.pose.orientation.w = std::cos(theta_seg / 2);
  proj.pose.orientation.z = std::sin(theta_seg / 2);
  proj_pub_.publish(proj);
}

bool PathFollower::rotate(geometry_msgs::Twist& cmd_vel)
{
  tf2::Vector3 meas_pos;
  double meas_rot;
  getRobotPose(meas_pos, meas_rot);
  double error = desired_orientation_ - meas_rot;

  if (std::abs(error) < max_angle_from_goal_)
    return true;

  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = kp_ * error;
  return false;
}

bool PathFollower::follow(geometry_msgs::Twist& cmd_vel)
{
  if (updateSegment())
    return true;
  double d, theta_e;
  findNearestPoint(segment_[0], segment_[1], d, theta_e);

  double C_theta_e = std::cos(theta_e);
  double gain = 10 * std::abs(C_theta_e);
  double angular_speed = speed_ * (-std::tan(theta_e) / l1_ - gain * d / C_theta_e);

  cmd_vel.linear.x = speed_;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = angular_speed;
  return false;
}

bool PathFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (state_ == ROTATING)
  {
    if (rotate(cmd_vel))
    {
      if (next_wp_index_ == 0)
      {
        next_wp_index_ = 1;
        state_ = FOLLOWING;
      }
      else if (next_wp_index_ == plan_.size() - 1)
        state_ = DONE;
      else
      {
        ROS_WARN_STREAM("Invalid state ROTATING with wp index " << next_wp_index_);
        state_ = DONE;
      }
    }
  }
  else if (state_ == FOLLOWING)
  {
    if (follow(cmd_vel))
    {
      desired_orientation_ = angle(plan_.at(next_wp_index_).pose.orientation);
      ROS_INFO_STREAM("final orientation: " << desired_orientation_);
      state_ = ROTATING;
    }
  }
  if (std::isnan(cmd_vel.angular.z))
    return false;
  cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, angular_velocity_max_), -angular_velocity_max_);
  return true;
}

void PathFollower::getRobotPose(tf2::Vector3& position, double& theta)
{
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  tf2::fromMsg(robot_pose.pose.position, position);

  theta = angle(robot_pose.pose.orientation);
}

tf2::Vector3 PathFollower::computeTargetPosition()
{
  geometry_msgs::PointStamped target_bot, target_map;
  target_bot.header.frame_id = costmap_ros_->getBaseFrameID();
  target_bot.point.x = l1_;
  tf_->transform(target_bot, target_map, costmap_ros_->getGlobalFrameID());
  tf2::Vector3 target;
  tf2::fromMsg(target_map.point, target);
  target.setZ(0);
  return target;
}

}  // namespace local
}  // namespace move_pguard
