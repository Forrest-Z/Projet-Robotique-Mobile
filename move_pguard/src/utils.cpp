#include "move_pguard/utils.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

namespace move_pguard
{
geometry_msgs::PoseStamped getRobotPose(const costmap_2d::Costmap2DROS& costmap)
{
  geometry_msgs::PoseStamped robot_pose;
  costmap.getRobotPose(robot_pose);
  return robot_pose;
}

double angle(const geometry_msgs::PoseStamped& pose_from, const geometry_msgs::PoseStamped& pose_to)
{
  tf2::Quaternion q_to;
  tf2::fromMsg(pose_to.pose.orientation, q_to);

  tf2::Quaternion q_from;
  tf2::fromMsg(pose_from.pose.orientation, q_from);

  tf2::Matrix3x3 relative_rotation(q_from.inverse() * q_to);

  double roll, pitch, yaw;
  relative_rotation.getEulerYPR(yaw, pitch, roll);
  return yaw;
}

double angle(const geometry_msgs::Quaternion& quat_msgs)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(quat_msgs, quaternion);

  tf2::Matrix3x3 rotation(quaternion);

  double roll, pitch, yaw;
  rotation.getEulerYPR(yaw, pitch, roll);
  return yaw;
}

float distance(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to)
{
  float dx = to.pose.position.x - from.pose.position.x;
  float dy = to.pose.position.y - from.pose.position.y;
  float dz = to.pose.position.z - from.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool isOutOfBound(const costmap_2d::Costmap2D& costmap, int x, int y)
{
  return 0 > x || (unsigned int)x >= costmap.getSizeInCellsX() || 0 > y || (unsigned int)y >= costmap.getSizeInCellsY();
}
}  // namespace move_pguard
