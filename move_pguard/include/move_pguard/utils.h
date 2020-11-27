#ifndef __COSTMAP_UTILS_H__
#define __COSTMAP_UTILS_H__

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace move_pguard
{
using index_t = unsigned int;

/**
 * Get the pose of the robot from the Costmap2DROS
 *
 * This function is used for compatibility accross ROS Kinetic and ROS Melodic.
 *
 *
 * @param costmap The costmap
 * @return The pose of the robot
 */
geometry_msgs::PoseStamped getRobotPose(const costmap_2d::Costmap2DROS& costmap);

/**
 * Measure the angle between the two planar orientation
 *
 * @param from The initial pose
 * @param to The final pose
 * @return The angle along the Z axis (in rad in [-pi, pi[)
 */
double angle(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to);

/**
 * Measure the distance between the two plannar position
 *
 * @param from The initial pose
 * @param to The final pose
 * @return The distance (in m) between the two position
 */
float distance(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to);

/**
 * Predicate that tests if a cell is out of the costmap
 *
 * @param costmap The costmap to test
 * @param x The X coordinate in the map grid
 * @param y The Y coordinate in the map grid
 * @return `true` if the coordinate is out of the map otherwise `false`
 */
bool isOutOfBound(const costmap_2d::Costmap2D& costmap, int x, int y);
}  // namespace move_pguard
#endif  // __COSTMAP_UTILS_H__
