#include <costmap_2d/costmap_2d_ros.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <string>
#include <vector>

#include "move_pguard/global/dijkstra_planner.h"
#include "move_pguard/move_pguard.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "move_pguard");
  ros::NodeHandle node_handle;

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tl(buffer);
  costmap_2d::Costmap2DROS global_costmap("map/global", buffer);
  costmap_2d::Costmap2DROS local_costmap("map/local", buffer);

  move_pguard::MovePGuard move_pguard("driver", &buffer, &global_costmap, &local_costmap);

  ros::spin();
  return 0;
}
