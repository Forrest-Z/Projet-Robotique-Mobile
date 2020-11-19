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

  tf::TransformListener transform_listener;
  costmap_2d::Costmap2DROS global_costmap("map/global", transform_listener);
  costmap_2d::Costmap2DROS local_costmap("map/local", transform_listener);

  move_pguard::MovePGuard move_pguard("driver", &transform_listener, &global_costmap, &local_costmap);

  ros::spin();
  return 0;
}
