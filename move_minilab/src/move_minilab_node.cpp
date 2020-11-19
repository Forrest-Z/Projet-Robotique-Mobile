#include <ros/console.h>
#include <ros/ros.h>

#include "move_minilab/move_minilab.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "move_minilab");
  ros::NodeHandle n;

  ROS_INFO_STREAM("Hello world !");

  move_minilab::MoveMinilab move;
  ros::spin();
  return 0;
}
