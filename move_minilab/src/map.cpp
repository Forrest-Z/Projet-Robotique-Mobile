#include "move_minilab/map.h"

#include <nav_msgs/GetMap.h>
#include <ros/console.h>

#include <stdexcept>

namespace move_minilab
{
Map::Map()
{
  ros::NodeHandle n;
  client_ = n.serviceClient<nav_msgs::GetMap>("/dynamic_map");
}

char Map::at(size_t pixel_x, size_t pixel_y)
{
  int width = map_.info.width;
  int height = map_.info.height;

  if (pixel_x < 0 || pixel_x > width || pixel_y < 0 || pixel_y > height)
    throw std::logic_error("Out of bounds!");
  return map_.data[pixel_y * width + pixel_x];
}

void Map::update()
{
  nav_msgs::GetMap querry;
  bool success = client_.call(querry);
  if (success)
  {
    ROS_DEBUG_STREAM("Successfuly retreaved the map");
    map_ = querry.response.map;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to retreave the map");
  }
}
}  // namespace move_minilab
