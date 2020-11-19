#ifndef __MAP_H__
#define __MAP_H__

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace move_minilab
{
class Map
{
public:
  Map();
  ~Map() = default;

  void update();

  char at(size_t pixel_x, size_t pixel_y);

  char operator[](size_t index);

private:

  ros::ServiceClient client_;
  nav_msgs::OccupancyGrid map_;
};
}  // namespace move_minilab
#endif  // __MAP_H__
