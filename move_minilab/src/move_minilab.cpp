#include "move_minilab/move_minilab.h"

#include <nav_msgs/GetMap.h>

namespace move_minilab
{
MoveMinilab::MoveMinilab()
{
  ros::NodeHandle n;
  map_.update();
}

MoveMinilab::~MoveMinilab()
{
}

}  // namespace move_minilab
