#ifndef __MOVE_MINILAB_H__
#define __MOVE_MINILAB_H__

#include "move_minilab/map.h"

namespace move_minilab
{
class MoveMinilab
{
public:
  MoveMinilab();
  ~MoveMinilab();

private:
  Map map_;
};
}  // namespace move_minilab

#endif  // __MOVE_MINILAB_H__
