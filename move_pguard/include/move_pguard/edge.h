#ifndef __EDGE_H__
#define __EDGE_H__

namespace move_pguard
{
  struct Edge {
    unsigned int predecessor;
    float cost;
    bool is_open;
  };
}
#endif // __EDGE_H__
