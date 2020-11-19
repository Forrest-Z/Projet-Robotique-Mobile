#include "move_pguard/bresenham_line.h"

#include <cmath>

namespace move_pguard
{
BresenhamLine::BresenhamLine(Pose2D start, Pose2D end)
  : start_i_(start.x), start_j_(start.y), end_i_(end.x), end_j_(end.y)
{
  di_ = end_i_ - start_i_;
  dj_ = end_j_ - start_j_;
  // Ensure |dj / di| < 1, swap X and Y if necessary
  is_steep_ = std::abs(dj_) >= std::abs(di_);
  if (is_steep_)
  {
    start_i_ = start.y;
    start_j_ = start.x;
    end_i_ = end.y;
    end_j_ = end.x;
    di_ = end_i_ - start_i_;
    dj_ = end_j_ - start_j_;
  }
  // Ensure di_ >= 0, iterate the other way around if necessary
  i_step_ = 1;
  if (di_ < 0)
  {
    i_step_ = -1;
    di_ *= -1;
  }
  // Ensure dj_ >= 0, iterate the other way around if necessary
  j_step_ = 1;
  if (dj_ < 0)
  {
    j_step_ = -1;
    dj_ *= -1;
  }
}

Pose2D BresenhamLine::getStart() const
{
  Pose2D pose{ .x = start_i_, .y = start_j_ };
  if (is_steep_)
  {
    pose.x = start_j_;
    pose.y = start_i_;
  }
  return pose;
}

Pose2D BresenhamLine::getEnd() const
{
  Pose2D pose{ .x = end_i_, .y = end_j_ };
  if (is_steep_)
  {
    pose.x = end_j_;
    pose.y = end_i_;
  }
  return pose;
}

int BresenhamLine::getStepX() const
{
  return is_steep_ ? j_step_ : i_step_;
}

int BresenhamLine::getStepY() const
{
  return is_steep_ ? i_step_ : j_step_;
}

std::list<Pose2D> BresenhamLine::line() const
{
  std::list<Pose2D> line;
  int f = 2 * dj_ - di_;  // Decision parameter
  int j = start_j_;
  for (int i = start_i_; i != end_i_; i += i_step_)
  {
    Pose2D pose{ .x = i, .y = j };
    if (is_steep_)
    {
      pose.x = j;
      pose.y = i;
    }
    line.push_back(pose);
    // Update the decision parameter and select the next cell
    if (f > 0)
    {
      j += j_step_;
      f -= 2 * di_;
    }
    f += 2 * dj_;
  }
  return line;
}

BresenhamLine::Iterator BresenhamLine::begin()
{
  Iterator itr(this, start_i_, start_j_, 2 * di_, 2 * dj_, 2 * dj_ - di_);
  return itr;
}

BresenhamLine::Iterator BresenhamLine::end()
{
  Iterator itr(this, end_i_, end_j_, 2 * di_, 2 * dj_, 0);
  return itr;
}

BresenhamLine::BresenhamLine::Iterator::Iterator(BresenhamLine* line, int i, int j, int two_di, int two_dj, int f)
  : line_(line), i_(i), j_(j), f_(f), two_di_(two_di), two_dj_(two_dj)
{
}

bool BresenhamLine::Iterator::operator==(const Iterator& other)
{
  return line_ == other.line_ && i_ == other.i_;
}

bool BresenhamLine::Iterator::operator!=(const Iterator& other)
{
  return !(*this == other);
}

Pose2D BresenhamLine::Iterator::operator*()
{
  Pose2D pose{ .x = i_, .y = j_ };
  if (line_->isSteep())
  {
    pose.x = j_;
    pose.y = i_;
  }
  return pose;
}

BresenhamLine::Iterator BresenhamLine::Iterator::operator++(int)
{
  Iterator old(*this);
  operator++();
  return old;
}

BresenhamLine::Iterator& BresenhamLine::Iterator::operator++()
{
  // Update the decision parameter and select the next cell
  if (f_ > 0)
  {
    j_ += line_->getStepJ();
    f_ -= two_di_;
  }
  f_ += two_dj_;
  i_ += line_->getStepI();
  return *this;
}
}  // namespace move_pguard
