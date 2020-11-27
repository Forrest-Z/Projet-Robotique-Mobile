#ifndef __BRESENHAM_LINE_H__
#define __BRESENHAM_LINE_H__

#include <iterator>
#include <list>

namespace move_pguard
{
struct Pose2D
{
  int x;
  int y;
};

/**
 * Implementation of the Bresenham line algorithm
 */
class BresenhamLine
{
public:
  class Iterator : public std::iterator<std::input_iterator_tag, Pose2D>
  {
  public:
    Iterator(BresenhamLine* line, int i, int j, int two_di, int two_dj, int f);
    Iterator(const Iterator& other) = default;

    Iterator& operator++();
    Iterator operator++(int);

    bool operator==(const Iterator& other);
    bool operator!=(const Iterator& other);

    Pose2D operator*();

  private:
    BresenhamLine* line_;
    int i_;
    int j_;
    int f_;
    int two_di_;
    int two_dj_;
  };
  /**
   * Construct a new Bresenham Line
   *
   * @param start The start pose
   * @param end The end pose
   */
  BresenhamLine(Pose2D start, Pose2D end);

  /**
   * Get the start pose
   *
   * @return the start pose
   */
  Pose2D getStart() const;

  /**
   * Get the end pose
   *
   * @return the end pose
   */
  Pose2D getEnd() const;

  /**
   * Get the step of the line on the X axis
   *
   * @return the step fo the line on the X axis
   */
  int getStepX() const;

  /**
   * Get the step of the line on the Y axis
   *
   * @return the step of the line of the Y axis
   */
  int getStepY() const;

  /**
   * Get the step of the line on the I axis
   *
   * The I axis depends on @ref{isSteep()}, when `true` it is the Y axis
   * otherwise it is the X axis
   *
   * @return The step of the line on the I axis
   */
  int getStepI() const
  {
    return i_step_;
  }

  /**
   * Get the step of the line on the J axis
   *
   * The J axis depends on @ref{isSteep()}, when `true` it is the X axis
   * otherwise it is the Y axis
   *
   * @return The step of the line on the J axis
   */
  int getStepJ() const
  {
    return j_step_;
  }

  /**
   * Predicate that test if the line is steep
   *
   * @return `true` if the line is steep; otherwise `false`
   */
  bool isSteep() const
  {
    return is_steep_;
  }

  /**
   * List all the points of the line
   *
   * @return A vector of the points for this line
   */
  std::list<Pose2D> line() const;

  /**
   * Give an iterator at the beginning of the line
   *
   * @return an iterator at the beginning of the line
   */
  Iterator begin();

  /**
   * Give an iterator at the end of the line
   *
   * @return an iterator at the end of the line
   */
  Iterator end();

protected:
  int start_i_;   /**< The start i coordinate (i is X if not steep otherwise Y) */
  int start_j_;   /**< The start j coordinate (j is Y if not steep otherwise X) */
  int end_i_;     /**< The end i coordinate (i is X if not steep otherwise Y) */
  int end_j_;     /**< The end j coordinate (j is Y if not steep otherwise X) */
  int di_;        /**< The difference between the i coordinates */
  int dj_;        /**< The difference between the j coordinates */
  int i_step_;    /**< The step on the i axis*/
  int j_step_;    /**< The step on the j axis*/
  bool is_steep_; /**< Flag for steep line */
};
}  // namespace move_pguard
#endif  // __BRESENHAM_LINE_H__
