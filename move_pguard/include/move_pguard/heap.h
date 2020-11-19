#ifndef __HEAP_H__
#define __HEAP_H__

#include <ostream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace move_pguard
{
/**
 * Heap data structure with the smallest element at the top.
 *
 * @tparam T The type to store
 * @tparam P The type of the priority
 */
template <typename T, typename P>
class Heap
{
public:
  /**
   * Insert an element
   *
   * @param element The element
   * @param priority The priority of the element
   */
  void insert(const T& element, const P& priority);

  /**
   * Get the top element
   *
   * @return the top element
   */
  const T& top() const;

  /**
   * Get the top element and its priority
   *
   * @param priority *[out]* The priority of the element
   * @return the top element
   */
  const T& top(P& priority) const;

  /**
   * Remove the top element
   */
  void pop();

  /**
   * Change the priority of an element
   *
   * @param element The element
   * @param priority The new priority
   */
  void changePriority(const T& element, const P& priority);

  /**
   * Get the priority of an element
   *
   * @param element The element
   * @return The priority of the element
   */
  const P& getPriority(const T& element);

  /**
   * Get the size of the heap
   *
   * @return The size of the heap
   */
  inline unsigned int size() const
  {
    return heap_.size();
  }

  /**
   * Predicate that test if an element is in the heap
   *
   * @param element The element to test
   * @return `true` if the element is in this heap; otherwise `false`
   */
  bool contains(const T& element);

  /**
   * Remove the elements from this heap
   */
  void clear();

  friend std::ostream& operator<<(std::ostream& os, const Heap<T, P>& heap)
  {
    unsigned int pow2 = 1;
    for (unsigned int i = 0; i < heap.size(); i++)
    {
      if (i == pow2 - 1)
      {
        os << '\n';
        pow2 <<= 1;
      }
      os << heap.heap_.at(i).second << ": " << (int)heap.heap_.at(i).first << '\t';
    }
    os << "\nDict: ";
    for (auto elem_pair : heap.dict_)
    {
      os << elem_pair.first << ": " << elem_pair.second << "\t";
    }
    os << std::endl;
    return os;
  }

private:
  /**
   * Get the index of the parent
   *
   * @param index The index of the child
   * @return The index of the parent
   * @throw std::invalid_argument When requesting the parent of index 0
   */
  unsigned int parent(unsigned int index) const;

  /**
   * Get the index of the left child
   *
   * @param index The index of the parent
   * @return The index of the left child
   */
  inline unsigned int left_child(unsigned int index) const
  {
    return 2 * (index + 1) - 1;
  }

  /**
   * Get the index of the right child
   *
   * @param index The index of the parent
   * @return The index of the right child
   */
  inline unsigned int right_child(unsigned int index) const
  {
    return 2 * (index + 1);
  }

  /**
   * Raise an element to its valid position
   *
   * @param index The index of the element to raise
   */
  void raise(unsigned int index);

  /**
   * Lower an element to its valid position
   *
   * @param index The index of the element to lower
   */
  void lower(unsigned int index);

  /**
   * Swap the elements at the given indexes
   *
   * @param i1 The first element to swap
   * @param i2 The second element to swap
   */
  void swap(unsigned int i1, unsigned int i2);

  std::vector<std::pair<P, T>> heap_;        /**< The actual heap */
  std::unordered_map<T, unsigned int> dict_; /**< Map to keep track of the position of the element in the heap */
};

template <typename T, typename P>
void Heap<T, P>::insert(const T& element, const P& priority)
{
  // Add the new element to the heap
  std::pair<P, T> elem_pair(priority, element);
  unsigned int index = heap_.size();
  heap_.push_back(elem_pair);
  // Store its index
  dict_.insert(std::pair<T, unsigned int>(element, index));
  // Sort the heap
  raise(index);
}

template <typename T, typename P>
const T& Heap<T, P>::top() const
{
  // Retrieve the first element
  const std::pair<P, T>& top_pair = heap_.at(0);
  return top_pair.second;
}

template <typename T, typename P>
const T& Heap<T, P>::top(P& priority) const
{
  // Retrieve the first element
  const std::pair<P, T>& top_pair = heap_.at(0);
  priority = top_pair.first;
  return top_pair.second;
}

template <typename T, typename P>
void Heap<T, P>::pop()
{
  // Store the previous top element
  T top_elem = top();
  // Swap with the last element and shorten the heap then sort the top
  // element
  swap(0, heap_.size() - 1);
  heap_.pop_back();
  // Erase the previous top element from the dictionnary
  auto itr = dict_.find(top_elem);
  dict_.erase(itr);
  lower(0);
}

template <typename T, typename P>
void Heap<T, P>::changePriority(const T& element, const P& priority)
{
  // Find the element in the heap
  unsigned int index = dict_.at(element);
  std::pair<P, T>& index_pair = heap_.at(index);
  // Set the new priority
  P old_priority = index_pair.first;
  index_pair.first = priority;
  // Sort the heap
  if (priority < old_priority)
  {
    raise(index);
  }
  else
  {
    lower(index);
  }
}

template <typename T, typename P>
const P& Heap<T, P>::getPriority(const T& element)
{
  unsigned int index = dict_.at(element);
  std::pair<P, T>& index_pair = heap_.at(index);
  return index_pair.first;
}

template <typename T, typename P>
bool Heap<T, P>::contains(const T& element)
{
  return dict_.find(element) != dict_.end();
}

template <typename T, typename P>
void Heap<T, P>::clear()
{
  heap_.clear();
  dict_.clear();
}

template <typename T, typename P>
unsigned int Heap<T, P>::parent(unsigned int index) const
{
  if (index == 0)
    throw std::logic_error("Heap::parent : Index 0 has no parent");
  return (index + 1) / 2 - 1;
}

template <typename T, typename P>
void Heap<T, P>::raise(unsigned int index)
{
  if (index == 0)
    // Cannot raise more, we are done
    return;
  // Find the parent of the element
  unsigned int parent_index = parent(index);
  // If the element is smaller than its parent, swap them and repeat
  if (heap_.at(index).first < heap_.at(parent_index).first)
  {
    swap(parent_index, index);
    raise(parent_index);
  }
}

template <typename T, typename P>
void Heap<T, P>::lower(unsigned int index)
{
  if (index + 1 > heap_.size())
    // Cannot lower more, we are done
    return;
  std::pair<P, T> index_pair = heap_.at(index);
  // Test if the left child is smaller
  unsigned int lchild = left_child(index);
  unsigned int rchild = right_child(index);
  if (lchild < heap_.size())
  {
    // The left child exists
    std::pair<P, T> lchild_pair = heap_.at(lchild);
    if (rchild < heap_.size())
    {
      // Both children exist
      std::pair<P, T> rchild_pair = heap_.at(rchild);
      // Select the smallest child, swap if necessary and keep lowering
      if (lchild_pair.first < rchild_pair.first)
      {
        if (lchild_pair.first < index_pair.first)
        {
          // the parent is bigger than its child, swap them, keep
          // lowering
          swap(lchild, index);
          lower(lchild);
        }
      }
      else
      {
        if (rchild_pair.first < index_pair.first)
        {
          // the parent is bigger than its child, swap them, keep
          // lowering
          swap(rchild, index);
          lower(rchild);
        }
      }
    }
    else if (lchild_pair.first < index_pair.first)
    {
      // The right child does not exist and the parent is bigger than
      // its child, swap them, keep lowering
      swap(lchild, index);
      lower(lchild);
    }
  }
  else if (rchild < heap_.size())
  {
    // The left child does not exist but the right child exists
    std::pair<P, T> rchild_pair = heap_.at(rchild);
    if (rchild_pair.first < index_pair.first)
    {
      // the parent is bigger than its child, swap them, keep lowering
      swap(rchild, index);
      lower(rchild);
    }
  }
}

template <typename T, typename P>
void Heap<T, P>::swap(unsigned int i1, unsigned int i2)
{
  if (i1 == i2)
    // Do not swap the same index, cancel
    return;
  // Swap in the heap
  std::pair<P, T> elem1 = heap_.at(i1);
  std::pair<P, T> elem2 = heap_.at(i2);
  heap_.at(i1) = elem2;
  heap_.at(i2) = elem1;
  // Update the indexes
  dict_.at(elem1.second) = i2;
  dict_.at(elem2.second) = i1;
}
}  // namespace move_pguard
#endif  // __HEAP_H__
