// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// CircularBuffer<T> is a double-ended circular buffer with some
// configurable capacity for holding elements of type T.
//
// When a CircularBuffer<T> has reached capacity and a new element is
// pushed in at either end, an element from the opposite end will be
// evicted to make space for it.
//
// It is thread-compatible. Users must provide thread safety externally if
// needed.

#ifndef EIGENMATH_TF2_EIGENMATH_TF2_CIRCULAR_BUFFER_H_
#define EIGENMATH_TF2_EIGENMATH_TF2_CIRCULAR_BUFFER_H_

#include <algorithm>
#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>

#include "genit/iterator_facade.h"

namespace eigenmath {
namespace tf2 {

// Forward declaration.
template <typename T>
class CircularBufferIterator;

// Double-ended circular buffer class.
// CircularBuffer contains raw storage for capacity() objects of type T.
//
// When the CircularBuffer is full(), i.e. when size() == capacity(),
// pushing a new element to one end will evict an element from the
// opposite end.
//
// When the CircularBuffer is not full(), pushing an element will place
// a copy of the pushed object into the appropriate uninitialized buffer
// slot via placement new.
template <typename T>
class CircularBuffer {
 public:
  using value_type = T;
  using pointer = T*;
  using const_pointer = const T*;
  using reference = T&;
  using const_reference = const T&;
  using size_type = int;
  using difference_type = int;

  typedef CircularBufferIterator<T> iterator;
  typedef CircularBufferIterator<const T> const_iterator;
  typedef std::reverse_iterator<iterator> reverse_iterator;
  typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

  CircularBuffer() : CircularBuffer(0) {}
  explicit CircularBuffer(size_type c)
      : capacity_(c), begin_(0), size_(0), space_(Allocate(capacity_)) {}

  CircularBuffer(const CircularBuffer& rhs) : CircularBuffer(rhs.capacity_) {
    size_ = rhs.size_;
    pointer p = space_;
    for (const auto& e : rhs) {
      Construct(p++, e);
    }
  }

  CircularBuffer(CircularBuffer&& rhs) noexcept
      : capacity_(rhs.capacity_),
        begin_(rhs.begin_),
        size_(rhs.size_),
        space_(rhs.space_) {
    rhs.capacity_ = 0;
    rhs.begin_ = 0;
    rhs.size_ = 0;
    rhs.space_ = nullptr;
  }

  void swap(CircularBuffer& rhs) noexcept {
    using std::swap;
    swap(capacity_, rhs.capacity_);
    swap(begin_, rhs.begin_);
    swap(size_, rhs.size_);
    swap(space_, rhs.space_);
  }

  friend void swap(CircularBuffer& a, CircularBuffer& b) noexcept { a.swap(b); }

  CircularBuffer& operator=(CircularBuffer rhs) noexcept {
    swap(rhs);
    return *this;
  }

  ~CircularBuffer() {
    clear();
    Deallocate(space_, capacity_);
  }

  // Reallocates and sets capacity. Items from the front up to the
  // new_capacity are kept, e.g. [0, new_capacity).
  // Post: capacity() == new_capacity
  // Post: size() == min(old_size, new_capacity)
  void ChangeCapacity(size_type new_capacity) {
    if (new_capacity == capacity_) {
      return;
    }
    CircularBuffer tmp(new_capacity);
    std::copy(std::make_move_iterator(begin()),
              std::make_move_iterator(begin() + std::min(size_, new_capacity)),
              std::back_inserter(tmp));
    swap(tmp);
  }

  // Push an item onto the beginning of the buffer. begin_ is moved
  // circularly to the left.
  // Requires: value_type is CopyConstructible and CopyAssignable.
  void push_front(const value_type& item) { emplace_front(item); }
  // Requires: value_type is MoveConstructible and MoveAssignable.
  void push_front(value_type&& item) { emplace_front(std::move(item)); }

  // Push an item onto the end of the buffer. begin_ is moved
  // circularly to the right if the buffer is full.
  // Requires: value_type is CopyConstructible and CopyAssignable.
  void push_back(const value_type& item) { emplace_back(item); }
  // Requires: value_type is MoveConstructible and MoveAssignable.
  void push_back(value_type&& item) { emplace_back(std::move(item)); }

  // Emplace an item onto the front of the buffer. begin_ is moved circularly to
  // the left.
  template <typename... Args>
  void emplace_front(Args&&... args) {
    if (!capacity_) {
      return;
    }
    begin_ = prevpos(begin_);
    if (full()) {
      Assign(space_ + begin_, std::forward<Args>(args)...);
      return;
    }
    ++size_;
    Construct(space_ + begin_, std::forward<Args>(args)...);
  }

  // Emplace an item onto the back of the buffer. begin_ is moved circularly to
  // the right if the buffer is full.
  template <typename... Args>
  void emplace_back(Args&&... args) {
    if (!capacity_) {
      return;
    }
    if (full()) {
      Assign(space_ + begin_, std::forward<Args>(args)...);
      begin_ = nextpos(begin_);
      return;
    }
    ++size_;
    Construct(space_ + logical_to_absolute(size_ - 1),
              std::forward<Args>(args)...);
  }

  // Remove & destroy the front element.
  void pop_front() {
    if (size_ == 0) {
      return;
    }
    Destroy(&front());
    begin_ = nextpos(begin_);
    --size_;
  }

  // Remove & destroy the back element.
  void pop_back() {
    if (size_ == 0) {
      return;
    }
    Destroy(&back());
    --size_;
  }

  iterator begin() { return iterator(this, 0); }
  const_iterator begin() const { return const_iterator(this, 0); }

  iterator end() { return iterator(this); }
  const_iterator end() const { return const_iterator(this); }

  reverse_iterator rbegin() { return reverse_iterator(end()); }
  const_reverse_iterator rbegin() const {
    return const_reverse_iterator(end());
  }
  reverse_iterator rend() { return reverse_iterator(begin()); }
  const_reverse_iterator rend() const {
    return const_reverse_iterator(begin());
  }

  const_reference front() const { return space_[begin_]; }
  reference front() { return space_[begin_]; }

  const_reference back() const {
    return space_[logical_to_absolute(size_ - 1)];
  }
  reference back() { return space_[logical_to_absolute(size_ - 1)]; }

  size_type size() const { return size_; }
  size_type capacity() const { return capacity_; }

  bool empty() const { return size_ == 0; }
  bool full() const { return size_ == capacity_; }

  // For pos >= 0, returns the item at logical position 'pos'.
  // For pos < 0, returns the item at logical position 'pos + size()'
  reference at(difference_type pos) {
    size_type logical = pos + (pos < 0) * size_;
    return space_[logical_to_absolute(logical)];
  }
  const_reference at(difference_type pos) const {
    size_type logical = pos + (pos < 0) * size_;
    return space_[logical_to_absolute(logical)];
  }

  reference operator[](size_type pos) { return at(pos); }
  const_reference operator[](size_type pos) const { return at(pos); }

  void clear() noexcept(noexcept(std::declval<pointer>()->~value_type())) {
    for (size_type i = 0; i < size_; ++i) {
      Destroy(space_ + logical_to_absolute(i));
    }
    begin_ = 0;
    size_ = 0;
  }

 private:
  template <typename... U>
  static void Assign(pointer p, U&&... v) {
    Destroy(p);
    Construct(p, std::forward<U>(v)...);
  }

  static void Assign(pointer p, const value_type& v) { *p = v; }

  static void Assign(pointer p, value_type&& v) { *p = std::move(v); }

  template <typename... U>
  static void Construct(pointer p, U&&... v) {
    new (p) value_type(std::forward<U>(v)...);
  }
  static void Destroy(pointer p) noexcept(noexcept(p->~value_type())) {
    p->~value_type();
  }

  static pointer Allocate(size_type n) {
    return std::allocator<value_type>().allocate(n);
  }

  static void Deallocate(pointer p, size_type n) {
    std::allocator<value_type>().deallocate(p, n);
  }

  reference at_absolute(size_type pos) { return space_[pos]; }
  const_reference at_absolute(size_type pos) const { return space_[pos]; }

  // Pre: logical in [0, size).
  size_type logical_to_absolute(size_type logical) const {
    size_type absolute = begin_ + logical;
    if (absolute >= capacity_) {
      absolute -= capacity_;
    }
    return absolute;
  }

  // Pre: absolute in [0, capacity).
  size_type absolute_to_logical(size_type absolute) const {
    size_type logical = capacity_ - begin_ + absolute;
    if (logical >= capacity_) {
      logical -= capacity_;
    }
    return logical;
  }

  // Pre: absolute in [0, capacity).
  size_type nextpos(size_type absolute) const {
    ++absolute;
    if (absolute == capacity_) {
      absolute = 0;
    }
    return absolute;
  }

  // Pre: absolute in [0, capacity).
  size_type prevpos(size_type absolute) const {
    if (absolute == 0) {
      absolute += capacity_;
    }
    --absolute;
    return absolute;
  }

  size_type capacity_;
  size_type begin_;
  size_type size_;
  value_type* space_;
};

// Iterators are invalidated by modification to the circular buffer.
template <typename T>
class CircularBufferIterator
    : public genit::IteratorFacade<CircularBufferIterator<T>, T&,
                                   std::random_access_iterator_tag> {
 private:
  using non_const_value_type = std::remove_const_t<T>;
  using non_const_iter_type = CircularBufferIterator<non_const_value_type>;

  using container_type =
      std::conditional_t<std::is_const_v<T>,
                         const CircularBuffer<non_const_value_type>,
                         CircularBuffer<T>>;

  friend class genit::IteratorFacadePrivateAccess<CircularBufferIterator>;

 public:
  CircularBufferIterator(container_type* cb, int logical_pos)
      : cb_(cb), logical_pos_(logical_pos) {}

  explicit CircularBufferIterator(container_type* cb)
      : CircularBufferIterator(cb, cb->size()) {}

  CircularBufferIterator() : CircularBufferIterator(nullptr, 0) {}

  // For const_iterator, this defines an implicit conversion from iterator.
  // For iterator, this defines a copy constructor.
  // NOLINTNEXTLINE(google-explicit-constructor)
  CircularBufferIterator(const non_const_iter_type& rhs)
      : CircularBufferIterator(rhs.GetContainerPointer(),
                               rhs.GetLogicalPosition()) {}

  // For access for conversion from non-const to const iterator.
  int GetLogicalPosition() const { return logical_pos_; }
  container_type* GetContainerPointer() const { return cb_; }

 private:
  T& Dereference() const { return cb_->at(logical_pos_); }
  bool IsEqual(const CircularBufferIterator& rhs) const {
    return logical_pos_ == rhs.logical_pos_;
  }
  void Advance(int n) { logical_pos_ += n; }
  void Increment() { ++logical_pos_; }
  void Decrement() { --logical_pos_; }
  int DistanceTo(const CircularBufferIterator& rhs) const {
    return rhs.logical_pos_ - logical_pos_;
  }

  container_type* cb_;  // not owned
  int logical_pos_;     // logical position in *cb_.
};

// relational operators
template <typename T>
inline bool operator==(const CircularBuffer<T>& a, const CircularBuffer<T>& b) {
  return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin());
}

template <typename T>
inline bool operator<(const CircularBuffer<T>& a, const CircularBuffer<T>& b) {
  return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end());
}

template <typename T>
inline bool operator!=(const CircularBuffer<T>& a, const CircularBuffer<T>& b) {
  return !(a == b);
}

template <typename T>
inline bool operator>(const CircularBuffer<T>& a, const CircularBuffer<T>& b) {
  return b < a;
}

template <typename T>
inline bool operator<=(const CircularBuffer<T>& a, const CircularBuffer<T>& b) {
  return !(a > b);
}

template <typename T>
inline bool operator>=(const CircularBuffer<T>& a, const CircularBuffer<T>& b) {
  return !(a < b);
}

}  // namespace tf2
}  // namespace eigenmath

#endif  // EIGENMATH_TF2_EIGENMATH_TF2_CIRCULAR_BUFFER_H_
