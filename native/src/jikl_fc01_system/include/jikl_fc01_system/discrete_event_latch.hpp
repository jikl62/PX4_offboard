#pragma once
#include <array>
#include <cstddef>
#include <deque>
#include <utility>

namespace fc01 {

// A bounded FIFO for discrete feedback observations. Continuous samples may
// continue replacing their live cache, but an event observation stays queued
// until the adapter presents it to Runtime::step. A normal callback therefore
// cannot clear an event that arrived earlier in the same spin_some(). The
// SystemNode removes the front entry only after the complete step (including
// its publication boundary) returns; a subsequent healthy sample is allowed
// to update the continuous cache but never acknowledges this queue. The
// bounded capacity is deliberate: an event storm fails closed instead of
// growing control-loop memory without limit.
template<class T, std::size_t Capacity = 32>
class DiscreteEventLatch {
  static_assert(Capacity > 0, "event latch capacity must be positive");
 public:
  bool push(const T& value) {
    if (values_.size() >= Capacity) { overflow_ = true; return false; }
    values_.push_back(value); return true;
  }
  bool empty() const { return values_.empty(); }
  std::size_t size() const { return values_.size(); }
  const T* front() const { return values_.empty() ? nullptr : &values_.front(); }
  void pop() { if (!values_.empty()) values_.pop_front(); }
  bool overflowed() const { return overflow_; }
  void clear() { values_.clear(); overflow_ = false; }
 private:
  std::deque<T> values_;
  bool overflow_{};
};

} // namespace fc01
