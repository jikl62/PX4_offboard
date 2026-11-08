#pragma once
#include <array>
#include <cstddef>
#include <deque>
#include <utility>

namespace fc01 {

// A bounded FIFO separate from the newest continuous cache. The adapter
// scans every queued safety payload before planning output, and acknowledges
// the batch only after the complete step and final sinks return. A later
// healthy callback cannot erase an earlier unsafe payload. Overflow is sticky
// and fails closed instead of growing control-loop memory without limit.
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
  template<class Observe> void for_each(Observe observe) const {
    for (const auto& value : values_) observe(value);
  }
  void pop() { if (!values_.empty()) values_.pop_front(); }
  void acknowledge_all() { values_.clear(); }
  bool overflowed() const { return overflow_; }
  void clear() { values_.clear(); overflow_ = false; }
 private:
  std::deque<T> values_;
  bool overflow_{};
};

} // namespace fc01
