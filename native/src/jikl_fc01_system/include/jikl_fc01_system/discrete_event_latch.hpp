#pragma once
// 离散安全事件锁存：FIFO 保存每个需要被控制周期看到的负面观测。
// 它与“最新连续测量”缓存分开，后来的正常消息不能覆盖本周期较早的危险事件。
// 上层只有在完整控制步骤和最终 sink 返回后才 acknowledge_all；溢出为粘性故障。
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
  // 队列容量固定，满时拒绝新项并保留 overflow_，不能靠无限扩容拖垮常驻控制进程。
  bool push(const T& value) {
    if (values_.size() >= Capacity) { overflow_ = true; return false; }
    values_.push_back(value); return true;
  }
  bool empty() const { return values_.empty(); }
  std::size_t size() const { return values_.size(); }
  const T* front() const { return values_.empty() ? nullptr : &values_.front(); }
  // 只读遍历全部保留事件；不能因为最后一项健康就跳过此前的失联、接管或 reset。
  template<class Observe> void for_each(Observe observe) const {
    for (const auto& value : values_) observe(value);
  }
  void pop() { if (!values_.empty()) values_.pop_front(); }
  // 确认仅清队列，不清溢出证据；clear 是显式重建操作，调用责任属于上层。
  void acknowledge_all() { values_.clear(); }
  bool overflowed() const { return overflow_; }
  void clear() { values_.clear(); overflow_ = false; }
 private:
  // 值按到达顺序保存；不存外部引用，避免 ROS 回调结束后悬空。
  std::deque<T> values_;
  bool overflow_{};
};

} // namespace fc01
