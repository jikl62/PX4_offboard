#pragma once
// 离散安全事件锁存：FIFO保存需要被控制步骤逐条观察的事件，包括危险与恢复边沿。
// 它与“最新连续测量”缓存分开，后来的正常消息不能覆盖本周期较早的危险事件。
// 上层只有在完整控制步骤和最终 sink 返回后才 acknowledge_all；溢出为粘性故障。
#include <array>
#include <cstddef>
#include <deque>
#include <utility>

namespace fc01 {

// 连续测量缓存只留最新值，本FIFO则保留离散观测的到达次序。节点先逐项观察再计算控制计划，
// 控制步与最终sink完整结束后才确认这批事件；正常尾帧不能洗掉失联/Kill/接管等中间事实。
// template中T为事件类型，Capacity为编译期容量；队列满时锁存溢出，拒绝靠无限扩容维持控制。
template <class T, std::size_t Capacity = 32> class DiscreteEventLatch {
  static_assert(Capacity > 0, "event latch capacity must be positive");

public:
  // 队列容量固定，满时拒绝新项并保留 overflow_，不能靠无限扩容拖垮常驻控制进程。
  bool push(const T& value) {
    if (values_.size() >= Capacity) {
      overflow_ = true;
      return false;
    }
    values_.push_back(value);
    return true;
  }
  bool empty() const {
    return values_.empty();
  }
  std::size_t size() const {
    return values_.size();
  }
  const T* front() const {
    // 返回成员元素的只读地址，空队列返回nullptr。pop/clear/销毁后不能继续保存这个借用指针。
    return values_.empty() ? nullptr : &values_.front();
  }
  // 只读遍历全部保留事件；不能因为最后一项健康就跳过此前的失联、接管或 reset。
  template <class Observe> void for_each(Observe observe) const {
    // observe是调用者给的函数/lambda，按值传入；const auto&逐项借用事件，不复制也不修改队列。
    for (const auto& value : values_)
      observe(value);
  }
  void pop() {
    if (!values_.empty())
      values_.pop_front();
  }
  // 确认仅清队列，不清溢出证据；clear 是显式重建操作，调用责任属于上层。
  void acknowledge_all() {
    values_.clear();
  }
  bool overflowed() const {
    return overflow_;
  }
  void clear() {
    // clear比acknowledge_all更强：连overflow_也重置；必须由明确重建路径调用，不能当普通健康更新。
    values_.clear();
    overflow_ = false;
  }

private:
  // 值按到达顺序保存；不存外部引用，避免 ROS 回调结束后悬空。
  std::deque<T> values_;
  bool overflow_{};
};

} // namespace fc01
