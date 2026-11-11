#pragma once
// 控制周期时序：区别本轮发布资格与仅供显示的耗时统计。
// 所有 now/start/done 为 monotonic 秒；绝不拿墙上时钟与单调时钟直接相减。
// CurrentControlCycle 的失败在本周期内不可恢复；ControlTiming 的统计不授予权限。
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fc01 {
// The publication budget begins before executor callbacks. A check must use a
// newly sampled monotonic time AFTER its snapshot, so the snapshot's own work
// cannot be omitted. Failure remains closed until the next cycle begins.
class CurrentControlCycle {
public:
  // 在执行 ROS 回调前开始预算，避免把回调耗时漏出控制超时计算。
  void begin(double now) {
    started_ = observed_ = now;
    callbacks_complete_ = false;
    valid_ = std::isfinite(now) && now >= 0;
  }
  // 只允许记录一次回调结束；时间无效/倒退则本轮资格永久失效。
  void callbacks_done(double now) {
    if (!valid_ || callbacks_complete_ || !std::isfinite(now) || now < started_) {
      valid_ = false;
      return;
    }
    observed_ = now;
    callbacks_complete_ = true;
  }
  // 调用方在完成快照后重新采时；这样快照构造自身的时间也计入截止预算。
  bool current(double now, double stall_limit) {
    if (!valid_ || !callbacks_complete_ || !std::isfinite(now) || !std::isfinite(stall_limit) ||
        stall_limit <= 0 || now < observed_ || now - started_ > stall_limit) {
      valid_ = false;
      return false;
    }
    observed_ = now;
    return true;
  }

private:
  bool valid_{}, callbacks_complete_{};
  double started_{}, observed_{};
};
// Observation of the last completed cycle; no timing metric grants authority.
// callback_seconds=回调耗时；control_seconds=回调后计算耗时；period_seconds=相邻起点间隔。
struct ControlTiming {
  bool valid{}, period_valid{};
  double callback_seconds{}, control_seconds{}, period_seconds{}, max_period_seconds{};
  std::uint64_t stall_count{};
  void observe(double start, double callbacks_done, double done, double stall_limit) {
    valid = std::isfinite(start) && std::isfinite(callbacks_done) && std::isfinite(done) &&
            callbacks_done >= start && done >= callbacks_done &&
            (previous_start_ < 0 || start >= previous_start_);
    // 无效观测不覆盖上一有效周期起点；period_valid=false 表示本次不能用周期值解释抖动。
    if (!valid) {
      period_valid = false;
      return;
    }
    callback_seconds = callbacks_done - start;
    control_seconds = done - callbacks_done;
    period_valid = previous_start_ >= 0;
    if (period_valid) {
      period_seconds = start - previous_start_;
      max_period_seconds = std::max(max_period_seconds, period_seconds);
      if (period_seconds > stall_limit)
        ++stall_count;
    }
    previous_start_ = start;
  }

private:
  double previous_start_{-1};
};
} // namespace fc01
