#pragma once
// 合作式 DDS 补读预算：保持原 spin_some 内部回调顺序，只决定能否再开始一轮。
// now/budget 单位为单调秒；回调计数/安全事件计数只能递增，倒退说明上下文无效。
// 它不能抢占某个阻塞回调；最终 CurrentControlCycle 和发布门仍负责回调后的控制截止检查。
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fc01 {
// 停止原因区分空闲、安全事件、时钟/预算/计数错误和资源上限，便于实际调度诊断。
enum class ReceivePassStop {
  Running,
  Idle,
  Event,
  InvalidClock,
  InvalidBudget,
  InvalidCounter,
  Budget,
  PassLimit
};
inline const char* receive_pass_stop_name(ReceivePassStop value) {
  switch (value) {
  case ReceivePassStop::Running:
    return "RUNNING";
  case ReceivePassStop::Idle:
    return "IDLE";
  case ReceivePassStop::Event:
    return "SAFETY_EVENT";
  case ReceivePassStop::InvalidClock:
    return "INVALID_MONOTONIC_CLOCK";
  case ReceivePassStop::InvalidBudget:
    return "INVALID_RECEIVE_BUDGET";
  case ReceivePassStop::InvalidCounter:
    return "COUNTER_REGRESSION";
  case ReceivePassStop::Budget:
    return "BUDGET_EXHAUSTED";
  case ReceivePassStop::PassLimit:
    return "PASS_LIMIT";
  }
  return "UNKNOWN";
}

// begin在控制周期开始时保存累计计数；每次next_pass只回答“还可不可以开始下一整轮spin_some”。
// 一轮内可能执行很多回调，调度器无法抢占正在执行的回调；max_passes=10限制的是轮数而非消息数。
// 第一轮不因接收预算已消耗完而被省略，但无效时钟/计数、或应先处理的新事件仍会拒绝它。
//
// callbacks累计所有接收回调，events累计“应先交给控制步骤”的观察：
// 后者可以是需要留证的危险变化，也可以是促使控制尽早运行的测量候选；其内容由适配层判断。
// 默认一见新event就结束额外补读，使中间观察先到Runtime/Core，避免健康尾帧覆盖它。
// Safety无输出观察可设置stop_on_events=false继续有限补读，但适配层仍必须保留每个离散载荷。
// 本类不解析测量、不修改超时/控制权；一次停止保持到下一周期begin，不被后来好消息重新打开。
class ReceivePassSchedule {
public:
  // 每个控制周期最多十轮；不是每轮最多十个回调，第一轮保持原无限 spin_some 语义。
  static constexpr std::size_t max_passes = 10;
  // 保存本周期预算起点和原计数；stop_on_events=false 只适用于保留全部离散证据的无输出安全观察。
  void begin(double now, double budget, std::uint64_t callbacks, std::uint64_t events,
             bool stop_on_events = true) {
    // begin代表新控制周期，不是每轮补读都调用；若每轮重开会把预算/上限不断续长。
    started_ = observed_ = now;
    budget_ = budget;
    callbacks_ = callbacks;
    events_ = events;
    passes_ = 0;
    stop_on_events_ = stop_on_events;
    reason_ = ReceivePassStop::Running;
    if (!std::isfinite(now) || now < 0)
      reason_ = ReceivePassStop::InvalidClock;
    else if (!std::isfinite(budget) || budget <= 0)
      reason_ = ReceivePassStop::InvalidBudget;
  }
  // 停止原因一旦设定保持到下一次 begin，后续正常时间不能使同周期重新开始补读。
  bool next_pass(double now, std::uint64_t callbacks, std::uint64_t events) {
    if (reason_ != ReceivePassStop::Running)
      return false;
    // 首拒绝顺序本身是诊断合同：时钟→计数→新事件→空闲→预算→轮数上限。
    // 后面即使同时有多个停止原因，也保留这里最先命中的那个。
    if (!std::isfinite(now) || now < observed_)
      return stop(ReceivePassStop::InvalidClock);
    observed_ = now;
    // 计数应是进程累计总量；小于上轮保存值不是“没消息”，而是当前观察上下文不可信。
    if (callbacks < callbacks_ || events < events_)
      return stop(ReceivePassStop::InvalidCounter);
    // 新安全事件优先结束补读，控制步骤必须先消费它，不能追新掩盖。
    if (stop_on_events_ && events != events_)
      return stop(ReceivePassStop::Event);
    // passes_是已批准轮数，初值0；if(passes_)等价于“至少批准过一轮”。
    // 因此第一轮不查空闲/预算/上限，之后才根据上一整轮有没有新回调决定是否继续。
    if (passes_) {
      if (callbacks == callbacks_)
        return stop(ReceivePassStop::Idle);
      if (now - started_ >= budget_)
        return stop(ReceivePassStop::Budget);
      if (passes_ >= max_passes)
        return stop(ReceivePassStop::PassLimit);
    }
    // 只有真的批准新一轮才提交计数基线并+1；它不表示该轮回调已经执行完成。
    // 调用者拿到true后才运行spin_some，结束后带新的累计计数再来询问。
    callbacks_ = callbacks;
    events_ = events;
    ++passes_;
    return true;
  }
  std::size_t pass_count() const {
    return passes_;
  }
  ReceivePassStop reason() const {
    return reason_;
  }
  const char* stop_reason() const {
    return receive_pass_stop_name(reason_);
  }
  double elapsed_seconds() const {
    // 最后一次合法时钟观察与begin起点之差；不是“每个回调都保证在此时间内返回”。
    return observed_ - started_;
  }

private:
  bool stop(ReceivePassStop reason) {
    // 参数reason是本次原因，reason_是保存到周期结束的成员；返回false让调用者退出补读。
    reason_ = reason;
    return false;
  }
  // started_/observed_ 是单调时间；callbacks_/events_ 只缓存前次允许轮的计数。
  double started_{}, observed_{}, budget_{};
  std::uint64_t callbacks_{}, events_{};
  bool stop_on_events_{true};
  std::size_t passes_{};
  ReceivePassStop reason_{ReceivePassStop::InvalidClock};
};
} // namespace fc01
