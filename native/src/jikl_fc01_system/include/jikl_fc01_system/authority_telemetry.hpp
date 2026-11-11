#pragma once
// PX4 遥控遥测适配：把选中的手动控制源和独立开关消息整理为控制权事实。
// 源 stamp 使用 PX4/Agent 同步后的微秒域；now 为本机单调秒，两类时钟分别验证。
// Facts 是观测结果而非执行命令；Kill 的已知 ON 不可被缺失、过期或无效时钟的 OFF 擦除。
#include "jikl_fc01_system/runtime.hpp"
#include <type_traits>
namespace fc01 {
// 编译期判断消息是否含四轴字段；旧/简化类型缺字段时不能伪造安全中杆和低油门。
template <class M, class = void> struct HasRcAxes : std::false_type {};
template <class M>
struct HasRcAxes<M,
                 std::void_t<decltype(std::declval<M>().roll), decltype(std::declval<M>().pitch),
                             decltype(std::declval<M>().yaw), decltype(std::declval<M>().throttle)>>
    : std::true_type {};
// Observation-only adapter shared by the real ROS node and deterministic tests.
// RC link readiness is derived from the selected ManualControlSetpoint source,
// not raw InputRc transport. ManualControlSwitches remains an independent
// RC_TAKEOVER requirement. Received Kill stays asserted until a newer OFF
// observation; staleness/unknown data must not turn a known Kill into OFF.
class AuthorityTelemetry {
public:
  // ready 与 fresh、源选择和源时钟分别记录，方便诊断具体哪条授权证据缺失。
  struct Facts {
    bool rc_link_ready{}, rc_switches_ready{}, kill{};
    bool manual_seen{}, manual_valid{}, rc_source_selected{}, sticks_moving{};
    bool switches_seen{}, switches_fresh{};
    bool rc_source_clock_valid{}, rc_switch_clock_valid{};
    std::uint8_t data_source{}, mode_slot{}, arm_switch{}, kill_switch{};
    std::uint64_t switch_generation{};
    bool rc_axes_safe{}, rc_aux_switches_safe{};
    double rc_throttle{NAN};
    std::uint64_t switch_sample_stamp{};
  };

  template <class Manual>
  void manual(const Manual& message, double now, std::uint64_t host_stamp = 0,
              double timeout = .5) {
    using M = std::decay_t<Manual>;

    const auto envelope = message.timestamp;
    const auto sample = message.timestamp_sample;

    // 拒绝无效、重复或过期的消息发布时间
    if (envelope == 0 || envelope <= manual_stamp_.value)
      return;
    if (host_stamp && !source_sample_in_clock(envelope, sample, host_stamp, timeout)) {
      manual_clock_rejected_ = true;
      return;
    }

    // 当前选中控制源失效时，及时撤销 RC 准入。
    // 即使原始采样时间没有推进，也不能继续沿用旧的有效状态。
    if (!message.valid || message.data_source != M::SOURCE_RC) {
      if (!manual_stamp_.accept(envelope, now))
        return;

      manual_seen_ = true;
      manual_clock_rejected_ = false;
      manual_valid_ = message.valid;
      data_source_ = message.data_source;
      source_rc_ = false;
      sticks_moving_ = false;
      axes_safe_ = false;
      throttle_ = NAN;
      return;
    }

    // 拒绝非法、重复或乱序的原始采样
    // 不调用 Stamp::accept()，避免污染其永久锁存状态
    if (sample == 0 || sample > envelope || sample <= manual_sample_stamp_.value)
      return;

    const double previous_sample = manual_sample_stamp_.received;

    // 两个时间戳均通过预检查后，更新跟踪器
    if (!manual_stamp_.accept(envelope, now))
      return;

    if (!manual_sample_stamp_.accept(sample, now))
      return;

    manual_timestamp_ = envelope;
    manual_previous_received_ = previous_sample;

    manual_seen_ = true;
    manual_clock_rejected_ = false;
    manual_valid_ = true;
    data_source_ = message.data_source;
    source_rc_ = true;
    sticks_moving_ = message.sticks_moving;
    observe_manual_axes(message);
  }

  template <class Switches>
  void switches(const Switches& message, double now, double timeout, std::uint64_t host_stamp = 0) {

    const auto envelope = message.timestamp;
    const auto sample = message.timestamp_sample;
    if (host_stamp && !source_sample_in_clock(envelope, sample, host_stamp, timeout)) {
      // An invalid-clock OFF cannot erase a known Kill. ON remains a veto.
      if (message.kill_switch == 1)
        kill_latched_ = true;
      switch_clock_rejected_ = true;
      return;
    }

    // 只接受时间戳有效且严格推进的样本
    if (envelope == 0 || sample == 0 || sample > envelope || envelope <= switch_stamp_.value ||
        sample <= sample_stamp_.value)
      return;

    const double previous_sample = sample_stamp_.received;

    if (!switch_stamp_.accept(envelope, now))
      return;

    if (!sample_stamp_.accept(sample, now))
      return;

    switch_timestamp_ = envelope;
    sample_timestamp_ = sample;
    sample_previous_received_ = previous_sample;

    observe_switch_values(message, timeout);
  }

  // 组合当前来源数量、失联标志、双时间戳和连续两帧进展；返回快照不改变内部缓存。
  Facts facts(double now, double timeout, bool flags_fresh, bool manual_lost,
              std::size_t manual_publishers, std::size_t switch_publishers,
              std::uint64_t host_stamp = 0) const {
    auto recent = [timeout](std::uint64_t envelope, std::uint64_t raw) {
      return raw > 0 && raw <= envelope && double(envelope - raw) / 1e6 < timeout;
    };
    // Two advancing raw observations within the existing freshness window are
    // required, including after a stale gap. This does not confirm RC mapping,
    // takeover ergonomics, or PX4 parameter configuration.
    auto progressed = [now, timeout](double previous) {
      return previous >= 0 && now >= previous && now - previous < timeout;
    };
    Facts out;
    const bool manual_clock =
        !manual_clock_rejected_ &&
        (!host_stamp || source_sample_in_clock(manual_timestamp_, manual_sample_stamp_.value,
                                               host_stamp, timeout));
    const bool switch_clock =
        !switch_clock_rejected_ &&
        (!host_stamp ||
         source_sample_in_clock(switch_timestamp_, sample_timestamp_, host_stamp, timeout));
    // 外壳时间新鲜不代表摇杆原始采样新鲜；两个跟踪器和前帧进展窗口须同时通过。
    const bool manual_fresh = manual_stamp_.fresh(now, timeout) &&
                              manual_sample_stamp_.fresh(now, timeout) &&
                              progressed(manual_previous_received_) &&
                              recent(manual_timestamp_, manual_sample_stamp_.value);
    out.manual_seen = manual_seen_;
    out.rc_source_clock_valid = manual_seen_ && manual_clock;
    out.rc_switch_clock_valid = switch_seen_ && switch_clock;
    out.manual_valid = manual_seen_ && manual_valid_;
    out.rc_source_selected = manual_seen_ && manual_valid_ && source_rc_;
    out.data_source = data_source_;
    out.sticks_moving = manual_publishers == 1 && flags_fresh && !manual_lost &&
                        out.rc_source_selected && manual_fresh && manual_clock && sticks_moving_;
    out.rc_link_ready = manual_publishers == 1 && flags_fresh && !manual_lost &&
                        out.rc_source_selected && manual_fresh && manual_clock;

    out.switches_seen = switch_seen_;
    out.switches_fresh = switch_publishers == 1 && switch_stamp_.fresh(now, timeout) &&
                         sample_stamp_.fresh(now, timeout) &&
                         progressed(sample_previous_received_) &&
                         recent(switch_timestamp_, sample_timestamp_) && switch_clock;
    out.rc_switches_ready = out.switches_fresh && kill_ == 3;
    out.kill = kill_latched_;
    out.mode_slot = switches_[0];
    out.arm_switch = switches_[1];
    out.kill_switch = kill_;
    out.switch_generation = generation_;
    out.rc_axes_safe = axes_safe_;
    out.rc_throttle = throttle_;
    out.switch_sample_stamp = sample_timestamp_;
    const auto off = [](std::uint8_t x) {
      return x == 0 || x == 3;
    };
    out.rc_aux_switches_safe = off(switches_[2]) && off(switches_[3]) && off(switches_[4]);
    return out;
  }

private:
  // 提交已通过双时间戳检查的原始开关载荷，timeout 单位秒；返回 void，不直接改变 Core 阶段。
  // 修改开关缓存、generation 和遥测 Kill 锁存；ON 立即锁存，OFF 仍须源采样贴近 envelope。
  // 必须在 switch/sample 时间水位提交后调用；无效时钟 ON 的早期否决仍留在 switches 中，不能合并。
  template <class Switches> void observe_switch_values(const Switches& message, double timeout) {
    // 保存模式/解锁/返航/盘旋/Offboard/Kill 的原始档位，generation 保留同周期 A→B→A 变化证据。
    const std::array<std::uint8_t, 6> next{message.mode_slot,       message.arm_switch,
                                           message.return_switch,   message.loiter_switch,
                                           message.offboard_switch, message.kill_switch};

    if (switch_seen_ && (next != switches_ || message.switch_changes != switch_changes_)) {
      ++generation_;
    }

    switches_ = next;
    switch_changes_ = message.switch_changes;
    switch_seen_ = true;
    switch_clock_rejected_ = false;
    kill_ = message.kill_switch;

    // ON 立即锁存；只有严格推进、与 envelope 足够接近的新 OFF 样本可以清除遥测层 Kill。
    if (kill_ == 1) {
      kill_latched_ = true;
    } else if (kill_ == 3 && sample_timestamp_ <= switch_timestamp_ &&
               double(switch_timestamp_ - sample_timestamp_) / 1e6 < timeout) {
      kill_latched_ = false;
    }
  }
  // 保存已接受 RC 源的四轴安全证据；输入为 PX4 归一化杆量，缺字段时明确无轴证据。
  // 只更新 throttle_/axes_safe_，无返回值；必须在 envelope/raw 双时间戳提交和源选择之后调用。
  // 本函数不能给无效源补成安全中杆，也不改变两个时钟或 RC 控制权。
  template <class Manual> void observe_manual_axes(const Manual& message) {
    using M = std::decay_t<Manual>;
    // 归一化 roll/pitch/yaw 需在 ±0.15 内；throttle 需接近 -1（低油门），才满足地面重准备轴安全。
    if constexpr (HasRcAxes<M>::value) {
      throttle_ = message.throttle;
      axes_safe_ = std::isfinite(message.roll) && std::isfinite(message.pitch) &&
                   std::isfinite(message.yaw) && std::isfinite(throttle_) &&
                   std::abs(message.roll) <= .15 && std::abs(message.pitch) <= .15 &&
                   std::abs(message.yaw) <= .15 && throttle_ >= -1.01 && throttle_ <= -.9;
    } else {
      axes_safe_ = false;
      throttle_ = NAN;
    }
  }
  // 分别记录选中源 envelope/raw 和开关 envelope/raw，避免一条新消息替另一来源续期。
  Stamp manual_stamp_, manual_sample_stamp_, switch_stamp_, sample_stamp_;
  std::uint64_t manual_timestamp_{}, switch_timestamp_{}, sample_timestamp_{}, generation_{};
  double manual_previous_received_{-1}, sample_previous_received_{-1};
  std::uint32_t switch_changes_{};
  std::uint8_t data_source_{}, kill_{};
  bool manual_seen_{}, manual_valid_{}, source_rc_{}, sticks_moving_{};
  bool switch_seen_{}, kill_latched_{};
  bool manual_clock_rejected_{}, switch_clock_rejected_{};
  bool axes_safe_{};
  double throttle_{NAN};
  std::array<std::uint8_t, 6> switches_{};
};
} // namespace fc01
