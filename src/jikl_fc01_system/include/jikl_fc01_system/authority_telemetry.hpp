#pragma once
#include "jikl_fc01_system/runtime.hpp"
#include <type_traits>
namespace fc01 {
// Observation-only adapter shared by the real ROS node and deterministic tests.
// RC link readiness is derived from the selected ManualControlSetpoint source,
// not raw InputRc transport. ManualControlSwitches remains an independent
// RC_TAKEOVER requirement. Received Kill stays asserted until a newer OFF
// observation; staleness/unknown data must not turn a known Kill into OFF.
class AuthorityTelemetry {
 public:
  struct Facts {
    bool rc_link_ready{},rc_switches_ready{},kill{};
    bool manual_seen{},manual_valid{},rc_source_selected{},sticks_moving{};
    bool switches_seen{},switches_fresh{};
    std::uint8_t data_source{},mode_slot{},arm_switch{},kill_switch{};
    std::uint64_t switch_generation{};
  };

template<class Manual>
void manual(const Manual& message, double now) {
    using M = std::decay_t<Manual>;

    const auto envelope = message.timestamp;
    const auto sample = message.timestamp_sample;

    // 拒绝无效、重复或过期的消息发布时间
    if (envelope == 0 || envelope <= manual_stamp_.value)
        return;

    // 当前选中控制源失效时，及时撤销 RC 准入。
    // 即使原始采样时间没有推进，也不能继续沿用旧的有效状态。
    if (!message.valid || message.data_source != M::SOURCE_RC) {
        if (!manual_stamp_.accept(envelope, now))
            return;

        manual_seen_ = true;
        manual_valid_ = message.valid;
        data_source_ = message.data_source;
        source_rc_ = false;
        sticks_moving_ = false;
        return;
    }

    // 拒绝非法、重复或乱序的原始采样
    // 不调用 Stamp::accept()，避免污染其永久锁存状态
    if (sample == 0 ||
        sample > envelope ||
        sample <= manual_sample_stamp_.value)
        return;

    const double previous_sample =
        manual_sample_stamp_.received;

    // 两个时间戳均通过预检查后，更新跟踪器
    if (!manual_stamp_.accept(envelope, now))
        return;

    if (!manual_sample_stamp_.accept(sample, now))
        return;

    manual_timestamp_ = envelope;
    manual_previous_received_ = previous_sample;

    manual_seen_ = true;
    manual_valid_ = true;
    data_source_ = message.data_source;
    source_rc_ = true;
    sticks_moving_ = message.sticks_moving;
}

template<class Switches>
void switches(const Switches& message,
              double now,
              double timeout) {

    const auto envelope = message.timestamp;
    const auto sample = message.timestamp_sample;

    // 只接受时间戳有效且严格推进的样本
    if (envelope == 0 ||
        sample == 0 ||
        sample > envelope ||
        envelope <= switch_stamp_.value ||
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

    const std::array<std::uint8_t, 6> next{
        message.mode_slot,
        message.arm_switch,
        message.return_switch,
        message.loiter_switch,
        message.offboard_switch,
        message.kill_switch
    };

    if (switch_seen_ &&
        (next != switches_ ||
         message.switch_changes != switch_changes_)) {
        ++generation_;
    }

    switches_ = next;
    switch_changes_ = message.switch_changes;
    switch_seen_ = true;
    kill_ = message.kill_switch;

    if (kill_ == 1) {
        kill_latched_ = true;
    }
    else if (kill_ == 3 &&
             sample_timestamp_ <= switch_timestamp_ &&
             double(switch_timestamp_ - sample_timestamp_) / 1e6 < timeout) {
        kill_latched_ = false;
    }
}

  Facts facts(double now,double timeout,bool flags_fresh,bool manual_lost,
      std::size_t manual_publishers,std::size_t switch_publishers) const{
    auto recent=[timeout](std::uint64_t envelope,std::uint64_t raw){
      return raw>0&&raw<=envelope&&double(envelope-raw)/1e6<timeout;
    };
    // Two advancing raw observations within the existing freshness window are
    // required, including after a stale gap. This does not confirm RC mapping,
    // takeover ergonomics, or PX4 parameter configuration.
    auto progressed=[now,timeout](double previous){
      return previous>=0&&now>=previous&&now-previous<timeout;
    };
    Facts out;
    const bool manual_fresh=manual_stamp_.fresh(now,timeout)&&manual_sample_stamp_.fresh(now,timeout)&&
      progressed(manual_previous_received_)&&recent(manual_timestamp_,manual_sample_stamp_.value);
    out.manual_seen=manual_seen_;
    out.manual_valid=manual_seen_&&manual_valid_;
    out.rc_source_selected=manual_seen_&&manual_valid_&&source_rc_;
    out.data_source=data_source_;
    out.sticks_moving=manual_publishers==1&&flags_fresh&&!manual_lost&&out.rc_source_selected&&
      manual_fresh&&sticks_moving_;
    out.rc_link_ready=manual_publishers==1&&flags_fresh&&!manual_lost&&out.rc_source_selected&&manual_fresh;

    out.switches_seen=switch_seen_;
    out.switches_fresh=switch_publishers==1&&switch_stamp_.fresh(now,timeout)&&sample_stamp_.fresh(now,timeout)&&
      progressed(sample_previous_received_)&&recent(switch_timestamp_,sample_timestamp_);
    out.rc_switches_ready=out.switches_fresh&&kill_==3;
    out.kill=kill_latched_;
    out.mode_slot=switches_[0];out.arm_switch=switches_[1];out.kill_switch=kill_;
    out.switch_generation=generation_;
    return out;
  }
 private:
  Stamp manual_stamp_,manual_sample_stamp_,switch_stamp_,sample_stamp_;
  std::uint64_t manual_timestamp_{},switch_timestamp_{},sample_timestamp_{},generation_{};
  double manual_previous_received_{-1},sample_previous_received_{-1};
  std::uint32_t switch_changes_{};
  std::uint8_t data_source_{},kill_{};
  bool manual_seen_{},manual_valid_{},source_rc_{},sticks_moving_{};
  bool switch_seen_{},kill_latched_{};
  std::array<std::uint8_t,6> switches_{};
};
} // namespace fc01
