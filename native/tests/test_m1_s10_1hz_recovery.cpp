// 低频着陆检测回归：20 Hz控制循环必须与1 Hz独立land样本的证明窗口正确配合。
// Harness只在new_land时推进land戳；普通tick推进其它反馈，防止缓存重复被误算成新离地证据。
// z为NED下正m、vz下正m/s；rising时减少z模拟爬升，测试不含真实动力学。
// Agent B r5: synthetic 1Hz PX4 land detector / supervision regression.
// No ROS graph, flight hardware, commands to FC01, or real parameters.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <cassert>
#include <iostream>
#include <string>
namespace a = fc01::attitude;
namespace f = fc01;

// 持有真实Runtime和合成状态；固定1.25 s land间隔上限用于容纳1 Hz源且仍拒绝长期冻结。
struct Harness {
  f::Config config;
  f::Runtime runtime;
  f::Snapshot s;
  double now{};
  bool rising{};
  Harness() : config(make_config()), runtime(config), s(authority_test::sample()) {
    authority_test::armed(runtime, s, now, "AUTHORITY_AUTONOMOUS");
    assert(runtime.attitude_core().phase() == a::Phase::ArmedGround);
    s.at_rest = false;
  }
  static f::Config make_config() {
    auto c = authority_test::config();
    c.attitude.autonomous_failsafe_confirmed = true;
    c.attitude.land_sample_max_gap = 1.25;
    c.attitude.land_timeout = 2.5;
    c.attitude.armed_dwell_seconds = 1.0;
    c.attitude.liftoff_dwell_seconds = .20;
    c.attitude.touchdown_dwell_seconds = .30;
    c.attitude.saturation_seconds = 100.;
    c.attitude.max_flight_seconds = 100.;
    return c;
  }
  f::Output tick() {
    now += .05;
    s.status_stamp += 50000;
    s.attitude.local_stamp += 50000;
    s.attitude.vertical_sample_stamp += 50000;
    if (rising && runtime.attitude_core().phase() == a::Phase::ThrustRamp)
      s.attitude.z -= .006;
    assert(runtime.request("PING", {}, s, now).accepted);
    return runtime.step(s, now);
  }
  f::Output new_land() {
    s.land_stamp += 1000000;
    return tick();
  }
  f::Output advance_ticks(int n) {
    f::Output out;
    for (int i = 0; i < n; ++i)
      out = tick();
    return out;
  }
  f::Output one_hz() {
    advance_ticks(19);
    return new_land();
  }
  void ramp() {
    for (int i = 0; i < 4 && runtime.attitude_core().phase() != a::Phase::ThrustRamp; ++i)
      one_hz();
    assert(runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  }
  void evidence_airborne() {
    s.feedback.landed = false;
    s.attitude.z = 1.8;
    s.attitude.vz = -.12;
    rising = true;
  }
  void ascend() {
    ramp();
    evidence_airborne();
    for (int i = 0; i < 3 && runtime.attitude_core().phase() != a::Phase::Ascend; ++i)
      one_hz();
    assert(runtime.attitude_core().phase() == a::Phase::Ascend);
  }
};

// 两次独立新land且高度/速度/接触条件一致后才从斜坡进入Ascend，控制tick本身不证明离地。
static void positive_1hz_is_not_impossible() {
  Harness h;
  h.advance_ticks(25); // Repeat detector message at 20 Hz for >1s.
  assert(h.runtime.attitude_core().phase() == a::Phase::ArmedGround);
  h.one_hz();
  h.one_hz();
  assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  h.evidence_airborne();
  const auto first = h.one_hz();
  assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  assert(first.attitude.publish);
  const auto second = h.one_hz();
  assert(h.runtime.attitude_core().phase() == a::Phase::Ascend);
  assert(second.attitude.publish);
  assert(h.runtime.attitude_core().reason() == std::string("ASCEND"));
  std::cout << "S10_R5_1HZ_TAKEOFF two independent land stamps => ASCEND: PASS\n";
}
// 冻结超过允许间隔后第一次新样本只能重新播种窗口，第二个连续新样本才可完成。
static void held_detector_cannot_fake_liftoff() {
  Harness h;
  h.ramp();
  h.evidence_airborne();
  h.new_land();
  for (int i = 0; i < 50; ++i) {
    const auto o = h.tick();
    assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
    assert(o.attitude.publish);
  }
  h.new_land(); // gap >1.25s MUST seed, not conclude previous dwell.
  assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  h.one_hz();
  assert(h.runtime.attitude_core().phase() == a::Phase::Ascend);
  std::cout << "S10_R5_FROZEN_STAMP long gap requires TWO new samples: PASS\n";
}
// 超长gap或landed弹跳各自重置离地证明，正常尾帧不能复用旧窗口。
static void separated_and_bouncing_evidence() {
  Harness h;
  h.ramp();
  h.evidence_airborne();
  h.new_land();
  h.advance_ticks(30); // 1.5s, violates 1.25s limit.
  h.new_land();
  assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  h.s.feedback.landed = true;
  h.one_hz();
  assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  h.s.feedback.landed = false;
  h.one_hz();
  assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  h.one_hz();
  assert(h.runtime.attitude_core().phase() == a::Phase::Ascend);
  std::cout << "S10_R5_GAP_AND_BOUNCE resets liftoff proof: PASS\n";
}
// 有高度但不向上、或向上但高度不足都不能证明离地，两个独立物理条件须同时成立。
static void height_and_speed_both_required() {
  for (int variant = 0; variant < 2; ++variant) {
    Harness h;
    h.ramp();
    h.s.feedback.landed = false;
    h.s.attitude.z = (variant == 0 ? 1.8 : 1.98);
    h.s.attitude.vz = (variant == 0 ? 0 : -.12);
    for (int i = 0; i < 3; ++i)
      h.one_hz();
    assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  }
  std::cout << "S10_R5_HEIGHT_SPEED negative gates: PASS\n";
}
// 回退再回到旧戳仍不是两次新证据；实际适配器还会额外锁存时钟回退。
static void timestamp_regression_cannot_revalidate_old_evidence() {
  Harness h;
  h.ramp();
  h.evidence_airborne();
  h.new_land();
  const auto last = h.s.land_stamp;
  h.s.land_stamp = last - 100;
  h.tick();
  assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  // Replaying an older stamp, then returning to the old stamp, is still not
  // two independent positive observations. PX4's real Stamp.accept will also
  // latch time_regressed on this condition and refuse flight.
  h.s.land_stamp = last;
  h.tick();
  assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  h.new_land();
  assert(h.runtime.attitude_core().phase() == a::Phase::ThrustRamp);
  h.one_hz();
  assert(h.runtime.attitude_core().phase() == a::Phase::Ascend);
  std::cout << "S10_R5_STAMP_REGRESSION old samples do not authorize liftoff: PASS\n";
}
// 爬升超时进入受监督下降；固定预算过期不能伪造PX4接管，也不能在不确定状态发起新起飞。
static void ascend_timeout_retains_guarded_custody_after_budget() {
  Harness h;
  h.ascend();
  const double since = h.now;
  bool descent = false;
  for (int i = 0; i < 215; ++i) {
    const auto o = h.tick();
    if (h.runtime.attitude_core().phase() == a::Phase::Descent) {
      descent = true;
      assert(o.attitude.publish);
      assert(o.attitude.thrust[2] <= 0);
      assert(h.runtime.attitude_core().reason() ==
                 std::string("ASCENT_TIMEOUT_SUPERVISED_RECOVERY") ||
             h.runtime.attitude_core().reason() == std::string("DESCENT"));
      const auto b = h.runtime.publication_context(h.s, h.now, true, true);
      assert(a::attitude_publication_policy(b, o.attitude).allowed);
      break;
    }
    assert(h.runtime.attitude_core().phase() == a::Phase::Ascend);
    assert(o.attitude.publish);
  }
  assert(descent && h.now - since >= h.config.attitude.ascent_timeout - .05);
  bool overdue = false;
  for (int i = 0; i < 650; ++i) {
    const auto o = h.tick();
    if (h.runtime.attitude_core().recovery_budget_exhausted()) {
      assert(o.attitude.publish);
      assert(h.runtime.attitude_core().fault_class() == a::FaultClass::ControlledDescent);
      assert(h.runtime.attitude_core().reason() ==
             std::string("RECOVERY_BUDGET_EXHAUSTED_GUARDED_CUSTODY_ACTIVE_HANDOFF_UNVERIFIED"));
      assert(h.s.feedback.armed && h.s.feedback.offboard);
      const auto& fb = h.runtime.publication_context(h.s, h.now, true, true).feedback;
      assert(std::string(h.runtime.attitude_core().recovery_responsibility(fb)) ==
             "COMPANION_GUARDED_DESCENT_ACTIVE_RECOVERY_OVERDUE_HANDOFF_UNVERIFIED");
      overdue = true;
    }
    assert(h.runtime.attitude_core().phase() == a::Phase::Descent ||
           h.runtime.attitude_core().phase() == a::Phase::RecoveryDescent);
    assert(o.attitude.publish);
  }
  assert(overdue);
  const auto no_rearm = h.tick();
  assert(no_rearm.attitude.publish && no_rearm.attitude.command == a::Command::None);
  assert(!h.runtime.request("TAKEOFF", {.6}, h.s, h.now).accepted);
  std::cout
      << "S10_R6.1_ASCENT_TIMEOUT supervised output across fixed recovery deadline, handoff UNVERIFIED: PASS\n";
}
int main() {
  positive_1hz_is_not_impossible();
  held_detector_cannot_fake_liftoff();
  separated_and_bouncing_evidence();
  height_and_speed_both_required();
  timestamp_regression_cannot_revalidate_old_evidence();
  ascend_timeout_retains_guarded_custody_after_budget();
}
