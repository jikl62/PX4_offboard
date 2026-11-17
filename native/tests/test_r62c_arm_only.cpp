// R6.2c 仅解锁台架回归：显式bench合同下只允许规范零推力和一次Arm/Disarm，不允许起飞。
// BH用真实Runtime、最终边界和最终提交回报；land仍为1 Hz。参数仅属拆桨离线合同，不作为部署建议。
// R6.2c offline Arm-only bench path. No ROS or physical hardware.
#define main existing_s10_not_invoked
#include "c_s10_independent/C_S10_SAFETY_ATTACKS.cpp"
#undef main
static f::Config bench_config() {
  auto c = config();
  c.attitude.bench_arm_only_enabled = true;
  c.attitude.takeoff_enabled = false;
  c.validate();
  return c;
}
// 保存各类假sink次数，每次发布逐项断言推力为零，命令先过对应最终策略再记wire提交。
struct BH {
  f::Runtime r{bench_config()};
  f::Snapshot s{snapshot()};
  double t{};
  int arms{}, disarms{}, modes{}, targets{}, heartbeats{};
  unsigned ticks{};
  BH() {
    r.require_attitude_sink_commit();
    assert(r.request("INTERACTIVE", {}, s, t).accepted);
    assert(r.request("AUTHORITY_AUTONOMOUS", {}, s, t).accepted);
    assert(r.request("ATTITUDE_READY", {}, s, t).accepted);
  }
  a::Output tick(bool newland = true) {
    t += .05;
    ++ticks;
    s.status_stamp += 1000;
    if (newland && ticks % 20 == 0)
      s.land_stamp += 1000;
    s.attitude.local_stamp += 1000;
    assert(r.request("PING", {}, s, t).accepted);
    auto o = r.step(s, t).attitude;
    if (o.publish) {
      assert(!a::nonzero(o.thrust));
      auto c = r.publication_context(s, t, true, true);
      const auto d = a::publish_attitude_at_boundary<Mode, Target>(
          c, o, 1000000 + std::uint64_t(t * 1e6),
          [&](const Target& x) {
            assert(x.thrust_body[2] == 0);
            ++targets;
          },
          [&](const Mode&) {
            ++heartbeats;
          });
      assert(d.allowed && targets == heartbeats);
    }
    if (o.command != a::Command::None) {
      auto c = r.publication_context(s, t, true, true);
      if (o.command == a::Command::Arm) {
        assert(a::publish_arm_at_boundary(c, [&] {
                 ++arms;
               }).allowed);
      } else if (o.command == a::Command::Disarm) {
        assert(a::publish_disarm_at_boundary(c, [&] {
                 ++disarms;
               }).allowed);
      } else if (o.command == a::Command::ReleaseOffboard) {
        assert(a::publish_release_at_boundary(c, [&] {
                 ++modes;
               }).allowed);
      } else {
        ++modes;
      }
      r.confirm_attitude_command_publication(o.command, true, 1000000 + std::uint64_t(t * 1e6), s,
                                             t);
    }
    return o;
  }
  void ready() {
    for (int i = 0; i < 65 && r.attitude_core().phase() != a::Phase::RequestOffboard; ++i)
      tick();
    assert(r.attitude_core().phase() == a::Phase::RequestOffboard && modes == 1);
    r.ack({r.pending(176).baseline + 3000, 176, 0, 1, 191, false}, t + .001);
    s.feedback.offboard = true;
    s.nav_state = 14;
    tick();
    assert(r.attitude_core().phase() == a::Phase::OffboardReady);
  }
  // 先准备，再验证TAKEOFF被台架策略禁止；ARM_ONLY经ACK和新armed状态才能BenchArmedGround。
  void armed() {
    ready();
    auto no = r.request("TAKEOFF", {.5}, s, t);
    assert(!no.accepted && no.message == "TAKEOFF_FORBIDDEN_IN_ARM_ONLY_BENCH_MODE");
    auto yes = r.request("ARM_ONLY", {}, s, t);
    assert(yes.accepted);
    auto o = tick();
    assert(o.command == a::Command::Arm && arms == 1);
    r.ack({r.pending(400).baseline + 3000, 400, 0, 1, 191, false}, t + .001);
    s.feedback.armed = true;
    tick();
    assert(r.attitude_core().phase() == a::Phase::BenchArmedGround);
  }
};
// 正常台架可进入PoweredGround并计划唯一上锁，整个期间不得出现ThrustRamp/Ascend/HeightHold。
void positive() {
  BH h;
  h.armed();
  bool saw_powered = false;
  for (int i = 0; i < 300 && !h.disarms; ++i) {
    auto o = h.tick();
    assert(!a::nonzero(o.thrust));
    auto p = h.r.attitude_core().phase();
    assert(p != a::Phase::ThrustRamp && p != a::Phase::Ascend && p != a::Phase::HeightHold);
    if (p == a::Phase::PoweredGround)
      saw_powered = true;
  }
  assert(saw_powered && h.disarms == 1 && h.arms == 1);
  for (int i = 0; i < 20; ++i) {
    h.tick();
    assert(h.disarms == 1);
  }
  std::cout << "R62C_BENCH_AUTO_DISARM_NO_RAMP PASS\n";
}
// 冻结land时不能仅按控制时间发上锁；保持无非零推力并披露等待。
void detector_freeze() {
  BH h;
  h.armed();
  for (int i = 0; i < 85; ++i)
    h.tick(false);
  assert(h.r.attitude_core().phase() == a::Phase::BenchArmedGround);
  assert(h.disarms == 0 && h.arms == 1);
  std::cout << "R62C_BENCH_FROZEN_DETECTOR_NO_DISARM PASS\n";
}
// 台架Vz超限仍须Safety并保存精确16位原因，不能盲目Disarm。
void ground_failure() {
  BH h;
  h.armed();
  h.s.attitude.vz = .251;
  auto o = h.tick();
  assert(!o.publish);
  assert(h.r.attitude_core().reason() == "BENCH_ARMED_GROUND_NOT_STABLE");
  assert(h.r.attitude_core().armed_ground_fault_evidence().failed_mask == 16U);
  assert(h.disarms == 0 && h.arms == 1);
  std::cout << "R62C_BENCH_VZ_FAILURE_BLOCKED PASS\n";
}
// 恶意非零输出即使Core错误也被最终sink拒绝；LAND在非静止地面只先进入原减推恢复。
void stop_and_boundary() {
  BH h;
  h.armed();
  auto c = h.r.publication_context(h.s, h.t, true, true);
  a::Output forbidden;
  forbidden.publish = true;
  forbidden.q = {1, 0, 0, 0};
  forbidden.thrust = {0, 0, -.01};
  // No nonzero output may pass any bench phase, even if Core is mistaken.
  assert(!a::attitude_publication_policy(c, forbidden).allowed);
  assert(a::attitude_publication_policy(c, forbidden).reason == a::BoundaryReason::StageOneNonzero);
  h.s.at_rest = false;
  auto no = h.r.request("LAND", {}, h.s, h.t);
  assert(no.accepted && h.r.attitude_core().phase() == a::Phase::PoweredGround);
  assert(h.disarms == 0);
  std::cout << "R62C_BENCH_NO_NONZERO_AT_FINAL_SINK_OR_BLIND_LAND PASS\n";
}
// 默认bench关闭；将bench与takeoff同时打开是非法配置，构造前须拒绝。
void defaults() {
  Harness h;
  h.ready();
  auto no = h.r.request("ARM_ONLY", {}, h.s, h.t);
  assert(!no.accepted && no.message == "ARM_ONLY_BENCH_MODE_DISABLED");
  auto c = config();
  c.attitude.bench_arm_only_enabled = true;
  bool rejected = false;
  try {
    c.validate();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);
  std::cout << "R62C_BENCH_DEFAULT_DENIED PASS\n";
}
int main() {
  defaults();
  positive();
  detector_freeze();
  ground_failure();
  stop_and_boundary();
  std::cout << "R62C_BENCH_OFFLINE_PASS\n";
}
