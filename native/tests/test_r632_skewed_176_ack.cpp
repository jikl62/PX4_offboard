// 实测负5.643 ms模式ACK反事实回归：候选ACK只能与发送后因果新模式共同确认，不能直接授权。
// Recorded176保存实测wire、水位和ACK绝对µs，其他反馈合成；普通测试关闭Arm/Takeoff以隔离模式入口。
// FC01 2026-10-16: replay the actual -5.643ms ACK and post-command mode evidence.
#include "review_clock_support.hpp"
#include <cmath>
#include <limits>

// options区分普通/bench路线；构造器要求实际假sink提交才可能建立ACK候选，未提交命令不具线因果。
struct Recorded176 {
  static constexpr std::uint64_t wire = 1792143533524121ULL;
  static constexpr std::uint64_t watermark = 1792143533329254ULL;
  static constexpr std::uint64_t ack_stamp = 1792143533518478ULL;
  static f::Config options(bool bench) {
    auto c = config(bench);
    // Match the real retest: exercise the flight ACK route with Arm/Takeoff shut.
    if (!bench)
      c.attitude.arm_enabled = c.attitude.takeoff_enabled = false;
    return c;
  }
  f::Runtime r;
  f::Snapshot s;
  double t = 1., sent_at{};
  Recorded176(bool committed = true, bool bench = false, std::uint64_t prior_ack = 0,
              int authority = 1)
      : r(options(bench)) {
    r.require_attitude_sink_commit();
    s.status_fresh = s.land_fresh = s.local_fresh = s.arming_state_known = true;
    s.status_stamp = s.land_stamp = watermark - 5000000;
    s.system = s.component = 1;
    s.nav_state = 15;
    s.feedback.exclusive = s.feedback.landed = s.feedback.preflight = true;
    s.at_rest = true;
    s.stationary_seconds = 3.;
    s.battery_fresh = s.battery_connected = s.power_valid = true;
    s.attitude.attitude_fresh = s.attitude.failsafe_fresh = true;
    s.attitude.z_valid = s.attitude.vz_valid = true;
    s.attitude.q = {1, 0, 0, 0};
    s.attitude.z = 2.;
    s.attitude.battery_voltage = 7.6;
    s.attitude.source_clock_validated = true;
    s.attitude.rc_link_ready = s.attitude.rc_switches_ready = true;
    if (prior_ack)
      r.ack({prior_ack, 176, 0, 1, 191, false}, t);
    assert(r.request("INTERACTIVE", {}, s, t).accepted);
    if (authority)
      assert(
          r.request(authority == 2 ? "AUTHORITY_RC" : "AUTHORITY_AUTONOMOUS", {}, s, t).accepted);
    assert(r.request("ATTITUDE_READY", {}, s, t).accepted);
    for (int i = 0; i < 60; ++i) {
      t += .05;
      s.status_stamp += 50000;
      s.land_stamp = s.status_stamp;
      s.attitude.local_stamp = s.status_stamp;
      assert(r.request("PING", {}, s, t).accepted);
      const auto out = r.step(s, t).attitude;
      if (out.command == a::Command::Offboard) {
        s.status_stamp = s.land_stamp = watermark;
        s.attitude.local_stamp = watermark;
        sent_at = t;
        r.confirm_attitude_command_publication(out.command, committed, wire, s, t);
        break;
      }
    }
    assert(r.attitude_core().phase() == a::Phase::RequestOffboard);
    assert(r.pending(176).published == committed);
  }
  void ack(std::uint64_t stamp = ack_stamp, int result = 0) {
    r.ack({stamp, 176, result, 1, 191, false}, t + .01);
  }
  a::Output tick(bool advance_status = true) {
    t += .05;
    if (advance_status)
      s.status_stamp = wire + std::uint64_t(std::llround((t - sent_at) * 1e6)) - 194867;
    s.land_stamp = s.attitude.local_stamp = s.status_stamp;
    assert(r.request("PING", {}, s, t).accepted);
    return r.step(s, t).attitude;
  }
  void mode() {
    s.feedback.offboard = true;
    s.nav_state = 14;
  }
  void settle() {
    for (int i = 0; i < 12 && r.attitude_core().phase() == a::Phase::RequestOffboard; ++i)
      tick();
  }
};

// 负偏差ACK本身仍未接受，只有新Offboard状态超过wire+100 ms才共同证明；不能因此打开起飞门。
static void recorded_negative_ack_needs_new_state() {
  Recorded176 h;
  h.ack();
  assert(!h.r.pending(176).accepted);
  for (int i = 0; i < 8; ++i)
    h.tick(); // ACK alone, with fresh non-Offboard status.
  assert(h.r.attitude_core().phase() == a::Phase::RequestOffboard);
  assert(!h.r.pending(176).accepted);
  h.mode();
  h.settle();
  assert(h.r.attitude_core().phase() == a::Phase::OffboardReady);
  assert(h.r.pending(176).accepted && h.r.pending(176).state_verified);
  assert(h.r.pending(176).ack_stamp == 1792143533518478ULL);
  assert(h.r.pending(176).verified_status_stamp > 1792143533624121ULL);
  assert(!h.s.feedback.armed);
  assert(!h.r.request("TAKEOFF", {.6}, h.s, h.t).accepted);
  std::cout << "REAL_MINUS_5_643MS_ACK_REQUIRES_NEW_DISARMED_OFFBOARD_STATE PASS\n";
}

// 缓存状态或等于因果floor都不足，必须严格更新后才ready。
static void cached_or_pre_floor_state_is_insufficient() {
  for (auto stamp :
       {Recorded176::watermark, Recorded176::wire + 50000, Recorded176::wire + 100000}) {
    Recorded176 h;
    h.ack();
    h.mode();
    h.s.status_stamp = stamp;
    for (int i = 0; i < 5; ++i)
      h.tick(false);
    assert(!h.r.pending(176).accepted);
    assert(h.r.attitude_core().phase() == a::Phase::RequestOffboard);
    h.settle();
    assert(h.r.attitude_core().phase() == a::Phase::OffboardReady);
  }
  std::cout << "CACHED_AND_100MS_FLOOR_STATUS_CANNOT_CONFIRM_CANDIDATE PASS\n";
}

// 旧水位、超偏差、远未来、错误身份/命令、错误接收时刻均不合法，不能借新模式补救。
static void invalid_ack_cannot_use_fresh_mode() {
  using Ack = f::g::Ack;
  const Ack cases[] = {{Recorded176::watermark, 176, 0, 1, 191, false},
                       {Recorded176::watermark - 1, 176, 0, 1, 191, false},
                       {Recorded176::wire - 100001, 176, 0, 1, 191, false},
                       {Recorded176::wire + 1000000, 176, 0, 1, 191, false},
                       {Recorded176::ack_stamp, 176, 0, 2, 191, false},
                       {Recorded176::ack_stamp, 176, 0, 1, 192, false},
                       {Recorded176::ack_stamp, 512, 0, 1, 191, false}};
  for (const auto& ack : cases) {
    Recorded176 h;
    h.r.ack(ack, h.t + .01);
    h.mode();
    h.settle();
    assert(!h.r.pending(176).accepted);
    assert(h.r.attitude_core().phase() == a::Phase::RequestOffboard);
  }
  for (double dt : {-.001, 5., std::numeric_limits<double>::quiet_NaN()}) {
    Recorded176 h;
    h.r.ack({Recorded176::ack_stamp, 176, 0, 1, 191, false}, h.t + dt);
    h.mode();
    h.settle();
    assert(!h.r.pending(176).accepted);
    // An ACK delivered at the deadline terminates the pending transaction.
    assert(h.r.attitude_core().phase() ==
           (dt >= 5. ? a::Phase::Safety : a::Phase::RequestOffboard));
  }
  std::cout << "OLD_OUT_OF_BUDGET_WRONG_TARGET_AND_LOCAL_RECEIPT_ACKS_REFUSED PASS\n";
}

// 先前已见ACK和重复ACK不能重新绑定新候选，观察计数保持真实。
static void prior_ack_and_duplicate_are_not_new_candidates() {
  Recorded176 old(true, false, Recorded176::ack_stamp);
  old.ack();
  old.mode();
  old.settle();
  assert(!old.r.pending(176).accepted);
  Recorded176 h;
  h.ack();
  const auto count = h.r.pending(176).ack_observations;
  h.ack();
  assert(h.r.pending(176).ack_observations == count);
  h.mode();
  h.settle();
  assert(h.r.pending(176).accepted);
  std::cout << "PREVIOUSLY_OBSERVED_ACK_AND_DUPLICATE_CANNOT_REBIND PASS\n";
}

// 十四类当前健康/身份/地面条件及Core stall仍否决候选确认。
static void candidate_cannot_override_current_safety_conditions() {
  for (int fault = 0; fault < 14; ++fault) {
    Recorded176 h;
    h.ack();
    h.mode();
    switch (fault) {
    case 0:
      h.s.arming_state_known = false;
      break;
    case 1:
      h.s.feedback.armed = true;
      break;
    case 2:
      h.s.feedback.landed = false;
      break;
    case 3:
      h.s.at_rest = false;
      break;
    case 4:
      h.s.attitude.source_clock_valid = false;
      break;
    case 5:
      h.s.attitude.source_clock_validated = false;
      break;
    case 6:
      h.s.status_fresh = false;
      break;
    case 7:
      h.s.feedback.exclusive = false;
      break;
    case 8:
      h.s.time_regressed = true;
      break;
    case 9:
      h.s.attitude.kill = true;
      break;
    case 10:
      h.s.nav_state = 15;
      break;
    case 11:
      h.s.system = 2;
      break;
    case 12:
      h.s.attitude.attitude_fresh = false;
      break;
    case 13:
      h.s.land_fresh = false;
      break;
    }
    h.settle();
    assert(!h.r.pending(176).accepted);
    assert(h.r.attitude_core().phase() != a::Phase::OffboardReady);
  }
  // The existing Core stall check must still reject the would-be confirmation.
  Recorded176 stalled;
  stalled.ack();
  stalled.mode();
  stalled.t += .4;
  stalled.s.status_stamp = Recorded176::wire + 200000;
  stalled.r.step(stalled.s, stalled.t);
  assert(stalled.r.attitude_core().phase() == a::Phase::Safety);
  assert(!stalled.r.pending(176).accepted);
  std::cout << "CANDIDATE_CANNOT_BYPASS_CURRENT_HEALTH_GROUND_AUTHORITY_OR_STALL PASS\n";
}

// 候选5 s超时、未最终提交或后续明确拒绝均阻止ready，不自动复活。
static void candidates_expire_and_uncommitted_commands_cannot_complete() {
  Recorded176 h;
  h.ack();
  for (int i = 0; i < 110 && h.r.attitude_core().phase() != a::Phase::Safety; ++i)
    h.tick();
  assert(h.r.attitude_core().phase() == a::Phase::Safety);
  assert(h.r.attitude_core().reason() == "OFFBOARD_ACK_TIMEOUT");
  h.mode();
  h.tick();
  h.ack();
  assert(!h.r.pending(176).accepted);
  Recorded176 denied(false);
  denied.ack();
  denied.mode();
  denied.settle();
  assert(!denied.r.pending(176).accepted);
  assert(denied.r.attitude_core().phase() != a::Phase::OffboardReady);
  Recorded176 rejected;
  rejected.ack();
  rejected.ack(Recorded176::ack_stamp + 1, 2);
  rejected.mode();
  rejected.settle();
  assert(!rejected.r.pending(176).accepted);
  assert(rejected.r.attitude_core().phase() == a::Phase::Safety);
  std::cout << "TIMEOUT_SAFETY_FINAL_SINK_AND_REFUSAL_REMAIN_EFFECTIVE PASS\n";
}

// 原正8.794/13.518 ms、bench state-only和释放176路线各自保持原合同。
static void fast_positive_acks_and_other_paths_are_preserved() {
  for (auto delta : {8794, 13518}) {
    Recorded176 h;
    h.ack(Recorded176::wire + delta);
    h.mode();
    h.settle();
    assert(h.r.pending(176).accepted);
    assert(h.r.attitude_core().phase() == a::Phase::OffboardReady);
  }
  Recorded176 bench(true, true);
  bench.ack();
  bench.mode();
  bench.settle();
  assert(!bench.r.pending(176).accepted); // Bench uses its existing state-only path.
  assert(bench.r.attitude_core().phase() == a::Phase::OffboardReady);
  Recorded176 release;
  release.ack();
  release.mode();
  release.settle();
  assert(release.r.request("STOP", {}, release.s, release.t).accepted);
  const auto out = release.tick();
  assert(out.command == a::Command::ReleaseOffboard);
  const auto wire =
      Recorded176::wire + std::uint64_t(std::llround((release.t - release.sent_at) * 1e6));
  release.r.confirm_attitude_command_publication(out.command, true, wire, release.s, release.t);
  release.r.ack({wire - 5643, 176, 0, 1, 191, false}, release.t + .01);
  assert(!release.r.pending(176).accepted);
  std::cout << "POSITIVE_8_794_13_518MS_ACKS_BENCH_AND_RELEASE_PATHS_PRESERVED PASS\n";
}

// 测试偏差边缘与准备期间STOP；确认入模式后才能规划新释放176，不能叠事务。
static void skew_budget_edges_and_preparation_stop() {
  for (auto offset : {0, 1, 99999}) {
    Recorded176 h;
    h.ack(Recorded176::wire - offset);
    h.mode();
    h.settle();
    assert(h.r.attitude_core().phase() == a::Phase::OffboardReady);
    assert(h.r.pending(176).accepted && h.r.pending(176).ack_confirmed_by_state);
  }
  Recorded176 h;
  h.ack();
  assert(h.r.request("STOP", {}, h.s, h.t).accepted);
  h.mode();
  a::Output out;
  for (int i = 0; i < 12 && h.r.attitude_core().phase() == a::Phase::RequestOffboard; ++i)
    out = h.tick();
  assert(h.r.attitude_core().phase() == a::Phase::ReleaseOffboard);
  // Core emits Release in the SAME step and Runtime tracks the new 176.
  assert(out.command == a::Command::ReleaseOffboard);
  assert(h.r.pending(176).requested && !h.r.pending(176).published);
  assert(h.r.attitude_core().phase() == a::Phase::ReleaseOffboard);
  std::cout << "SKEW_BUDGET_EDGES_AND_STOP_DURING_PREPARATION PASS\n";
}

// 未选择权限或当前RC证据丢失不得确认，恢复当前完整RC事实后才可完成。
static void candidate_requires_current_selected_authority() {
  Recorded176 unset(true, false, 0, 0);
  unset.ack();
  unset.mode();
  unset.settle();
  assert(!unset.r.pending(176).accepted);
  assert(unset.r.attitude_core().phase() == a::Phase::RequestOffboard);
  for (bool lose_link : {false, true}) {
    Recorded176 h(true, false, 0, 2);
    h.ack();
    h.mode();
    if (lose_link)
      h.s.attitude.rc_link_ready = false;
    else
      h.s.attitude.rc_switches_ready = false;
    h.settle();
    assert(!h.r.pending(176).accepted);
    assert(h.r.attitude_core().phase() == a::Phase::RequestOffboard);
    h.s.attitude.rc_link_ready = h.s.attitude.rc_switches_ready = true;
    h.settle();
    assert(h.r.pending(176).accepted);
    assert(h.r.attitude_core().phase() == a::Phase::OffboardReady);
  }
  std::cout << "CANDIDATE_REQUIRES_CURRENT_LOCKED_AUTHORITY_AND_RC_TELEMETRY PASS\n";
}

// 接管请求仍等待待决Offboard的ACK/因果状态完成，然后才用零流释放模式，不能覆盖尚未解决的176。
static void takeover_waits_for_pending_offboard_before_release() {
  for (bool skewed : {true, false})
    for (bool causal : {false, true}) {
      Recorded176 h;
      h.ack(skewed ? Recorded176::ack_stamp : Recorded176::wire + 8794);
      h.mode();
      h.s.status_stamp = Recorded176::wire + (causal ? 200000 : 50000);
      h.s.land_stamp = h.s.attitude.local_stamp = h.s.status_stamp;
      assert(h.r.request("AUTHORITY_TAKEOVER", {}, h.s, h.t).accepted);
      assert(h.r.attitude_core().operator_release_requested());
      assert(h.r.attitude_core().phase() == a::Phase::RequestOffboard);
      a::Output out;
      if (!causal) {
        for (int i = 0; i < 3; ++i) {
          out = h.tick(false);
          assert(out.command == a::Command::None);
        }
        assert(h.r.attitude_core().phase() == a::Phase::RequestOffboard);
      }
      for (int i = 0; i < 12 && h.r.attitude_core().phase() == a::Phase::RequestOffboard; ++i)
        out = h.tick(causal ? false : true);
      assert(h.r.attitude_core().phase() == a::Phase::ReleaseOffboard);
      assert(out.command == a::Command::ReleaseOffboard && !a::nonzero(out.thrust));
      assert(h.r.pending(176).requested && !h.r.pending(176).published);
    }
  std::cout << "TAKEOVER_WAITS_PENDING_OFFBOARD_ACK_AND_STATE_BEFORE_NEW_RELEASE_176 PASS\n";
}

int main() {
  recorded_negative_ack_needs_new_state();
  cached_or_pre_floor_state_is_insufficient();
  invalid_ack_cannot_use_fresh_mode();
  prior_ack_and_duplicate_are_not_new_candidates();
  candidate_cannot_override_current_safety_conditions();
  candidates_expire_and_uncommitted_commands_cannot_complete();
  fast_positive_acks_and_other_paths_are_preserved();
  skew_budget_edges_and_preparation_stop();
  takeover_waits_for_pending_offboard_before_release();
  candidate_requires_current_selected_authority();
}
