// 诊断可区分首因、核对测距趋势，但不能代替实际反馈或改变人工RC权限。
#include "vision_test_support.hpp"
#include "jikl_fc01_system/range_trend_diagnostics.hpp"
#include <iostream>
using namespace vision_test;
int main() {
  {
    auto c = config(); c.attitude.visual.enabled = false;
    VisionFlight q(c);
    auto early = q.r.publication_context(q.s, q.t, true, true).feedback;
    early.preflight = false;
    q.r.begin_attitude_safety_observation();
    q.r.observe_attitude_safety_event(early, a::Core::SafetyEvent::Status, q.t);
    need(!q.r.attitude_core().queued_safety_evidence().recorded,
         "temporary preparation refusal polluted task first cause");
    q.warm(); q.manual_arm();
    auto bad = q.r.publication_context(q.s, q.t, true, true).feedback;
    bad.failsafe = true;
    q.r.observe_attitude_safety_event(bad, a::Core::SafetyEvent::Failsafe, q.t);
    need(q.r.attitude_core().queued_safety_evidence().mask == 2U,
         "actual task failsafe hidden by old preparation cause");
  }
  for (unsigned bit = 0; bit < 10; ++bit) {
    auto c = config(); c.attitude.visual.enabled = false;
    VisionFlight q(c); q.warm(); q.manual_arm();
    auto bad = q.r.publication_context(q.s, q.t, true, true).feedback;
    switch (bit) {
    case 0: bad.kill = true; break;
    case 1: bad.failsafe = true; break;
    case 2: bad.calibration = true; break;
    case 3: bad.hil = true; break;
    case 4: bad.preflight = false; break;
    case 5: bad.battery_ready = false; break;
    case 6: bad.rc_link_ready = false; break;
    case 7: bad.rc_axes_safe = false; break;
    case 8: bad.rc_sticks_moving = true; break;
    case 9: bad.rc_aux_switches_safe = false; break;
    }
    q.r.begin_attitude_safety_observation();
    q.r.observe_attitude_safety_event(bad, a::Core::SafetyEvent::Authority, q.t);
    const auto first = q.r.attitude_core().queued_safety_evidence();
    need(first.recorded && (first.mask & (1U << bit)), "queued cause missing");
    need(first.status_stamp == bad.status_stamp && first.switch_stamp == bad.rc_switch_sample_stamp,
         "queued source watermarks missing");
    auto next = bad; next.failsafe = true;
    q.r.observe_attitude_safety_event(next, a::Core::SafetyEvent::Failsafe, q.t + .01);
    q.tick();
    need(q.r.attitude_core().queued_safety_evidence().mask == first.mask,
         "later event overwrote first cause");
    need(q.r.attitude_core().phase() == a::Phase::Safety && !q.o.attitude.publish &&
             q.r.attitude_core().reason() == "RC_QUEUED_SAFETY_CONDITION",
         "diagnostics changed original veto");
  }
  a::RangeTrendDiagnostics d;
  const auto observe = [&](std::uint64_t s, double z, double range, bool valid = true,
                           unsigned reset = 0, double variance = .01) {
    d.observe(s, valid, z, range, variance, 0, reset, .5);
  };
  observe(1000000, 4., .1); observe(1250000, 3.9, .2);
  auto e = d.evidence(true);
  need(e.available && e.compared && e.same_direction && std::abs(e.fused_rise_m - .1) < 1e-9,
       "relative upward trends not comparable");
  need(!d.evidence(false).available, "stale range remained current");
  observe(1500000, 3.8, .1);
  need(d.evidence(true).compared && !d.evidence(true).same_direction,
       "opposite trend not visible");
  observe(1750000, 3.7, .2, true, 1);
  need(!d.evidence(true).compared, "range reset bridged old baseline");
  observe(2000000, 3.6, .3, false);
  need(!d.evidence(true).available, "invalid range remained current");
  observe(2250000, 3.5, .4); need(!d.evidence(true).compared, "invalid gap bridged");
  observe(2250000, 3.4, .5); need(!d.evidence(true).available, "same stamp changed payload");
  observe(2500000, 3.3, .6, true, 0, NAN);
  need(!d.evidence(true).available, "NaN variance accepted");
  observe(2750000, 3.2, -.1); need(!d.evidence(true).available, "negative range accepted");
  observe(3000000, 3.1, .8); observe(4000000, 3., .9);
  need(!d.evidence(true).compared, "sample gap bridged old range");
  observe(3999999, 3., .9); need(!d.evidence(true).available, "regressed sample accepted");
  std::cout << "DIAGNOSTICS_PASS 10 queued cause bits + range trend/reset/freshness/invalid\n";
}
