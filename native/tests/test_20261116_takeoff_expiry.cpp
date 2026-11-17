// 两个层级复现：local仍新鲜不等于旧上升仍在关联期限内；不连接ROS或飞控。
#include "ramp_guard_test_support.hpp"
#include <iostream>
struct ExpiryConfig {
  double land_sample_max_gap{1.25}, land_timeout{2.5}, local_timeout{.5},
      liftoff_dwell_seconds{.2}, liftoff_height_m{.12}, liftoff_speed_mps{.08};
};
static std::uint64_t stamp(double t) {
  return 1000000ULL + std::uint64_t(t * 1e6 + .5);
}
static a::TakeoffEvidence warm(const ExpiryConfig& c) {
  a::TakeoffEvidence e;
  e.observe(c, 0, 1, stamp(0), true, stamp(0), .13, .2);
  e.observe(c, .25, 1, stamp(0), true, stamp(.25), .18, .2);
  for (int i = 2; i <= 10; ++i) {
    double t = i * .25;
    e.observe(c, t, 1, stamp(0), true, stamp(t), .18, 0);
  }
  e.observe(c, 2.5, 1, stamp(2.5), false, stamp(2.5), .18, 0);
  return e;
}
int main() {
  ExpiryConfig c;
  for (double t : {2.75, 2.750001, 2.8, 3.000001}) {
    auto e = warm(c);
    const bool confirmed = e.observe(c, t, 1, stamp(t), false, stamp(2.5), .18, 0);
    require_guard(confirmed == (t <= 2.75), "motion expiry must apply without a new local sample");
  }
  GuardFlight q;
  q.ground_ramp(3.);
  for (int i = 0; i < 8; ++i) {
    q.observe(.13 + .01 * i, .2);
    q.advance(.05, false, true);
  }
  require_guard(q.r.attitude_core().liftoff_evidence().rise_confirmed, "rise fixture not proven");
  q.observe(.20, 0);
  for (int i = 0; i < 45; ++i)
    q.advance(.05, false, true);
  q.s.feedback.landed = false;
  q.s.at_rest = false;
  q.advance(.05, true, false);
  for (int i = 0; i < 5; ++i)
    q.advance(.05, i == 4, false);
  require_guard(q.r.attitude_core().phase() == a::Phase::ThrustRamp &&
                    !q.r.attitude_core().liftoff_confirmed() && q.o.attitude.publish,
                "expired rise cannot advance to Ascend; existing ramp custody must remain");
  std::cout << "TAKEOFF_EXPIRY_PASS helper boundaries + Runtime late detector\n";
}
