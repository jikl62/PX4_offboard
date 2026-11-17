// Runtime到最终sink整链回归：缺XY与动力准备条件时，仅原有第一阶段零推力准备可工作。
// sample的z=2 m是NED参考；故意缺XY/VXY/heading/电池RC动力就绪，确保准备不会隐式升级到Arm。
#include "jikl_fc01_system/runtime.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <cassert>
#include <iostream>
namespace a = fc01::attitude;
struct Mode {
  std::uint64_t timestamp{};
  bool position{}, velocity{}, acceleration{}, attitude{}, body_rate{}, thrust_and_torque{},
      direct_actuator{};
};
struct Target {
  std::uint64_t timestamp{};
  float yaw_sp_move_rate{};
  std::array<float, 4> q_d{};
  std::array<float, 3> thrust_body{};
  bool reset_integral{}, fw_control_yaw_wheel{};
};
// 构造可信未解锁、接地静止和Z/Vz/姿态，保留无视觉垂向准备的实际依赖范围。
static fc01::Snapshot sample() {
  fc01::Snapshot s;
  s.status_fresh = s.land_fresh = s.local_fresh = true;
  s.stationary_seconds = 3;
  s.status_stamp = s.land_stamp = 1000;
  s.feedback.exclusive = s.feedback.landed = s.at_rest = true;
  s.attitude.attitude_fresh = s.attitude.failsafe_fresh = s.attitude.z_valid = s.attitude.vz_valid =
      true;
  s.attitude.q = {1, 0, 0, 0};
  s.attitude.z = 2;
  s.attitude.vz = 0;
  // Deliberately no XY/VXY/heading or stage-two battery/RC readiness.
  return s;
}
int main() {
  fc01::Config c;
  c.profile = "attitude_altitude";
  fc01::Runtime r(c);
  auto s = sample();
  assert(r.request("ATTITUDE_READY", {}, s, 0).accepted);
  int targets = 0, heartbeats = 0;
  bool requested = false;
  // 按50 ms推进并用正确176 ACK与新Offboard状态完成准备；每个真实假sink都核对推力严格零。
  for (int i = 0; i <= 30; ++i) {
    const double t = i * .05;
    s.status_stamp += 1000;
    s.land_stamp += 1000;
    auto out = r.step(s, t);
    if (out.attitude.command == a::Command::Offboard) {
      requested = true;
      const auto b = r.pending(176).baseline;
      r.ack({b + 1, 512, 0, 1, 191, false}, t + .001);
      assert(!r.pending(176).accepted);
      r.ack({b + 2, 176, 0, 1, 191, false}, t + .002);
      s.feedback.offboard = true;
      s.nav_state = 14;
    }
    if (out.attitude.publish) {
      auto ctx = r.publication_context(s, t, true, true);
      assert(!ctx.config.flight_actions_enabled && !ctx.config.parameters_confirmed &&
             !ctx.operator_lease_live);
      auto d = a::publish_attitude_at_boundary<Mode, Target>(
          ctx, out.attitude, 123,
          [&](const Target& x) {
            ++targets;
            for (float value : x.thrust_body)
              assert(value == 0);
          },
          [&](const Mode&) {
            ++heartbeats;
          });
      assert(d.allowed);
      assert(!r.pending(400).sent);
    }
  }
  assert(requested && r.attitude_core().phase() == a::Phase::OffboardReady &&
         targets == heartbeats && targets > 0);
  // 断开会话后的旧准备输出要在最终边界拒绝，再由Runtime走原零推力模式释放，不偷偷Arm。
  auto out = r.step(s, 1.55);
  auto ctx = r.publication_context(s, 1.55, false, true);
  auto denied = a::publish_attitude_at_boundary<Mode, Target>(
      ctx, out.attitude, 124,
      [&](const Target&) {
        ++targets;
      },
      [&](const Mode&) {
        ++heartbeats;
      });
  assert(!denied.allowed && denied.reason == a::BoundaryReason::SessionDisconnected);
  const int before = heartbeats;
  r.disconnect(s, 1.56);
  out = r.step(s, 1.6);
  assert(out.attitude.publish && out.attitude.command == a::Command::ReleaseOffboard &&
         out.attitude.thrust[2] == 0 && heartbeats == before);
  ctx = r.publication_context(s, 1.6, false, true, true);
  auto release = a::publish_release_at_boundary(ctx, [&] {
    ++targets;
  });
  assert(release.allowed);
  std::cout
      << "runtime_to_final_boundary_stage_one_ready_with_four_gates_false_one_shot_and_disconnect: PASS\n";
}
