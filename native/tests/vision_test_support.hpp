#pragma once
// 视觉集成离线夹具：用合成反馈走真实 RC 起飞、Runtime/Core 和最终双发布边界。
// 不连接 ROS/PX4；目标与心跳 sink 仅在本进程保存消息、计数，不代表飞控接收或实际飞行。
// 只有测试配置启用视觉；自动 Arm 仍关闭，生产 visual_navigation_enabled=false 和 1.5 m 路径不在此修改。
// Entirely synthetic feedback. Exercise the production Runtime and final sinks;
// no ROS includes, PX4 publishers, fabricated permissions or automatic ARM.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <stdexcept>

namespace vision_test {
namespace f = fc01;
namespace a = fc01::attitude;
namespace v = fc01::vision;
inline void need(bool ok, const char* reason) {
  if (!ok)
    throw std::runtime_error(reason);
}
// 测试参数显式启用既有人工 RC 路线和视觉；每次仍用 Config::validate 校验，不制造自动解锁权限。
inline f::Config config() {
  auto c = authority_test::config();
  auto& p = c.attitude;
  p.rc_navigation_enabled = true;
  p.arm_enabled = false;
  p.rc_takeover_confirmed = true;
  p.rc_navigation_mode_slot = 1;
  p.max_flight_seconds = 30;
  p.max_tilt_deg = 10;
  p.land_sample_max_gap = 1.25;
  p.height_tolerance = .15;
  p.vertical.max_climb = .3;
  p.visual.enabled = true;
  c.validate();
  return c;
}
// 最小消息替身保留真实编码字段，用于检验最终边界产生的心跳/姿态目标，不是 ROS 消息发布者。
struct RcMode {
  std::uint64_t timestamp{};
  bool position{}, velocity{}, acceleration{}, attitude{}, body_rate{}, thrust_and_torque{},
      direct_actuator{};
};
struct RcTarget {
  std::uint64_t timestamp{};
  float yaw_sp_move_rate{};
  std::array<float, 4> q_d{};
  std::array<float, 3> thrust_body{};
  bool reset_integral{}, fw_control_yaw_wheel{};
};
// t 为单调模拟秒；源戳单位 µs。目标/心跳分别记本轮与累计提交，自动 Arm/Offboard 命令独立计数。
struct VisionFlight {
  f::Config c;
  f::Runtime r;
  f::Snapshot s{authority_test::sample()};
  double t{};
  f::Output o;
  unsigned published{}, arm_commands{}, mode_commands{}, target_commits{}, heartbeat_commits{};
  bool target_committed{}, heartbeat_committed{};
  RcTarget last_target{};
  RcMode last_mode{};
  a::Decision last_decision{};
  // 初始状态为未解锁 Stabilized、安全 RC 开关和非导航档；显式要求最终 sink 提交才能推进责任状态。
  explicit VisionFlight(f::Config cfg = config()) : c(std::move(cfg)), r(c) {
    s.nav_state = 15;
    s.attitude.source_clock_validated = true;
    s.attitude.rc_link_ready = s.attitude.rc_switches_ready = true;
    s.attitude.rc_source_clock_valid = s.attitude.rc_switch_clock_valid = true;
    s.attitude.rc_source_selected = s.attitude.rc_axes_safe = s.attitude.rc_aux_switches_safe =
        true;
    s.attitude.rc_mode_slot = c.attitude.rc_navigation_mode_slot == 1 ? 6 : 1;
    s.attitude.rc_arm_switch = s.attitude.rc_kill_switch = 3;
    s.attitude.rc_switch_sample_stamp = 1000;
    s.attitude.local_stamp = s.attitude.vertical_sample_stamp = 1000;
    s.attitude.z_deriv = s.attitude.vz = 0;
    r.require_attitude_sink_commit();
  }
  void set_visual(v::Input value) {
    s.attitude.visual_input = std::move(value);
  }
  // 按真实 dt 推进各独立源戳，然后调用 Runtime；只有最终边界允许且实际假 sink 返回才确认本地提交。
  void tick(bool commit = true, double dt = .05) {
    need(std::isfinite(dt) && dt > 0, "INVALID_SYNTHETIC_DT");
    t += dt;
    const auto advance = static_cast<std::uint64_t>(std::llround(dt * 1e6));
    s.status_stamp += advance;
    s.land_stamp += advance;
    s.attitude.local_stamp += advance;
    s.attitude.vertical_sample_stamp += advance;
    s.attitude.rc_switch_sample_stamp += advance;
    s.attitude.source_host_stamp = s.status_stamp;
    target_committed = heartbeat_committed = false;
    o = r.step(s, t);
    if (o.attitude.command == a::Command::Arm)
      ++arm_commands;
    if (o.attitude.command == a::Command::Offboard)
      ++mode_commands;
    if (o.attitude.publish) {
      const auto context = r.publication_context(s, t, true, true);
      last_decision = commit ? a::publish_attitude_at_boundary<RcMode, RcTarget>(
                                   context, o.attitude, s.status_stamp,
                                   [&](const RcTarget& target) {
                                     last_target = target;
                                     ++target_commits;
                                     target_committed = true;
                                   },
                                   [&](const RcMode& mode) {
                                     last_mode = mode;
                                     ++heartbeat_commits;
                                     heartbeat_committed = true;
                                   })
                             : a::attitude_publication_policy(context, o.attitude);
      if (commit) {
        r.confirm_rc_output(last_decision.allowed, t);
        if (last_decision.allowed)
          ++published;
      }
    } else
      last_decision =
          a::attitude_publication_policy(r.publication_context(s, t, true, true), o.attitude);
  }
  void warm() {
    for (int i = 0; i < 75; ++i)
      tick();
  }
  // 先模拟新 RC 解锁开关，再给独立新 armed_time/人工解锁原因；两步反馈不能由视觉消息代替。
  void manual_arm() {
    s.attitude.rc_arm_switch = 1;
    ++s.attitude.operator_switch_generation;
    tick();
    s.feedback.armed = true;
    s.armed_time = 10;
    s.latest_arming_reason = 2;
    s.at_rest = false;
    tick();
  }
  // 先模拟新导航档操作，再给实际 Offboard 状态；不向飞控发送进入模式的自动命令。
  void navigation() {
    s.attitude.rc_mode_slot = c.attitude.rc_navigation_mode_slot;
    ++s.attitude.operator_switch_generation;
    tick();
    s.feedback.offboard = true;
    s.nav_state = 14;
    tick();
  }
  // 完整准备/人工解锁/切档后必须到 ArmedGround，同时断言整段未生成自动 Arm 或 Offboard 命令。
  void join() {
    warm();
    manual_arm();
    navigation();
    need(r.attitude_core().phase() == a::Phase::ArmedGround, "SYNTHETIC_RC_JOIN_FAILED");
    need(arm_commands == 0 && mode_commands == 0, "SYNTHETIC_ROUTE_EMITTED_ARM_OR_OFFBOARD");
  }
  // 调用者给相对地面上正 height(m)/up_speed(m/s)，写入 PX4 NED 时用 z=2-height、vz=z_deriv=-up_speed。
  void observe(double height, double up_speed = 0) {
    s.attitude.z = 2 - height;
    s.attitude.vz = s.attitude.z_deriv = -up_speed;
  }
  // 用每步 0.01 m 的合成爬升进入平台保持；每步检查最终提交许可，不把该理想轨迹当机体动力学。
  void reach_hold() {
    join();
    for (int i = 0; i < 100 && r.attitude_core().phase() != a::Phase::ThrustRamp; ++i)
      tick();
    need(r.attitude_core().phase() == a::Phase::ThrustRamp, "SYNTHETIC_RAMP_NOT_REACHED");
    double height = 0;
    const double target = c.attitude.rc_target_height;
    while (height < target - 1e-9) {
      height = std::min(target, height + .01);
      observe(height, .2);
      s.feedback.landed = s.at_rest = false;
      tick();
      need(last_decision.allowed, "SYNTHETIC_ASCENT_BOUNDARY_DENIED");
    }
    observe(target);
    for (int i = 0; i < 160 && r.attitude_core().phase() != a::Phase::HeightHold; ++i)
      tick();
    need(r.attitude_core().phase() == a::Phase::HeightHold, "SYNTHETIC_HEIGHT_HOLD_NOT_REACHED");
    need(last_decision.allowed, "SYNTHETIC_HEIGHT_HOLD_BOUNDARY_DENIED");
  }
};
} // namespace vision_test
