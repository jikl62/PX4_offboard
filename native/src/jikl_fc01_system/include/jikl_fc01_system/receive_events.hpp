#pragma once
// 接收补读的事件边界：告诉调度器何时必须先给真实控制步骤处理当前观测。
// 普通推进的连续数值可以追到最新值；可输出任务遇离散/无效/跳变事件先停额外补读。
// Safety 完全静默时可按预算继续追赶，但事件仍独立保留逐条扫描，不能被正常尾帧覆盖。
// 返回 true 只影响接收调度，不直接允许发布、触发新的飞行模式或清除故障。
#include "jikl_fc01_system/authority_telemetry.hpp"
#include "jikl_fc01_system/attitude_diagnostics.hpp"
#include <tuple>

namespace fc01 {
// 这些谓词只决定“先停止额外收包，把本条交给控制步骤”；不授权输出，不直接清故障。
// 普通推进的连续数值可追到最新，离散变化和无效反馈则必须先被控制器观察。
// 把来源、Kill、档位、时钟和开关代数纳入比较，保留同周期短暂接管的可见性。
inline auto authority_receive_key(const AuthorityTelemetry::Facts& f) {
  // make_tuple把相关字段组成一个可整体比较的值；这里用作变化检测，不是把元组解释成授权结果。
  return std::make_tuple(f.rc_link_ready, f.rc_switches_ready, f.kill, f.manual_seen,
                         f.manual_valid, f.rc_source_selected, f.sticks_moving,
                         f.rc_source_clock_valid, f.rc_switch_clock_valid, f.data_source,
                         f.mode_slot, f.arm_switch, f.kill_switch, f.switch_generation,
                         f.rc_axes_safe, f.rc_aux_switches_safe);
}
// PX4 模式/解锁/失效/校准/可选模式变化均属离散事件，时间戳本身推进不算事件。
template <class M> bool status_receive_event(const M& a, const M& b) {
  // a为上一值、b为新值；lambda只收本次借用参数，返回元组副本。!=检测任一离散字段变化。
  const auto key = [](const M& m) {
    return std::make_tuple(m.system_id, m.component_id, m.arming_state, m.armed_time,
                           m.latest_arming_reason, m.nav_state, m.failsafe,
                           m.failsafe_and_user_took_over, m.calibration_enabled,
                           m.rc_calibration_in_progress, m.hil_state, m.pre_flight_checks_pass,
                           m.power_input_valid, m.can_set_nav_states_mask);
  };
  return key(a) != key(b);
}
// landed 和 at_rest 的任一边沿影响离地/落地/恢复证据，不能跳过。
template <class M> bool land_receive_event(const M& a, const M& b) {
  return a.landed != b.landed || a.at_rest != b.at_rest;
}
// 遥控/Offboard 丢失和严重故障的变化需要现有安全逻辑逐次观察。
template <class M> bool flags_receive_event(const M& a, const M& b) {
  return a.manual_control_signal_lost != b.manual_control_signal_lost ||
         a.offboard_control_signal_lost != b.offboard_control_signal_lost ||
         a.fd_critical_failure != b.fd_critical_failure;
}
template <class M> bool battery_receive_event(const M& a, const M& b) {
  // 电池低频，电压的每次变化也保留；低压阈值属于上层控制器，接收调度器不另造“安全电压”。
  return a.connected != b.connected || a.warning != b.warning || a.voltage_v != b.voltage_v;
}
// z、vz、z_deriv 均来自 PX4 NED：m、m/s、m/s，向下为正；此处不建立地面零点。
template <class M>
bool local_receive_event(const M& a, const M& b, const attitude::Config& c,
                         bool allow_unused_horizontal = false) {
  const auto key = [](const M& m) {
    return std::make_tuple(m.xy_valid, m.z_valid, m.v_xy_valid, m.v_z_valid,
                           m.heading_good_for_control, m.dead_reckoning, m.xy_reset_counter,
                           m.z_reset_counter, m.vxy_reset_counter, m.vz_reset_counter,
                           m.heading_reset_counter);
  };
  // 有效位、航向质量、推算和所有估计器 reset 变化先处理，即便新的数值仍看似正常。
  if (key(a) != key(b))
    return true;
  for (double value : {double(b.z), double(b.vz), double(b.z_deriv)})
    if (!std::isfinite(value))
      return true;
  // 纯垂向路线不用明确无效的水平估计；不能让重复NaN每次都中止补读，害得有效Z/Vz滞留DDS队列。
  // 有效位/reset变化及“自称有效却非有限”的水平字段仍属于事件，不能借此绕过保护。
  // 纯垂向消费者可忽略明确标无效的水平 NaN，防止未使用字段让新鲜 Z/Vz 卡在 DDS 队列后方。
  if ((!allow_unused_horizontal || b.xy_valid) && (!std::isfinite(b.x) || !std::isfinite(b.y)))
    return true;
  if ((!allow_unused_horizontal || b.v_xy_valid) && (!std::isfinite(b.vx) || !std::isfinite(b.vy)))
    return true;
  if ((!allow_unused_horizontal || b.heading_good_for_control) && !std::isfinite(b.heading))
    return true;
  // 差异和恢复一致都要保留；漏掉恢复边沿会把两次独立异常错误拼成一段连续告警。
  // 速度差仅是独立诊断：保留差异样本，也保留恢复一致边沿，避免把两次事件拼成一次持续超时。
  if (c.feedback_guard.enabled) {
    const bool before = std::abs(double(a.vz) - a.z_deriv) > c.feedback_guard.rate_tolerance_mps;
    const bool after = std::abs(double(b.vz) - b.z_deriv) > c.feedback_guard.rate_tolerance_mps;
    if (after || before != after)
      return true;
  }
  // 未解锁静止计时也使用Vz；离开/回到静止带先交控制步骤观察，本函数不建立地面零点。
  // 离开和回到静止 Vz 带都影响地面静止计时，必须先让节点看见，不能凭后帧续成静止。
  if (c.ground_vz_tolerance > 0 && (std::abs(double(a.vz)) <= c.ground_vz_tolerance) !=
                                       (std::abs(double(b.vz)) <= c.ground_vz_tolerance))
    return true;
  // 只复用已配置的估计器跳变界限，跳变必须先被处理，不能被下一条正常样本替换掉。
  return (c.z_jump_limit_m > 0 && std::abs(double(b.z) - a.z) > c.z_jump_limit_m) ||
         (c.vz_jump_limit_mps > 0 && std::abs(double(b.vz) - a.vz) > c.vz_jump_limit_mps);
}
// 姿态 reset 或无效四元数保留为事件；有效实测姿态（含大倾角）按连续反馈处理。
// 此谓词不检查倾角阈值；不要把它误认为独立的大倾角停机保护。
template <class M> bool attitude_receive_event(const M& a, const M& b, const attitude::Config& c) {
  if (a.quat_reset_counter != b.quat_reset_counter)
    return true;
  const attitude::Quaternion q{b.q[0], b.q[1], b.q[2], b.q[3]};
  const auto diagnostic = attitude::attitude_diagnostics(q, {1, 0, 0, 0});
  // 有效实测姿态按连续量处理，即使倾角较大也不在此中断补读，避免状态/估计器/权限事件被积压。
  // 无效四元数仍是事件，须先到既有独立健康门；配置参数保留以维持统一API。
  (void)c;
  // (void)c显式说明此函数当前不使用配置；不是删除对姿态质量的检查，下一行仍拒绝无效四元数。
  return !diagnostic.valid;
}
} // namespace fc01
