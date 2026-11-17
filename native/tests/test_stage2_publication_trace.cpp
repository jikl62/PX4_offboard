// 同一原始输入在基线/当前发布策略上回放：精确比较拒绝原因、三次采样与非原子sink顺序。
// 仅假Mode/Target和内存记录；不使用ROS/DDS，不提交飞控消息。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main original_publication_regression_main
#include "test_publication_boundary.cpp"
#undef main
#pragma GCC diagnostic pop
#include <iomanip>

// 按单个索引破坏一种独立证据，也保留好输入；只改本次局部上下文，不改变生产配置。
static void change_context(a::BoundaryContext& c, int variant) {
  switch (variant) {
  case 1:
    c.shutdown = true;
    break;
  case 2:
    c.feedback.status_fresh = false;
    break;
  case 3:
    c.feedback.land_fresh = false;
    break;
  case 4:
    c.feedback.local_fresh = false;
    break;
  case 5:
    c.feedback.source_clock_valid = false;
    break;
  case 6:
    c.feedback.kill = true;
    break;
  case 7:
    c.feedback.operator_takeover = true;
    break;
  case 8:
    c.feedback.exclusive = false;
    break;
  case 9:
    c.reference_ready = false;
    break;
  case 10:
    c.control_fresh = false;
    break;
  case 11:
    c.session_connected = false;
    break;
  case 12:
    c.operator_lease_live = false;
    break;
  case 13:
    c.config.flight_actions_enabled = false;
    break;
  case 14:
    c.feedback.armed = !c.feedback.armed;
    break;
  case 15:
    c.feedback.landed = false;
    break;
  case 16:
    c.feedback.z = std::numeric_limits<double>::quiet_NaN();
    break;
  case 17:
    c.feedback.z = -100;
    break;
  case 18:
    c.authority.takeover_latched = true;
    break;
  case 19:
    c.yaw_reference = .3;
    break;
  case 20:
    c.feedback.rc_axes_safe = false;
    break;
  case 21:
    c.feedback.rc_sticks_moving = true;
    break;
  case 22:
    c.rc_arm_evidence_pending = true;
    break;
  case 23:
    c.rc_navigation_zero_allowed = true;
    break;
  case 24:
    c.feedback.armed_time++;
    break;
  case 25:
    c.feedback.rc_mode_slot = 6;
    break;
  default:
    break;
  }
}

// 返回准备/常规/人工RC三类合成上下文；RC目标1.5m等生产候选值不由本测试重写。
static a::BoundaryContext trace_context(int route) {
  auto c = context(route != 0);
  if (route == 2) {
    c.config.rc_navigation_enabled = c.config.rc_takeover_confirmed = true;
    c.config.arm_enabled = c.config.autonomous_failsafe_confirmed = false;
    c.config.max_flight_seconds = 30;
    c.config.max_tilt_deg = 10;
    c.authority = {fc01::authority::Mode::RcTakeover, fc01::authority::Mode::RcTakeover, true,
                   false};
    c.feedback.source_clock_validated = c.feedback.rc_link_ready = c.feedback.rc_switches_ready =
        true;
    c.feedback.rc_source_clock_valid = c.feedback.rc_switch_clock_valid = true;
    c.feedback.rc_source_selected = c.feedback.rc_axes_safe = c.feedback.rc_aux_switches_safe =
        true;
    c.feedback.rc_arm_switch = 3;
    c.feedback.rc_mode_slot = 1;
  }
  return c;
}

// 完整phase枚举×路线×单项坏证据×精确double推力；拒绝原因必须严格一致，不能只比较allowed。
static void policy_matrix() {
  for (int route = 0; route < 3; ++route)
    for (int phase = 0; phase <= int(a::Phase::RcWaitNavigation); ++phase)
      for (int variant = 0; variant <= 25; ++variant)
        for (int thrust = 0; thrust < 6; ++thrust) {
          auto c = trace_context(route);
          c.phase = static_cast<a::Phase>(phase);
          change_context(c, variant);
          a::Output output;
          output.publish = true;
          output.q = {1, 0, 0, 0};
          if (thrust == 1)
            output.thrust[2] = -.4;
          if (thrust == 2)
            output.thrust[2] = std::numeric_limits<double>::denorm_min();
          if (thrust == 3)
            output.thrust[0] = -.1;
          if (thrust == 4)
            output.thrust[2] = std::numeric_limits<double>::infinity();
          if (thrust == 5) {
            output.thrust[2] = -.4;
            output.visual_motion = true;
          }
          const auto decision = a::attitude_publication_policy(c, output);
          std::cout << "POLICY " << route << ':' << phase << ':' << variant << ':' << thrust << ':'
                    << decision.allowed << ':' << a::name(decision.reason) << '\n';
        }
}

// 在初查、目标前、目标后分别使证据失效，或令任一sink抛异常；记录尝试状态和有序调用串。
static void resampled_matrix() {
  for (int changed_at = 0; changed_at < 5; ++changed_at)
    for (int sink_failure = 0; sink_failure < 3; ++sink_failure) {
      auto c = context();
      a::Output output;
      output.publish = true;
      output.q = {1, 0, 0, 0};
      a::PublicationAttempt evidence;
      std::string calls, result;
      unsigned sampled = 0;
      try {
        const auto decision = a::publish_attitude_resampled<Mode, Target>(
            [&] {
              calls += 'C';
              if (++sampled == unsigned(changed_at))
                c.feedback.status_fresh = false;
              return c;
            },
            output, 123,
            [&](const Target& target) {
              calls += 'T';
              assert(target.timestamp == 123 && target.thrust_body[2] == 0);
              if (sink_failure == 1)
                throw std::runtime_error("target-fixture");
            },
            [&](const Mode& mode) {
              calls += 'H';
              assert(mode.timestamp == 123 && mode.attitude);
              if (sink_failure == 2)
                throw std::runtime_error("heartbeat-fixture");
            },
            &evidence);
        result = a::name(decision.reason);
      } catch (const std::exception& error) {
        result = error.what();
      }
      std::cout << "PAIR " << changed_at << ':' << sink_failure << ':' << calls << ':' << result
                << ':' << int(evidence.stage) << ':' << evidence.decision.allowed << ':'
                << a::name(evidence.decision.reason) << ':' << evidence.target_committed
                << evidence.heartbeat_committed << evidence.sink_exception << '\n';
    }
}

// 所有输出只供严格动态差分；原三次检查与目标/心跳的非原子事实均有独立可见记录。
int main() {
  policy_matrix();
  resampled_matrix();
  return 0;
}
