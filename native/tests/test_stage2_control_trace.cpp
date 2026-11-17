// 第二阶段动态行为对照：同一确定性驱动分别链接 5198d33 与重构头文件。
// 所有 double 用 hexfloat 输出，状态/故障文本/权限证据逐项输出；比较器要求逐字节一致，不使用浮点容差。
// 本程序只操作纯内存 Runtime 和假 sink，不启动 ROS、Agent、systemd、串口或真实飞机。
#define main stage2_existing_rc_assertions
#include "test_rc_navigation.cpp"
#undef main
#include "jikl_fc01_system/authority_telemetry.hpp"
#include <iomanip>
#include <limits>

namespace v = fc01::vision;
namespace vel = jikl_velocity;

// 输出同一控制步全部垂向计划和关键状态；NED 反馈转出的高度/速度已在生产代码中定义。
static void trace_core(const char* label, unsigned row, const a::Core& core, const a::Output& out) {
  std::cout << label << ',' << row << ',' << a::name(core.phase()) << ',' << core.reason() << ','
            << core.event() << ',' << core.flight_started() << ',' << core.liftoff_confirmed()
            << ',' << core.target_height_settled() << ',' << core.navigation_allowed() << ','
            << core.reference_z() << ',' << core.rc_prestream_commits() << ','
            << core.rc_ground_reference_qualified() << ',' << core.rc_ground_reference_samples()
            << ',' << core.rc_manual_arm_proven() << ',' << core.rc_manual_armed_time() << ','
            << core.rc_fault_recovery_ready() << ',' << core.takeover_reason() << ','
            << core.authority_state().takeover_latched << ',' << a::name(core.fault_class()) << ','
            << out.publish << ',' << static_cast<int>(out.command) << ',' << out.visual_motion;
  for (double value : out.q)
    std::cout << ',' << value;
  for (double value : out.thrust)
    std::cout << ',' << value;
  std::cout << ',' << out.height << ',' << out.speed << ',' << out.target_height << ','
            << out.velocity_target << ',' << out.saturated << ',' << out.slew_limited << ','
            << out.vertical_closed_loop << ',' << out.takeoff_thrust_limited << ','
            << out.proportional_thrust << ',' << out.integral_thrust << ',' << out.raw_thrust << ','
            << out.thrust_ceiling << '\n';
}

// 覆盖冻结/退积分/物理饱和/起飞 ceiling/估计器 reset/非法输入，保留 double 的原始精确结果。
static void vertical_trace() {
  const auto config = production_no_vision_rc_config().attitude.vertical;
  for (unsigned scenario = 0; scenario < 16; ++scenario) {
    a::VerticalController controller(config);
    controller.reset(scenario % 3 * .20);
    for (unsigned row = 0; row < 256; ++row) {
      a::VerticalSample s{
          2., 2. - double(row % 43) * .035, double(int(row % 15) - 7) * .04, true, true, true, 0,
          0};
      const double dt = row % 5 == 0 ? .10 : .025;
      if (scenario == 1 && row == 40)
        controller.freeze_integral(true);
      if (scenario == 1 && row == 80)
        controller.freeze_integral(false);
      if (scenario == 2 && row == 80)
        controller.retain_lift_reserve();
      if (scenario == 3 && row >= 80)
        s.z_reset = 1;
      if (scenario == 4 && row == 80)
        s.fresh = false;
      if (scenario == 5 && row == 80)
        s.z0 = std::numeric_limits<double>::infinity();
      if (scenario == 6 && row == 80)
        s.z = -4.;
      const double ceiling = scenario == 7 ? double(row % 17) * .04 : .8;
      const double target = scenario == 8 ? 7. : scenario == 9 ? -.1 : 1.5;
      auto out =
          controller.step(s, target, scenario == 10 && row == 80 ? 0 : dt,
                          scenario == 11 ? -.20 : .07, scenario == 12 && row == 80 ? -1. : ceiling);
      if (scenario >= 13 && row % 9 == 0)
        out = controller.ramp(double(int(row % 15) - 2) * .1, dt);
      std::cout << "vertical," << scenario << ',' << row << ',' << out.healthy << ',' << out.reason
                << ',' << out.saturated << ',' << out.slew_limited << ',' << out.thrust << ','
                << out.height << ',' << out.speed << ',' << out.velocity_target << ','
                << out.closed_loop << ',' << out.takeoff_limited << ',' << out.proportional_thrust
                << ',' << out.integral_thrust << ',' << out.raw_thrust << ',' << out.thrust_ceiling
                << ',' << controller.integral() << '\n';
    }
  }
}

// 覆盖重复戳、回退、缺样、诊断速度差和硬位置积分残差；锁存后仍继续喂健康尾帧检验不自清。
static void feedback_trace() {
  for (unsigned scenario = 0; scenario < 8; ++scenario) {
    a::VerticalFeedbackConfig config;
    config.enabled = scenario != 0;
    a::VerticalFeedbackGuard guard(config);
    std::uint64_t stamp = 1000000;
    double z = 2.;
    for (unsigned row = 0; row < 100; ++row) {
      if (scenario != 1 || row % 3)
        stamp += 50000;
      if (scenario == 2 && row == 30)
        stamp -= 100000;
      if (scenario == 3 && row == 30)
        stamp += 500000;
      const double rate = scenario == 4 ? -.10 : 0.;
      z += rate * .05;
      const auto& e =
          guard.observe(stamp, scenario == 5 && row > 30 ? z - .5 : z, scenario == 6 ? .7 : rate,
                        scenario == 7 && row == 30 ? NAN : rate, .3);
      std::cout << "feedback," << scenario << ',' << row << ',' << e.checked << ',' << e.healthy
                << ',' << e.suspect << ',' << e.position_window_checked << ',' << e.rate_warning
                << ',' << e.rate_error_mps << ',' << e.position_rate_error_mps << ','
                << e.disagreement_seconds << ',' << e.sample_stamp << ',' << e.disagreement_samples
                << ',' << e.reason << '\n';
    }
  }
}

struct TraceManual {
  static constexpr std::uint8_t SOURCE_RC = 1;
  std::uint64_t timestamp{}, timestamp_sample{};
  bool valid{true};
  std::uint8_t data_source{1};
  bool sticks_moving{};
  double roll{}, pitch{}, yaw{}, throttle{-1};
};
struct TraceSwitches {
  std::uint64_t timestamp{}, timestamp_sample{};
  std::uint8_t mode_slot{1}, arm_switch{3}, return_switch{3}, loiter_switch{3}, offboard_switch{3},
      kill_switch{3};
  std::uint32_t switch_changes{};
};
// 构造多来源、无效源、Kill ON/OFF 与 envelope/raw 不同步；只读取 Facts，不构造任何动作。
static void authority_trace() {
  f::AuthorityTelemetry telemetry;
  TraceManual manual;
  TraceSwitches sw;
  for (unsigned row = 0; row < 256; ++row) {
    const auto stamp = 1000000 + std::uint64_t(row) * 50000;
    const double now = row * .05;
    manual.timestamp = sw.timestamp = stamp;
    if (row % 7)
      manual.timestamp_sample = sw.timestamp_sample = stamp;
    manual.valid = row % 17 != 0;
    manual.data_source = row % 23 == 0 ? 2 : 1;
    manual.roll = row % 13 == 0 ? .3 : 0;
    manual.pitch = row % 19 == 0 ? NAN : 0;
    manual.throttle = row % 29 == 0 ? -.8 : -1.;
    manual.sticks_moving = row % 11 == 0;
    sw.mode_slot = row % 9 == 0 ? 6 : 1;
    sw.kill_switch = row % 5 == 0 ? 1 : 3;
    sw.switch_changes = row / 3;
    telemetry.manual(manual, now, stamp, .5);
    telemetry.switches(sw, now, .5, row % 31 == 0 ? stamp + 1000000 : stamp);
    const auto facts =
        telemetry.facts(now, .5, true, row % 37 == 0, row % 41 == 0 ? 2 : 1, 1, stamp);
    std::cout << "authority," << row << ',' << facts.rc_link_ready << ',' << facts.rc_switches_ready
              << ',' << facts.kill << ',' << facts.manual_seen << ',' << facts.manual_valid << ','
              << facts.rc_source_selected << ',' << facts.sticks_moving << ','
              << facts.switches_seen << ',' << facts.switches_fresh << ','
              << facts.rc_source_clock_valid << ',' << facts.rc_switch_clock_valid << ','
              << unsigned(facts.data_source) << ',' << unsigned(facts.mode_slot) << ','
              << unsigned(facts.arm_switch) << ',' << unsigned(facts.kill_switch) << ','
              << facts.switch_generation << ',' << facts.rc_axes_safe << ','
              << facts.rc_aux_switches_safe << ',' << facts.rc_throttle << ','
              << facts.switch_sample_stamp << '\n';
  }
}

// 在生产候选 1.5m/人工 Arm/视觉关闭参数下逐tick存证；每轮独立重建后再验证安全地面恢复。
static void rc_trace() {
  for (unsigned episode = 0; episode < 12; ++episode) {
    Flight q(production_no_vision_rc_config());
    unsigned frame = 0;
    const auto advance = [&] {
      q.t += .05;
      q.s.status_stamp += 50000;
      q.s.land_stamp += 50000;
      q.s.attitude.local_stamp += 50000;
      q.s.attitude.vertical_sample_stamp += 50000;
      q.s.attitude.rc_switch_sample_stamp += 50000;
      q.s.attitude.source_host_stamp = q.s.status_stamp;
      q.o = q.r.step(q.s, q.t);
      std::string calls;
      if (q.o.attitude.publish) {
        const auto context = q.r.publication_context(q.s, q.t, true, true);
        const auto decision = a::publish_attitude_at_boundary<RcMode, RcTarget>(
            context, q.o.attitude, q.s.status_stamp,
            [&](const RcTarget&) {
              calls += "target;";
            },
            [&](const RcMode&) {
              calls += "heartbeat;";
            });
        if (decision.allowed) {
          ++q.published;
          q.r.confirm_rc_output(true, q.t);
        }
        std::cout << "sink," << episode << ',' << frame << ',' << decision.allowed << ','
                  << a::name(decision.reason) << ',' << calls << '\n';
      }
      trace_core("rc", episode * 1000 + frame++, q.r.attitude_core(), q.o.attitude);
    };
    for (int i = 0; i < 75; ++i)
      advance();
    q.s.attitude.rc_arm_switch = 1;
    ++q.s.attitude.operator_switch_generation;
    advance();
    if (episode % 4 != 1) {
      q.s.feedback.armed = true;
      q.s.armed_time = episode + 10;
      q.s.latest_arming_reason = 2;
      q.s.at_rest = false;
    }
    advance();
    q.s.attitude.rc_mode_slot = q.c.attitude.rc_navigation_mode_slot;
    ++q.s.attitude.operator_switch_generation;
    advance();
    q.s.feedback.offboard = episode % 4 != 2;
    q.s.nav_state = q.s.feedback.offboard ? 14 : 15;
    advance();
    for (unsigned row = 0; row < 150; ++row) {
      if (episode % 4 == 0 && row > 100) {
        q.s.feedback.landed = false;
        q.s.attitude.z = 2. - (row - 100) * .005;
        q.s.attitude.vz = q.s.attitude.z_deriv = -.1;
      }
      // 故障实验不走假 sink 的 allowed 断言；它们必须保存无发布计划和恢复诊断。
      if (episode % 4 == 3 && row == 20)
        q.s.attitude.kill = true;
      if (episode % 4 == 3 && row == 25)
        q.s.attitude.kill = false;
      advance();
    }
    if (episode % 4 == 0) {
      double height = q.r.attitude_core().reference_z() - q.s.attitude.z;
      for (unsigned row = 0; row < 190 && q.r.attitude_core().phase() != a::Phase::HeightHold;
           ++row) {
        height = std::min(1.5, height + .015);
        q.s.attitude.z = q.r.attitude_core().reference_z() - height;
        q.s.attitude.vz = q.s.attitude.z_deriv = height < 1.5 ? -.30 : 0.;
        advance();
      }
      need(q.r.attitude_core().phase() == a::Phase::HeightHold,
           "strict control trace reaches original no-vision 1.5m HeightHold");
      for (unsigned row = 0; row < 10; ++row)
        advance();
      const auto land = q.r.request("LAND", {}, q.s, q.t);
      std::cout << "rc_land," << episode << ',' << land.accepted << ',' << land.message << '\n';
      need(land.accepted, "strict trace retains original landing request");
      for (unsigned row = 0; row < 260 && q.r.attitude_core().phase() != a::Phase::SafeDisarmed;
           ++row) {
        height = std::max(0., height - .01);
        q.s.attitude.z = q.r.attitude_core().reference_z() - height;
        q.s.attitude.vz = q.s.attitude.z_deriv = height > 0 ? .20 : 0.;
        if (height == 0.)
          q.s.feedback.landed = q.s.at_rest = true;
        advance();
        if (q.o.attitude.command == a::Command::Disarm) {
          bool committed = false;
          const auto decision =
              a::publish_disarm_at_boundary(q.r.publication_context(q.s, q.t, true, true), [&] {
                committed = true;
                q.r.confirm_attitude_command_publication(a::Command::Disarm, true,
                                                         q.s.status_stamp + 1, q.s, q.t);
              });
          std::cout << "rc_disarm," << episode << ',' << decision.allowed << ','
                    << a::name(decision.reason) << ',' << committed << '\n';
          need(committed, "strict trace disarm reaches only the guarded fake command sink");
          q.r.ack({q.r.pending(400).baseline + 1, 400, 0, 1, 191, false}, q.t + .001);
          q.s.feedback.armed = false;
        }
      }
      need(q.r.attitude_core().phase() == a::Phase::SafeDisarmed,
           "strict trace requires new ground and actual disarmed feedback");
    }
    q.s.feedback.armed = q.s.feedback.offboard = false;
    q.s.feedback.landed = q.s.at_rest = true;
    q.s.nav_state = 15;
    q.s.attitude.rc_arm_switch = 3;
    q.s.attitude.rc_mode_slot = 6;
    q.s.attitude.z = 2.;
    q.s.attitude.vz = q.s.attitude.z_deriv = 0;
    for (unsigned row = 0; row < 60; ++row)
      advance();
    std::cout << "rc_reprepare," << episode << ',' << q.r.rc_restart_allowed(q.s) << ','
              << q.r.reprepare_rc(q.s, q.t) << '\n';
  }
}

// 程序 Arm 路线单独对照 ACK、新 armed、取消和迟到事件，覆盖从 RequestArm 提取出的完整事务分支。
static void requested_arm_trace() {
  for (unsigned scenario = 0; scenario < 6; ++scenario) {
    auto c = config();
    c.attitude.autonomous_failsafe_confirmed = true;
    f::Runtime runtime(c);
    auto snapshot = sample();
    double now = 0;
    ready(runtime, snapshot, now, "AUTHORITY_AUTONOMOUS");
    need(runtime.request("TAKEOFF", {1.}, snapshot, now).accepted,
         "programmatic trace receives explicit synthetic takeoff authorization");
    for (unsigned row = 0; row < 180; ++row) {
      if (row == 1) {
        if (scenario == 0 || scenario == 1)
          runtime.ack({runtime.pending(400).baseline + 1, 400,
                       static_cast<std::uint8_t>(scenario == 0 ? 0 : 2), 1, 191, false},
                      now + .001);
        if (scenario < 3)
          snapshot.feedback.armed = true;
        if (scenario == 4)
          runtime.request("STOP", {}, snapshot, now);
      }
      if (row == 125 && (scenario == 3 || scenario == 4))
        snapshot.feedback.armed = true;
      const auto out = tick(runtime, snapshot, now);
      trace_core("requested_arm", scenario * 1000 + row, runtime.attitude_core(), out.attitude);
    }
  }
}

// 对 Config 首异常文本进行交叉故障组合对照，确保拆分后依然保留原短路和错误优先级。
static void config_trace() {
  for (unsigned bits = 0; bits < 256; ++bits) {
    auto c = production_no_vision_rc_config().attitude;
    if (bits & 1)
      c.rc_target_height = -1;
    if (bits & 2)
      c.feedback_guard.dwell_seconds = 0;
    if (bits & 4)
      c.loop_hz = NAN;
    if (bits & 8)
      c.land_sample_max_gap = 8;
    if (bits & 16)
      c.recovery_seconds = 100;
    if (bits & 32)
      c.command_timeout = 3;
    if (bits & 64)
      c.vertical.maximum = .1;
    if (bits & 128)
      c.takeoff_reserve_thrust = .8;
    std::cout << "config," << bits << ',';
    try {
      c.validate();
      std::cout << "valid\n";
    } catch (const std::invalid_argument& error) {
      std::cout << error.what() << '\n';
    }
  }
}

// 视觉目标与估计状态独立推进；覆盖3种模式、改变来源、重复载荷、reset 和撤权，禁止取得真实控制权限。
static void vision_trace() {
  for (unsigned scenario = 0; scenario < 8; ++scenario) {
    v::Config c;
    c.enabled = true;
    v::Controller controller(c);
    v::Input in;
    in.state_publishers = in.command_publishers = 1;
    in.state.frame_id = in.command.frame_id = "fc01_vision_ned";
    in.state.body_frame_id = "base_link_frd";
    in.state.source_id = "trace-camera";
    in.command.source_id = "trace-guidance";
    in.state.initialized = in.state.tracking = in.state.calibration_valid = in.command.valid = true;
    in.state.valid_fields = v::Position | v::Velocity | v::Attitude | v::Quality;
    in.state.position = in.state.velocity = {0, 0, 0};
    in.state.attitude = {1, 0, 0, 0};
    in.state.quality = .9;
    in.command.position_valid = in.command.sticks_valid = true;
    in.command.position = {.2, -.1, -.2};
    in.command.sticks = {.2, .3, -.4, .1};
    for (unsigned row = 0; row < 60; ++row) {
      const double now = 10. + row * .05;
      in.ros_now_ns = 100000000000ULL + std::uint64_t(row) * 50000000;
      in.state.sequence = in.command.sequence = row + 1;
      in.state.stamp_ns = in.command.stamp_ns = in.ros_now_ns;
      in.state.received_at = in.command.received_at = now;
      in.command.mode = static_cast<v::Mode>(1 + row / 20);
      if (row == 25) {
        if (scenario == 1)
          in.state.source_id = "changed-camera";
        if (scenario == 2)
          in.state.reset_counter = 1;
        if (scenario == 3)
          in.revocation_generation = 1;
        if (scenario == 4)
          in.state.stamp_ns -= 1000000000;
        if (scenario == 5)
          in.state.position[0] = 3.;
        if (scenario == 6)
          in.command.mode = v::Mode::Land;
        if (scenario == 7)
          in.command.sequence = row;
      }
      const auto out = controller.step(in, now, .5, .1, .05);
      std::cout << "vision," << scenario << ',' << row << ',' << out.active << ','
                << out.request_land << ',' << int(out.mode) << ',' << out.reason << ','
                << out.target_height << ',' << out.yaw << ',' << out.state_sequence << ','
                << out.command_sequence << ',' << out.reset_counter << ','
                << controller.current(in, now);
      for (double value : out.q)
        std::cout << ',' << value;
      for (double value : out.velocity_target)
        std::cout << ',' << value;
      for (double value : out.acceleration_target)
        std::cout << ',' << value;
      std::cout << '\n';
    }
  }
}

// 保留通用/仿真路线的速度滤波和停止捕获：沿既有阶段进入、运行和降落，不迁移到 RC 生产参数。
static void velocity_trace() {
  vel::Config c;
  c.height = 1.;
  c.settle_time = .1;
  vel::Core core(c);
  vel::Feedback feedback;
  feedback.fresh = feedback.valid = feedback.heading_ok = feedback.preflight = feedback.exclusive =
      feedback.landed = true;
  core.receive({}, 0);
  need(core.start(feedback, 0), "velocity trace starts on safe synthetic ground");
  for (unsigned row = 0; row < 300; ++row) {
    const double now = row * .05;
    if (row > 45)
      feedback.offboard = true;
    if (row > 48)
      feedback.armed = true;
    if (row > 50) {
      feedback.landed = false;
      feedback.position.z = -1.;
    }
    if (row > 270) {
      feedback.landed = true;
      feedback.armed = false;
    }
    vel::Velocity command;
    if (row > 100 && row < 180)
      command = {.4, -.3, .1, .2};
    core.receive(command, now);
    if (row == 240)
      core.land(feedback, now, "trace_land");
    const auto out = core.step(feedback, now);
    std::cout << "velocity," << row << ',' << vel::name(core.phase()) << ',' << core.reason() << ','
              << int(out.control) << ',' << int(out.command) << ',' << out.position.x << ','
              << out.position.y << ',' << out.position.z << ',' << out.velocity.x << ','
              << out.velocity.y << ',' << out.velocity.z << ',' << out.yaw << ',' << out.yaw_rate
              << '\n';
  }
}

int main() {
  std::cout << std::hexfloat;
  config_trace();
  vertical_trace();
  feedback_trace();
  authority_trace();
  rc_trace();
  requested_arm_trace();
  vision_trace();
  velocity_trace();
  // 同一驱动末尾仍执行原完整 RC 断言集（含原 1.5m 起飞/降落和连续恢复），不以差分代替原合同。
  char name[] = "stage2_trace";
  char* argv[] = {name, nullptr};
  return stage2_existing_rc_assertions(1, argv);
}
