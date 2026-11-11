#pragma once
// 姿态高度主链的配置、输入快照、状态枚举和输出合同。
// Config 的动作门、反馈的新鲜度及权限必须分别成立；结构体中的数值或布尔默认值不表示真实硬件状态已确认。
#include "jikl_fc01_system/attitude_reference.hpp"
#include "jikl_fc01_system/vertical_controller.hpp"
#include "jikl_fc01_system/vertical_feedback_guard.hpp"
#include "jikl_fc01_system/authority.hpp"
#include "jikl_fc01_system/vision_navigation.hpp"
#include <cstdint>

namespace fc01::attitude {
// 配置按准备、RC人工授权、垂向闭环、视觉外环和恢复策略分组；validate 在构造 Core 时执行。
struct Config {
  // 四个门分别允许动力动作、程序解锁、起飞和已人工确认参数；RC 路线要求程序 arm_enabled=false。
  bool flight_actions_enabled{}, arm_enabled{}, takeoff_enabled{}, parameters_confirmed{};
  bool rc_navigation_enabled{};
  // RC 起飞目标为向上相对高度 m；人工解锁到选择导航的最长等待为 s。
  double rc_target_height{1.5}, rc_arm_to_navigation_timeout{8};
  // PX4 六档导航选择位置，不等于 nav_state 枚举；实际候选 YAML 可覆盖默认槽位。
  std::int64_t rc_navigation_mode_slot{6};
  // Dedicated no-propeller engineering mode, disabled by default. Normal TAKEOFF is forbidden.
  // 拆桨专用解锁观察路径；开启时必须禁止普通 TAKEOFF，不能靠等时间进入推力爬坡。
  bool bench_arm_only_enabled{};
  // Bench-only estimator grace. Upward height still uses the flight liftoff
  // threshold; these values cannot admit an ordinary TAKEOFF or touchdown.
  double bench_ground_down_tolerance_m{.50}, bench_ground_vz_tolerance_mps{.25};
  // 这两个确认位对应不同控制权策略，不能用“有遥控消息”替代已确认的接管合同。
  bool rc_takeover_confirmed{}, autonomous_failsafe_confirmed{};
  // 高度与推力增益/包络来自候选 YAML；FeedbackGuard 不另选传感器；visual.enabled 控制视觉链是否存在。
  VerticalConfig vertical;
  VerticalFeedbackConfig feedback_guard;
  vision::Config visual;
  // 预发送/地面证据时长为 s，loop_hz 为 Hz，stall_seconds 是相邻控制步允许的最大秒间隔。
  double prestream_seconds{1.2}, ground_seconds{2}, loop_hz{20}, stall_seconds{.3};
  // 各数据源有独立秒级时限；状态/着陆低频反馈与姿态/本地位置的限制不能混用。
  double attitude_timeout{.5}, local_timeout{.5}, status_timeout{2.5}, land_timeout{2.5},
      flags_timeout{2.5};
  // 控制台租约、事务确认期限单位 s；地面速度容差为 |NED vz| 的 m/s 幅值。
  double console_timeout{3}, command_timeout{5}, ground_vz_tolerance{.1};
  // 着陆检测新样本之间允许的最大主机秒间隔；超时会重新累计地面证据。
  double land_sample_max_gap{.5};
  // Flight-only supervised envelope; values MUST be validated on the actual airframe.
  // 这些负值是尚未确认飞行包络的哨兵；地面停留/离地证据时长为 s，高度为 m，上升速度幅值为 m/s。
  double armed_dwell_seconds{-.1}, liftoff_dwell_seconds{-.1}, liftoff_height_m{-.1},
      liftoff_speed_mps{-.1};
  // 触地连续确认时长为 s；高度下界/位置跳变为 m，垂速跳变为 m/s；不能在任务中悄悄放宽。
  double touchdown_dwell_seconds{-.1}, height_lower_margin_m{-.1}, z_jump_limit_m{-.1},
      vz_jump_limit_mps{-.1};
  // Normal TAKEOFF only: bounded, time-ramped lift reserve above historical hover.
  // Zero preserves R6.3.1 behavior; value is NOT an airborne calibration.
  // 归一化起飞推力余量只在悬停候选以上有界爬升；它不是实测校准，也不是空中最低推力。
  double takeoff_reserve_thrust{0.0};
  // 斜坡/稳定/着陆时长单位 s，高度误差为 m，速度容差为向上速度的绝对 m/s。
  double ramp_seconds{-1}, settle_seconds{-1}, landing_seconds{-1}, height_tolerance{-1},
      speed_tolerance{-1};
  // 上升/下降/饱和/任务最长时间为s；max_tilt_deg为目标倾角上限，也用于RC地面/零流桥接准入。
  double ascent_timeout{-1}, descent_timeout{-1}, saturation_seconds{-1}, max_flight_seconds{-1},
      max_tilt_deg{-1};
  // Recovery completion deadline, not an output-authorization lease. One
  // budget per flight; LAND/console loss/shutdown/bounce cannot renew it.
  // Expiry must be disclosed, but cannot prove PX4 has taken airborne custody.
  // 恢复预算只记录本次托管的绝对完成期限，不能在超时后宣称已落地或 PX4 已安全接管。
  double recovery_seconds{15}, px4_land_ack_seconds{5};
  bool px4_land_fallback_confirmed{};
  // 只有实际验证的 PX4 Land 后备方案才填写外部证据标识；非空文本自身仍不是飞行成功证明。
  std::string px4_land_evidence_id;
  // 先检查视觉与原包络关系，再检查RC、准备和动力参数；抛异常让启动入口拒绝不一致配置。
  void validate() const {
    visual.validate();
    // 视觉必须沿现有RC路线且不得超过原高度、速度、倾角包络，接口存在不构成独立控制权。
    if (visual.enabled &&
        (!rc_navigation_enabled || visual.max_tilt_deg > max_tilt_deg ||
         visual.max_height > vertical.max_height ||
         visual.max_vertical_speed > std::min(vertical.max_climb, vertical.max_descent)))
      throw std::invalid_argument("VISUAL_NAVIGATION_REQUIRES_RC_ROUTE_AND_EXISTING_ENVELOPE");
    // 人工RC路线只允许人工解锁，并要求明确起飞许可、短任务、有限倾角与有效档位。
    if (rc_navigation_enabled &&
        (!flight_actions_enabled || arm_enabled || !takeoff_enabled || !parameters_confirmed ||
         !rc_takeover_confirmed || autonomous_failsafe_confirmed || bench_arm_only_enabled ||
         rc_navigation_mode_slot < 1 || rc_navigation_mode_slot > 6 ||
         !std::isfinite(rc_target_height) || rc_target_height <= 0 ||
         rc_target_height >= vertical.max_height || !std::isfinite(rc_arm_to_navigation_timeout) ||
         rc_arm_to_navigation_timeout < 1 || rc_arm_to_navigation_timeout > 10 ||
         max_flight_seconds > 30 || max_tilt_deg > 10))
      throw std::invalid_argument("INVALID_MANUAL_RC_NAVIGATION_CONFIG");
    validate_preparation();
    validate_flight_limits();
  }
  // 校验准备、时间窗口、bench 与恢复合同；无参数/返回值，失败抛原有首个 invalid_argument。
  // 只读配置，不改状态或参数。必须在视觉/RC 交叉校验之后、完整动力限制之前，保留原异常优先级。
  void validate_preparation() const {
    feedback_guard.validate();
    for (double value :
         {prestream_seconds, ground_seconds, loop_hz, stall_seconds, attitude_timeout,
          local_timeout, status_timeout, land_timeout, flags_timeout, console_timeout,
          command_timeout, ground_vz_tolerance, land_sample_max_gap, bench_ground_down_tolerance_m,
          bench_ground_vz_tolerance_mps})
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("INVALID_PREPARATION_PARAMETER");
    if (bench_ground_down_tolerance_m > .50 || bench_ground_vz_tolerance_mps > .25)
      throw std::invalid_argument("BENCH_GROUND_GRACE_OUTSIDE_BOUNDED_LIMITS");
    if (land_sample_max_gap >= land_timeout)
      throw std::invalid_argument("INVALID_LAND_SAMPLE_GAP");
    if (!std::isfinite(recovery_seconds) || recovery_seconds <= 0 || recovery_seconds > 60 ||
        !std::isfinite(px4_land_ack_seconds) || px4_land_ack_seconds <= 0 ||
        px4_land_ack_seconds > 5)
      throw std::invalid_argument("INVALID_BOUNDED_RECOVERY_POLICY");
    if (px4_land_fallback_confirmed && px4_land_evidence_id.empty())
      throw std::invalid_argument("PX4_LAND_FALLBACK_NEEDS_EXTERNAL_EVIDENCE_ID");
    // 准备时长、至少2秒静止、20..50Hz频率与5秒命令期限属于现有合同，不因排版改变。
    if (prestream_seconds < 1.1 || ground_seconds < 2 || loop_hz < 20 || loop_hz > 50 ||
        command_timeout != 5)
      throw std::invalid_argument("PRESTREAM_REST_RATE_OR_ACK_TIMEOUT_INVALID");
    if (bench_arm_only_enabled &&
        (!flight_actions_enabled || !arm_enabled || takeoff_enabled || !parameters_confirmed))
      throw std::invalid_argument("ARM_ONLY_REQUIRES_EXCLUSIVE_GATES_TAKEOFF_DISABLED");
  }
  // 仅动作门开启时核验完整动力包络；推力无量纲、时间 s、高度 m、速度 m/s、倾角 deg。
  // 无参数/返回值，失败抛原错误码；不修改配置，不推测实机参数。保持确认声明→vertical→飞行限制顺序。
  void validate_flight_limits() const {
    if (flight_actions_enabled) { //flight_actions_enabled=false 时，不执行完整飞行动力参数校验。
      if (!parameters_confirmed)
        throw std::invalid_argument("FLIGHT_PARAMETERS_NOT_CONFIRMED_BY_OPERATOR");
      // 开放动作后先确认人工参数声明，再校验动力参数；失败直接拒绝启动。
      vertical.validate();
      for (double value :
           {ramp_seconds, settle_seconds, landing_seconds, height_tolerance, speed_tolerance,
            ascent_timeout, descent_timeout, saturation_seconds, max_flight_seconds, max_tilt_deg,
            armed_dwell_seconds, liftoff_dwell_seconds, liftoff_height_m, liftoff_speed_mps,
            touchdown_dwell_seconds, height_lower_margin_m, z_jump_limit_m, vz_jump_limit_mps})
        if (!std::isfinite(value) || value <= 0)
          throw std::invalid_argument("EXPLICIT_VALIDATED_FLIGHT_LIMITS_REQUIRED");
      if (!std::isfinite(takeoff_reserve_thrust) || takeoff_reserve_thrust < 0 ||
          takeoff_reserve_thrust > .06 ||
          vertical.hover + takeoff_reserve_thrust > vertical.maximum)
        throw std::invalid_argument("INVALID_BOUNDED_TAKEOFF_RESERVE");
      if (armed_dwell_seconds < .1 || liftoff_height_m >= vertical.max_height ||
          height_lower_margin_m >= vertical.max_height || z_jump_limit_m >= vertical.max_height ||
          touchdown_dwell_seconds < .1)
        throw std::invalid_argument("FLIGHT_OBSERVATION_ENVELOPE_INVALID");
      if (max_tilt_deg >= 90)
        throw std::invalid_argument("MAX_TILT_MUST_BE_BELOW_90_DEGREES");
    }
  }
};
// ROS适配器每轮生成的实际飞机快照；姿态为Hamilton wxyz，机体系FRD，位置/速度为世界NED。
struct Feedback {
  // 视觉实际状态和目标命令使用各自消息合同；即使存在输入，也必须 visual.enabled 和导航阶段同时成立。
  vision::Input visual_input;
  Quaternion q{1, 0, 0, 0};
  std::uint8_t quat_reset{}, z_reset{}, vz_reset{};
  // z 是PX4向下位置m，vz向下为正m/s，stationary_seconds为严格地面静止秒数；电池电压单位V。
  double z{}, vz{}, stationary_seconds{}, battery_voltage{NAN};
  // 源消息时间戳单位µs，用于严格新反馈与事务因果判断，不等同主机steady秒时钟。
  std::uint64_t status_stamp{}, land_stamp{};
  bool attitude_fresh{}, local_fresh{}, status_fresh{}, land_fresh{}, failsafe_fresh{};
  // Pure-Core fixtures have no host clock. The ROS adapter always overwrites
  // both fields from its checked source timestamps at every snapshot.
  // 纯Core旧fixture的valid默认值不代表实机同步；ROS每轮必须覆盖valid和validated两位。
  bool source_clock_valid{true}, source_clock_validated{};
  // 已核对的同步微秒域主机时间；不得与PX4自己的 armed_time 直接跨域比较。
  std::uint64_t source_host_stamp{}; // synchronized adapter clock; never armed_time
  // 接收新消息与测量本身新鲜是不同条件，这两位用于排除新信封包装旧测量。
  bool local_sample_fresh{}, attitude_sample_fresh{};
  bool rc_source_clock_valid{}, rc_switch_clock_valid{};
  // 实际状态有效位、动力/着陆/模式及来源唯一性由适配器提供；没有新鲜证据时不得据缓存授权。
  bool z_valid{}, vz_valid{}, armed{}, landed{}, at_rest{}, offboard{}, failsafe{}, exclusive{},
      time_regressed{};
  bool calibration{}, hil{}, preflight{}, battery_ready{}, manual_ready{};
  bool stabilized_selectable{}, stabilized_selected{};
  // manual_ready above is a legacy observation, never an authority predicate.
  // RC链路、开关遥测、Kill、人工接管和Offboard丢信号必须分开处理；Kill优先于普通降落或解锁。
  bool rc_link_ready{}, rc_switches_ready{}, kill{}, operator_takeover{}, offboard_signal_lost{};
  // Observation diagnostics only. These do not replace the route predicates.
  bool rc_source_seen{}, rc_source_valid{}, rc_source_selected{}, rc_sticks_moving{};
  bool rc_switches_seen{}, rc_switches_fresh{};
  // 保留PX4原枚举：档位1..6，开关1=ON/3=OFF；未知值不能视作安全OFF。
  std::uint8_t rc_data_source{}, rc_mode_slot{}, rc_arm_switch{}, rc_kill_switch{};
  bool rc_axes_safe{}, rc_aux_switches_safe{};
  // 选中的RC油门归一化值；只有来源、时间和轴安全检查均通过才可用于人工授权证据。
  double rc_throttle{NAN};
  // 前者是开关源采样µs；后者是PX4启动相对解锁时刻µs，只能与同一PX4源的历史armed_time比较。
  std::uint64_t rc_switch_sample_stamp{}, armed_time{};
  // PX4最近解锁原因枚举；人工RC路线要求RC开关原因，不能以任何armed=true替代人工解锁。
  std::uint8_t latest_arming_reason{};
  std::uint64_t operator_switch_generation{};
  std::uint64_t local_stamp{};
  bool xy_valid{}, vxy_valid{}, land_selectable{}, auto_land_selected{};
  // PX4 NED 向下位置导数 m/s，和vz修正算法不同；差异仅诊断，不是新的高度来源。
  double z_deriv{NAN}; // PX4 down-position derivative, never a new altitude source
  std::uint64_t vertical_sample_stamp{}; // raw PX4 timestamp_sample, not message timestamp
};
// 把状态与已确认策略组合为权限输入，仍由authority::evaluate独立裁决。
inline authority::Inputs authority_inputs(const Config& c, const Feedback& s) {
  return {s.rc_link_ready, s.rc_switches_ready, c.rc_takeover_confirmed,
          c.autonomous_failsafe_confirmed};
}
// Runtime提供命令ACK与独立PX4状态的摘要；同编号400的Arm/Disarm重复ACK不能直接认领本轮事务。
struct Transaction {
  bool accepted{}, rejected{}, timed_out{};
  // For reused Command 400, ACK cannot identify the operation. Runtime
  // supplies independent, CURRENT safe-ground PX4 evidence after final commit.
  bool ambiguous{}, state_evidenced{};
};
// Arm-only has a disarmed ground anchor and can publish only exact-zero
// thrust. Motor vibration can prevent PX4 at_rest while it is armed. This
// observation is ONLY for that bench route, never for ordinary touchdown.
// 拆桨专用地面速度/高度观察；允许有界估计漂移，不能拿它替代普通飞行触地证明。
inline bool bench_vertical_observation(const Config& c, const Feedback& s, double z0) {
  const double height = z0 - s.z;
  return std::isfinite(z0) && std::isfinite(s.z) && std::isfinite(s.vz) &&
         height <= std::min(c.height_tolerance, c.liftoff_height_m) &&
         height >= -c.bench_ground_down_tolerance_m &&
         std::abs(s.vz) <= c.bench_ground_vz_tolerance_mps;
}
// 按位保留失败项：1未解锁、2非Offboard、4未着地、8高度越限、16垂速越限；用于同帧诊断。
inline std::uint32_t bench_ground_failed_mask(const Config& c, const Feedback& s, double z0) {
  const double height = z0 - s.z;
  return (!s.armed ? 1U : 0U) | (!s.offboard ? 2U : 0U) | (!s.landed ? 4U : 0U) |
         ((!std::isfinite(height) || height > std::min(c.height_tolerance, c.liftoff_height_m) ||
           height < -c.bench_ground_down_tolerance_m)
              ? 8U
              : 0U) |
         ((!std::isfinite(s.vz) || std::abs(s.vz) > c.bench_ground_vz_tolerance_mps) ? 16U : 0U);
}
// 仍要求新鲜有效遥测、Offboard和着地；专用宽容只作用于bench路径。
inline bool bench_ground_observation(const Config& c, const Feedback& s, double z0) {
  return c.bench_arm_only_enabled && !c.takeoff_enabled && s.status_fresh && s.land_fresh &&
         s.local_fresh && s.z_valid && s.vz_valid && s.offboard && s.landed &&
         bench_vertical_observation(c, s, z0);
}
// Post-stop observation only: an old barometric origin is not evidence that
// a DISARMED fixed bench has taken off. Callers must separately establish a
// new Disarmed status and a new at_rest dwell. This cannot authorize a 400
// command, Armed ground admission, TAKEOFF, or ordinary flight touchdown.
// 已独立上锁后的bench观察不用旧气压高度判定离地；调用者另外证明新状态和静止停留。
inline bool bench_disarmed_ground_observation(const Config& c, const Feedback& s) {
  return c.bench_arm_only_enabled && !c.takeoff_enabled && !s.armed && s.status_fresh &&
         s.land_fresh && s.local_fresh && s.z_valid && s.vz_valid && s.landed &&
         std::isfinite(s.z) && std::isfinite(s.vz);
}
// 阶段顺序也用于区间判断，枚举不可随意重排。主线：准备→Offboard→解锁/RC汇合→爬坡→上升→定高→下降→上锁→释放。
// Safety为故障锁存；Rc三阶段仅人工授权；BenchArmedGround只允许零推力；Recovery/Px4LandPending仍需健康受限输出。
enum class Phase {
  Idle,
  SelfCheck,
  Prestream,
  RequestOffboard,
  OffboardReady,
  RequestArm,
  ArmedGround,
  BenchArmedGround,
  ThrustRamp,
  Ascend,
  HeightHold,
  Descent,
  RecoveryDescent,
  Px4LandPending,
  PoweredGround,
  Landed,
  RequestDisarm,
  SafeDisarmed,
  ReleaseOffboard,
  Safety,
  RcPrestream,
  RcWaitArm,
  RcWaitNavigation
};
// 把内部阶段映射为既有日志协议字符串；中文化不翻译协议值，以免破坏解析和下游工具。
inline const char* name(Phase phase) {
  switch (phase) {
  case Phase::Idle:
    return "IDLE";
  case Phase::SelfCheck:
    return "SELF_CHECK";
  case Phase::Prestream:
    return "PRESTREAM";
  case Phase::RequestOffboard:
    return "REQUEST_OFFBOARD";
  case Phase::OffboardReady:
    return "OFFBOARD_READY";
  case Phase::RequestArm:
    return "REQUEST_ARM";
  case Phase::ArmedGround:
    return "ARMED_GROUND";
  case Phase::BenchArmedGround:
    return "BENCH_ARMED_GROUND";
  case Phase::ThrustRamp:
    return "THRUST_RAMP";
  case Phase::Ascend:
    return "ASCEND";
  case Phase::HeightHold:
    return "HEIGHT_HOLD";
  case Phase::Descent:
    return "DESCENT";
  case Phase::PoweredGround:
    return "POWERED_GROUND_UNCONFIRMED";
  case Phase::Landed:
    return "LANDED";
  case Phase::RecoveryDescent:
    return "RECOVERY_DESCENT";
  case Phase::Px4LandPending:
    return "PX4_LAND_PENDING";
  case Phase::RequestDisarm:
    return "REQUEST_DISARM";
  case Phase::SafeDisarmed:
    return "SAFE_DISARMED";
  case Phase::ReleaseOffboard:
    return "RELEASE_OFFBOARD";
  case Phase::Safety:
    return "SAFETY";
  case Phase::RcPrestream:
    return "RC_PRESTREAM";
  case Phase::RcWaitArm:
    return "RC_READY_WAIT_ARM";
  case Phase::RcWaitNavigation:
    return "RC_ARMED_WAIT_NAVIGATION";
  }
  return "UNKNOWN";
}
// Core只生成动作意图：None无命令，Offboard进入外控，Arm/Disarm解上锁，ReleaseOffboard地面释放，Px4Land请求后备降落。
enum class Command { None, Offboard, Arm, Disarm, ReleaseOffboard, Px4Land };
// 故障分类描述地面取消/受控下降/PX4交接或空中未知状态；它不能代替着陆或真实接管证据。
enum class FaultClass { None, GroundAbort, ControlledDescent, Px4Handoff, UnknownAirborne };
// 保留故障分类稳定字符串，未知分类保守显示空中状态未知。
inline const char* name(FaultClass f) {
  switch (f) {
  case FaultClass::None:
    return "NONE";
  case FaultClass::GroundAbort:
    return "GROUND_ABORT";
  case FaultClass::ControlledDescent:
    return "CONTROLLED_DESCENT";
  case FaultClass::Px4Handoff:
    return "PX4_HANDOFF";
  case FaultClass::UnknownAirborne:
    return "UNKNOWN_AIRBORNE";
  }
  return "UNKNOWN_AIRBORNE";
}
// 计算计划而非已执行事实；publish表示拟发布，最终sink还会重采样上下文，command也需Runtime确认实际提交。
struct Output {
  bool visual_motion{};
  // q 为姿态目标 wxyz；thrust为机体FRD归一化向量，上推力写成z负值，准备阶段保持精确零。
  bool publish{};
  Quaternion q{1, 0, 0, 0};
  std::array<double, 3> thrust{};
  Command command{Command::None};
  double height{}, speed{}, target_height{}, velocity_target{};
  bool saturated{}, slew_limited{};
  bool vertical_closed_loop{}, takeoff_thrust_limited{};
  double proportional_thrust{}, integral_thrust{}, raw_thrust{}, thrust_ceiling{};
};
// Captured within the same Core::step as the ARMED_GROUND fault.  This is
// evidence only: it grants no output authority and changes no fault predicate.
// 故障触发的同一控制步保存完整样本，避免之后新回调覆盖真正触发条件。
struct ArmedGroundFaultEvidence {
  bool valid{};
  std::uint32_t failed_mask{}; // 1=armed, 2=offboard, 4=landed, 8=height, 16=vz
  bool armed{}, offboard{}, landed{};
  double z0{}, z{}, vz{}, height{}, up_speed{}, height_limit{}, vz_limit{};
  std::uint64_t status_stamp{}, land_stamp{}, local_stamp{};
  double downward_height_limit{}; // populated for bench faults only
};
// 权限选择/复位失败的地面证据快照；只用于解释拒绝，不授予输出权限。
struct AuthorityGroundEvidence {
  bool valid{};
  std::uint32_t failed_mask{};
  double stationary_seconds{}, vz{};
  std::uint64_t status_stamp{}, land_stamp{}, local_stamp{};
};
} // namespace fc01::attitude
