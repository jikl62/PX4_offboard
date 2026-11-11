#pragma once
// 飞控最终发布策略：对每一条真实 sink 调用独立检查当前健康、状态和权限。
// 这是无 ROS/DDS、无副作用的拒绝优先策略；不夹紧输出、不重置故障、不推进飞行状态机。
// BoundaryContext 是当次发布的不可变快照，生产节点在目标和心跳前分别重取，不能复用过期的通过结论。
//
// 从调用链阅读：Core计算计划 → 本文件返回Decision → publication_boundary实际调用sink。
// “计划正确”和“现在仍准许发”是两个时间点的问题；本文件重复健康/权限门是有意的最后一道检查。
// 这里全部是只读策略；allow只准许调用者继续，绝不等于目标已经发出，更不等于PX4采用。
// 各策略按源码顺序返回首个拒绝，不收集所有错误；交换两个if可能改变可观察的故障原因。
// 正常准备零流、人工RC待证零流、bench零流、飞行非零推力及地面模式释放有不同合同，
// 不要把下面看似重复的条件合并成一个“只要healthy就都能发”的通用开关。
#include "jikl_fc01_system/attitude_altitude_core.hpp"
#include <string>

namespace fc01::attitude {
// 这份上下文由Runtime/节点从当前配置、快照和Core状态组装，策略通过const引用只读借用。
// 它不是新的控制权对象；同一份“已通过”上下文不能在稍后的另一个sink前继续当作最新证据。
struct BoundaryContext {
  std::string profile;
  Phase phase{Phase::Idle};
  Config config;
  Feedback feedback;
  // session/lease是对应路线的有效操作者连接/租约，不等同于“必须打开图形窗口”。
  // control_fresh由整个控制周期的截止预算产生，不能仅凭源消息新鲜就补成true。
  bool shutdown{}, session_connected{}, operator_lease_live{}, control_fresh{};
  bool preparation_lease_valid{true}, flight_started{}, reference_ready{};
  // PX4 NED z 地面零点，单位 m，向下为正；飞行高度由 reference_z-feedback.z 得到，向上为正。
  double reference_z{};
  std::uint8_t reference_z_reset{}, reference_vz_reset{}, reference_quat_reset{};
  authority::State authority;
  bool offboard_observed{};
  // confirmed表示Core已经观察的证明，pending表示未结束的意图，committed表示本地实际出口提交。
  // 三种bool描述不同层次；默认false不能被解释成已证明飞机上锁或接地。
  bool ground_confirmed{}, arm_confirmed{}, ground_release_pending{}, arm_cancel_pending{};
  bool disarm_command_committed{};
  bool bench_arm_only_active{}, bench_disarm_proven{}, arm_command_committed{};
  // 台架释放阶段仍对最新单帧 z/vz 不连续性检查；不能在 flight_started 清除后失去最后一层保护。
  bool bench_sample_guard_ready{};
  double bench_sample_z{}, bench_sample_vz{};
  std::uint64_t bench_arm_status_baseline{}, bench_arm_local_baseline{};
  std::uint64_t bench_disarm_status_floor{}, bench_disarm_local_floor{}, bench_disarm_land_floor{};
  bool px4_land_command_committed{};
  std::uint64_t touchdown_baseline{};
  double yaw_reference{};
  // 仅已有 HeightHold 视觉计划及同一当前输入可置真；绝非新 VelocityCommand 诊断入口的权限。
  bool visual_motion_allowed{};
  Quaternion visual_target_q{1, 0, 0, 0};
  bool rc_manual_arm_proven{}, rc_arm_evidence_pending{}, rc_navigation_zero_allowed{};
  std::uint64_t rc_manual_armed_time{};
};
// 拒绝原因按物理健康、权限、阶段和输出分别表达；序列化字符串保持旧协议不变。
enum class BoundaryReason {
  Allowed,
  WrongProfile,
  Shutdown,
  SessionDisconnected,
  OperatorLease,
  ControlStall,
  OutputDisabled,
  InvalidOutput,
  StageOneNonzero,
  WrongOutputPhase,
  FlightActions,
  ArmEnabled,
  TakeoffEnabled,
  ParametersConfirmed,
  InvalidFlightConfig,
  ArmPhase,
  ArmDisarmed,
  ArmGround,
  FlightPhase,
  ArmedOffboard,
  FlightReference,
  InvalidThrust,
  Conflict,
  TelemetryStale,
  Failsafe,
  CalibrationHil,
  AttitudeInvalid,
  VerticalInvalid,
  NonfiniteTelemetry,
  TimeRegression,
  SourceClock,
  EstimatorReset,
  BatteryPreflight,
  FlightEnvelope,
  AuthorityUnset,
  AuthorityNotLocked,
  AuthorityChanged,
  OperatorTakeover,
  RcLink,
  RcUnconfirmed,
  RcTelemetry,
  AutonomousUnconfirmed,
  InvalidAuthority,
  CommandedAttitude,
  DisarmPhase,
  DisarmGround,
  ReleasePhase,
  DisarmTransaction,
  LandFallback,
  VisualInput,
  VisualAttitude
};
inline const char* name(BoundaryReason reason) {
  switch (reason) {
  case BoundaryReason::VisualInput:
    return "VISUAL_INPUT_EXPIRED_OR_CHANGED";
  case BoundaryReason::VisualAttitude:
    return "VISUAL_ATTITUDE_NOT_MATCHING_PLAN";
  case BoundaryReason::Allowed:
    return "ALLOWED";
  case BoundaryReason::WrongProfile:
    return "WRONG_PROFILE";
  case BoundaryReason::Shutdown:
    return "SHUTDOWN_LATCHED";
  case BoundaryReason::SessionDisconnected:
    return "OPERATOR_SESSION_DISCONNECTED";
  case BoundaryReason::OperatorLease:
    return "LIVE_OPERATOR_LEASE_REQUIRED";
  case BoundaryReason::ControlStall:
    return "CONTROL_LOOP_STALL";
  case BoundaryReason::OutputDisabled:
    return "OUTPUT_DISABLED";
  case BoundaryReason::InvalidOutput:
    return "INVALID_WIRE_OUTPUT";
  case BoundaryReason::StageOneNonzero:
    return "STAGE_ONE_NONZERO_THRUST";
  case BoundaryReason::WrongOutputPhase:
    return "OUTPUT_WRONG_PHASE";
  case BoundaryReason::FlightActions:
    return "FLIGHT_ACTIONS_DISABLED";
  case BoundaryReason::ArmEnabled:
    return "ARM_DISABLED";
  case BoundaryReason::TakeoffEnabled:
    return "TAKEOFF_DISABLED";
  case BoundaryReason::ParametersConfirmed:
    return "FLIGHT_PARAMETERS_NOT_CONFIRMED";
  case BoundaryReason::InvalidFlightConfig:
    return "INVALID_FLIGHT_CONFIGURATION";
  case BoundaryReason::ArmPhase:
    return "ARM_WRONG_PHASE";
  case BoundaryReason::ArmDisarmed:
    return "ARM_REQUIRES_DISARMED";
  case BoundaryReason::ArmGround:
    return "ARM_REQUIRES_OFFBOARD_GROUND_REFERENCE";
  case BoundaryReason::FlightPhase:
    return "NONZERO_WRONG_FLIGHT_PHASE";
  case BoundaryReason::ArmedOffboard:
    return "NONZERO_REQUIRES_ARMED_OFFBOARD";
  case BoundaryReason::FlightReference:
    return "FLIGHT_REFERENCE_REQUIRED";
  case BoundaryReason::InvalidThrust:
    return "INVALID_MULTICOPTER_THRUST";
  case BoundaryReason::Conflict:
    return "PUBLISHER_OR_TELEMETRY_CONFLICT";
  case BoundaryReason::TelemetryStale:
    return "DDS_STATUS_LAND_OR_FAILSAFE_STALE";
  case BoundaryReason::Failsafe:
    return "PX4_FAILSAFE";
  case BoundaryReason::CalibrationHil:
    return "CALIBRATION_OR_HIL_ACTIVE";
  case BoundaryReason::AttitudeInvalid:
    return "ATTITUDE_STALE_OR_INVALID";
  case BoundaryReason::VerticalInvalid:
    return "Z_VZ_STALE_OR_INVALID";
  case BoundaryReason::NonfiniteTelemetry:
    return "NONFINITE_VERTICAL_TELEMETRY";
  case BoundaryReason::TimeRegression:
    return "TELEMETRY_TIME_REGRESSION";
  case BoundaryReason::SourceClock:
    return "DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE";
  case BoundaryReason::EstimatorReset:
    return "ESTIMATOR_RESET_AFTER_CONTROL_SAMPLE";
  case BoundaryReason::BatteryPreflight:
    return "BATTERY_PREFLIGHT_REQUIRED";
  case BoundaryReason::FlightEnvelope:
    return "FLIGHT_ENVELOPE_EXCEEDED";
  case BoundaryReason::AuthorityUnset:
    return "AUTHORITY_MODE_UNSET";
  case BoundaryReason::AuthorityNotLocked:
    return "AUTHORITY_MODE_NOT_LOCKED";
  case BoundaryReason::AuthorityChanged:
    return "AUTHORITY_CHANGED_DURING_ATTEMPT";
  case BoundaryReason::OperatorTakeover:
    return "OPERATOR_TAKEOVER_LATCHED";
  case BoundaryReason::RcLink:
    return "RC_LINK_REQUIRED";
  case BoundaryReason::RcUnconfirmed:
    return "RC_TAKEOVER_CONTRACT_UNCONFIRMED";
  case BoundaryReason::RcTelemetry:
    return "RC_TAKEOVER_TELEMETRY_REQUIRED";
  case BoundaryReason::AutonomousUnconfirmed:
    return "AUTONOMOUS_FAILSAFE_UNCONFIRMED";
  case BoundaryReason::InvalidAuthority:
    return "INVALID_AUTHORITY_MODE";
  case BoundaryReason::CommandedAttitude:
    return "COMMAND_TARGET_NOT_LEVEL_LOCKED_YAW";
  case BoundaryReason::DisarmPhase:
    return "DISARM_WRONG_PHASE";
  case BoundaryReason::DisarmGround:
    return "DISARM_REQUIRES_NEW_FRESH_CONFIRMED_GROUND";
  case BoundaryReason::ReleasePhase:
    return "RELEASE_WRONG_PHASE_OR_GROUND";
  case BoundaryReason::DisarmTransaction:
    return "DISARM_ALREADY_COMMITTED_NO_RETRY";
  case BoundaryReason::LandFallback:
    return "PX4_LAND_FALLBACK_CONTRACT_OR_CURRENT_POSITION_UNVERIFIED";
  }
  return "UNKNOWN_DENIED";
}
// 缺省Decision是拒绝；调用者要同时读取allowed和reason，不能把枚举数字当作bool成功码。
struct Decision {
  bool allowed{};
  BoundaryReason reason{BoundaryReason::InvalidOutput};
};
inline Decision deny(BoundaryReason reason) {
  // 聚合初始化依次填写结构体的allowed、reason字段，不修改任何外部状态。
  return {false, reason};
}
inline Decision allow() {
  return {true, BoundaryReason::Allowed};
}
// 将统一 authority 合同映射为发布原因，不能在最终 sink 使用比 Core 更宽松的授权。
inline Decision boundary_authority(const BoundaryContext& c) {
  switch (authority::evaluate(c.authority, authority_inputs(c.config, c.feedback))) {
  case authority::Reason::Allowed:
    return allow();
  case authority::Reason::Unset:
    return deny(BoundaryReason::AuthorityUnset);
  case authority::Reason::NotLocked:
    return deny(BoundaryReason::AuthorityNotLocked);
  case authority::Reason::Changed:
    return deny(BoundaryReason::AuthorityChanged);
  case authority::Reason::Takeover:
    return deny(BoundaryReason::OperatorTakeover);
  case authority::Reason::RcLink:
    return deny(BoundaryReason::RcLink);
  case authority::Reason::RcUnconfirmed:
    return deny(BoundaryReason::RcUnconfirmed);
  case authority::Reason::RcTelemetry:
    return deny(BoundaryReason::RcTelemetry);
  case authority::Reason::AutonomousUnconfirmed:
    return deny(BoundaryReason::AutonomousUnconfirmed);
  case authority::Reason::Invalid:
    return deny(BoundaryReason::InvalidAuthority);
  }
  return deny(BoundaryReason::InvalidAuthority);
}
// RC 准备三阶段都只允许零推力；stage_one 还包括未解锁的普通 Offboard 准备阶段。
inline bool rc_stage(Phase p) {
  return p == Phase::RcPrestream || p == Phase::RcWaitArm || p == Phase::RcWaitNavigation;
}
inline bool stage_one(Phase p) {
  // 这是“只可零推力准备”的分类，不是Phase数值大小或可自动进入下一阶段的授权。
  return p == Phase::Idle || p == Phase::SelfCheck || p == Phase::Prestream ||
         p == Phase::RequestOffboard || p == Phase::OffboardReady || rc_stage(p);
}
inline bool nonzero(const std::array<double, 3>& thrust) {
  // const引用不复制三轴数组；||只需一轴不等于0就为真。很小的double也不能算“安全零”。
  return thrust[0] != 0 || thrust[1] != 0 || thrust[2] != 0;
}
// 检查动作总门、程序/人工解锁路线、起飞/台架门和参数确认；异常配置统一拒绝。
inline Decision flight_gates(const Config& c) {
  if (!c.flight_actions_enabled)
    return deny(BoundaryReason::FlightActions);
  if (!c.arm_enabled && !c.rc_navigation_enabled)
    return deny(BoundaryReason::ArmEnabled);
  if (!c.takeoff_enabled && !c.bench_arm_only_enabled)
    return deny(BoundaryReason::TakeoffEnabled);
  if (!c.parameters_confirmed)
    return deny(BoundaryReason::ParametersConfirmed);
  // 只将配置值不合法的invalid_argument映射成稳定拒绝原因；内存等其他异常仍向外传播，
  // 不用catch(...)把无法完成校验伪装为一个普通可重试等待。
  try {
    c.validate();
  } catch (const std::invalid_argument&) {
    return deny(BoundaryReason::InvalidFlightConfig);
  }
  return allow();
}
// 先处理关闭/周期时效/接管/时钟，再检查源独占及各反馈；按原次序返回首个原因。
inline Decision boundary_health(const BoundaryContext& c) {
  if (c.profile != "attitude_altitude")
    return deny(BoundaryReason::WrongProfile);
  // 关闭期间只保留已定义的受控下降、地面上锁/释放及取消处置阶段，不重新开放起飞。
  // !(A && B)表示除去一条“当前阶段且已有取消意图”的窄例外；其他恢复阶段也只允许继续处置。
  // 命中例外仍须通过下方所有当前健康/权限/输出门，不是在shutdown时新开一个起飞入口。
  if (c.shutdown && !(c.phase == Phase::RequestArm && c.arm_cancel_pending) &&
      c.phase != Phase::Descent && c.phase != Phase::RecoveryDescent &&
      c.phase != Phase::Px4LandPending && c.phase != Phase::PoweredGround &&
      c.phase != Phase::Landed && c.phase != Phase::RequestDisarm &&
      c.phase != Phase::ReleaseOffboard &&
      !(c.phase == Phase::RequestOffboard && c.ground_release_pending && !c.feedback.armed &&
        c.feedback.landed && c.feedback.at_rest))
    return deny(BoundaryReason::Shutdown);
  if (!c.control_fresh)
    return deny(BoundaryReason::ControlStall);
  // s是c.feedback的只读别名，避免反复写长成员路径；没有新采样，也不会产生第二份反馈。
  const auto& s = c.feedback;
  // 任何已锁存接管、Kill、飞控接管或已观察 Offboard 后的信号丢失都否决输出。
  if (c.authority.takeover_latched || s.kill || s.operator_takeover ||
      ((c.offboard_observed || c.flight_started || c.phase == Phase::OffboardReady) &&
       s.offboard_signal_lost))
    return deny(BoundaryReason::OperatorTakeover);
  if (c.authority.locked && c.authority.mode != c.authority.attempt_mode)
    return deny(BoundaryReason::AuthorityChanged);
  if (s.time_regressed)
    return deny(BoundaryReason::TimeRegression);
  if (!s.source_clock_valid)
    return deny(BoundaryReason::SourceClock);
  const bool bench_release =
      c.bench_arm_only_active && c.arm_confirmed && c.phase == Phase::ReleaseOffboard;
  // 已活动任务或仍解锁时，z/vz/姿态 reset 不得悄悄改变既定参考。
  if (c.reference_ready && (c.flight_started || s.armed || bench_release) &&
      (s.z_reset != c.reference_z_reset || s.vz_reset != c.reference_vz_reset ||
       s.quat_reset != c.reference_quat_reset))
    return deny(BoundaryReason::EstimatorReset);
  if (!s.exclusive)
    return deny(BoundaryReason::Conflict);
  if (!s.status_fresh || !s.land_fresh || !s.failsafe_fresh)
    return deny(BoundaryReason::TelemetryStale);
  if (s.failsafe)
    return deny(BoundaryReason::Failsafe);
  if (s.calibration || s.hil)
    return deny(BoundaryReason::CalibrationHil);
  if (!s.attitude_fresh || !normalized(s.q))
    return deny(BoundaryReason::AttitudeInvalid);
  if (!s.local_fresh || !s.z_valid || !s.vz_valid)
    return deny(BoundaryReason::VerticalInvalid);
  if (!std::isfinite(s.z) || !std::isfinite(s.vz) || !std::isfinite(s.stationary_seconds))
    return deny(BoundaryReason::NonfiniteTelemetry);
  // 最终出口可能看到不同于Core计算时的新上下文；bench即使已清flight_started，仍保留相邻控制观察跳变门。
  // 此门检查相对最近实际观察的z/vz不连续，不把旧armed地面z0的累计漂移误当成跳变。
  if (bench_release && (!c.bench_sample_guard_ready || !std::isfinite(c.bench_sample_z) ||
                        !std::isfinite(c.bench_sample_vz) ||
                        std::abs(s.z - c.bench_sample_z) > c.config.z_jump_limit_m ||
                        std::abs(s.vz - c.bench_sample_vz) > c.config.vz_jump_limit_mps))
    return deny(BoundaryReason::FlightEnvelope);
  return allow();
}
// 程序解锁只允许 RequestArm 的当前未解锁地面状态；RC 导航模式明确禁止程序解锁。
inline Decision arm_publication_policy(const BoundaryContext& c) {
  // 先拒绝整条人工RC路线，再检查其他门；保留该顺序，不能因遥测健康就允许程序代替人工Arm。
  if (c.config.rc_navigation_enabled)
    return deny(BoundaryReason::ArmEnabled);
  const auto healthy = boundary_health(c);
  if (!healthy.allowed)
    return healthy;
  const auto gates = flight_gates(c.config);
  if (!gates.allowed)
    return gates;
  const auto authority = boundary_authority(c);
  if (!authority.allowed)
    return authority;
  if (c.phase != Phase::RequestArm)
    return deny(BoundaryReason::ArmPhase);
  if (!c.session_connected)
    return deny(BoundaryReason::SessionDisconnected);
  if (!c.operator_lease_live)
    return deny(BoundaryReason::OperatorLease);
  const auto& s = c.feedback;
  if (s.armed)
    return deny(BoundaryReason::ArmDisarmed);
  if (!c.flight_started || !c.reference_ready || !s.offboard || !s.landed || !s.at_rest ||
      s.stationary_seconds < c.config.ground_seconds ||
      std::abs(s.vz) > c.config.ground_vz_tolerance)
    return deny(BoundaryReason::ArmGround);
  if (!s.battery_ready || !s.preflight)
    return deny(BoundaryReason::BatteryPreflight);
  return allow();
}
// 程序上锁只在新鲜、比 touchdown 基线更新的地面证据成立时允许；不凭历史 landed 上锁。
inline Decision disarm_publication_policy(const BoundaryContext& c) {
  if (c.phase != Phase::RequestDisarm)
    return deny(BoundaryReason::DisarmPhase);
  // 本地publish返回不证明PX4收到/执行，但也不能据此假定“没发到”而重发。
  // Arm/Disarm共用400，不能拿一条无法归属的错误ACK授权第二次例行Disarm。
  if (c.disarm_command_committed)
    return deny(BoundaryReason::DisarmTransaction);
  const auto health = boundary_health(c);
  if (!health.allowed)
    return health;
  const auto gates = flight_gates(c.config);
  if (!gates.allowed)
    return gates;
  const auto authority = boundary_authority(c);
  if (!authority.allowed)
    return authority;
  const auto& s = c.feedback;
  // 台架例外必须有专门解锁后新状态/本地/land 因果水位，不能泛化为任意地面上锁旁路。
  const bool bench =
      c.config.bench_arm_only_enabled && c.bench_arm_only_active && c.bench_disarm_proven &&
      c.arm_command_committed && s.status_stamp > c.bench_arm_status_baseline &&
      s.local_stamp > c.bench_arm_local_baseline && s.status_stamp >= c.bench_disarm_status_floor &&
      s.local_stamp >= c.bench_disarm_local_floor && s.land_stamp >= c.bench_disarm_land_floor &&
      bench_ground_observation(c.config, s, c.reference_z);
  // 只要配置或本轮进入bench，就必须满足完整bench专用证明；不能失败后回落到更宽松的普通地面分支。
  if ((c.config.bench_arm_only_enabled || c.bench_arm_only_active) && !bench)
    return deny(BoundaryReason::DisarmGround);
  if (!c.flight_started || !c.arm_confirmed || !c.reference_ready || !c.ground_confirmed ||
      !s.armed || !s.offboard || !s.landed || (!s.at_rest && !bench) ||
      s.land_stamp <= c.touchdown_baseline || !std::isfinite(c.reference_z) ||
      (!bench && (std::abs(c.reference_z - s.z) > c.config.height_tolerance ||
                  std::abs(s.vz) > c.config.speed_tolerance)))
    return deny(BoundaryReason::DisarmGround);
  return allow();
}
// PX4 Land 后备需用户确认合同、证据编号、当前水平定位/速度及可选模式，且仅提交一次。
inline Decision land_fallback_publication_policy(const BoundaryContext& c) {
  const auto health = boundary_health(c);
  if (!health.allowed)
    return health;
  const auto gates = flight_gates(c.config);
  if (!gates.allowed)
    return gates;
  const auto auth = boundary_authority(c);
  if (!auth.allowed)
    return auth;
  const auto& s = c.feedback;
  if (c.phase != Phase::Px4LandPending || !c.flight_started || !c.arm_confirmed ||
      c.px4_land_command_committed || !c.config.px4_land_fallback_confirmed ||
      c.config.px4_land_evidence_id.empty() || !s.armed || !s.offboard || !s.local_fresh ||
      !s.xy_valid || !s.vxy_valid || !s.land_selectable || !s.battery_ready)
    return deny(BoundaryReason::LandFallback);
  return allow();
}
// 只识别未开始动力托管且未确认解锁的准备取消；其旧地面零点漂移不应永久卡住退出。
// 这只是分类函数；实际释放仍需新鲜上锁/接地/静止、当前Z/Vz、时间/唯一性等最终门。
// 曾经确认解锁的任务仍保留原参考高度要求，不能借准备取消例外退出飞行。
inline bool unarmed_preparation_release(const BoundaryContext& c) {
  return !c.flight_started && !c.arm_confirmed;
}
// 当最新最终上下文已证明操作者切出Offboard，未起飞的地面准备就取消原176计划。
// 原计划依据的模式事实已过时，不能向操作者的新模式再多发一次切换；这不扩大到飞行阶段。
inline bool external_ground_mode_departure(const BoundaryContext& c) {
  const auto& s = c.feedback;
  return c.profile == "attitude_altitude" && c.phase == Phase::ReleaseOffboard &&
         unarmed_preparation_release(c) && c.offboard_observed && s.status_fresh && s.land_fresh &&
         s.local_fresh && s.failsafe_fresh && s.exclusive && !s.time_regressed && !s.failsafe &&
         !s.kill && !s.armed && !s.offboard && s.landed && s.at_rest && s.z_valid && s.vz_valid &&
         std::isfinite(s.z) && std::isfinite(s.vz);
}
// 释放阶段重验未解锁、接地静止、Offboard 和可切 Stabilized；只有尚未飞行准备才可忽略旧零点漂移。
inline Decision release_publication_policy(const BoundaryContext& c) {
  if (c.phase != Phase::ReleaseOffboard)
    return deny(BoundaryReason::ReleasePhase);
  const auto health = boundary_health(c);
  if (!health.allowed)
    return health;
  const auto& s = c.feedback;
  // 关闭动力门的地面准备也必须能正常释放模式，而这时飞行动力包络可能尚未填写。
  // 三目表达式仅在地面释放选已有合法容差或原固定后备值，不修改配置，也不用于授权起飞。
  const double ground_height_tolerance =
      c.config.height_tolerance > 0 ? c.config.height_tolerance : .30;
  const double ground_speed_tolerance =
      c.config.speed_tolerance > 0 ? c.config.speed_tolerance : c.config.ground_vz_tolerance;
  const bool bench = c.config.bench_arm_only_enabled && c.bench_arm_only_active &&
                     c.arm_confirmed && !c.config.takeoff_enabled && !c.flight_started &&
                     c.ground_confirmed;
  // Core已完成专门的上锁后静止新样本窗口；bench最终释放只复查当前上锁地面合同。
  // 正常飞行结束仍查原参考高度/速度容差，不能把两者都改成忽略旧z0。
  const bool vertical = bench ? bench_disarmed_ground_observation(c.config, s)
                              : ((unarmed_preparation_release(c) ||
                                  std::abs(c.reference_z - s.z) <= ground_height_tolerance) &&
                                 std::abs(s.vz) <= ground_speed_tolerance);
  if (!s.status_fresh || !s.land_fresh || s.armed || !s.landed || !s.at_rest || !s.offboard ||
      !s.stabilized_selectable || !c.reference_ready || !std::isfinite(c.reference_z) || !vertical)
    return deny(BoundaryReason::ReleasePhase);
  return allow();
}
// 检查已通过健康/参考门的目标是否匹配锁定 yaw 或同一视觉计划；c/output 只读，无状态副作用。
// 返回子检查的首个拒绝原因；NED/FRD 姿态沿用 Hamilton wxyz，角度计算及浮点顺序保持原样。
// 仅供总策略在既有位置调用，不能独立作为发布许可；不采新上下文、不改变目标、不重新分配权限。
inline Decision attitude_target_policy(const BoundaryContext& c, const Output& output) {
  // normalized返回optional<Quaternion>：不存在有效结果就拒绝；不会在这里改写原目标output.q。
  const auto commanded = normalized(output.q);
  if (!commanded || !std::isfinite(c.yaw_reference))
    return deny(BoundaryReason::CommandedAttitude);
  // Hamilton[w,x,y,z]：只有yaw旋转时w=cos(yaw/2)、z=sin(yaw/2)，roll/pitch目标为0。
  // yaw_reference是弧度，且须来自本轮已锁参考或已提交视觉yaw，不能直接跟随任意实测机头。
  Quaternion level{std::cos(c.yaw_reference / 2), 0, 0, std::sin(c.yaw_reference / 2)};
  // 视觉运动只在明确开启且已离地 HeightHold 时允许，并与当前视觉计划四元数及倾角相符。
  if (output.visual_motion) {
    if (!c.config.visual.enabled || c.phase != Phase::HeightHold || !c.flight_started ||
        !c.visual_motion_allowed || c.feedback.landed)
      return deny(BoundaryReason::VisualInput);
    const auto expected = normalized(c.visual_target_q);
    if (!expected)
      return deny(BoundaryReason::VisualAttitude);
    // *expected解引用optional，取出已验证的目标副本；之前已检查非空，不是解引用悬空指针。
    level = *expected;
    // 旋转矩阵zz元素等于目标倾角cos值；clamp只防反三角函数舍入越界，不修改发送目标。
    const double cosine = std::clamp(
        1 - 2 * ((*commanded)[1] * (*commanded)[1] + (*commanded)[2] * (*commanded)[2]), -1., 1.);
    if (std::acos(cosine) * 180 / 3.14159265358979323846 >
        std::min(c.config.visual.max_tilt_deg, c.config.max_tilt_deg) + 1e-8)
      return deny(BoundaryReason::VisualAttitude);
  }
  // 四元数点积绝对值给出最短转角；普通定高目标要接近锁定 yaw 的水平姿态，视觉目标采用更严同计划容差。
  // 单位四元数q与-q表示同一姿态，所以先abs(dot)；2*acos才是两姿态的最短转角（弧度）。
  // 普通目标容差2度先换成弧度，视觉目标须与同一计划在1e-5弧度内一致。
  double dot = 0;
  for (unsigned k = 0; k < 4; ++k)
    dot += (*commanded)[k] * level[k];
  const double angle = 2 * std::acos(std::clamp(std::abs(dot), 0., 1.));
  if (!std::isfinite(angle) ||
      angle > (output.visual_motion ? 1e-5 : 2. * 3.14159265358979323846 / 180.))
    return deny(output.visual_motion ? BoundaryReason::VisualAttitude
                                     : BoundaryReason::CommandedAttitude);
  return allow();
}
// 检查 RC 准备阶段的零流、人工 Arm 证据与开关合同；参数均来自本次上下文，无状态副作用。
// thrust_nonzero 必须由原始 double 推力计算；返回原首个拒绝原因，保留 pending_zero 的窄例外。
// 仅在总策略完成目标姿态检查且确认 rc_stage 后调用；不授予解锁或导航权、不跨时钟比较 armed_time。
inline Decision rc_preparation_output_policy(const BoundaryContext& c, bool thrust_nonzero) {
  const auto& s = c.feedback;
  if (!c.config.rc_navigation_enabled || thrust_nonzero)
    return deny(BoundaryReason::StageOneNonzero);
  const auto gates = flight_gates(c.config);
  if (!gates.allowed)
    return gates;
  if (!c.session_connected)
    return deny(BoundaryReason::SessionDisconnected);
  if (!s.source_clock_validated || !RcNavigationStart::rc_live(s) || !s.rc_axes_safe ||
      s.rc_sticks_moving || !s.rc_aux_switches_safe)
    return deny(BoundaryReason::RcTelemetry);
  if (!s.battery_ready || !s.preflight)
    return deny(BoundaryReason::BatteryPreflight);
  if (!s.landed || std::abs(c.reference_z - s.z) > c.config.height_tolerance ||
      std::abs(s.vz) > c.config.ground_vz_tolerance)
    return deny(BoundaryReason::ArmGround);
  if (!s.stabilized_selected && !s.offboard)
    return deny(BoundaryReason::ArmedOffboard);
  // 等待新的人工解锁证据期间可维持原零心跳；必须仍在非 Offboard、非导航档，不能借此直接起飞。
  const bool pending_zero = c.rc_arm_evidence_pending && !c.rc_manual_arm_proven &&
                            !c.flight_started &&
                            (c.phase == Phase::RcWaitArm || c.phase == Phase::RcWaitNavigation) &&
                            !s.offboard && s.rc_mode_slot != c.config.rc_navigation_mode_slot;
  // navigation_zero是人工解锁已证明但导航开关/模式仍异步配对的窄零流；pending_zero则还没证明人工Arm。
  // 两者均不是flight_started，也不能给非零推力授权；RC原准入机另管固定配对期限。
  const bool navigation_zero = c.rc_navigation_zero_allowed && c.rc_manual_arm_proven &&
                               !c.flight_started && c.phase == Phase::RcWaitNavigation;
  if (c.phase == Phase::RcWaitNavigation && c.rc_manual_arm_proven && !navigation_zero)
    return deny(BoundaryReason::RcTelemetry);
  if (!c.rc_manual_arm_proven && (s.armed || s.rc_arm_switch == 1) && !pending_zero)
    return deny(BoundaryReason::ArmDisarmed);
  // armed_time是PX4本次解锁事件身份，latest_arming_reason=2是RC开关原因；
  // 与manual证明严格匹配，不能用上一轮已解锁事实授权这一轮。
  if (s.armed && !pending_zero &&
      (!c.rc_manual_arm_proven || s.rc_arm_switch != 1 || s.armed_time != c.rc_manual_armed_time ||
       s.latest_arming_reason != 2))
    return deny(BoundaryReason::ArmDisarmed);
  if (s.offboard &&
      (!s.armed || (s.rc_mode_slot != c.config.rc_navigation_mode_slot && !navigation_zero) ||
       s.offboard_signal_lost))
    return deny(BoundaryReason::ArmedOffboard);
  if (c.phase != Phase::RcWaitNavigation && !pending_zero &&
      (s.rc_arm_switch != 3 || s.rc_mode_slot == c.config.rc_navigation_mode_slot || s.offboard))
    return deny(BoundaryReason::OperatorTakeover);
  return boundary_authority(c);
}
// 姿态目标必须先通过原始 double 推力检查，避免转 float 下溢把微小非零错误伪装成零推力。
inline Decision attitude_publication_policy(const BoundaryContext& c, const Output& output) {
  if (!output.publish)
    return deny(BoundaryReason::OutputDisabled);
  for (double x : output.thrust)
    if (!std::isfinite(x) || x < -1 || x > 1)
      return deny(BoundaryReason::InvalidOutput);
  // 例如微小double非零在转float时可能变0；必须在转换之前拒绝准备阶段非零。
  // 这里拒绝错误计划，不夹紧、不替它补成零，也不修改Core的原输出。
  const bool thrust_nonzero = nonzero(output.thrust);
  if ((stage_one(c.phase) || c.config.bench_arm_only_enabled) && thrust_nonzero)
    return deny(BoundaryReason::StageOneNonzero);
  if (!normalized(output.q))
    return deny(BoundaryReason::InvalidOutput);
  const auto healthy = boundary_health(c);
  if (!healthy.allowed)
    return healthy;
  // 状态白名单只是必要条件；后续仍分别检查参考、目标姿态、RC 条件和非零推力阶段。
  switch (c.phase) {
  case Phase::Prestream:
  case Phase::RequestOffboard:
  case Phase::OffboardReady:
  case Phase::RequestArm:
  case Phase::ArmedGround:
  case Phase::BenchArmedGround:
  case Phase::ThrustRamp:
  case Phase::Ascend:
  case Phase::HeightHold:
  case Phase::Descent:
  case Phase::RecoveryDescent:
  case Phase::Px4LandPending:
  case Phase::PoweredGround:
  case Phase::Landed:
  case Phase::RequestDisarm:
  case Phase::ReleaseOffboard:
    break;
  case Phase::RcPrestream:
  case Phase::RcWaitArm:
  case Phase::RcWaitNavigation:
    break;
  default:
    return deny(BoundaryReason::WrongOutputPhase);
  }
  if (!c.reference_ready)
    return deny(BoundaryReason::FlightReference);
  // 零推力、普通准备和模式释放也要检查目标姿态/yaw；不能靠“推力反正是0”跳过目标合同。
  // 推力原始double非零检查已在健康门之前，首拒绝优先级保持不动。
  const auto target = attitude_target_policy(c, output);
  if (!target.allowed)
    return target;
  // RC 准备仍要来源同步、中杆低油门、辅助开关安全、飞行前检查；允许的待证零推力不等于解锁确认。
  if (rc_stage(c.phase))
    return rc_preparation_output_policy(c, thrust_nonzero);
  // 释放只允许零推力并委托专门地面策略；不是绕开飞行动作门的动力入口。
  if (c.phase == Phase::ReleaseOffboard) {
    if (thrust_nonzero)
      return deny(BoundaryReason::StageOneNonzero);
    // 未解锁地面释放用零流维持到PX4模式转移；它还服务于动力门全关的准备取消。
    // 因此这里不要求飞行动作门开启，但仍要求专门地面释放策略全部通过。
    return release_publication_policy(c);
  }
  // 未解锁准备使用准备租约，动作门关闭仍可预发送零心跳；任何已解锁状态都被拒绝。
  if (stage_one(c.phase)) {
    // 关闭UI时，尚未结束的176仍可能生效；这条窄例外只维持未解锁地面的零准备流等原事务结果。
    // 它只豁免当前session_connected，不省略preparation_lease和其他上方门。
    const bool cancelled_ground_mode = c.phase == Phase::RequestOffboard &&
                                       c.ground_release_pending && unarmed_preparation_release(c) &&
                                       !c.feedback.armed && c.feedback.landed && c.feedback.at_rest;
    if (!c.session_connected && !cancelled_ground_mode)
      return deny(BoundaryReason::SessionDisconnected);
    if (!c.preparation_lease_valid)
      return deny(BoundaryReason::OperatorLease);
    if (c.feedback.armed)
      return deny(BoundaryReason::ArmDisarmed);
    if (c.phase == Phase::OffboardReady && !c.feedback.offboard)
      return deny(BoundaryReason::ArmedOffboard);
    return allow(); // 动作门关闭仍可做零推力准备，不意味着可以Arm或起飞。
  }
  const auto gates = flight_gates(c.config);
  if (!gates.allowed)
    return gates;
  const auto authority = boundary_authority(c);
  if (!authority.allowed)
    return authority;
  if (c.config.rc_navigation_enabled &&
      (c.phase == Phase::ArmedGround || c.phase == Phase::ThrustRamp || c.phase == Phase::Ascend ||
       c.phase == Phase::HeightHold)) {
    const auto& s = c.feedback;
    if (!c.rc_manual_arm_proven || !RcNavigationStart::rc_live(s) || !s.rc_axes_safe ||
        s.rc_sticks_moving || !s.rc_aux_switches_safe || s.rc_arm_switch != 1 ||
        s.rc_mode_slot != c.config.rc_navigation_mode_slot ||
        s.armed_time != c.rc_manual_armed_time || s.latest_arming_reason != 2)
      return deny(BoundaryReason::RcTelemetry);
  }
  if (!c.flight_started || !c.feedback.offboard)
    return deny(BoundaryReason::FlightReference);
  // RequestArm/ArmedGround/RequestDisarm 的零目标可在前述健康和授权完整时发出；非零目标继续检查。
  if (!thrust_nonzero)
    return allow(); // 此处仅省略后面的非零动力检查，上方健康/目标/权限/托管门均已执行。
  if (c.phase != Phase::ThrustRamp && c.phase != Phase::Ascend && c.phase != Phase::HeightHold &&
      c.phase != Phase::Descent && c.phase != Phase::RecoveryDescent &&
      c.phase != Phase::Px4LandPending && c.phase != Phase::PoweredGround &&
      c.phase != Phase::Landed)
    return deny(BoundaryReason::FlightPhase);
  if (!c.feedback.armed)
    return deny(BoundaryReason::ArmedOffboard);
  // 多旋翼升力只使用 FRD 负 z，水平机体推力必须为 0，幅值不得超过现有垂向上限。
  if (output.thrust[0] != 0 || output.thrust[1] != 0 || output.thrust[2] > 0 ||
      -output.thrust[2] > c.config.vertical.maximum)
    return deny(BoundaryReason::InvalidThrust);
  if (!c.feedback.battery_ready)
    return deny(BoundaryReason::BatteryPreflight);
  // 这些已进入下降/地面处置的阶段不因UI会话结束而遗弃armed+Offboard托管，
  // 但健康、控制权、当前armed、推力范围/方向与电池门仍在上方全部执行。
  if (c.phase != Phase::Descent && c.phase != Phase::RecoveryDescent &&
      c.phase != Phase::Px4LandPending && c.phase != Phase::PoweredGround &&
      c.phase != Phase::Landed) {
    if (!c.session_connected)
      return deny(BoundaryReason::SessionDisconnected);
    if (!c.operator_lease_live)
      return deny(BoundaryReason::OperatorLease);
  }
  // 相对高度包络向上为正；允许原地面下界余量，但不改变最大飞行高度。
  if (!std::isfinite(c.reference_z) ||
      c.reference_z - c.feedback.z > c.config.vertical.max_height ||
      c.reference_z - c.feedback.z < -c.config.height_lower_margin_m)
    return deny(BoundaryReason::FlightEnvelope);
  return allow();
}
// 仅把动作门序列化到诊断，不依据输出 JSON 的显示状态改变任何权限。
inline std::string flight_gates_json(const Config& c) {
  // lambda把bool变成JSON字面量true/false；只读显示不是修改动作门的命令入口。
  auto boolean = [](bool b) {
    return b ? "true" : "false";
  };
  return std::string("\"flight_actions_enabled\":") + boolean(c.flight_actions_enabled) +
         ",\"arm_enabled\":" + boolean(c.arm_enabled) +
         ",\"takeoff_enabled\":" + boolean(c.takeoff_enabled) +
         ",\"flight_parameters_confirmed\":" + boolean(c.parameters_confirmed) +
         ",\"bench_arm_only_enabled\":" + boolean(c.bench_arm_only_enabled);
}
} // namespace fc01::attitude
