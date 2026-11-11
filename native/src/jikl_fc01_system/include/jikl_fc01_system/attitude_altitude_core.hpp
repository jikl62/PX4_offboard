#pragma once
// 原生姿态高度状态机：把当前PX4/RC事实、已校验配置和Runtime事务结果变成姿态/推力计划。
// 阅读顺序：prepare/rc_startup建立参考→step检查权限健康→各Phase计算→apply转FRD推力→外层最终发布。
// 本类是纯计算单线程状态，不拥有ROS/串口；计划不等于已发出，离地/触地/上锁必须靠独立新反馈证明。
//
// 先分清本文件中的四种“结果”：
//   Feedback 是外层组装的事实快照；新鲜度/有效位决定其中的数值能否使用。
//   Output 是本次计算的发布计划；publish=true 也只表示请求外层再检查、再发布。
//   note_*_publication 收到的是本地发布出口返回的提交结果；这仍不能证明 PX4 采用了目标。
//   新 status/land/local 等反馈才提供动作后的状态证据；ACK 只描述命令事务，不能代替物理反馈。
//
// 普通起飞的两条路线在 ArmedGround 汇合，前面的授权过程不同：
//   控制台路线：prepare → SelfCheck → Prestream → RequestOffboard → OffboardReady，
//               再由 takeoff 请求程序解锁。arm_only是另行授权的拆桨bench，不进入正常爬升。
//               当前人工 RC 配置会拒绝这两个程序解锁入口。
//   人工 RC 路线：rc_startup 交给 RcNavigationStart 证明地面、零预发送、人工解锁和导航选择；
//                它不发送程序 Arm/Offboard 命令，证据齐全后才加入本 Core 的 ArmedGround。
//
// 变量名末尾的下划线表示 Core 持有的成员，能跨 step 保存；局部 out 每次重新创建。
// 常见缩写：c_ 是已校验配置，s 是本帧反馈，q 是四元数，z0_ 是地面 NED 坐标。
// PX4 世界系 NED 的 z/vz 向下为正；本控制器使用 height=z0-z、speed=-vz，向上为正。
// FRD 机体系 z 同样向下；apply 将非负推力幅值写成 {0,0,-thrust}，量纲为归一化比例。
// 因而 -0.5 不是 -0.5 N，也不能仅凭该比例断言真实升力或已离地。
#include "jikl_fc01_system/attitude_types.hpp"
#include "jikl_fc01_system/takeoff_evidence.hpp"
#include "jikl_fc01_system/rc_navigation.hpp"
#include <memory>

namespace fc01::attitude {
// 承担任务阶段、参考、权限与恢复托管；不得在注释或结构整理中重排安全门、浮点运算和状态修改。
class Core {
public:
  // 配置校验后才构造垂向控制器；只有动力门开启才存在vertical_，构造本身不授予动力。
  explicit Core(Config config = {})
      : c_(config), visual_(config.visual), feedback_guard_(config.feedback_guard),
        recovery_feedback_guard_(config.feedback_guard) {
    c_.validate();
    // vertical 是局部配置副本，只调整此控制器使用的地面下界；不改调用者传入的 YAML。
    // unique_ptr 表示垂向控制器由 Core 独占；对象随 Core 销毁，不需要在每条早退手工 delete。
    // 动作门关闭时指针为空；后面的动力阶段必须由原准入门保证，不能凭有一个 step 调用绕过。
    if (c_.flight_actions_enabled) {
      auto vertical = c_.vertical;
      vertical.ground_lower_margin = c_.height_lower_margin_m;
      vertical_ = std::make_unique<VerticalController>(vertical);
    }
  }
  // 只读诊断入口；phase/reason/event分别表示当前阶段、等待/故障原因、最近参考等事件。
  Phase phase() const {
    return phase_;
  }
  // const std::string& 是只读引用：避免复制字符串，但引用只在当前 Core 仍存活时有效。
  // 之后 step/fault 可能改写同一成员；需要保存历史时，调用者应自行复制，而不是长期保存引用。
  const std::string& reason() const {
    return reason_;
  }
  const std::string& event() const {
    return event_;
  }
  const VerticalFeedbackEvidence& vertical_feedback_evidence() const {
    return feedback_guard_.evidence();
  }
  bool active() const {
    // 枚举的区间比较依赖 attitude_types.hpp 中 Phase 的既有排列；不能按名称随意重排枚举。
    // active 指“仍有控制流程需推进”，不表示已解锁、已离地或已允许非零输出。
    return (phase_ >= Phase::SelfCheck && phase_ <= Phase::ReleaseOffboard) ||
           phase_ == Phase::RcPrestream || phase_ == Phase::RcWaitArm ||
           phase_ == Phase::RcWaitNavigation;
  }
  // 只由实际sink回报成功提交；连续预发送按提交计数，已提交视觉偏航才可成为下轮水平参考。
  void note_rc_publication(bool allowed, double now) {
    if (c_.rc_navigation_enabled)
      rc_start_.note_commit(allowed, now);
    if (allowed && navigation_allowed() && visual_result_.active && now >= visual_plan_at_ &&
        now - visual_plan_at_ <= c_.stall_seconds) {
      committed_yaw_ = visual_result_.yaw;
      committed_yaw_set_ = true;
    }
  }
  const vision::Result& visual_result() const {
    return visual_result_;
  }
  // 视觉计划必须仍在导航授权阶段、未过控制时限，且实际状态/目标合同此刻仍有效。
  bool visual_current(const Feedback& s, double now) const {
    return c_.visual.enabled && navigation_allowed() && visual_result_.active &&
           now >= visual_plan_at_ && now - visual_plan_at_ <= c_.stall_seconds &&
           visual_.current(s.visual_input, now);
  }
  // 视觉撤销立即停止运动并请求既有下降路径；失败仍锁存故障，不另造绕过健康门的降落输出。
  void visual_land(const Feedback& s, double now, const std::string& reason) {
    visual_result_.active = false;
    visual_result_.request_land = true;
    visual_result_.reason = reason;
    visual_.stop();
    const auto failure = descend(s, now);
    if (!failure.empty())
      fault(failure);
    else
      reason_ = "VISUAL_RECOVERY_" + reason;
  }
  // 下列RC查询暴露独立地面锚点和人工解锁事实；它们是证据，不是自动Arm请求。
  unsigned rc_prestream_commits() const {
    return rc_start_.commits();
  }
  bool rc_ground_reference_qualified() const {
    return rc_start_.ground_reference_qualified();
  }
  unsigned rc_ground_reference_samples() const {
    return rc_start_.ground_reference_samples();
  }
  double rc_ground_reference_seconds() const {
    return rc_start_.ground_reference_seconds();
  }
  bool rc_manual_arm_proven() const {
    return rc_start_.manual_arm_proven();
  }
  std::uint64_t rc_manual_armed_time() const {
    return rc_start_.manual_armed_time();
  }
  // 健康门仍需成立；只有尚在固定配对窗口时可以桥接精确零输出。
  bool rc_arm_evidence_pending(const Feedback& s, double now) const {
    return c_.rc_navigation_enabled && health(s).empty() &&
           rc_start_.pending_zero_allowed(c_, s, now);
  }
  // 人工解锁已证明但模式/档位异步到达时维持零流；不得借此产生非零推力。
  bool rc_navigation_zero_allowed(const Feedback& s, double now) const {
    return c_.rc_navigation_enabled && health(s).empty() &&
           rc_start_.navigation_zero_allowed(c_, s, now);
  }
  // 重新准备分三类：正常任务完成、尚未加入的准备重试、故障后的独立地面恢复；任一都要当前安全RC和新水位。
  bool rc_restart_allowed(const Feedback& s) const {
    const bool completed = rc_joined_ && (phase_ == Phase::SafeDisarmed || phase_ == Phase::Idle);
    const bool preparation_retry = phase_ == Phase::Safety && !rc_joined_ &&
                                   rc_preparation_retry_reason(reason_) &&
                                   s.rc_switch_sample_stamp > fault_rc_switch_baseline_;
    // 故障可恢复白名单、完整新地面证据、无本周期不安全事件及四路新样本必须同时成立。
    const bool recovered_fault =
        phase_ == Phase::Safety && rc_fault_recoverable_ && rc_recovery_ready_ &&
        !recovery_event_blocked_ && rc_recovery_sample_safe(s) &&
        s.status_stamp > fault_status_baseline_ && s.land_stamp > fault_land_baseline_ &&
        s.local_stamp > fault_local_baseline_ &&
        s.rc_switch_sample_stamp > fault_rc_switch_baseline_;
    // 最后仍要求未解锁、非Offboard、Stabilized、着地静止、电池预检和解锁/导航开关OFF，不能用“之前安全”替代。
    return c_.rc_navigation_enabled && !recovery_event_blocked_ &&
           (!restart_required_ || recovered_fault) &&
           (completed || preparation_retry || recovered_fault) && health(s).empty() &&
           s.source_clock_validated && s.source_host_stamp > 0 && !s.armed && !s.offboard &&
           !s.operator_takeover && s.landed && s.at_rest && s.stabilized_selected &&
           s.battery_ready && s.preflight && s.rc_switch_sample_stamp > task_rc_switch_baseline_ &&
           s.stationary_seconds >= c_.ground_seconds && std::abs(s.vz) <= c_.ground_vz_tolerance &&
           RcNavigationStart::rc_live(s) && s.rc_axes_safe && s.rc_aux_switches_safe &&
           !s.rc_sticks_moving && s.rc_arm_switch == 3 && s.rc_mode_slot > 0 &&
           s.rc_mode_slot <= 6 && s.rc_mode_slot != c_.rc_navigation_mode_slot;
  }
  // 延期仅能发生在两个发布出口均未调用之前，而且只能是已识别的样本年龄跨界，不能是时钟不连续。
  // 它不授予任何输出权限，也不清除Safety/故障锁存。
  // 只在尚无Arm意图、无提交/接管/故障、未解锁的RC准备阶段，允许源样本年龄过期后退回Idle重建。
  bool defer_rc_preparation(const Feedback& s, double now, const std::string& reason) {
    if (!c_.rc_navigation_enabled || reason.empty() ||
        (phase_ != Phase::RcPrestream && phase_ != Phase::RcWaitArm) ||
        (rc_start_.phase() != RcStartPhase::Prestream &&
         rc_start_.phase() != RcStartPhase::WaitArm) ||
        !rc_start_.no_arm_intent_observed() || !rc_start_.anchor_valid() || !z_set_ ||
        !std::isfinite(now) || now < 0 || now < rc_observed_at_ || rc_joined_ || flight_started_ ||
        restart_required_ || fault_class_ != FaultClass::None || authority_.takeover_latched ||
        arm_confirmed_ || arm_cancel_pending_ || arm_command_committed_ || disarm_in_flight_ ||
        disarm_command_committed_ || disarm_outcome_uncertain_ || offboard_command_committed_ ||
        release_command_committed_ || offboard_observed_ || px4_land_committed_ ||
        operator_release_requested_)
      return false;
    // 外层另外证明“最近收到”和“原始样本确实推进”；此处只暂缓源年龄超限这一项。
    // 其余反馈、操作者意图和参考条件仍逐项成立，不能扩大为忽略健康门。
    if (s.time_regressed || !s.source_clock_validated || !s.source_host_stamp || !s.exclusive ||
        !s.status_fresh || !s.land_fresh || !s.failsafe_fresh || s.failsafe || s.calibration ||
        s.hil || s.armed || s.offboard || !s.landed || !s.at_rest || !s.stabilized_selected ||
        s.kill || s.operator_takeover || !RcNavigationStart::rc_live(s) || !s.rc_axes_safe ||
        !s.rc_aux_switches_safe || s.rc_sticks_moving || s.rc_arm_switch != 3 ||
        s.rc_mode_slot == 0 || s.rc_mode_slot > 6 || s.rc_mode_slot == c_.rc_navigation_mode_slot ||
        !s.battery_ready || !s.preflight || !s.z_valid || !s.vz_valid || !std::isfinite(s.z) ||
        !std::isfinite(s.vz) || !std::isfinite(s.stationary_seconds) ||
        s.stationary_seconds < c_.ground_seconds || std::abs(s.vz) > c_.ground_vz_tolerance ||
        std::abs(z0_ - s.z) > c_.height_tolerance || s.z_reset != z_reset_ ||
        s.vz_reset != vz_reset_ || s.quat_reset != reference_.reset_counter())
      return false;
    const auto q = normalized(s.q);
    if (!q || std::acos(std::clamp(1 - 2 * ((*q)[1] * (*q)[1] + (*q)[2] * (*q)[2]), -1., 1.)) *
                      180 / 3.141592653589793 >
                  c_.max_tilt_deg)
      return false;
    // 此窄路径只重建本任务RC准备与参考，不清除Runtime的命令历史/全局因果水位。
    rc_start_ = RcNavigationStart{};
    rc_operator_live_ = false;
    rc_observed_at_ = now;
    reference_.clear();
    z_set_ = false;
    authority_ = {};
    last_ = now;
    enter(Phase::Idle, now);
    reason_ = event_ = reason;
    return true;
  }
  // flight_started表示已承担任务托管，不代表真实已离地；真正离地由liftoff证据记录。
  bool flight_started() const {
    return flight_started_;
  }
  // 程序路线从Arm请求开始承担任务托管；RC路线从准入汇合开始承担托管。
  // liftoff_confirmed保存本次任务曾取得的离地证据，降落期间仍保留；
  // 它既不表示当前一定在空中，也不表示目标高度已稳定。target_height_settled另查阶段。
  bool liftoff_confirmed() const {
    return liftoff_evidenced_;
  }
  bool target_height_settled() const {
    return liftoff_evidenced_ && phase_ == Phase::HeightHold;
  }
  // 必须已证实离地且目标高度稳定、仍处任务托管、非bench、无交接请求，才可启用水平导航。
  bool navigation_allowed() const {
    return target_height_settled() && flight_started_ && !bench_arm_only_active_ &&
           !operator_release_requested_;
  }
  const LiftoffEvidence& liftoff_evidence() const {
    return takeoff_evidence_.evidence();
  }
  // 对外展示区分解锁、未确认离地、爬升、目标稳定和降落，避免把推力爬坡误报为成功飞行。
  const char* takeoff_stage() const {
    if (phase_ == Phase::Safety)
      return "FAULT";
    if (!flight_started_)
      return arm_confirmed_ ? "COMPLETE" : "NOT_STARTED";
    switch (phase_) {
    case Phase::RequestArm:
      return "ARMING";
    case Phase::ArmedGround:
    case Phase::BenchArmedGround:
      return "ARMED_GROUND";
    case Phase::ThrustRamp:
      return "LIFTOFF_UNCONFIRMED";
    case Phase::Ascend:
      return liftoff_evidenced_ ? "CLIMBING" : "LIFTOFF_UNCONFIRMED";
    case Phase::HeightHold:
      return navigation_allowed() ? "TARGET_SETTLED" : "LIFTOFF_UNCONFIRMED";
    default:
      return "LANDING";
    }
  }
  // 以下bench证据只能解释拆桨路线，不适用于普通起飞/触地授权。
  bool bench_arm_only_active() const {
    return bench_arm_only_active_;
  }
  bool bench_disarm_proven() const {
    return bench_disarm_proven_;
  }
  bool bench_sample_guard_ready() const {
    return bench_arm_only_active_ && last_sample_set_;
  }
  double bench_sample_z() const {
    return last_sample_z_;
  }
  double bench_sample_vz() const {
    return last_sample_vz_;
  }
  std::uint64_t bench_arm_status_baseline() const {
    return bench_arm_status_baseline_;
  }
  std::uint64_t bench_arm_local_baseline() const {
    return bench_arm_local_baseline_;
  }
  std::uint64_t bench_disarm_status_floor() const {
    return bench_disarm_status_floor_;
  }
  std::uint64_t bench_disarm_local_floor() const {
    return bench_disarm_local_floor_;
  }
  std::uint64_t bench_disarm_land_floor() const {
    return bench_disarm_land_floor_;
  }
  double bench_ground_dwell_seconds() const {
    return land_dwell_first_time_ < 0
               ? 0
               : std::max(0., land_dwell_last_time_ - land_dwell_first_time_);
  }
  std::uint64_t bench_ground_first_land_stamp() const {
    return land_dwell_first_stamp_;
  }
  std::uint64_t bench_ground_last_land_stamp() const {
    return land_dwell_last_stamp_;
  }
  bool bench_wait_overdue(double now) const {
    return bench_arm_only_active_ && flight_started_ && std::isfinite(now) &&
           now - flight_since_ > std::min(8.0, c_.max_flight_seconds);
  }
  // 仅bench或三动作门全闭的地面诊断可采用已提交后的独立新模式状态；普通动力飞行不走此宽容。
  bool ground_mode_state_only_allowed() const {
    return c_.bench_arm_only_enabled ||
           (!c_.flight_actions_enabled && !c_.arm_enabled && !c_.takeoff_enabled);
  }
  bool disarm_requested() const {
    return phase_ == Phase::RequestDisarm || phase_ == Phase::SafeDisarmed;
  }
  // 只读地面z0和姿态/垂向reset基准，供最终发布边界重核；坐标z0沿用PX4 NED向下m。
  bool reference_ready() const {
    return z_set_;
  }
  double reference_z() const {
    return z0_;
  }
  std::uint8_t reference_z_reset() const {
    return z_reset_;
  }
  std::uint8_t reference_vz_reset() const {
    return vz_reset_;
  }
  std::uint8_t reference_quat_reset() const {
    return reference_.reset_counter();
  }
  // 这是只读接收调度提示：true要求尽快结束额外补读，让step先看到当前候选样本。
  // 它不推进guard/参考，不自行故障，也不授予输出权限。
  bool local_requires_control_step(std::uint64_t stamp, double z, double vz, double z_deriv) const {
    // 非有限值、累计跳变和地面离开必须保留，后续正常回调不能把中间异常覆盖掉。
    if (!std::isfinite(z) || !std::isfinite(vz) || !std::isfinite(z_deriv))
      return true;
    // 地面准入使用本轮证明窗，不能沿用结束任务的原点或上个控制周期的准备快照。
    // 即使后续回调又回到锚点，也要留下期间曾离开锚点的证据。
    if (phase_ == Phase::Safety && rc_recovery_since_ >= 0 &&
        std::abs(z - rc_recovery_z_) > c_.height_tolerance)
      return true;
    if (c_.rc_navigation_enabled && !rc_joined_ && rc_start_.phase() == RcStartPhase::Prestream &&
        rc_start_.ground_reference_height_changed(z, c_.height_tolerance))
      return true;
    // 还要与上次控制观察比较：0→0.6→1.2→0.6的相邻差都小于1，
    // 但中途相对上次控制值已超过1；仅比较相邻回调会漏掉这种累计变化。
    if ((flight_started_ || (bench_arm_only_active_ && bench_disarmed_seen_)) && last_sample_set_ &&
        (std::abs(z - last_sample_z_) > c_.z_jump_limit_m ||
         std::abs(vz - last_sample_vz_) > c_.vz_jump_limit_mps))
      return true;
    const double height = z0_ - z;
    const bool stopped_bench = bench_arm_only_active_ && bench_disarmed_seen_;
    if (flight_started_ && !stopped_bench &&
        ((z_set_ && (height > c_.vertical.max_height ||
                     height < -(bench_arm_only_active_ ? c_.bench_ground_down_tolerance_m
                                                       : c_.height_lower_margin_m))) ||
         std::abs(vz) > c_.vertical.max_climb + c_.vertical.max_descent + 2 * c_.vz_jump_limit_mps))
      return true;
    // 复制guard的上次控制状态进行预览；vz与z_deriv相等也可能违反位置积分窗口。
    // 预览副本可变，真实guard不能因仅做接收调度判断而提前推进。
    if (c_.feedback_guard.enabled && flight_started_ && !bench_arm_only_active_ &&
        phase_ >= Phase::ThrustRamp && phase_ <= Phase::RequestDisarm) {
      auto preview = feedback_guard_;
      const auto& evidence = preview.observe(stamp, z, vz, z_deriv, c_.local_timeout);
      if (evidence.suspect || !evidence.healthy)
        return true;
    }
    // RC解锁/导航配对绑定此前捕获的未解锁锚点；
    // 不能让后来的回调覆盖配对窗口中曾发生的位移。
    if (c_.rc_navigation_enabled && !rc_joined_ && rc_start_.anchor_valid() &&
        (rc_start_.phase() == RcStartPhase::WaitArm ||
         rc_start_.phase() == RcStartPhase::WaitNavigation) &&
        (std::abs(rc_start_.z0() - z) > c_.height_tolerance ||
         std::abs(vz) > c_.ground_vz_tolerance))
      return true;
    if (phase_ == Phase::SelfCheck && prep_since_ >= 0 &&
        std::abs(z - prep_z_) > c_.height_tolerance)
      return true;
    if (!z_set_)
      return false;
    // 普通地面采用向上相对高度和|NED vz|；bench单独限定更宽的向下漂移，不能互换。
    const auto normal_ground = [&](double speed) {
      return std::abs(height) <= c_.height_tolerance && std::abs(vz) <= speed;
    };
    const auto bench_ground = [&] {
      return height <= std::min(c_.height_tolerance, c_.liftoff_height_m) &&
             height >= -c_.bench_ground_down_tolerance_m &&
             std::abs(vz) <= c_.bench_ground_vz_tolerance_mps;
    };
    if (phase_ == Phase::ArmedGround && !normal_ground(c_.ground_vz_tolerance))
      return true;
    if (phase_ == Phase::BenchArmedGround && !bench_ground())
      return true;
    if (phase_ == Phase::PoweredGround || phase_ == Phase::RequestDisarm ||
        phase_ == Phase::SafeDisarmed) {
      if (bench_arm_only_active_) {
        // 已确认上锁的bench允许有限估计漂移；但真正step仍需新的上锁/静止证据，
        // 这里仅决定要不要尽早计算，不能替step补出地面授权。
        if (!bench_disarmed_seen_ && !external_bench_disarm_pending_ && !bench_ground())
          return true;
      } else if (!normal_ground(c_.speed_tolerance))
        return true;
    }
    if (phase_ == Phase::Landed && !normal_ground(c_.speed_tolerance))
      return true;
    // 一旦正在累计触地/稳定窗口，瞬时不满足也必须交给step清窗；
    // 普通高空下降没有这项清窗要求，可以继续补读连续反馈。
    if ((phase_ == Phase::Descent || phase_ == Phase::RecoveryDescent ||
         phase_ == Phase::Px4LandPending) &&
        land_dwell_first_time_ >= 0 && !normal_ground(c_.speed_tolerance))
      return true;
    if (phase_ == Phase::Ascend && dwell_since_ >= 0 &&
        (std::abs(height - target_) > c_.height_tolerance || std::abs(vz) > c_.speed_tolerance))
      return true;
    if (phase_ == Phase::ThrustRamp) {
      if (!ramp_feedback_active_ && height >= std::min(target_, c_.liftoff_height_m))
        return true;
      const auto& rise = takeoff_evidence_.evidence();
      if (rise.local_samples &&
          (height < c_.liftoff_height_m || (!rise.rise_confirmed && -vz < c_.liftoff_speed_mps)))
        return true;
    }
    return false;
  }
  const ArmedGroundFaultEvidence& armed_ground_fault_evidence() const {
    return armed_ground_fault_evidence_;
  }
  const AuthorityGroundEvidence& authority_ground_evidence() const {
    return authority_ground_evidence_;
  }
  // 按位记录权限地面门：已解锁/未着陆/未静止/静止不足/垂速越限，供同帧诊断。
  std::uint32_t authority_ground_failed_mask(const Feedback& s) const {
    return (s.armed ? 1U : 0U) | (!s.landed ? 2U : 0U) | (!s.at_rest ? 4U : 0U) |
           ((!std::isfinite(s.stationary_seconds) || s.stationary_seconds < c_.ground_seconds)
                ? 8U
                : 0U) |
           ((!std::isfinite(s.vz) || std::abs(s.vz) > c_.ground_vz_tolerance) ? 16U : 0U);
  }
  // 优先使用最近实际提交的视觉偏航，否则沿准备时捕获的本地偏航；不强制世界正北。
  double yaw_reference() const {
    return committed_yaw_set_ ? committed_yaw_ : reference_.yaw();
  }
  // 水平目标保持偏航，roll/pitch目标为零；wxyz四元数由yaw半角构造，单位rad。
  Quaternion level_target() const {
    const double yaw = yaw_reference();
    return {std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)};
  }
  const authority::State& authority_state() const {
    return authority_;
  }
  const std::string& takeover_reason() const {
    return takeover_reason_;
  }
  bool restart_required() const {
    return restart_required_;
  }
  bool rc_fault_recovery_ready() const {
    return rc_recovery_ready_ && !recovery_event_blocked_;
  }
  // 恢复诊断分别保留首次故障、最新故障和当前阻塞项；计数只计独立推进的源消息。
  struct RcRecoveryDiagnostics {
    std::string first_fault_reason, latest_fault_reason, block_reason;
    bool recoverable{}, ready{}, event_blocked{}, window_active{};
    double elapsed_s{}, required_s{};
    unsigned status_samples{}, land_samples{}, local_samples{};
    std::uint64_t status_stamp{}, land_stamp{}, local_stamp{};
  };
  // 只读生成快照，不因状态窗口刷新增加地面时长或新样本数。
  RcRecoveryDiagnostics rc_recovery_diagnostics() const {
    return {first_fault_reason_,
            latest_fault_reason_,
            rc_recovery_block_reason_,
            rc_fault_recoverable_,
            rc_fault_recovery_ready(),
            recovery_event_blocked_,
            rc_recovery_since_ >= 0,
            rc_recovery_since_ < 0 ? 0
                                   : std::max(0., rc_recovery_observed_at_ - rc_recovery_since_),
            c_.ground_seconds,
            rc_recovery_status_samples_,
            rc_recovery_land_samples_,
            rc_recovery_local_samples_,
            rc_recovery_last_status_,
            rc_recovery_last_land_,
            rc_recovery_last_local_};
  }
  // Safety中继续只读更新故障因果水位及独立地面恢复证据；不再生成动力计划。
  void observe_safety_terminal(const Feedback& s, double now) {
    if (phase_ == Phase::Safety) {
      observe_fault_watermarks(s);
      observe_external_terminal(s, now);
      observe_rc_recovery(s, now);
    }
  }
  bool offboard_observed() const {
    return offboard_observed_;
  }
  bool ground_confirmed() const {
    return ground_confirmed_;
  }
  bool ground_release_pending() const {
    return ground_release_pending_;
  }
  bool arm_confirmed() const {
    return arm_confirmed_;
  }
  bool arm_cancel_pending() const {
    return arm_cancel_pending_;
  }
  bool arm_command_committed() const {
    return arm_command_committed_;
  }
  bool disarm_command_committed() const {
    return disarm_command_committed_;
  }
  bool disarm_outcome_uncertain() const {
    return disarm_outcome_uncertain_;
  }
  // 只由Runtime在最终动作发布出口返回结果后调用；committed=true才更新命令/状态基线。
  // 计划中的Output::command不证明VehicleCommand已经发出。
  void note_command_publication(Command command, bool committed, std::uint64_t land_stamp = 0,
                                std::uint64_t status_stamp = 0) {
    if (!committed)
      return;
    if (command == Command::Offboard)
      offboard_command_committed_ = true;
    if (command == Command::ReleaseOffboard)
      release_command_committed_ = true;
    if (command == Command::Arm || command == Command::Disarm || command == Command::Offboard ||
        command == Command::ReleaseOffboard)
      status_baseline_ = std::max(status_baseline_, status_stamp);
    if (command == Command::Arm)
      arm_command_committed_ = true;
    if (command == Command::Disarm) {
      disarm_command_committed_ = true;
      disarm_in_flight_ = true;
      disarm_land_commit_stamp_ = land_stamp;
    }
    if (command == Command::Px4Land) {
      px4_land_committed_ = true;
      handoff_status_baseline_ = status_stamp;
    }
  }
  std::uint64_t touchdown_baseline() const {
    return touchdown_baseline_;
  }
  FaultClass fault_class() const {
    return fault_class_;
  }
  double recovery_deadline() const {
    return recovery_deadline_;
  }
  bool recovery_budget_exhausted() const {
    return recovery_budget_exhausted_;
  }
  bool px4_land_command_committed() const {
    return px4_land_committed_;
  }
  bool recovery_progress_observed() const {
    return recovery_progress_observed_;
  }
  bool px4_land_mode_observed() const {
    return px4_land_mode_observed_;
  }
  // 这里只披露当前控制责任，不断言PX4已安全降落。
  // 遥测过期、Kill或进程失效时，本进程无法保证继续Offboard机动；
  // 这些情况依赖另行验证的PX4 failsafe，不能从本地状态字符串推断成功。
  // 明确谁仍承担下降/交接责任；输出文案不把超时、ACK或本地提交写成“已安全落地”。
  const char* recovery_responsibility(const Feedback& s) const {
    if (s.kill)
      return "EMERGENCY_KILL_NO_ROUTINE_DISARM";
    if (external_ground_disarmed_confirmed_)
      return "GROUND_DISARMED_CONFIRMED";
    if (phase_ == Phase::Safety)
      return s.status_fresh && !s.offboard ? "PX4_FAILSAFE_OBSERVED_BUT_CONTROL_UNVERIFIED"
                                           : "PX4_FAILSAFE_PENDING_UNKNOWN_AIRBORNE";
    if (phase_ == Phase::RecoveryDescent || phase_ == Phase::Px4LandPending) {
      if (recovery_budget_exhausted_)
        return "COMPANION_GUARDED_DESCENT_ACTIVE_RECOVERY_OVERDUE_HANDOFF_UNVERIFIED";
      return phase_ == Phase::RecoveryDescent
                 ? "CONTROLLED_DESCENT_ACTIVE_BOUNDED_RECOVERY"
                 : "PX4_HANDOFF_REQUEST_PENDING_COMPANION_DESCENT_ACTIVE";
    }
    if (phase_ == Phase::PoweredGround || phase_ == Phase::Landed)
      return "COMPANION_SLEW_LIMITED_POWER_REMOVAL_AND_NEW_GROUND_PROOF";
    if (phase_ == Phase::Descent)
      return "COMPANION_CONTROLLED_DESCENT_ACTIVE";
    if (arm_cancel_pending_ && phase_ == Phase::RequestArm)
      return "ARM_400_OUTCOME_UNCERTAIN_GUARDED_HEARTBEAT";
    if (disarm_in_flight_ && s.armed && s.offboard)
      return "DISARM_400_OUTCOME_UNCERTAIN_GUARDED_RECOVERY";
    if (operator_release_requested_ && s.offboard && active())
      return "OPERATOR_HANDOVER_REQUESTED_NOT_PX4_CONFIRMED";
    if (phase_ == Phase::SafeDisarmed || phase_ == Phase::ReleaseOffboard)
      return "DISARMED_GROUND_PX4_MODE_RELEASE_PENDING";
    return flight_started_ ? "COMPANION_OFFBOARD_ATTITUDE_CONTROL_ACTIVE"
                           : "DISARMED_OR_PREPARATION_NO_FLIGHT";
  }
  // RC路线先识别未加入阶段的显式接管；飞行中杆量/轴不安全也视为干预，之后比较开关代数。
  bool operator_intervened(const Feedback& s) const {
    if (c_.rc_navigation_enabled && !rc_joined_)
      return s.operator_takeover;
    if (c_.rc_navigation_enabled && flight_started_ && (s.rc_sticks_moving || !s.rc_axes_safe))
      return true;
    return s.operator_takeover || (active() && s.operator_switch_generation != switch_generation_);
  }
  // 按Kill、人工干预、PX4 failsafe、丢流/退出Offboard和RC失联顺序判断，保持原优先级。
  const char* takeover_cause(const Feedback& s) const {
    if (s.kill)
      return "KILL_OR_TERMINATION";
    if (operator_intervened(s) && !operator_release_requested_)
      return "EXPLICIT_OPERATOR_INTERVENTION";
    if (s.failsafe)
      return "PX4_FAILSAFE";
    if (offboard_observed_ && s.offboard_signal_lost)
      return "OFFBOARD_HEARTBEAT_LOST";
    if (offboard_observed_ && s.status_fresh && !s.offboard && phase_ != Phase::ReleaseOffboard &&
        phase_ != Phase::SafeDisarmed)
      return "PX4_LEFT_OFFBOARD";
    if (flight_started_ && authority_.mode == authority::Mode::RcTakeover) {
      if (!s.rc_link_ready)
        return "RC_LINK_LOST";
      if (!s.rc_switches_ready)
        return "RC_TAKEOVER_TELEMETRY_LOST";
    }
    return nullptr;
  }
  // RC路线用最近健康RC观察，控制台路线用交互PING租约；都使用同一主机单调秒域并拒绝回退。
  bool operator_lease_live(double now) const {
    if (c_.rc_navigation_enabled)
      return rc_operator_live_ && std::isfinite(now) && now >= rc_observed_at_ &&
             now - rc_observed_at_ <= c_.stall_seconds;
    return interactive_ && std::isfinite(now) && ping_ >= 0 && now >= ping_ &&
           now - ping_ <= c_.console_timeout;
  }
  // 非交互准备不要求控制台PING；进入交互后须持续满足原租约。
  bool preparation_lease_valid(double now) const {
    return !interactive_ || operator_lease_live(now);
  }
  // 显式启用控制台租约并记录当前单调时刻；它不是解锁或飞行许可。
  void live_console(double now) {
    interactive_ = true;
    ping_ = now;
  }
  // 只有已启用交互会话才续期，不从任意输入隐式生成操作员同意。
  void ping(double now) {
    if (interactive_)
      ping_ = now;
  }
  // 只能Idle且未锁定时选择接管策略；同时证明无输出、无事务及安全地面。
  std::string select_authority(authority::Mode mode, const Feedback& s, double now,
                               bool outputs_active, bool pending_command, bool publication_fault) {
    if (authority_.locked)
      return "AUTHORITY_MODE_LOCKED";
    if (phase_ != Phase::Idle || flight_started_)
      return "AUTHORITY_SELECTION_REQUIRES_IDLE";
    if (mode != authority::Mode::RcTakeover && mode != authority::Mode::AutonomousNoRc)
      return "INVALID_AUTHORITY_MODE";
    auto why = safe_authority_ground(s, now, outputs_active, pending_command, publication_fault);
    if (!why.empty())
      return why;
    authority_.mode = mode;
    authority_.attempt_mode = authority::Mode::Unset;
    return {};
  }
  // 权限复位不能清通用/发布完整性故障；必须任务已停止且独立地面门再次成立。
  std::string reset_authority(const Feedback& s, double now, bool outputs_active,
                              bool pending_command, bool publication_fault) {
    // 普通故障/发布完整性故障不能通过操作者API被清除。
    if (restart_required_ || (phase_ == Phase::Safety && !authority_.takeover_latched))
      return "AUTHORITY_SAFETY_RESTART_REQUIRED";
    if (active())
      return "AUTHORITY_MODE_LOCKED";
    auto why = safe_authority_ground(s, now, outputs_active, pending_command, publication_fault);
    if (!why.empty())
      return why;
    authority_ = {};
    flight_started_ = false;
    bench_arm_only_active_ = false;
    offboard_observed_ = false;
    z_set_ = false;
    external_bench_disarm_pending_ = false;
    offboard_command_committed_ = release_command_committed_ = false;
    bench_disarm_proven_ = bench_disarmed_seen_ = false;
    operator_release_requested_ = false;
    reference_.clear();
    switch_generation_ = s.operator_switch_generation;
    takeover_reason_.clear();
    enter(Phase::Idle, now);
    reason_ = "AUTHORITY_RESET_TO_UNSET_SAFE_GROUND";
    return {};
  }
  // 完整交接需要独立证明PX4已真实离开Offboard；
  // armed+Offboard时仅收到接管请求，先作为受控降落请求处理。
  std::string request_operator_takeover(const Feedback& s, double now) {
    if (!s.status_fresh)
      return "TAKEOVER_NEEDS_FRESH_PX4_STATUS";
    if (!s.offboard) {
      release_to_operator("PX4_MODE_HANDOVER_OBSERVED");
      return {};
    }
    operator_release_requested_ = true;
    // 与STOP相同，先完成待决Offboard事务，再计划地面释放用的第二条176。
    // 否则迟到的176 ACK候选可能与释放命令碰撞，失去阶段归属。
    if (phase_ == Phase::RequestOffboard && !s.armed) {
      ground_release_pending_ = true;
      reason_ = "TAKEOVER_REQUEST_WAIT_OFFBOARD_ACK_AND_STATUS";
      return {};
    }
    if (s.armed || phase_ == Phase::RequestArm) {
      const auto why = descend(s, now);
      if (!why.empty())
        return why;
      reason_ = "TAKEOVER_REQUEST_PENDING_CONTROLLED_GROUND_RECOVERY";
    } else if (z_set_) {
      enter(Phase::ReleaseOffboard, now);
      reason_ = "TAKEOVER_REQUEST_PENDING_GROUND_MODE_ACK";
    } else
      return "TAKEOVER_NO_GROUND_REFERENCE";
    return {};
  }
  bool operator_release_requested() const {
    return operator_release_requested_;
  }
  bool handover_observed() const {
    return authority_.takeover_latched && !offboard_observed_;
  }
  // 确认应让出控制时锁存OPERATOR_TAKEOVER_LATCHED；后续只能通过原独立地面恢复规则开启新任务。
  void release_to_operator(const std::string& why) {
    authority_.takeover_latched = true;
    authority_.locked = true;
    takeover_reason_ = why;
    fault("OPERATOR_TAKEOVER_LATCHED", false);
  }
  // 控制台准备入口只走非RC路线；新任务需健康、未解锁、着地静止、非Offboard及可用策略。
  std::string prepare(const Feedback& s, double now) {
    if (c_.rc_navigation_enabled)
      return "RC_NAVIGATION_USES_PHYSICAL_SWITCH_AUTHORIZATION";
    if (phase_ == Phase::Safety)
      return "SAFETY_RESTART_ON_SAFE_GROUND_REQUIRED";
    if (active())
      return "TASK_ACTIVE";
    if (authority_.locked)
      return "AUTHORITY_SAFE_GROUND_RESET_REQUIRED";
    auto why = health(s);
    if (!why.empty())
      return why;
    if (s.armed || !s.landed || !s.at_rest || s.offboard)
      return "PREPARE_REQUIRES_DISARMED_REST_OUTSIDE_OFFBOARD";
    if (!std::isfinite(now))
      return "INVALID_MONOTONIC_TIME";
    // bench必须在准备锁定之前选择有效的已审策略；
    // 关闭动力门的诊断仍可使用零输出专用的UNSET策略，不能据此解锁。
    if (c_.bench_arm_only_enabled) {
      const auto policy = authority::evaluate(authority_, authority_inputs(c_, s), false);
      if (policy != authority::Reason::Allowed)
        return authority::name(policy);
    }
    // 锁定本轮策略后捕获偏航；清任务局部参考/恢复状态，不推测真实已离地。
    authority_.locked = true;
    authority_.attempt_mode = authority_.mode;
    offboard_observed_ = false;
    switch_generation_ = s.operator_switch_generation;
    reference_.clear();
    reference_.observe(s.q, s.quat_reset, false);
    z_set_ = false;
    flight_started_ = false;
    bench_arm_only_active_ = false;
    ground_abort_ = false;
    ground_confirmed_ = false;
    ground_release_pending_ = false;
    external_bench_disarm_pending_ = false;
    offboard_command_committed_ = release_command_committed_ = false;
    liftoff_evidenced_ = false;
    arm_confirmed_ = false;
    clear_ramp_feedback();
    arm_cancel_pending_ = arm_command_committed_ = false;
    disarm_in_flight_ = disarm_command_committed_ = disarm_outcome_uncertain_ = false;
    disarm_land_commit_stamp_ = 0;
    recovery_deadline_ = -1;
    px4_land_committed_ = px4_land_attempted_ = false;
    recovery_budget_exhausted_ = false;
    px4_land_mode_observed_ = recovery_progress_observed_ = external_ground_disarmed_confirmed_ =
        false;
    recovery_last_local_stamp_ = 0;
    fault_class_ = FaultClass::None;
    last_sample_set_ = false;
    operator_release_requested_ = false;
    armed_ground_fault_evidence_ = {};
    clear_disarmed_prep();
    command_sent_ = false;
    last_ = now;
    event_ = "LOCAL_YAW_REFERENCE_CAPTURED";
    enter(Phase::SelfCheck, now);
    return {};
  }
  // 它与TAKEOFF是两个入口，沿用同样的健康/权限/ACK及最终发布门，
  // 但永远不授予进入ThrustRamp的许可。
  // 拆桨专用请求：程序解锁门开、起飞门关且已Offboard准备；与TAKEOFF严格互斥。
  std::string arm_only(const Feedback& s, double now) {
    if (c_.rc_navigation_enabled)
      return "RC_NAVIGATION_FORBIDS_AUTOMATIC_ARM";
    if (!c_.bench_arm_only_enabled)
      return "ARM_ONLY_BENCH_MODE_DISABLED";
    if (!c_.flight_actions_enabled || !c_.arm_enabled || c_.takeoff_enabled ||
        !c_.parameters_confirmed)
      return "ARM_ONLY_EXCLUSIVE_GATES_REQUIRED";
    if (restart_required_)
      return "AUTHORITY_SAFETY_RESTART_REQUIRED";
    if (takeover_cause(s))
      return "OPERATOR_TAKEOVER_LATCHED";
    const auto policy = authority::evaluate(authority_, authority_inputs(c_, s));
    if (policy != authority::Reason::Allowed)
      return authority::name(policy);
    if (phase_ != Phase::OffboardReady)
      return "ARM_ONLY_REQUIRES_OFFBOARD_READY";
    if (!interactive_ || ping_ < 0 || now < ping_ || now - ping_ > .5)
      return "LIVE_INTERACTIVE_CONSOLE_REQUIRED";
    auto why = health(s);
    if (!why.empty())
      return why;
    if (!s.offboard || s.armed || !s.landed || !s.at_rest || !z_set_ ||
        s.stationary_seconds < c_.ground_seconds)
      return "ARM_ONLY_REQUIRES_CONFIRMED_DISARMED_OFFBOARD_GROUND_REFERENCE";
    if (!s.battery_ready || !s.preflight)
      return "BATTERY_PREFLIGHT_REQUIRED";
    if (std::abs(s.vz) > c_.ground_vz_tolerance)
      return "ARM_ONLY_GROUND_VZ_NOT_STABLE";
    z0_ = s.z;
    z_reset_ = s.z_reset;
    vz_reset_ = s.vz_reset;
    event_ = "ARM_ONLY_DISARMED_GROUND_REFERENCE_RECAPTURED";
    target_ = command_height_ = 0;
    flight_started_ = true;
    bench_arm_only_active_ = true;
    flight_since_ = now;
    bench_disarm_proven_ = bench_disarmed_seen_ = false;
    bench_arm_status_baseline_ = bench_arm_land_baseline_ = bench_arm_local_baseline_ = 0;
    bench_disarm_status_floor_ = bench_disarm_local_floor_ = bench_disarm_land_floor_ = 0;
    last_sample_set_ = false;
    ground_confirmed_ = false;
    liftoff_evidenced_ = false;
    arm_confirmed_ = false;
    arm_cancel_pending_ = arm_command_committed_ = false;
    disarm_in_flight_ = disarm_command_committed_ = disarm_outcome_uncertain_ = false;
    disarm_land_commit_stamp_ = 0;
    vertical_->reset(0);
    feedback_guard_.reset();
    enter(Phase::RequestArm, now);
    return {};
  }
  // 控制台普通起飞入口；生产RC路线明确拒绝此入口，避免程序Arm替代人工解锁。
  std::string takeoff(const Feedback& s, double height, double now) {
    if (c_.rc_navigation_enabled)
      return "RC_NAVIGATION_FORBIDS_CONSOLE_TAKEOFF";
    if (c_.bench_arm_only_enabled)
      return "TAKEOFF_FORBIDDEN_IN_ARM_ONLY_BENCH_MODE";
    if (!c_.flight_actions_enabled)
      return "FLIGHT_ACTIONS_DISABLED";
    if (!c_.arm_enabled)
      return "ARM_DISABLED";
    if (!c_.takeoff_enabled)
      return "TAKEOFF_DISABLED";
    if (!c_.parameters_confirmed)
      return "FLIGHT_PARAMETERS_NOT_CONFIRMED";
    if (restart_required_)
      return "AUTHORITY_SAFETY_RESTART_REQUIRED";
    if (takeover_cause(s))
      return "OPERATOR_TAKEOVER_LATCHED";
    const auto policy = authority::evaluate(authority_, authority_inputs(c_, s));
    if (policy != authority::Reason::Allowed)
      return authority::name(policy);
    if (phase_ != Phase::OffboardReady)
      return "TAKEOFF_REQUIRES_OFFBOARD_READY";
    if (!interactive_ || ping_ < 0 || now < ping_ || now - ping_ > .5)
      return "LIVE_INTERACTIVE_CONSOLE_REQUIRED";
    auto why = health(s);
    if (!why.empty())
      return why;
    if (!s.offboard || s.armed || !s.landed || !s.at_rest || !z_set_ ||
        s.stationary_seconds < c_.ground_seconds)
      return "TAKEOFF_REQUIRES_CONFIRMED_DISARMED_OFFBOARD_GROUND_REFERENCE";
    if (!s.battery_ready || !s.preflight)
      return "BATTERY_PREFLIGHT_REQUIRED";
    // 目标向上高度必须在明确配置的包络内部；边界相等也拒绝，不能以默认值猜测。
    if (!std::isfinite(height) || height <= 0 || height >= c_.vertical.max_height)
      return "HEIGHT_OUTSIDE_REVIEWED_ENVELOPE";
    // SELF_CHECK与本次起飞请求之间可能相隔很久，所以在本次完全验证的未解锁地面准入重捕z0。
    // 不能把旧准备时的气压原点直接带进ArmedGround，也不允许空中重置原点来原谅估计器故障。
    if (std::abs(s.vz) > c_.ground_vz_tolerance)
      return "TAKEOFF_GROUND_VZ_NOT_STABLE";
    z0_ = s.z;
    z_reset_ = s.z_reset;
    vz_reset_ = s.vz_reset;
    // 只有当前已证明未解锁地面才重捕获z0；空中/估计器故障不能用重设零点消失。
    event_ = "TAKEOFF_DISARMED_GROUND_Z_REFERENCE_RECAPTURED";
    target_ = height;
    command_height_ = 0;
    flight_started_ = true;
    bench_arm_only_active_ = false;
    flight_since_ = now;
    clear_ramp_feedback();
    last_sample_set_ = false;
    ground_confirmed_ = false;
    liftoff_evidenced_ = false;
    arm_confirmed_ = false;
    arm_cancel_pending_ = arm_command_committed_ = false;
    disarm_in_flight_ = disarm_command_committed_ = disarm_outcome_uncertain_ = false;
    disarm_land_commit_stamp_ = 0;
    vertical_->reset(0);
    feedback_guard_.reset();
    enter(Phase::RequestArm, now);
    return {};
  }
  // 降落/取消入口先撤销视觉；按尚未加入RC、尚未起飞、未决Arm、bench和普通飞行分别处理。
  std::string descend(const Feedback& s, double now) {
    visual_.stop();
    visual_result_.active = false;
    if (c_.rc_navigation_enabled && !rc_joined_) {
      release_to_operator("RC_PREPARATION_CANCELLED_NO_AUTOMATIC_ARM_OR_DISARM");
      return {};
    }
    if (phase_ == Phase::Safety)
      return "SAFETY_RELEASED_TO_PX4_OPERATOR";
    if (phase_ == Phase::ReleaseOffboard || phase_ == Phase::SafeDisarmed)
      return {};
    if (!flight_started_) {
      if (s.armed)
        return "UNEXPECTED_ARMED_CANNOT_RECOVER_WITH_STAGE_ONE";
      if (phase_ == Phase::RequestOffboard) {
        // 本请求返回后，先前176仍可能在PX4生效；继续严格零流，
        // 等待ACK/新Offboard状态的原事务结果，再进行地面模式释放。
        ground_release_pending_ = true;
        reason_ = "WAIT_OFFBOARD_RESULT_BEFORE_GROUND_RELEASE";
        return {};
      }
      if (s.offboard && z_set_) {
        enter(Phase::ReleaseOffboard, now);
        reason_ = "GROUND_RELEASE_HANDSHAKE_REQUIRED";
      } else {
        enter(Phase::Idle, now);
        reason_ = "DISARMED_PREPARATION_RELEASED";
      }
      return {};
    }
    // 已计划/已提交Arm不能撤回；取消只置arm_cancel_pending_并等独立新状态，不发送第二个Arm。
    if (phase_ == Phase::RequestArm) {
      // 400可能已经通过最终发布出口；LAND无法撤回已发送命令。
      // 保持最终门保护的精确零目标，等PX4新状态；取消后不再计划第二条Arm。
      arm_cancel_pending_ = true;
      reason_ = arm_command_committed_ ? "ARM_COMMITTED_CANCEL_PENDING_PX4_STATUS"
                                       : "ARM_PLANNED_CANCEL_PENDING_FINAL_SINK_RESULT";
      return {};
    }
    if (!s.armed)
      return "ACTIVE_FLIGHT_NOT_ARMED";
    // bench不得调用普通下降控制（可能产生非零推力）；只在新地面证据成立后走原零推力上锁握手。
    if (bench_arm_only_active_) {
      if (phase_ == Phase::RequestDisarm || phase_ == Phase::Landed ||
          phase_ == Phase::PoweredGround)
        return {};
      // 普通下降控制器可能算出非零推力，所以拆桨bench不能调用它。
      // 此请求只进入专用零流地面观察，后面仍需新样本；请求本身不是地面证明，也不授权Disarm。
      if (!health(s).empty() || !z_set_ || !bench_ground_observation(c_, s, z0_))
        return "BENCH_GROUND_UNCONFIRMED_OPERATOR_TAKEOVER_REQUIRED";
      land_baseline_ = s.land_stamp;
      enter(Phase::PoweredGround, now);
      reason_ = "BENCH_GROUND_DISARM_REQUESTED_NO_RAMP";
      return {};
    }
    if (phase_ != Phase::Descent && phase_ != Phase::RecoveryDescent &&
        phase_ != Phase::Px4LandPending && phase_ != Phase::Landed &&
        phase_ != Phase::RequestDisarm) {
      command_height_ = std::max(0., z0_ - s.z);
      land_baseline_ = s.land_stamp;
      // 普通取消结合着陆检测、高度和速度判断地面候选；尚有离地迹象时不得直接断流。
      const bool plausible_ground = !pending_ramp_lift(s) && health(s).empty() && s.landed &&
                                    std::abs(z0_ - s.z) <= c_.height_tolerance &&
                                    std::abs(s.vz) <= c_.speed_tolerance;
      ground_abort_ = plausible_ground;
      fault_class_ = plausible_ground ? FaultClass::GroundAbort : FaultClass::ControlledDescent;
      // 解锁后或电机仍转时at_rest可能为false；不能仅据此宣称已在空中或立即断流。
      // PoweredGround仍负责按变化率降至零，并要求新的检测器样本。
      enter(plausible_ground ? Phase::PoweredGround : Phase::Descent, now);
    }
    return {};
  }
  // 控制台退出撤销租约并尝试原下降路径；打包报告等工作由外层等控制生命周期结束后执行。
  void disconnect(const Feedback& s, double now) {
    interactive_ = false;
    ping_ = -1;
    if (active()) {
      auto why = descend(s, now);
      if (!why.empty())
        fault(why);
    }
  }
  // 未进入飞行托管时只安排地面模式释放；未决Offboard事务仍需等结果，不能立即遗忘。
  void stop_disarmed(const Feedback& s, double now) {
    if (!flight_started_ && phase_ != Phase::Safety) {
      if (phase_ == Phase::RequestOffboard) {
        ground_release_pending_ = true;
        return;
      }
      if (s.offboard && z_set_)
        enter(Phase::ReleaseOffboard, now);
      else
        enter(Phase::Idle, now);
    }
  }
  // 外层不可恢复拒绝通过统一fault锁存，避免多条路径各自清状态。
  void block(const std::string& reason) {
    fault(reason);
  }
  // 仅尚无解锁意图的RC准备允许将已识别的时限拒绝标为可地面恢复，不能覆盖非零/未知部分发布。
  bool block_rc_preparation_timing(const std::string& reason, const Feedback& s) {
    if ((phase_ != Phase::RcPrestream && phase_ != Phase::RcWaitArm) || !c_.rc_navigation_enabled ||
        !rc_start_.no_arm_intent_observed() || rc_joined_ || flight_started_ || restart_required_ ||
        fault_class_ != FaultClass::None || authority_.takeover_latched || s.armed || s.offboard)
      return false;
    fault(reason);
    rc_fault_recoverable_ = true;
    return true;
  }
  // 本轮最终控制时限不新鲜就清地面恢复窗口，早期健康观察不得抵消最终超时。
  void clear_rc_timing_recovery() {
    clear_rc_recovery("RC_RECOVERY_CONTROL_LOOP_NOT_FRESH");
  }
  // 开始一个接收批次前清本批次否决位；随后任意不安全离散事件都会重新置位。
  void begin_safety_observation() {
    recovery_event_blocked_ = false;
  }
  // 离散安全来源分别为姿态/垂向/状态/failsafe/权限/电池/着陆；枚举仅选择检查分支，不直接推进正常阶段。
  enum class SafetyEvent { Attitude, Vertical, Status, Failsafe, Authority, Battery, Land };
  // 规划前逐个检查排队载荷，健康观察不推进正常阶段也不清故障。
  // 未汇合RC仍用原等待/重置策略；一旦已有解锁意图，该策略会阻塞本次准入。
  bool observe_safety_event(const Feedback& s, SafetyEvent source, double now) {
    if (phase_ == Phase::Safety) {
      observe_fault_watermarks(s);
      // 连续快照末帧健康也不能覆盖排队的坏事件；它会重新开始地面恢复证明。
      // Safety里的坏事件重启地面证明；同批次首个阻塞原因保持，后续坏帧不能抹掉否决。
      std::string blocked = rc_recovery_sample_block_reason(s);
      if (rc_recovery_since_ >= 0 &&
          (s.quat_reset != rc_recovery_quat_reset_ || s.z_reset != rc_recovery_z_reset_ ||
           s.vz_reset != rc_recovery_vz_reset_))
        blocked = "RC_RECOVERY_ESTIMATOR_RESET";
      if (blocked.empty() && source == SafetyEvent::Vertical && rc_recovery_since_ >= 0 &&
          std::abs(s.z - rc_recovery_z_) > c_.height_tolerance)
        blocked = "RC_RECOVERY_HEIGHT_DISCONTINUITY";
      if (blocked.empty() && source == SafetyEvent::Vertical && c_.feedback_guard.enabled) {
        const auto& e = recovery_feedback_guard_.observe(s.vertical_sample_stamp, s.z, s.vz,
                                                         s.z_deriv, c_.local_timeout);
        if (!e.healthy)
          blocked = e.reason;
      }
      if (!blocked.empty()) {
        // 保留本批第一个坏事件的解释和否决位；后面的坏事件不能把首因或否决冲掉。
        const auto reason = recovery_event_blocked_ ? rc_recovery_block_reason_
                                                    : "RC_RECOVERY_QUEUED_EVENT_" + blocked;
        recovery_event_blocked_ = true;
        clear_rc_recovery(reason);
      }
      return true;
    }
    // 拒绝时先推进已见源水位，避免故障后复用同一旧帧当作“新恢复证据”。
    // [&]构造局部lambda并按引用捕获本次作用域；它同步调用，不会被保存到下一轮。
    // 统一拒绝时先抬高故障水位，再走RC原准入规则或fault；返回true表示事件已处理，并非健康。
    const auto reject = [&](const std::string& why) {
      last_status_stamp_ = std::max(last_status_stamp_, s.status_stamp);
      last_land_stamp_ = std::max(last_land_stamp_, s.land_stamp);
      last_local_stamp_ = std::max(last_local_stamp_, s.local_stamp);
      last_rc_switch_stamp_ = std::max(last_rc_switch_stamp_, s.rc_switch_sample_stamp);
      if (c_.rc_navigation_enabled && !rc_joined_)
        rc_startup(s, now, why);
      else
        fault(why);
      return true;
    };
    // 已经armed或已承担任务托管时锁定参考，reset/jump必须拒绝，不能按普通未解锁准备宽容。
    const bool estimator_active = flight_started_ || s.armed;
    // 姿态事件独立检查可归一化四元数和活动期reset，不用“角度大”替代数值无效。
    if (source == SafetyEvent::Attitude) {
      if (!normalized(s.q))
        return reject("ATTITUDE_STALE_OR_INVALID");
      if (estimator_active && reference_.ready() && s.quat_reset != reference_.reset_counter())
        return reject("ACTIVE_QUATERNION_RESET");
      // 垂向事件检查有效位、有限值、重置、累计跳变及物理包络；同一估计器导数检查保留。
    } else if (source == SafetyEvent::Vertical) {
      if (!s.z_valid || !s.vz_valid)
        return reject("Z_VZ_STALE_OR_INVALID");
      if (!std::isfinite(s.z) || !std::isfinite(s.vz) ||
          (c_.feedback_guard.enabled && !std::isfinite(s.z_deriv)))
        return reject("NONFINITE_VERTICAL_TELEMETRY");
      if (c_.rc_navigation_enabled && !rc_joined_ && rc_start_.phase() == RcStartPhase::Prestream &&
          rc_start_.ground_reference_height_changed(s.z, c_.height_tolerance))
        return reject("RC_PRESTREAM_GROUND_CHANGED");
      if (estimator_active && z_set_ && (s.z_reset != z_reset_ || s.vz_reset != vz_reset_))
        return reject("ACTIVE_VERTICAL_ESTIMATOR_RESET");
      if (estimator_active && last_sample_set_ &&
          (std::abs(s.z - last_sample_z_) > c_.z_jump_limit_m ||
           std::abs(s.vz - last_sample_vz_) > c_.vz_jump_limit_mps))
        return reject("UNEXPLAINED_Z_VZ_ESTIMATOR_JUMP");
      if (flight_started_ &&
          (z0_ - s.z > c_.vertical.max_height ||
           z0_ - s.z < -(bench_arm_only_active_ ? c_.bench_ground_down_tolerance_m
                                                : c_.height_lower_margin_m) ||
           std::abs(s.vz) >
               c_.vertical.max_climb + c_.vertical.max_descent + 2 * c_.vz_jump_limit_mps))
        return reject("HEIGHT_OR_SPEED_ENVELOPE_EXCEEDED");
      if (c_.feedback_guard.enabled && flight_started_ && !bench_arm_only_active_ && s.armed &&
          phase_ >= Phase::ThrustRamp && phase_ <= Phase::RequestDisarm) {
        const auto& evidence = feedback_guard_.observe(s.vertical_sample_stamp, s.z, s.vz,
                                                       s.z_deriv, c_.local_timeout);
        if (!evidence.healthy)
          return reject(evidence.reason);
      }
    } else if (source == SafetyEvent::Battery) {
      if (!s.battery_ready)
        return reject("BATTERY_NOT_READY");
      // 触地证据一旦反弹就清连续计时；不得用稍后的landed=true恢复同一个坏样本。
    } else if (source == SafetyEvent::Land) {
      if ((phase_ == Phase::Landed || phase_ == Phase::RequestDisarm ||
           phase_ == Phase::SafeDisarmed || phase_ == Phase::ReleaseOffboard) &&
          (!s.landed || (!s.at_rest && !bench_arm_only_active_))) {
        ground_confirmed_ = false;
        clear_land_dwell();
        return true;
      }
      if (c_.rc_navigation_enabled && !rc_joined_ &&
          (!s.landed ||
           (!s.at_rest && !s.armed && s.rc_arm_switch != 1 && rc_start_.no_arm_intent_observed())))
        return reject("RC_PRESTREAM_GROUND_CHANGED");
    } else if (source == SafetyEvent::Status || source == SafetyEvent::Failsafe ||
               source == SafetyEvent::Authority) {
      if (c_.rc_navigation_enabled && !rc_joined_) {
        if (s.kill || s.failsafe || s.calibration || s.hil || !s.preflight || !s.battery_ready ||
            !RcNavigationStart::rc_live(s) || !s.rc_axes_safe || s.rc_sticks_moving ||
            !s.rc_aux_switches_safe)
          return reject("RC_QUEUED_SAFETY_CONDITION");
      }
      if (const auto cause = takeover_cause(s)) {
        if (s.offboard && s.armed && !s.kill && !s.failsafe && s.status_fresh &&
            std::string(cause) == "EXPLICIT_OPERATOR_INTERVENTION") {
          const auto why = request_operator_takeover(s, now);
          if (!why.empty())
            fault(why);
          return phase_ == Phase::Safety;
        } else
          release_to_operator(cause);
        return true;
      }
      if (source == SafetyEvent::Status && !s.status_fresh)
        return reject("DDS_STATUS_LAND_OR_FAILSAFE_STALE");
      if (source == SafetyEvent::Status && flight_started_ && !bench_arm_only_active_ &&
          phase_ >= Phase::ArmedGround && phase_ != Phase::RequestDisarm &&
          phase_ != Phase::ReleaseOffboard && !s.armed)
        return reject("UNEXPECTED_DISARM_ACTIVE_FLIGHT");
      if (source == SafetyEvent::Status && (s.calibration || s.hil))
        return reject("CALIBRATION_OR_HIL_ACTIVE");
      if (source == SafetyEvent::Status && flight_started_ && !s.battery_ready)
        return reject("BATTERY_NOT_READY");
    }
    return false;
  }
  // 一次 step 只使用传入的这份 s，不读取 ROS、不等待新消息、不直接操作电机。
  // const Feedback& 借用外层快照且不能经此引用修改；mode/arm/px4_land 按值传入，
  // 是 Runtime 对 176模式、400解/上锁、21降落事务的本次摘要，不是可在这里改写的 Runtime 状态。
  // now/elapsed 是主机单调秒，源 stamp 是微秒；同域可相减，跨域不能直接相减。
  //
  // 按以下顺序阅读本函数：
  //   1. 终态/RC准入分流；2. 时间、接管、健康、权限和估计器门；
  //   3. 处理外部上锁及任务期限；4. 形成默认零推力计划；5. 各阶段更新；6. 公共输出校验。
  // if/else if 链通常只执行一个阶段分支；但 enter 会当场改变 phase_，后面的独立 if 会看到新值。
  // 例如 SafeDisarmed 后的 ReleaseOffboard 故意在同一调用继续处理，不能改成只看函数入口的阶段。
  // return out 保留当前局部计划；return {} 构造默认的空 Output，丢弃本次局部发布/动作计划。
  // 两种 return 都不会撤回已经写入成员的 phase_/故障/证据，外层仍需执行自己的最终发布门。
  Output step(const Feedback& s, double now, Transaction mode, Transaction arm,
              Transaction px4_land = {}) {
    // 每次重新初始化，publish/command/推力采用 Output 的默认值；旧任务输出不会自动被带过来。
    Output out;
    // 故障锁存时仅观察新地面证据并返回无发布计划；不靠下一健康帧直接恢复动力。
    if (phase_ == Phase::Safety) {
      observe_safety_terminal(s, now);
      return out;
    }
    // 这些“最后见过的源戳”供 fault 记录故障因果下界，不表示这一帧已经获准参与控制。
    last_local_stamp_ = s.local_stamp;
    last_rc_switch_stamp_ = s.rc_switch_sample_stamp;
    if (c_.rc_navigation_enabled) {
      rc_operator_live_ = RcNavigationStart::rc_live(s);
      rc_observed_at_ = now;
      // 尚未汇合就直接返回 RC 准入结果；下方程序 Arm/常规阶段不会抢先运行。
      // READY 的零流、真正的人工 armed 证据和离地认证分别属于不同的门。
      if (!rc_joined_)
        return rc_startup(s, now);
    }
    last_status_stamp_ = s.status_stamp;
    last_land_stamp_ = s.land_stamp;
    // flight_started_ 表示本 Core 已承担本次动力任务的托管责任，不等于“飞起来了”。
    // 若只做准备却看到 armed，不能擅自把外部已解锁的飞机认作本任务继续加推力。
    if (s.armed && !flight_started_) {
      fault("UNEXPECTED_ARMED_STAGE_ONE_REFUSED");
      return out;
    }
    if (!active())
      return out;
    // 原始周期差直接用于stall与控制积分；不得先限幅elapsed而掩盖真正停顿。
    const double elapsed = now - last_;
    // 先保存本次观察时刻，再拒绝回退/停顿；失败后的成员状态不会因 return 自动回滚。
    last_ = now;
    if (!std::isfinite(now) || !std::isfinite(elapsed) || elapsed < 0) {
      fault("TIME_REGRESSION");
      return out;
    }
    if (elapsed > c_.stall_seconds) {
      fault("CONTROL_LOOP_STALL");
      return out;
    }
    // 必须先有本地提交，再有严格晚于handoff_status_baseline_的新模式状态，才算观察到模式转移；
    // 不能证明PX4选中的飞行任务确实在执行，ACK自身也不能解除本机托管。
    if (phase_ == Phase::Px4LandPending && px4_land_committed_ && s.status_fresh && !s.offboard &&
        s.status_stamp > handoff_status_baseline_) {
      px4_land_mode_observed_ = s.auto_land_selected;
      fault_class_ = FaultClass::Px4Handoff;
      release_to_operator(
          s.auto_land_selected
              ? (px4_land.accepted
                     ? "PX4_LAND_ACK_AND_NEW_MODE_OBSERVED_CONTROL_UNVERIFIED"
                     : "PX4_LAND_NEW_MODE_OBSERVED_ACK_UNATTRIBUTED_CONTROL_UNVERIFIED")
              : "PX4_OTHER_MODE_OBSERVED_CONTROL_UNVERIFIED");
      return out;
    }
    if (s.status_fresh && s.offboard)
      offboard_observed_ = true;
    // if条件中的局部声明：cause是本if内的原因指针；空指针表示没有接管原因。
    // 判定已发生接管，与仅收到“请求接管”的意图不同，所以下面保留两条处理路径。
    if (const auto cause = takeover_cause(s)) {
      if (s.offboard && s.armed && !s.kill && !s.failsafe && s.status_fresh &&
          std::string(cause) == "EXPLICIT_OPERATOR_INTERVENTION") {
        const auto requested = request_operator_takeover(s, now);
        if (!requested.empty()) {
          fault(requested);
          return out;
        }
        // 物理开关已动不等于PX4已切换模式；若仍armed+Offboard，
        // 先维持健康门保护下的受控下降，不能静默遗弃仍由外部目标控制的飞机。
      } else {
        fault_class_ =
            (s.armed && !verified_ground(s)) ? FaultClass::Px4Handoff : FaultClass::GroundAbort;
        release_to_operator(cause);
        return out;
      }
    }
    // 统一健康拒绝早于任何闭环运算；armed时无法保证空中状态，故障分类保守保留未知。
    // health 返回首个拒绝原因；空字符串才表示这些公共数据门已通过。
    // 此处失败就早退，后面的参考更新/积分运算都不会执行，也不把坏输入补成零继续算。
    const auto why = health(s);
    if (!why.empty()) {
      fault_class_ = s.armed ? FaultClass::UnknownAirborne : FaultClass::GroundAbort;
      fault(why);
      return out;
    }
    if (external_bench_disarm_pending_ && s.armed) {
      fault("BENCH_REARM_AFTER_EXTERNAL_DISARM_FORBIDDEN");
      return out;
    }
    // 尚未托管动力时，交互准备租约过期会终止准备。已经托管后的控制台失联则在后面走下降，
    // 避免把“窗口关闭”直接当成“飞机已经落地，可以立即丢弃控制流”。
    if (!flight_started_ && interactive_ &&
        (ping_ < 0 || now < ping_ || now - ping_ > c_.console_timeout)) {
      fault("PREPARATION_CONSOLE_LEASE_EXPIRED");
      return out;
    }
    // && 只在左侧为真时才计算右侧；括号内用 || 表示任一必要动作门关闭都拒绝。
    // 程序解锁与人工 RC 是二选一的合法路径；takeoff 与专用 bench 也是不同配置合同。
    if (flight_started_ &&
        (!c_.flight_actions_enabled || (!c_.arm_enabled && !c_.rc_navigation_enabled) ||
         (!c_.takeoff_enabled && !c_.bench_arm_only_enabled) || !c_.parameters_confirmed)) {
      fault("FLIGHT_GATE_CLOSED");
      return out;
    }
    if (flight_started_) {
      const auto policy = authority::evaluate(authority_, authority_inputs(c_, s));
      if (policy != authority::Reason::Allowed) {
        fault(authority::name(policy));
        return out;
      }
      if (!s.battery_ready) {
        fault("BATTERY_NOT_READY");
        return out;
      }
    }
    // 即使 bench 已出现上锁反馈，在地面完成证明前也不能放宽参考/reset保护。
    const bool estimator_active =
        flight_started_ || s.armed || (bench_arm_only_active_ && bench_disarmed_seen_);
    // estimator_active为真时禁止姿态reset重建；为假时observe允许按新reset重新捕获yaw。
    // 此调用本身不证明地面静止，准备/RC准入与后面的垂向重建仍各自保留地面证据门。
    // auto 推导的是 ReferenceResult 枚举；它描述这次“观察”有没有重建或拒绝参考。
    // reference_ 自己持有捕获的 yaw/reset；s.q 是实测姿态，不是本函数要发送的目标姿态。
    const auto ref = reference_.observe(s.q, s.quat_reset, estimator_active);
    if (ref == ReferenceResult::ActiveReset) {
      fault("ACTIVE_QUATERNION_RESET");
      return out;
    }
    if (ref == ReferenceResult::Rebased)
      event_ = "DISARMED_QUATERNION_REBASED";
    // 垂向reset与姿态reset同样受托管状态约束，不能用重新设z0绕过飞行故障。
    if (z_set_ && (s.z_reset != z_reset_ || s.vz_reset != vz_reset_)) {
      if (estimator_active) {
        fault("ACTIVE_VERTICAL_ESTIMATOR_RESET");
        return out;
      }
      if (!s.landed || !s.at_rest || s.stationary_seconds < c_.ground_seconds) {
        fault("GROUND_RESET_REFERENCE_NOT_STABLE");
        return out;
      }
      z0_ = s.z;
      z_reset_ = s.z_reset;
      vz_reset_ = s.vz_reset;
      event_ = "DISARMED_Z_REFERENCE_REBASED";
    }
    // 这个枚举区间仍要求 Offboard 托管；模式已离开就不能继续使用旧外部目标。
    // ReleaseOffboard 在区间之外，因为该阶段正在等待模式真实离开。
    if (phase_ >= Phase::OffboardReady && phase_ <= Phase::RequestDisarm && !s.offboard) {
      fault_class_ = s.armed ? FaultClass::Px4Handoff : FaultClass::GroundAbort;
      fault("OFFBOARD_LOST_OR_OPERATOR_MODE_CHANGED");
      return out;
    }
    // 跳变检查必须早于bench外部上锁的提前返回；
    // 有限连续漂移宽容不能把突然跳变洗成安全地面或模式释放证据。
    // 同一控制观察间的累计跳变也检查，接收端相邻帧平滑不能掩盖累计大变化。
    if (estimator_active && last_sample_set_ &&
        (std::abs(s.z - last_sample_z_) > c_.z_jump_limit_m ||
         std::abs(s.vz - last_sample_vz_) > c_.vz_jump_limit_mps)) {
      fault_class_ = FaultClass::UnknownAirborne;
      fault("UNEXPLAINED_Z_VZ_ESTIMATOR_JUMP");
      return {};
    }
    if (estimator_active) {
      last_sample_z_ = s.z;
      last_sample_vz_ = s.vz;
      last_sample_set_ = true;
    }
    // 只有原监测健康位失败才停机；速度差suspect/rate_warning继续是诊断，不改PI和限幅。
    if (c_.feedback_guard.enabled && flight_started_ && !bench_arm_only_active_ && s.armed &&
        phase_ >= Phase::ThrustRamp && phase_ <= Phase::RequestDisarm) {
      // const auto& 借用 guard 内保存的证据，不复制整个结构；observe 本身会推进它的样本窗口。
      // 本地引用只在这段读取，不在后续再次 observe 后把旧引用当作冻结的历史证据。
      const auto& evidence =
          feedback_guard_.observe(s.vertical_sample_stamp, s.z, s.vz, s.z_deriv, c_.local_timeout);
      // vz与z_deriv差只用于诊断；这里拒绝的是guard的硬健康失败。
      // 原积分、抗饱和、制动和物理包络照常保留。
      if (!evidence.healthy) {
        fault_class_ = FaultClass::UnknownAirborne;
        fault(evidence.reason);
        return {};
      }
    }
    // 本地Disarm已提交且PX4新状态已上锁后，改用上锁后地面合同。
    // 静止证明完成前保留flight_started_，健康/参考reset/样本跳变/权限门继续有效。
    if (bench_arm_only_active_ && phase_ == Phase::RequestDisarm && bench_disarm_proven_ &&
        disarm_command_committed_ && !s.armed && s.status_stamp > status_baseline_)
      bench_disarmed_seen_ = true;
    if (bench_arm_only_active_ && bench_disarmed_seen_ && s.armed) {
      fault("BENCH_REARM_DURING_DISARM_VERIFICATION");
      return out;
    }
    if (flight_started_ && phase_ >= Phase::ArmedGround && phase_ != Phase::RequestDisarm &&
        phase_ != Phase::ReleaseOffboard && !s.armed) {
      if (bench_arm_only_active_ && arm_confirmed_ && !disarm_in_flight_) {
        // PX4/RC可能先于本机主动上锁；这只算外部状态观察，不能追认本机Disarm命令已成功。
        if (s.status_stamp <= status_baseline_ || !bench_disarmed_ground_observation(c_, s)) {
          fault("BENCH_EXTERNAL_DISARM_GROUND_UNCONFIRMED");
          return out;
        }
        if (!external_bench_disarm_pending_) {
          external_bench_disarm_pending_ = true;
          external_bench_status_baseline_ = s.status_stamp;
          external_bench_land_baseline_ = s.land_stamp;
          enter(Phase::PoweredGround, now);
        }
        if (dwell_bench_ground(verified_ground(s) &&
                                   s.status_stamp > external_bench_status_baseline_ &&
                                   s.land_stamp > external_bench_land_baseline_,
                               s, now)) {
          // 把已完成的外部上锁观察保留到SafeDisarmed及释放出口；
          // 它不声称本机发过或成功过400。
          external_bench_disarm_pending_ = false;
          bench_disarmed_seen_ = true;
          flight_started_ = false;
          ground_confirmed_ = true;
          enter(Phase::SafeDisarmed, now);
          reason_ = "BENCH_EXTERNAL_DISARM_NEW_SAFE_GROUND_EVIDENCED";
          return out;
        }
        out.publish = true;
        out.q = reference_.level();
        out.height = z0_ - s.z;
        out.speed = -s.vz;
        reason_ = "BENCH_EXTERNAL_DISARM_WAIT_NEW_SAFE_GROUND_NO_400";
        return out;
      }
      if (disarm_in_flight_ && s.status_stamp > status_baseline_ &&
          s.land_stamp > disarm_land_commit_stamp_ && verified_ground(s)) {
        // 真实状态是独立证据；只有对应事务另有可归属ACK，才可在原因中写ACK成功。
        if (disarm_terminal(arm)) {
          enter(Phase::SafeDisarmed, now);
          flight_started_ = false;
          disarm_in_flight_ = false;
          reason_ = arm.accepted ? "DISARM_ACK_AND_FRESH_SAFE_GROUND_STATUS"
                                 : "DISARMED_FRESH_GROUND_STATE_ACK_UNATTRIBUTED";
          return out;
        }
        reason_ = "DISARMED_STATUS_PRESENT_AWAITING_400_ACK";
      } else {
        fault_class_ = FaultClass::UnknownAirborne;
        fault(disarm_in_flight_ ? "DISARMED_WHILE_GROUND_UNCONFIRMED_EXTERNAL_FAILSAFE_REQUIRED"
                                : "UNEXPECTED_DISARM_ACTIVE_FLIGHT");
        return out;
      }
    }
    // 控制台路线失去租约，或任一路线超过本次任务绝对时限，就请求原下降路径。
    // 这里调用 descend 会立即改 phase_；本次 step 仍继续往下，使用新阶段计算有界输出。
    // RC 路线不依赖 UI 的 ping；RequestArm 另有“命令可能已发出”的取消/新状态处理。
    if (flight_started_ && phase_ != Phase::RequestArm && phase_ <= Phase::HeightHold &&
        ((!c_.rc_navigation_enabled &&
          (!interactive_ || ping_ < 0 || now - ping_ > c_.console_timeout)) ||
         now - flight_since_ > c_.max_flight_seconds))
      descend(s, now);
    // 准备自检先证明地面；只有三动作门全关的零推力诊断可用独立位置稳定证明，不授权真实起飞。
    if (phase_ == Phase::SelfCheck) {
      if (s.armed || !s.landed || !s.at_rest) {
        fault("GROUND_STATE_CHANGED_DURING_SELF_CHECK");
        return out;
      }
      // 任何动力动作和授权准入仍需原低Vz连续静止证明。
      // 仅当全部动作门关闭，才允许用独立地面/位置稳定证据做未解锁零推力Offboard准备；
      // 这一诊断例外不能变成Arm就绪。
      // strict 是外层连续低垂速静止证明；diagnostics_only 必须三个动力动作门全关。
      // 下面的 !(A && B) 中 B 会被短路：只有确属诊断配置才推进备用位置稳定窗口。
      const bool strict = s.stationary_seconds >= c_.ground_seconds;
      const bool diagnostics_only =
          !c_.flight_actions_enabled && !c_.arm_enabled && !c_.takeoff_enabled;
      if (!strict && !(diagnostics_only && observe_disarmed_ground_prep(s, now))) {
        reason_ = "WAITING_FOR_VALID_GROUND_REFERENCE";
        return out;
      }
      z0_ = s.z;
      z_reset_ = s.z_reset;
      vz_reset_ = s.vz_reset;
      z_set_ = true;
      enter(Phase::Prestream, now);
      // 这里不 return，故同一帧会初始化零输出并进入下方 Prestream 分支；entered_ 刚写为 now。
    }
    // 默认计划是保持已捕获yaw的水平姿态，推力仍为零；高度z0-z向上、速度-vz向上。
    // level_target 仅给出水平姿态和选定 yaw；非零推力必须由具体阶段经 apply 单独写入。
    out.publish = true;
    out.q = level_target();
    out.height = z0_ - s.z;
    out.speed = -s.vz;
    out.target_height = command_height_;
    if (!std::isfinite(out.height) || !std::isfinite(out.speed)) {
      fault("NONFINITE_RELATIVE_HEIGHT");
      return {};
    }
    if (flight_started_) {
      // 三目运算 A ? B : C 只选一边：bench 的地面漂移宽容不扩散到正常飞行。
      // height 下界写成 -lower_margin，因为相对地面向下才是负高度。
      const double lower_margin =
          bench_arm_only_active_ ? c_.bench_ground_down_tolerance_m : c_.height_lower_margin_m;
      const bool stopped_bench = bench_arm_only_active_ && bench_disarmed_seen_ && !s.armed;
      if (!stopped_bench && (out.height > c_.vertical.max_height || out.height < -lower_margin ||
                             std::abs(s.vz) > c_.vertical.max_climb + c_.vertical.max_descent +
                                                  2 * c_.vz_jump_limit_mps)) {
        if (bench_arm_only_active_)
          capture_bench_ground_fault(s);
        fault_class_ = FaultClass::UnknownAirborne;
        fault("HEIGHT_OR_SPEED_ENVELOPE_EXCEEDED");
        return {};
      }
      // 实测姿态有效但较陡，仍然是应继续接收的反馈，不能在本路线自动变成姿态数值无效。
      // 无效/NaN四元数由health和最终边界拒绝，reset仍由上方保护；
      // 把有效大姿态错误变成Safety曾会中断接收，不能重新引入这种规则。
    }
    // 恢复预算到期只披露超期：不能把它当着陆证明或无健康控制地任意停止armed+Offboard流。
    if (recovery_deadline_ >= 0 && s.armed && !verified_ground(s) && now >= recovery_deadline_) {
      // 这里已经过健康、权限、估计器和包络门；超时仅披露尚无交接/落地证据。
      // 继续同一反馈闭环及有界/slew限制的下降仍只是保留托管，
      // 不保证持续无效推力、硬进程崩溃或硬件异常下也一定能安全完成。
      if (!recovery_budget_exhausted_)
        event_ = "RECOVERY_BUDGET_EXHAUSTED_NO_CONFIRMED_HANDOFF";
      recovery_budget_exhausted_ = true;
    }
    // 阶段分派从这里开始。该 if/else if 链本帧只选一个分支；分支内部可显式计算转场帧输出。
    // 非 RC 的 Prestream 以阶段时间计准备时长；RC 的真实提交计数在 RcNavigationStart 单独持有。
    // 持续零预发送满原时长后才计划一次Offboard命令，并锁住当时status水位。
    if (phase_ == Phase::Prestream) {
      if (now - entered_ >= c_.prestream_seconds) {
        enter(Phase::RequestOffboard, now);
        status_baseline_ = s.status_stamp;
        // command_sent_ 在 Core 内表示“本阶段已计划命令”，不是已经发送。
        // 外层最终 sink 返回后才通过 note_command_publication 更新 committed 字段。
        command_sent_ = true;
        out.command = Command::Offboard;
      }
    } else if (phase_ == Phase::RequestOffboard) {
      // 事务拒绝/超时保持原故障；ACK及严格新的模式反馈（或明确地面诊断例外）才进入准备完成。
      if (mode.rejected || mode.timed_out) {
        fault("OFFBOARD_COMMAND_REJECTED_OR_TIMEOUT");
        return {};
      }
      // mode.accepted是Runtime审核后的事务摘要，包含关联ACK，或176偏斜候选与独立新状态的窄核验路径。
      // 明确关闭动力门的地面诊断还另有窄状态证据例外；不能只看任意一条ACK就推进。
      // 无论哪条路径，右侧仍必须看到晚于 status_baseline_ 的真实 Offboard 状态；ACK不能替代它。
      if ((mode.accepted || (ground_mode_state_only_allowed() && offboard_command_committed_ &&
                             s.source_clock_validated)) &&
          s.offboard && s.status_stamp > status_baseline_) {
        if (ground_release_pending_) {
          ground_release_pending_ = false;
          enter(Phase::ReleaseOffboard, now);
          reason_ = mode.accepted ? "GROUND_RELEASE_AFTER_OFFBOARD_ACK_AND_STATUS"
                                  : "GROUND_RELEASE_AFTER_NEW_OFFBOARD_STATE_ACK_UNATTRIBUTED";
        } else {
          enter(Phase::OffboardReady, now);
          reason_ = mode.accepted ? "ACK_ACCEPTED_AND_NEW_OFFBOARD_DISARMED_STATUS"
                                  : "ZERO_ONLY_OFFBOARD_NEW_STATE_EVIDENCED_ACK_UNATTRIBUTED";
        }
      } else if (now - entered_ >= c_.command_timeout) {
        fault(mode.accepted ? "OFFBOARD_ACK_ACCEPTED_STATUS_TIMEOUT" : "OFFBOARD_ACK_TIMEOUT");
        return {};
      }
    } else if (phase_ == Phase::OffboardReady) {
      // OffboardReady只是准备完成，哪怕配置允许动力，本分支也保持默认精确零推力。
      // 只有另一次获准的takeoff/arm_only请求或独立RC准入才能推进动力任务。
    } else if (phase_ == Phase::RequestArm) {
      // Arm只计划一次，ACK和物理状态分别观察；取消/迟到armed反馈转受监督恢复，不盲重试400。
      if (!advance_requested_arm(s, now, arm, elapsed, out))
        return {};
    } else if (phase_ == Phase::ArmedGround) {
      // 人工RC或程序路线汇合后，按同帧 armed/Offboard/着地/高度/速度证据决定动力前观察。
      // 位掩码保留“同一触发帧哪些条件同时失败”：1未解锁、2非Offboard、4未着地、
      // 8高度超差、16垂速超差。| 是按位或，便于报告一次故障中的多个事实，不是互斥原因。
      const std::uint32_t failed_mask = (!s.armed ? 1U : 0U) | (!s.offboard ? 2U : 0U) |
                                        (!s.landed ? 4U : 0U) |
                                        (std::abs(out.height) > c_.height_tolerance ? 8U : 0U) |
                                        (std::abs(out.speed) > c_.ground_vz_tolerance ? 16U : 0U);
      if (failed_mask) {
        // 立刻保存导致本次拒绝的精确输入；如果等step后重新取样，
        // 新的健康帧可能把不可逆Safety的真实触发事实掩盖。
        armed_ground_fault_evidence_ = {true,
                                        failed_mask,
                                        s.armed,
                                        s.offboard,
                                        s.landed,
                                        z0_,
                                        s.z,
                                        s.vz,
                                        out.height,
                                        out.speed,
                                        c_.height_tolerance,
                                        c_.ground_vz_tolerance,
                                        s.status_stamp,
                                        s.land_stamp,
                                        s.local_stamp};
        fault_class_ = FaultClass::UnknownAirborne;
        fault("ARMED_GROUND_NOT_STABLE");
        return {};
      }
      // 已有bench观察表明PX4解锁后可能立即报告at_rest=false，故它不在上面的五项拒绝中。
      // dwell_land不是单纯等armed_dwell_seconds：只在严格新的land源戳到来时完成，
      // 期间坏条件/长间隔会清窗；缓存反复读到landed=true不会自动获得足够证据。
      if (dwell_land(s.land_stamp > 0, now, c_.armed_dwell_seconds, s.land_stamp)) {
        land_baseline_ = s.land_stamp;
        enter(Phase::ThrustRamp, now);
      }
    } else if (phase_ == Phase::BenchArmedGround) {
      // 拆桨bench永远零外部推力，不进入ThrustRamp；着陆检测新样本决定何时转上锁观察。
      if (now - flight_since_ > std::min(8.0, c_.max_flight_seconds)) {
        // 1Hz着陆检测不更新不能被解释为允许上锁；缺证据时仍保持受保护零流并要求人工处理。
        // 本次任务绝对时限仍会请求原安全恢复路径，但不会省略新地面证据。
        reason_ = "BENCH_TIMEOUT_ZERO_THRUST_OPERATOR_DISARM_REQUIRED";
      }
      // bench等待新的真实land源戳，外部推力始终精确零；
      // 再久的等待或一次LAND请求也不允许转入ThrustRamp。
      const auto failed_mask = bench_ground_failed_mask(c_, s, z0_);
      if (failed_mask) {
        capture_bench_ground_fault(s);
        fault_class_ = FaultClass::UnknownAirborne;
        fault("BENCH_ARMED_GROUND_NOT_STABLE");
        return {};
      }
      if (dwell_land(s.land_stamp > 0, now, c_.armed_dwell_seconds, s.land_stamp)) {
        land_baseline_ = s.land_stamp;
        enter(Phase::PoweredGround, now);
        reason_ = "BENCH_ARM_ONLY_GROUND_OBSERVED_NO_RAMP";
      }
    } else if (phase_ == Phase::ThrustRamp) {
      // 尚未证明离地时有绝对爬升期限；超时不再加速，按地面候选或下降路径受限减推力。
      if (!advance_thrust_ramp(s, now, elapsed, out))
        return {};
    } else if (phase_ == Phase::Ascend || phase_ == Phase::HeightHold || phase_ == Phase::Descent ||
               phase_ == Phase::RecoveryDescent || phase_ == Phase::Px4LandPending) {
      // 上升/保持与下降/恢复共用同一垂向闭环；descending只决定目标轨迹向上或向下推进。
      // 这是可修改的本地 bool：同帧出现超时或着陆检测矛盾时，会从上升切为下降计算。
      // 下方不能改成只在函数入口拍一次阶段快照，否则会错过同帧改写的 phase_。
      bool descending = phase_ == Phase::Descent || phase_ == Phase::RecoveryDescent ||
                        phase_ == Phase::Px4LandPending;
      // 已飞行却报着地先撤销导航，继续受监督下降并重新采触地证据；不能据一帧立即断动力。
      if (!descending && s.landed) {
        // 检测器在飞行阶段报着地只会撤销导航，不足以立即去掉动力。
        // 继续受保护的垂向控制，重新累计正常的严格新触地窗口。
        visual_.stop();
        visual_result_.active = false;
        ground_abort_ = false;
        // 此处捕获“当前实际高度”作为下降轨迹起点；target_ 是任务目标，不是当前高度。
        command_height_ = std::max(0., out.height);
        land_baseline_ = s.land_stamp;
        fault_class_ = FaultClass::ControlledDescent;
        enter(Phase::Descent, now);
        reason_ = "FLIGHT_DETECTOR_CONTRADICTION_NAVIGATION_REVOKED_SUPERVISED_DESCENT";
        descending = true;
      }
      // 起飞超时意味着未完成导航，不意味着可遗弃仍armed+Offboard的飞机；沿原受控恢复。
      if (phase_ == Phase::Ascend && now - entered_ >= c_.ascent_timeout) {
        // 上升未完成不是离开仍armed+Offboard飞机的许可。
        // 本阶段转受监督恢复，之后最终发布门仍要再检查；
        // 这一步不宣称已经降落，也不宣称PX4已接管。
        const auto recovery = descend(s, now);
        if (!recovery.empty()) {
          fault_class_ = FaultClass::UnknownAirborne;
          fault(recovery);
          return {};
        }
        reason_ = "ASCENT_TIMEOUT_SUPERVISED_RECOVERY";
        if (phase_ == Phase::PoweredGround) {
          if (!apply(vertical_->ramp(0, elapsed), out, now))
            return {};
          out.target_height = command_height_;
          return out;
        }
        descending = true;
      }
      // 普通下降超时后仍继续闭环，单独建立恢复期限；已有 deadline 不会被后续请求续期。
      // std::min 只撤掉高于现高度的旧上升轨迹，不重置真实推力 slew 或积分状态。
      if (phase_ == Phase::Descent && now - entered_ >= c_.descent_timeout) {
        if (recovery_deadline_ < 0)
          recovery_deadline_ = now + c_.recovery_seconds;
        recovery_entry_height_ = out.height;
        recovery_last_local_stamp_ = s.local_stamp;
        command_height_ = std::min(command_height_, std::max(0., out.height));
        enter(Phase::RecoveryDescent, now);
        fault_class_ = FaultClass::ControlledDescent;
        reason_ = "DESCENT_TIMEOUT_CONTROLLED_RECOVERY_ACTIVE_HANDOFF_UNVERIFIED";
      }
      if (phase_ == Phase::RecoveryDescent || phase_ == Phase::Px4LandPending) {
        // 只有独立新 local 样本才确认恢复进展；重复读取相同高度不能算一次新下降证据。
        if (s.local_stamp > recovery_last_local_stamp_) {
          recovery_last_local_stamp_ = s.local_stamp;
          if (out.height < recovery_entry_height_ - c_.height_tolerance)
            recovery_progress_observed_ = true;
        }
        // 撤掉残留上升轨迹；不重置推力slew状态，也不降低着地检测/高度/速度证据门。
        command_height_ = std::min(command_height_, std::max(0., out.height));
        // Land后备方案需显式验证配置、可用位置/模式等门全过；不是超时就盲发21。
        // attempted 在计划当帧置位，失败不自动重试；本地提交和真实模式转移分别在别处确认。
        if (phase_ == Phase::RecoveryDescent && !px4_land_attempted_ && land_fallback_ready(s)) {
          px4_land_attempted_ = true;
          enter(Phase::Px4LandPending, now);
          handoff_status_baseline_ = s.status_stamp;
          out.command = Command::Px4Land;
          command_sent_ = true;
          reason_ = "PX4_LAND_REQUEST_PLANNED_CONTINUOUS_GUARDED_RECOVERY";
        } else if (phase_ == Phase::Px4LandPending && (px4_land.rejected || px4_land.timed_out ||
                                                       now - entered_ >= c_.px4_land_ack_seconds)) {
          // ACK拒绝或缺失不等于要停掉健康闭环，也不授权重发21/176/400。
          // 返回恢复下降，保留原本那一个绝对完成期限。
          enter(Phase::RecoveryDescent, now);
          reason_ = "PX4_LAND_ACK_OR_MODE_UNRESOLVED_NO_RETRY_GUARDED_RECOVERY";
        }
      }
      // 视觉开关与离地/定高/权限全部成立才调用已有外环；新速度诊断并未自动连接到这里。
      // && 短路保证视觉关闭时完全跳过本外环；当前无视觉1.5m路线不依赖视觉消息。
      // visual_.step 同时验证输入和推进外环目标；其结果仍只是给原高度/姿态链的计划。
      if (navigation_allowed() && c_.visual.enabled) {
        visual_result_ = visual_.step(s.visual_input, now, out.height, yaw_reference(), elapsed);
        if (visual_result_.request_land) {
          visual_land(s, now, visual_result_.reason);
          descending = true;
          out.q = level_target();
          if (phase_ == Phase::Safety)
            return {};
        } else if (visual_result_.active) {
          target_ = visual_result_.target_height;
          out.q = visual_result_.q;
          out.visual_motion = true;
          visual_plan_at_ = now;
          reason_ = "VISUAL_" + visual_result_.reason;
        }
      }
      // 同一闭环处理轨迹前馈、速度反馈和推力限幅；失败直接保留原fault，不补发默认零推力。
      if (!height_feedback(s, out, now, elapsed, descending))
        return {};
      // 只有轨迹到目标且实测高度/向上速度连续稳定，才HeightHold，随后才允许导航。
      // dwell 要求连续满足：轨迹已到目标、当前不报着地、实测高度误差和速度均足够小。
      // 目标值到了1.5m不能直接当成实测飞机已到1.5m；settle计时中断就重新累计。
      if (phase_ == Phase::Ascend && liftoff_evidenced_ &&
          dwell(command_height_ >= target_ && !s.landed &&
                    std::abs(out.height - target_) <= c_.height_tolerance &&
                    std::abs(out.speed) <= c_.speed_tolerance,
                now, c_.settle_seconds))
        enter(Phase::HeightHold, now);
      // 触地需新的land样本及小高度/小速度连续成立；at_rest留到减推力后再要求，避免电机振动造成循环依赖。
      if (descending && dwell_land(s.landed && s.land_stamp > land_baseline_ &&
                                       std::abs(out.height) <= c_.height_tolerance &&
                                       std::abs(out.speed) <= c_.speed_tolerance,
                                   now, c_.touchdown_dwell_seconds, s.land_stamp)) {
        // 带动力触地时at_rest可能仍为false，待有界减推力后再要求静止，避免形成循环依赖。
        // at_rest也不能被借用为离地证明。
        touchdown_baseline_ = s.land_stamp;
        enter(Phase::Landed, now);
      }
    } else if (phase_ == Phase::PoweredGround) {
      // 仍带动力的地面候选不是终端Safety：按slew降至零并获取新的触地/静止证据，反弹则回下降。
      if (bench_arm_only_active_) {
        // 零推力Arm-only bench已经着地但可能仍有振动，不能照搬正常落地后的at_rest门。
        // 专用路径要求至少ground_seconds（配置至少2秒）的新检测器停留，
        // 并见到Arm之后的新status/local；本分支不运行垂向控制器，也不产生非零命令。
        if (!bench_ground_observation(c_, s, z0_)) {
          capture_bench_ground_fault(s);
          fault_class_ = FaultClass::UnknownAirborne;
          fault("BENCH_GROUND_OBSERVATION_LOST_OPERATOR_INTERVENTION_REQUIRED");
          return {};
        }
        // bench 上锁前重新证明地面，三个来源各自必须晚于本次 Arm 捕获的水位。
        // std::max 选择两个 land 因果下界中更晚的一个，不能借用较早任务的着陆样本。
        const bool new_ground = arm_command_committed_ && arm_confirmed_ && !liftoff_evidenced_ &&
                                s.armed && s.status_stamp > bench_arm_status_baseline_ &&
                                s.local_stamp > bench_arm_local_baseline_ &&
                                s.land_stamp > std::max(land_baseline_, bench_arm_land_baseline_);
        if (dwell_bench_ground(new_ground, s, now)) {
          // enter 会清 land dwell，所以先把首个证据戳复制到局部 first，再写下一阶段基线。
          const auto first = land_dwell_first_stamp_;
          bench_disarm_status_floor_ = s.status_stamp;
          bench_disarm_local_floor_ = s.local_stamp;
          bench_disarm_land_floor_ = s.land_stamp;
          bench_disarm_proven_ = ground_confirmed_ = true;
          enter(Phase::RequestDisarm, now);
          touchdown_baseline_ = first;
          status_baseline_ = s.status_stamp;
          out.command = Command::Disarm;
          command_sent_ = disarm_in_flight_ = true;
          reason_ = "BENCH_NEW_GROUND_DWELL_DISARM_REQUESTED";
        } else
          reason_ = bench_wait_overdue(now) ? "BENCH_TIMEOUT_ZERO_THRUST_OPERATOR_DISARM_REQUIRED"
                                            : "BENCH_WAIT_NEW_GROUND_DWELL_NO_AT_REST_REQUIREMENT";
      } else {
        // 在新地面证明齐备前，这个阶段仍承担armed时减动力的责任，
        // 它不是“进了Safety即可不再处理”的终态。
        if (!s.landed || std::abs(out.height) > c_.height_tolerance ||
            std::abs(out.speed) > c_.speed_tolerance) {
          if (bench_arm_only_active_) {
            fault_class_ = FaultClass::UnknownAirborne;
            fault("BENCH_GROUND_OBSERVATION_LOST_OPERATOR_INTERVENTION_REQUIRED");
            return {};
          }
          ground_abort_ = false;
          fault_class_ = FaultClass::ControlledDescent;
          land_baseline_ = s.land_stamp;
          enter(Phase::Descent, now);
          command_height_ = std::max(0., out.height);
          const auto hold = vertical_->step(vertical_sample(s), command_height_, elapsed);
          if (!apply(hold, out, now))
            return {};
        } else {
          // ramp(0) 的目标是零，不表示本帧瞬间置零；它沿原 slew 每秒变化限制逐渐减推力。
          auto control = vertical_->ramp(0, elapsed);
          if (!apply(control, out, now))
            return {};
          if (dwell_land(s.at_rest && control.thrust <= 1e-9, now, c_.touchdown_dwell_seconds,
                         s.land_stamp)) {
            // 后续Landed继续新样本停留和最终发布检查，这里还不能宣称Disarm完成。
            const auto first = land_dwell_first_stamp_;
            enter(Phase::Landed, now);
            land_baseline_ = first;
          }
        }
      }
    } else if (phase_ == Phase::Landed) {
      // 已触地仍持续有界减推力；独立新地面停留后才计划一次Disarm，不靠历史landed缓存。
      if (!s.landed || std::abs(out.height) > c_.height_tolerance ||
          std::abs(out.speed) > c_.speed_tolerance) {
        ground_confirmed_ = false;
        if (bench_arm_only_active_) {
          fault_class_ = FaultClass::UnknownAirborne;
          fault("BENCH_LANDED_CONFIRMATION_LOST_OPERATOR_INTERVENTION_REQUIRED");
          return {};
        }
        if (health(s).empty() && s.armed) {
          land_baseline_ = s.land_stamp;
          enter(Phase::Descent, now);
          command_height_ = std::max(0., out.height);
          const auto hold = vertical_->step(vertical_sample(s), command_height_, elapsed);
          if (!apply(hold, out, now))
            return {};
          // 反弹转场帧就计算受限闭环，仍走同一最终发布门，不故意留一个控制空帧。
        } else {
          fault_class_ = FaultClass::UnknownAirborne;
          fault("LANDED_CONFIRMATION_LOST");
          return {};
        }
      } else {
        auto control = vertical_->ramp(0, elapsed);
        if (!apply(control, out, now))
          return {};
        const bool confirmed = dwell_land(s.at_rest && s.land_stamp > land_baseline_ &&
                                              std::abs(out.height) <= c_.height_tolerance &&
                                              std::abs(out.speed) <= c_.speed_tolerance,
                                          now, c_.landing_seconds, s.land_stamp);
        if (confirmed && control.thrust <= 1e-9) {
          if (disarm_in_flight_) {
            // 这个标志表示已有上锁意图/事务仍待结果；已过 sink 的400不能被反弹撤回。
            // 不允许重新落地后盲发第二条400，因为迟到ACK仍可能属于前一条。
            // 保持流并等待原上锁事务结果及独立新上锁状态，不撤回、不盲重发旧Disarm。
            if (disarm_terminal(arm) && !s.armed && s.status_stamp > status_baseline_ &&
                s.land_stamp > disarm_land_commit_stamp_) {
              disarm_in_flight_ = false;
              enter(Phase::SafeDisarmed, now);
              flight_started_ = false;
            } else if (arm.rejected && s.armed && !disarm_command_committed_) {
              disarm_in_flight_ = false;
              land_baseline_ = s.land_stamp;
              enter(Phase::PoweredGround, now);
              reason_ = "POST_BOUNCE_DISARM_REJECTED_REACQUIRE_GROUND_EVIDENCE";
            } else if (arm.ambiguous || arm.timed_out ||
                       (arm.rejected && disarm_command_committed_)) {
              disarm_outcome_uncertain_ = true;
              reason_ = arm.ambiguous ? "AMBIGUOUS_400_ACK_POST_BOUNCE_NO_RETRY"
                                      : "POST_BOUNCE_DISARM_OUTCOME_UNCERTAIN_NO_BLIND_RETRY";
            } else
              reason_ = "PREVIOUS_DISARM_SUBMITTED_AWAIT_ACK_AND_STATE";
          } else if (!disarm_outcome_uncertain_) {
            // 同样先复制窗口证据再 enter；进入阶段会清局部计时和 command_sent_。
            const auto first = land_dwell_first_stamp_;
            ground_confirmed_ = true;
            enter(Phase::RequestDisarm, now);
            touchdown_baseline_ = first;
            status_baseline_ = s.status_stamp;
            out.command = Command::Disarm;
            command_sent_ = true;
            disarm_in_flight_ = true; // 本处仅计划；Runtime另外记录最终发布出口是否真实提交。
          } else
            reason_ = "DISARM_TIMEOUT_NO_BLIND_RETRY_EXTERNAL_REVIEW_REQUIRED";
        }
      }
    } else if (phase_ == Phase::RequestDisarm) {
      // 400上锁一旦提交便不可撤回；地面反弹仍保持健康下降托管，ACK不明时禁止盲重试。
      if (bench_arm_only_active_ && bench_disarm_proven_) {
        // 一旦看到本次上锁后的新 disarmed 状态，就改用上锁后地面合同；
        // 尚未看到时仍按 bench armed 合同检查，不能用 !armed 默认值补出触地证明。
        const bool ground = bench_disarmed_seen_ && !s.armed
                                ? bench_disarmed_ground_observation(c_, s)
                                : bench_ground_observation(c_, s, z0_);
        if (!ground) {
          capture_bench_ground_fault(s);
          fault_class_ = FaultClass::UnknownAirborne;
          fault("BENCH_DISARM_GROUND_BOUNCE_OPERATOR_INTERVENTION_REQUIRED");
          return {};
        }
        if (!s.armed) {
          // Arm/Disarm复用400，终结需要本次Disarm提交、独立新上锁状态及静止地面停留。
          // 电机已经上锁之后，at_rest仍是必须条件。
          if (dwell_bench_ground(disarm_command_committed_ && disarm_terminal(arm) &&
                                     verified_ground(s) && s.status_stamp > status_baseline_ &&
                                     s.land_stamp > disarm_land_commit_stamp_,
                                 s, now)) {
            disarm_in_flight_ = false;
            enter(Phase::SafeDisarmed, now);
            flight_started_ = false;
            reason_ = arm.accepted ? "BENCH_DISARM_ACK_AND_NEW_REST_GROUND_DWELL"
                                   : "BENCH_DISARM_NEW_REST_GROUND_DWELL_ACK_UNATTRIBUTED";
            return {};
          }
          reason_ = "BENCH_DISARMED_WAIT_NEW_REST_GROUND_DWELL";
        } else if (arm.ambiguous || arm.rejected || arm.timed_out ||
                   now - entered_ >= c_.command_timeout) {
          disarm_outcome_uncertain_ = true;
          reason_ = "BENCH_DISARM_OUTCOME_UNCERTAIN_NO_BLIND_RETRY";
        } else
          reason_ = "BENCH_DISARM_WAIT_NEW_PX4_DISARMED_STATE";
      } else {
        // 事务终态 + 新上锁状态 + 晚于实际Disarm提交的land样本 + 当前地面合同必须同时成立。
        // disarm_terminal 内部允许归属ACK或已提交后的独立状态证据，复用400的歧义由Runtime处理。
        if (disarm_terminal(arm) && !s.armed && s.status_stamp > status_baseline_ &&
            s.land_stamp > disarm_land_commit_stamp_ && verified_ground(s)) {
          disarm_in_flight_ = false;
          enter(Phase::SafeDisarmed, now);
          flight_started_ = false;
          reason_ = arm.accepted ? "DISARM_ACK_AND_NEW_DISARMED_LANDED_STATUS"
                                 : "DISARMED_NEW_SAFE_GROUND_STATE_ACK_UNATTRIBUTED";
          return {};
        }
        if (!ground_confirmed_ || !s.landed || !s.at_rest || s.land_stamp <= touchdown_baseline_ ||
            std::abs(out.height) > c_.height_tolerance ||
            std::abs(out.speed) > c_.speed_tolerance) {
          ground_confirmed_ = false;
          if (!s.armed) {
            fault_class_ = FaultClass::UnknownAirborne;
            fault("DISARMED_BUT_GROUND_LOST_EXTERNAL_FAILSAFE_REQUIRED");
            return {};
          }
          // 400已提交后无法撤回；反弹不会取消它，也不能让健康armed+Offboard失去托管。
          // 同帧改用原有界垂向下降，继续走最终发布检查。
          if (bench_arm_only_active_) {
            fault_class_ = FaultClass::UnknownAirborne;
            fault("BENCH_DISARM_GROUND_BOUNCE_OPERATOR_INTERVENTION_REQUIRED");
            return {};
          }
          fault_class_ = FaultClass::ControlledDescent;
          land_baseline_ = s.land_stamp;
          enter(Phase::Descent, now);
          command_height_ = std::max(0., out.height);
          if (!apply(vertical_->step(vertical_sample(s), command_height_, elapsed), out, now))
            return {};
          reason_ = "DISARM_SUBMITTED_GROUND_BOUNCE_SUPERVISED_DESCENT";
        } else if (arm.rejected && !disarm_command_committed_) {
          disarm_in_flight_ = false;
          land_baseline_ = s.land_stamp;
          enter(Phase::PoweredGround, now);
          if (!apply(vertical_->ramp(0, elapsed), out, now))
            return {};
          reason_ = "DISARM_REJECTED_NEW_GROUND_PROOF_REQUIRED";
        } else if (arm.ambiguous || arm.rejected || arm.timed_out ||
                   now - entered_ >= c_.command_timeout) {
          disarm_outcome_uncertain_ = true;
          reason_ = arm.ambiguous ? "AMBIGUOUS_400_ACK_HOLD_GUARDED_OUTPUT_NO_RETRY"
                                  : "DISARM_ACK_OR_STATUS_UNCERTAIN_HOLD_GUARDED_OUTPUT";
        }
      }
    } else if (phase_ == Phase::SafeDisarmed) {
      // 独立上锁且安全地面确认后才进入地面模式释放，不能把发出Disarm当成完成。
      if (!verified_ground(s) || s.armed) {
        fault("SAFE_DISARMED_GROUND_LOST");
        return {};
      }
      enter(Phase::ReleaseOffboard, now);
      // 下方是独立 if，故本帧即可开始安全地面模式释放；这里故意没有等下个20Hz周期。
    }
    // 仍Offboard就保持精确零流直到新模式反馈；已经离开Offboard则不再用心跳干扰PX4。
    if (phase_ == Phase::ReleaseOffboard) {
      // 地面模式释放未完成且PX4仍Offboard时保持精确零流；
      // 已观察到新模式后，就不应仅因还在等ACK继续发Offboard心跳。
      // 已离开Offboard时publish=false，外层仍可按独立动作最终门处理释放事务。
      // “动作计划”和“本轮是否还需要配套姿态心跳”不是同一个开关。
      out.publish = s.offboard;
      if (s.armed || !s.landed || !s.at_rest || !s.status_fresh || !s.land_fresh) {
        fault("RELEASE_REQUIRES_DISARMED_GROUND");
        return {};
      }
      if (!s.offboard) {
        if (!command_sent_ && s.stabilized_selected) {
          enter(Phase::Idle, now);
          return {};
        }
        if (command_sent_ && mode.accepted && s.stabilized_selected &&
            s.status_stamp > status_baseline_) {
          enter(Phase::Idle, now);
          reason_ = "RELEASE_ACK_AND_NEW_STABILIZED_STATUS";
          return {};
        }
        if (ground_mode_state_only_allowed() && release_command_committed_ &&
            s.source_clock_validated && s.stabilized_selected &&
            s.status_stamp > status_baseline_) {
          enter(Phase::Idle, now);
          reason_ = "ZERO_ONLY_RELEASE_NEW_STATE_EVIDENCED_ACK_UNATTRIBUTED";
          return {};
        }
        // 操作者可能已切入Altitude等PX4模式；未开始动力任务的准备若见到严格新非Offboard状态，
        // 不再多发一条176。这个例外不能用于已经开始的动力任务。
        if (!flight_started_ && !arm_confirmed_ && s.status_fresh && s.landed && s.at_rest &&
            s.status_stamp > status_baseline_) {
          enter(Phase::Idle, now);
          reason_ = "EXTERNAL_DISARMED_GROUND_MODE_OBSERVED_NO_RELEASE_176";
          return {};
        }
      }
      if (!command_sent_) {
        if (!s.stabilized_selectable) {
          fault("STABILIZED_MODE_NOT_AVAILABLE_FOR_RELEASE");
          return {};
        }
        command_sent_ = true;
        status_baseline_ = s.status_stamp;
        out.command = Command::ReleaseOffboard;
      } else if (mode.rejected || mode.timed_out || now - entered_ >= c_.command_timeout) {
        fault("RELEASE_MODE_ACK_OR_STATUS_TIMEOUT");
        return {};
      }
    }
    // 返回前逐轴拒绝非有限推力、bench非零推力及无效四元数；这里不替错误计划补零。
    // 这些校验通过后，最终sink仍会再核对当前上下文。
    for (double value : out.thrust)
      if (!std::isfinite(value)) {
        fault("NONFINITE_OUTPUT");
        return {};
      }
    // 再次用原double值作精确零检查；不是“很小就算零”，也不是转为float后被舍入为零。
    if (bench_arm_only_active_ &&
        (out.thrust[0] != 0 || out.thrust[1] != 0 || out.thrust[2] != 0)) {
      fault_class_ = FaultClass::UnknownAirborne;
      fault("BENCH_NONZERO_THRUST_FORBIDDEN");
      return {};
    }
    // normalized 返回 optional<Quaternion>：!optional 检查是否存在有效结果，
    // 这里仅核验可归一化，不用计算得到的副本覆盖原 out.q，不偷偷改变最终数值。
    if (!normalized(out.q)) {
      fault("INVALID_OUTPUT_QUATERNION");
      return {};
    }
    if (recovery_budget_exhausted_ && s.armed && s.offboard && !verified_ground(s))
      reason_ = "RECOVERY_BUDGET_EXHAUSTED_GUARDED_CUSTODY_ACTIVE_HANDOFF_UNVERIFIED";
    return out;
  }

private:
  // 推进程序 Arm 事务阶段；s 为同帧 NED/wxyz 反馈，now/elapsed 为单调秒，arm 为 Runtime 事务摘要。
  // 返回 false 对应原 step 的空计划早退，true 则继续原公共尾部；通过 out 引用保留同帧受限下降/零输出。
  // 仅写 Core 的 Arm/恢复阶段、证据及垂向状态，不发送命令；必须在公共健康/权限/参考检查之后调用。
  // ACK、严格新 armed 状态、迟到 Arm 恢复与原取消优先级完全保留，不能在 helper 返回后再处理另一阶段。
  bool advance_requested_arm(const Feedback& s, double now, const Transaction& arm, double elapsed,
                             Output& out) {
    if (!interactive_ || ping_ < 0 || now - ping_ > c_.console_timeout)
      arm_cancel_pending_ = true;
    // 只有严格晚于基线的真实PX4 armed状态才能说明已提交Arm的物理结果；ACK本身做不到。
    if (s.armed && s.status_stamp > status_baseline_) {
      if (bench_arm_only_active_ && !arm_confirmed_) {
        bench_arm_status_baseline_ = s.status_stamp;
        bench_arm_land_baseline_ = s.land_stamp;
        bench_arm_local_baseline_ = s.local_stamp;
      }
      arm_confirmed_ = true;
      // 先处理已取消/被拒/超时后的迟到armed，再考虑普通成功分支。
      // 若顺序反过来，迟到物理状态可能把已取消任务重新推进为正常起飞。
      if (arm_cancel_pending_ || arm.rejected || arm.timed_out) {
        const bool plausible_ground =
            bench_arm_only_active_ ? bench_ground_observation(c_, s, z0_)
                                   : (s.landed && std::abs(out.height) <= c_.height_tolerance &&
                                      std::abs(out.speed) <= c_.speed_tolerance);
        if (bench_arm_only_active_ && !plausible_ground) {
          fault_class_ = FaultClass::UnknownAirborne;
          fault("BENCH_LATE_ARM_GROUND_UNCONFIRMED_OPERATOR_INTERVENTION_REQUIRED");
          return false;
        }
        fault_class_ = plausible_ground ? FaultClass::GroundAbort : FaultClass::ControlledDescent;
        land_baseline_ = s.land_stamp;
        enter(plausible_ground ? Phase::PoweredGround : Phase::Descent, now);
        command_height_ = std::max(0., out.height);
        // 转场当帧仍用有界零目标减推力或闭环下降；
        // 不能对仍armed+Offboard用一个默认空计划冒充受控减推力。
        if (phase_ == Phase::PoweredGround) {
          if (!apply(vertical_->ramp(0, elapsed), out, now))
            return false;
        } else if (!apply(vertical_->step(vertical_sample(s), command_height_, elapsed), out, now))
          return false;
        reason_ = "LATE_ARM_STATUS_SUPERVISED_GROUND_RECOVERY";
      } else if (arm.accepted ||
                 (bench_arm_only_active_ && arm_command_committed_ && s.source_clock_validated) ||
                 (!bench_arm_only_active_ && arm_command_committed_ && arm.state_evidenced)) {
        // 上面的三条成功子句分别是：可归属ACK、bench的已提交+同步新状态例外、
        // 正常TAKEOFF的已提交+Runtime独立状态证据例外；均在外层已看到新armed状态后才到这里。
        // 正常TAKEOFF的窄状态例外只在最终400已提交后接受独立新外部Arm证据。
        // Runtime另外核验时钟、armed_time、解锁原因、RC关闭、唯一代数及此前无400历史。
        // 任意armed布尔值或普通“新status”不能自动推进起飞。
        enter(bench_arm_only_active_ ? Phase::BenchArmedGround : Phase::ArmedGround, now);
        if (!arm.accepted)
          reason_ = bench_arm_only_active_
                        ? "BENCH_ARM_NEW_STATE_EVIDENCED_ACK_UNATTRIBUTED"
                        : "TAKEOFF_ARM_NEW_EXTERNAL_STATE_EVIDENCED_ACK_UNATTRIBUTED";
      } else if (now - entered_ >= c_.command_timeout) {
        arm_cancel_pending_ = true;
        reason_ = "ARMED_WITHOUT_MATCHED_ACK_SUPERVISED_RECOVERY_REQUIRED";
      }
    } else if (arm_cancel_pending_ && (!arm_command_committed_ && !command_sent_)) {
      // 此分支连Arm计划都未形成；仅通过原有地面模式握手释放零推力准备。
      if (!s.armed && s.landed && s.at_rest) {
        enter(Phase::ReleaseOffboard, now);
        reason_ = "UNSENT_ARM_CANCEL_GROUND_MODE_RELEASE";
      }
    } else if (!command_sent_ && !arm_cancel_pending_) {
      // 在本阶段只形成一次Arm计划；下一次step不会再计划同一动作，是否发出由外层另记。
      command_sent_ = true;
      status_baseline_ = s.status_stamp;
      out.command = Command::Arm;
    } else if (arm.rejected && arm_cancel_pending_ && !s.armed &&
               s.status_stamp > status_baseline_ && s.landed && s.at_rest) {
      enter(Phase::ReleaseOffboard, now);
      reason_ = "ARM_REJECTED_AND_NEW_DISARMED_STATUS_GROUND_RELEASE";
    } else if (arm.rejected || arm.timed_out || now - entered_ >= c_.command_timeout) {
      arm_cancel_pending_ = true;
      // ACK超时不能证明Arm没到PX4；保持最终门保护的零目标并等待状态，绝不自动再发400。
      reason_ = arm.rejected ? "ARM_REJECTED_AWAIT_NEW_DISARMED_STATUS"
                             : "ARM_OUTCOME_UNCERTAIN_WAIT_PX4_STATUS_NO_NEW_TASK";
    }
    return true;
  }
  // 推进起飞斜坡及离地证据；s 使用原 PX4 NED/源微秒，now/elapsed 为单调秒，out 为本次已有计划。
  // 返回 false 必须让 step 立即返回空计划，true 才继续公共发布前校验；不直接发布、不把时间或推力当作离地。
  // 会写垂向积分/slew、轨迹、离地窗口和阶段；先绝对超时恢复，再在仍为 ThrustRamp 时推进原分段斜坡。
  // 保留同帧超时减推力、反馈提前制动、检测器证明与 Ascend 切换顺序，不能把这些判断合并为单次阶段快照。
  bool advance_thrust_ramp(const Feedback& s, double now, double elapsed, Output& out) {
    if (now - entered_ >= c_.ascent_timeout) {
      const bool plausible_ground = !pending_ramp_lift(s) && s.landed &&
                                    std::abs(out.height) <= c_.height_tolerance &&
                                    std::abs(out.speed) <= c_.speed_tolerance;
      fault_class_ = plausible_ground ? FaultClass::GroundAbort : FaultClass::ControlledDescent;
      land_baseline_ = s.land_stamp;
      enter(plausible_ground ? Phase::PoweredGround : Phase::Descent, now);
      reason_ = "LIFTOFF_TIMEOUT_ACTIVE_POWER_RECOVERY";
      // 超时后不再加速，连转场当帧也必须通过原slew减推力；默认零Output不是渐变减推力。
      if (phase_ == Phase::PoweredGround) {
        const auto reduce = vertical_->ramp(0, elapsed);
        if (!apply(reduce, out, now))
          return false;
      } else {
        command_height_ = std::max(0., out.height);
        const auto hold = vertical_->step(vertical_sample(s), command_height_, elapsed);
        if (!apply(hold, out, now))
          return false;
      }
    }
    if (phase_ == Phase::ThrustRamp) {
      // 开环和提前反馈共用原起飞时间斜坡上限；高度估计漂移可以触发制动，
      // 却不能使推力增加速度超过已配置斜坡。
      // 第一段按时间到hover候选，第二段增加有界reserve；时间/推力只能改变计划，不能证明离地。
      const double ramp_age = std::max(0., now - entered_);
      // 归一化推力上限沿既有分段斜坡，保持原乘加次序，避免中文化引入数值变化。
      // 可按阶段时间ramp_age手算：若hover=0.50、reserve=0.04，则t=0.5*ramp_seconds时
      // demand=0.25，t=1*ramp_seconds时为0.50，t=2*ramp_seconds时为0.54。
      // 这只是公式例子；实际输出还要经过slew/包络/提前高度制动，且这些值不证明真实悬停。
      const double ramp_demand =
          c_.vertical.hover * std::min(1., ramp_age / c_.ramp_seconds) +
          c_.takeoff_reserve_thrust *
              std::clamp((ramp_age - c_.ramp_seconds) / c_.ramp_seconds, 0., 1.);
      // 着陆检测器可能晚于真实爬升，所以高度阈值可先改变控制律，不能先认证离地。
      // 此开关一旦置位便保留；高度回落不能重新开启开环加速。
      // 健康/reset/跳变/权限和原绝对起飞时限仍全部有效。
      // 高度达到小阈值可提前接入反馈制动，但不是离地认证；锁存此控制律，回落不能重启开环加速。
      if (!ramp_feedback_active_ && out.height >= std::min(target_, c_.liftoff_height_m)) {
        ramp_feedback_active_ = true;
        // 先把轨迹接到当前高度（限在地面与目标之间），再保留闭环可用升力余量。
        // 不重置已有slew；若之后高度又低于阈值，ramp_feedback_active_仍为true。
        command_height_ = std::clamp(out.height, 0., target_);
        vertical_->retain_lift_reserve();
        event_ = "RAMP_HEIGHT_GUARD_FEEDBACK_ENGAGED";
      }
      if (ramp_feedback_active_) {
        if (!height_feedback(s, out, now, elapsed, false, ramp_demand))
          return false;
        reason_ = "RAMP_HEIGHT_GUARD_WAIT_NEW_LIFTOFF_EVIDENCE";
      } else {
        // 第一段到悬停候选，第二段加可选有界余量，避免真实离地需稍大推力时停在候选值。
        // 时间和推力本身均不证明离地；下方独立land与Z/VZ证据仍必须满足。
        auto control = vertical_->ramp(ramp_demand, elapsed);
        if (!apply(control, out, now))
          return false;
      }
      // 离地认证要求新的着陆检测和独立高度/向上速度样本；成立后保留轨迹、积分和slew进入Ascend。
      if (takeoff_evidence_.observe(c_, now, land_baseline_, s.land_stamp, s.landed,
                                    s.vertical_sample_stamp, out.height, out.speed)) {
        // 检测器待证期间可能已接入高度反馈；认证离地后保留轨迹、积分和slew连续性，不从零重启。
        out.target_height = command_height_;
        liftoff_evidenced_ = true;
        enter(Phase::Ascend, now);
      }
    }
    // 到时或到悬停推力都不是离地证明；只由上方独立证据更新liftoff_evidenced_。
    return true;
  }
  // 接受已通过恢复窗口与导数检查的样本，分别累计 status/local/land 的严格新源戳，now 为单调秒。
  // 无返回值，仅写本轮恢复证据水位、计数和最后 NED 样本；不授予恢复、不清故障。
  // 必须在 discontinuity 处理与垂向检查之后、恢复就绪计算之前；独立 DDS 源不能强迫成对推进。
  void record_rc_recovery_sources(const Feedback& s, double now) {
    // 各DDS源独立推进；两次land之间status只是正常缓存，不能因此清掉健康窗口。
    // 但整个窗口内每个来源仍必须提供足够的独立新观察。
    if (s.status_stamp > rc_recovery_last_status_)
      ++rc_recovery_status_samples_;
    if (s.local_stamp > rc_recovery_last_local_)
      ++rc_recovery_local_samples_;
    if (s.land_stamp > rc_recovery_last_land_) {
      ++rc_recovery_land_samples_;
      rc_recovery_last_land_at_ = now;
    }
    rc_recovery_last_status_ = s.status_stamp;
    rc_recovery_last_land_ = s.land_stamp;
    rc_recovery_last_local_ = s.local_stamp;
    rc_recovery_last_switch_ = s.rc_switch_sample_stamp;
    rc_recovery_last_vertical_ = s.vertical_sample_stamp;
    rc_recovery_last_z_ = s.z;
    rc_recovery_last_vz_ = s.vz;
    rc_recovery_last_deriv_ = s.z_deriv;
  }
  // 只更新恢复等待文本；参数是本轮是否重开证明及垂向窗口是否成立，返回 void。
  // rc_recovery_ready_ 和各源计数已由 observe_rc_recovery 算好，保留原首个未满足条件优先级。
  // 必须最后调用，不移动/删除真实门槛，也不能靠修改显示文本授予新任务或动力权限。
  void update_rc_recovery_reason(bool discontinuity, bool position_window_checked) {
    if (rc_recovery_ready_)
      rc_recovery_block_reason_.clear();
    else if (discontinuity)
      rc_recovery_block_reason_ = "RC_RECOVERY_NEW_PROOF_AFTER_DISCONTINUITY";
    else if (rc_recovery_status_samples_ < 2)
      rc_recovery_block_reason_ = "RC_RECOVERY_WAIT_STATUS_PROGRESS";
    else if (rc_recovery_local_samples_ < 3)
      rc_recovery_block_reason_ = "RC_RECOVERY_WAIT_LOCAL_PROGRESS";
    else if (rc_recovery_land_samples_ < 3)
      rc_recovery_block_reason_ = "RC_RECOVERY_WAIT_LAND_SAMPLES";
    else if (c_.feedback_guard.enabled && !position_window_checked)
      rc_recovery_block_reason_ = "RC_RECOVERY_WAIT_VERTICAL_POSITION_WINDOW";
    else
      rc_recovery_block_reason_ = "RC_RECOVERY_WAIT_GROUND_DWELL";
  }
  // 即使已停止输出，新的坏事件仍推进故障水位；下一地面恢复必须晚于这些观察。
  void observe_fault_watermarks(const Feedback& s) {
    // 输出停止后仍可能出现第二个故障；其因果下界要包含Safety期间新见反馈，
    // 即使该反馈刚清了恢复窗也必须保留。提高水位本身不授予恢复许可。
    last_status_stamp_ = std::max(last_status_stamp_, s.status_stamp);
    last_land_stamp_ = std::max(last_land_stamp_, s.land_stamp);
    last_local_stamp_ = std::max(last_local_stamp_, s.local_stamp);
    last_rc_switch_stamp_ = std::max(last_rc_switch_stamp_, s.rc_switch_sample_stamp);
  }
  // 恢复白名单只针对任务/测量失败；未知原因、时钟回退和发布完整性故障不能靠RC复位清除。
  static bool rc_recoverable_fault(const std::string& reason) {
    // 部分任务/测量故障可经独立安全地面证明结束；未知原因、发布完整性和时钟回退仍锁住。
    // 拨动RC开关不能修复已不可信的执行/传输域。
    return rc_preparation_retry_reason(reason) || reason == "RC_ARM_SOURCE_BEFORE_READY" ||
           reason == "RC_ARM_STATE_BEFORE_READY" || reason == "OPERATOR_TAKEOVER_LATCHED" ||
           reason == "ATTITUDE_STALE_OR_INVALID" || reason == "Z_VZ_STALE_OR_INVALID" ||
           reason == "NONFINITE_VERTICAL_TELEMETRY" ||
           reason == "DDS_STATUS_LAND_OR_FAILSAFE_STALE" ||
           reason == "DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE" || reason == "PX4_FAILSAFE" ||
           reason == "BATTERY_NOT_READY" || reason == "ACTIVE_QUATERNION_RESET" ||
           reason == "ACTIVE_VERTICAL_ESTIMATOR_RESET" || reason == "RC_ARMED_ESTIMATOR_RESET" ||
           reason == "RC_ARMED_GROUND_REFERENCE_NOT_STABLE" ||
           reason == "UNEXPLAINED_Z_VZ_ESTIMATOR_JUMP" ||
           reason == "VERTICAL_FEEDBACK_INPUT_INVALID" ||
           reason == "VERTICAL_FEEDBACK_SAMPLE_GAP" || reason == "VERTICAL_VELOCITY_INCONSISTENT" ||
           reason == "VERTICAL_POSITION_RATE_INCONSISTENT" ||
           reason == "RC_QUEUED_SAFETY_CONDITION" ||
           reason == "HEIGHT_OR_SPEED_ENVELOPE_EXCEEDED" || reason == "ARMED_GROUND_NOT_STABLE" ||
           reason == "OFFBOARD_LOST_OR_OPERATOR_MODE_CHANGED" ||
           reason == "LANDED_CONFIRMATION_LOST" || reason == "SAFE_DISARMED_GROUND_LOST" ||
           reason == "UNEXPECTED_DISARM_ACTIVE_FLIGHT" ||
           reason == "DISARMED_WHILE_GROUND_UNCONFIRMED_EXTERNAL_FAILSAFE_REQUIRED" ||
           reason == "DISARMED_BUT_GROUND_LOST_EXTERNAL_FAILSAFE_REQUIRED";
  }
  // 把当前独立地面恢复的拒绝条件逐项解释；空串才表示这一帧可加入新证明窗口。
  std::string rc_recovery_sample_block_reason(const Feedback& s) const {
    const auto unhealthy = health(s);
    if (!unhealthy.empty())
      return unhealthy;
    if (!s.source_clock_validated || !s.source_host_stamp)
      return "RC_RECOVERY_SOURCE_CLOCK_VALIDATION_REQUIRED";
    if (s.armed)
      return "RC_RECOVERY_DISARM_REQUIRED";
    if (s.offboard)
      return "RC_RECOVERY_NON_OFFBOARD_REQUIRED";
    if (s.operator_takeover)
      return "RC_RECOVERY_OPERATOR_TAKEOVER_ACTIVE";
    if (!s.landed || !s.at_rest)
      return "RC_RECOVERY_LANDED_AT_REST_REQUIRED";
    if (!s.stabilized_selected)
      return "RC_RECOVERY_STABILIZED_REQUIRED";
    if (!s.preflight || !s.battery_ready)
      return "RC_RECOVERY_BATTERY_PREFLIGHT_REQUIRED";
    if (s.stationary_seconds < c_.ground_seconds)
      return "RC_RECOVERY_STATIONARY_DWELL";
    if (std::abs(s.vz) > c_.ground_vz_tolerance)
      return "RC_RECOVERY_GROUND_SPEED_EXCEEDED";
    if (!RcNavigationStart::rc_live(s))
      return "RC_RECOVERY_FRESH_SAFE_RC_REQUIRED";
    if (!s.rc_axes_safe || !s.rc_aux_switches_safe || s.rc_sticks_moving)
      return "RC_RECOVERY_RC_AXES_OR_AUX_UNSAFE";
    if (s.rc_arm_switch != 3)
      return "RC_RECOVERY_ARM_SWITCH_OFF_REQUIRED";
    if (s.rc_mode_slot == 0 || s.rc_mode_slot > 6 || s.rc_mode_slot == c_.rc_navigation_mode_slot)
      return "RC_RECOVERY_NAVIGATION_SWITCH_OFF_REQUIRED";
    if (c_.feedback_guard.enabled && (!s.vertical_sample_stamp || !std::isfinite(s.z_deriv)))
      return "RC_RECOVERY_VERTICAL_SAMPLE_REQUIRED";
    return {};
  }
  // 只读包装与诊断同一谓词，避免“显示可恢复”与实际门槛分裂。
  bool rc_recovery_sample_safe(const Feedback& s) const {
    return rc_recovery_sample_block_reason(s).empty();
  }
  // 只清地面证明计时/计数和恢复专用导数积分；任务故障、命令历史及源故障水位保留。
  void clear_rc_recovery(std::string reason = "RC_RECOVERY_WAIT_NEW_GROUND") {
    rc_recovery_ready_ = false;
    rc_recovery_since_ = -1;
    rc_recovery_land_samples_ = rc_recovery_status_samples_ = rc_recovery_local_samples_ = 0;
    rc_recovery_last_land_ = rc_recovery_last_status_ = rc_recovery_last_local_ = 0;
    rc_recovery_last_switch_ = rc_recovery_last_vertical_ = 0;
    rc_recovery_block_reason_ = std::move(reason);
    recovery_feedback_guard_.reset();
  }
  // 故障后仅积累严格晚于故障的未解锁地面新样本；源独立推进，不要求同一回调齐发。
  void observe_rc_recovery(const Feedback& s, double now) {
    const bool observation_time_regressed = std::isfinite(now) && now < rc_recovery_observed_at_;
    if (std::isfinite(now))
      rc_recovery_observed_at_ = now;
    if (!c_.rc_navigation_enabled) {
      clear_rc_recovery("RC_RECOVERY_ROUTE_DISABLED");
      return;
    }
    if (!rc_fault_recoverable_) {
      clear_rc_recovery("RC_RECOVERY_FAULT_REQUIRES_RESTART");
      return;
    }
    if (recovery_event_blocked_) {
      clear_rc_recovery(rc_recovery_block_reason_);
      return;
    }
    if (!std::isfinite(now)) {
      clear_rc_recovery("RC_RECOVERY_INVALID_OBSERVATION_TIME");
      return;
    }
    const auto unsafe = rc_recovery_sample_block_reason(s);
    if (!unsafe.empty()) {
      clear_rc_recovery(unsafe);
      return;
    }
    if (s.status_stamp <= fault_status_baseline_) {
      clear_rc_recovery("RC_RECOVERY_WAIT_NEW_STATUS");
      return;
    }
    if (s.land_stamp <= fault_land_baseline_) {
      clear_rc_recovery("RC_RECOVERY_WAIT_NEW_LAND");
      return;
    }
    if (s.local_stamp <= fault_local_baseline_) {
      clear_rc_recovery("RC_RECOVERY_WAIT_NEW_LOCAL");
      return;
    }
    if (s.rc_switch_sample_stamp <= fault_rc_switch_baseline_) {
      clear_rc_recovery("RC_RECOVERY_WAIT_NEW_RC_SWITCHES");
      return;
    }
    // 观察期间reset/跳变会重开证明窗；新任务只能在已证明未解锁地面之后捕获自己的原点。
    // 上一任务的气压原点不能证明下一次起飞安全。
    // “不连续”与“硬故障”分开：此处只重开新地面证明窗，不清原Safety或重获授权。
    // &&保证仅已开始的窗口才比较其锚点；括号内任一回退/reset/同戳改数/长间隔都使证据作废。
    const bool discontinuity =
        rc_recovery_since_ >= 0 &&
        (observation_time_regressed || now < rc_recovery_since_ ||
         s.quat_reset != rc_recovery_quat_reset_ || s.z_reset != rc_recovery_z_reset_ ||
         s.vz_reset != rc_recovery_vz_reset_ || s.local_stamp < rc_recovery_last_local_ ||
         s.status_stamp < rc_recovery_last_status_ || s.land_stamp < rc_recovery_last_land_ ||
         s.rc_switch_sample_stamp < rc_recovery_last_switch_ ||
         s.vertical_sample_stamp < rc_recovery_last_vertical_ ||
         (s.local_stamp == rc_recovery_last_local_ &&
          (s.z != rc_recovery_last_z_ || s.vz != rc_recovery_last_vz_)) ||
         (c_.feedback_guard.enabled && s.vertical_sample_stamp == rc_recovery_last_vertical_ &&
          (s.z != rc_recovery_last_z_ || s.vz != rc_recovery_last_vz_ ||
           s.z_deriv != rc_recovery_last_deriv_)) ||
         (s.local_stamp > rc_recovery_last_local_ &&
          double(s.local_stamp - rc_recovery_last_local_) * 1e-6 > c_.local_timeout) ||
         std::abs(s.z - rc_recovery_z_) > c_.height_tolerance ||
         (s.land_stamp != rc_recovery_last_land_ &&
          now - rc_recovery_last_land_at_ >
              std::min(c_.land_sample_max_gap, c_.land_timeout * .5)));
    if (discontinuity)
      clear_rc_recovery("RC_RECOVERY_NEW_PROOF_AFTER_DISCONTINUITY");
    if (rc_recovery_since_ < 0) {
      rc_recovery_since_ = now;
      rc_recovery_z_ = s.z;
      rc_recovery_z_reset_ = s.z_reset;
      rc_recovery_vz_reset_ = s.vz_reset;
      rc_recovery_quat_reset_ = s.quat_reset;
    }
    const auto& vertical = recovery_feedback_guard_.observe(s.vertical_sample_stamp, s.z, s.vz,
                                                            s.z_deriv, c_.local_timeout);
    if (c_.feedback_guard.enabled && !vertical.healthy) {
      clear_rc_recovery(vertical.reason);
      return;
    }
    record_rc_recovery_sources(s, now);
    // 检测器新观察本身必须跨过地面停留时间；
    // 突发后冻结时，主机空跑tick不能把一小段land证据凑够完整窗口。
    // 实际land新样本必须覆盖ground_seconds，还需status≥2、local≥3、land≥3及启用时的位置积分窗口。
    // ready仅表示“可考虑重新准备”的证据齐备，rc_restart_allowed仍需当前安全门和新RC样本。
    // 1e-9秒只处理double累加误差，不减少所需独立样本，也不允许突发后一串空tick补满时间。
    rc_recovery_ready_ =
        rc_recovery_last_land_at_ - rc_recovery_since_ + 1e-9 >= c_.ground_seconds &&
        now - rc_recovery_last_land_at_ <= std::min(c_.land_sample_max_gap, c_.land_timeout * .5) &&
        rc_recovery_land_samples_ >= 3 && rc_recovery_status_samples_ >= 2 &&
        rc_recovery_local_samples_ >= 3 &&
        (!c_.feedback_guard.enabled || vertical.position_window_checked);
    update_rc_recovery_reason(discontinuity, vertical.position_window_checked);
  }
  // PX4 Land后备需外部已验证标识、有效XY/Z、可选Land及当前armed+Offboard；不能仅靠本地超时启用。
  bool land_fallback_ready(const Feedback& s) const {
    return c_.px4_land_fallback_confirmed && !c_.px4_land_evidence_id.empty() && s.local_fresh &&
           s.xy_valid && s.vxy_valid && s.z_valid && s.vz_valid && s.land_selectable &&
           s.status_fresh && s.armed && s.offboard && !s.failsafe;
  }
  // 让出控制后仍只读观察独立新上锁地面；观察成功是终态证据，不追认本地命令已成功。
  void observe_external_terminal(const Feedback& s, double now) {
    // 让出控制后仍只读观察；确认地面需故障后的新status与至少两个严格新land样本跨足时长。
    const bool ground = std::isfinite(now) && !s.time_regressed && s.status_fresh &&
                        s.status_stamp > fault_status_baseline_ && !s.armed && s.land_fresh &&
                        s.land_stamp > fault_land_baseline_ && verified_ground(s);
    const bool confirmed = dwell_land(ground, now, c_.ground_seconds, s.land_stamp);
    if (!ground)
      external_ground_disarmed_confirmed_ = false;
    if (confirmed)
      external_ground_disarmed_confirmed_ = true;
  }
  // 上锁完成可由可归属ACK或已实际提交后的独立状态证据组成；Runtime负责400歧义处理。
  bool disarm_terminal(const Transaction& transaction) const {
    return transaction.accepted || (disarm_command_committed_ && transaction.state_evidenced);
  }
  // 选择/复位控制权的公共门：活控制台、无输出、无未决事务、健康且安全地面，故障不得从该API清除。
  std::string safe_authority_ground(const Feedback& s, double now, bool outputs_active,
                                    bool pending_command, bool publication_fault) {
    if (publication_fault)
      return "AUTHORITY_PUBLICATION_FAULT_RESTART_REQUIRED";
    if (!operator_lease_live(now))
      return "LIVE_INTERACTIVE_CONSOLE_REQUIRED";
    if (outputs_active)
      return "AUTHORITY_OUTPUTS_MUST_BE_STOPPED";
    if (pending_command)
      return "AUTHORITY_PENDING_TRANSACTION";
    auto why = health(s);
    if (!why.empty())
      return why;
    if (s.kill || s.operator_takeover)
      return "OPERATOR_TAKEOVER_LATCHED";
    const auto mask = authority_ground_failed_mask(s);
    authority_ground_evidence_ = {true,           mask,         s.stationary_seconds, s.vz,
                                  s.status_stamp, s.land_stamp, s.local_stamp};
    if (mask)
      return "AUTHORITY_SAFE_GROUND_REQUIRED";
    return {};
  }
  // 统一按时间/源、唯一性、低频状态、failsafe、姿态和垂向检查；首个失败原因稳定保留用于诊断。
  std::string health(const Feedback& s) const {
    if (s.time_regressed)
      return "TELEMETRY_TIME_REGRESSION";
    if (!s.source_clock_valid)
      return "DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE";
    if (!s.exclusive)
      return "PUBLISHER_OR_TELEMETRY_CONFLICT";
    if (!s.status_fresh || !s.land_fresh || !s.failsafe_fresh)
      return "DDS_STATUS_LAND_OR_FAILSAFE_STALE";
    if (s.failsafe)
      return "PX4_FAILSAFE";
    if (s.calibration || s.hil)
      return "CALIBRATION_OR_HIL_ACTIVE";
    if (!s.attitude_fresh || !normalized(s.q))
      return "ATTITUDE_STALE_OR_INVALID";
    if (!s.local_fresh || !s.z_valid || !s.vz_valid)
      return "Z_VZ_STALE_OR_INVALID";
    if (!std::isfinite(s.z) || !std::isfinite(s.vz) || !std::isfinite(s.stationary_seconds))
      return "NONFINITE_VERTICAL_TELEMETRY";
    return {};
  }
  // 此打包函数在step已过健康及估计器门后使用，true占位不是自行生成新的硬件有效证据。
  VerticalSample vertical_sample(const Feedback& s) const {
    return {z0_, s.z, s.vz, true, true, true, s.z_reset, s.vz_reset};
  }
  // 把健康垂向结果写为FRD负z推力，并更新持续物理饱和计时；计划值不等于实际PX4采用。
  bool apply(const VerticalOutput& value, Output& output, double now) {
    if (!value.healthy) {
      fault(value.reason);
      return false;
    }
    if (value.saturated) {
      if (saturated_since_ < 0)
        saturated_since_ = now;
      if (now - saturated_since_ >= c_.saturation_seconds) {
        fault("PERSISTENT_THRUST_SATURATION");
        return false;
      }
    } else
      saturated_since_ = -1;
    // value.thrust 是0..1范围内的非负幅值；FRD机体z轴向下，向上推力因此取负号。
    // 例如 value.thrust=0.5 写成 {0,0,-0.5}；倾斜姿态时这是沿机体轴，不是世界系固定竖直力。
    output.thrust = {0, 0, -value.thrust};
    output.saturated = value.saturated;
    output.slew_limited = value.slew_limited;
    output.vertical_closed_loop = value.closed_loop;
    output.takeoff_thrust_limited = value.takeoff_limited;
    output.proportional_thrust = value.proportional_thrust;
    output.integral_thrust = value.integral_thrust;
    output.raw_thrust = value.raw_thrust;
    output.thrust_ceiling = value.thrust_ceiling;
    output.velocity_target = value.velocity_target;
    return true;
  }
  // 取消可能先于step到达，需看当前高度是否已经跨阈值，避免错把有离地迹象当可直接减为零的地面。
  bool pending_ramp_lift(const Feedback& s) const {
    // LAND/disconnect可能先于下一次step到来；也要检查请求当时的高度，
    // 不能等step锁存阈值后才承认当前已有可能离地迹象。
    return phase_ == Phase::ThrustRamp &&
           (ramp_feedback_active_ || z0_ - s.z >= std::min(target_, c_.liftoff_height_m));
  }
  // 尚未通过检测器认证的起飞和已认证飞行共用高度反馈；任务目标约束轨迹。
  // 速度闭环仍使用原动力包络，不能把起飞斜坡候选当成永久最低推力。
  // 推进向上高度轨迹：下降按max_descent，普通上升按max_climb，视觉定高可双向移动；elapsed保持原周期。
  bool height_feedback(const Feedback& s, Output& out, double now, double elapsed, bool descending,
                       double takeoff_ceiling = 1.) {
    // command_height_ 是上一帧的“平滑轨迹点”，target_ 才是任务最终目标；都以地面向上m计。
    // 三目表达式三条路：下降朝0每秒减max_descent；视觉HeightHold可上下调轨迹；
    // 普通上升每秒加max_climb但不超过target。乘elapsed把m/s换成此帧可走的m。
    const double previous_height_target = command_height_;
    command_height_ =
        descending ? std::max(0., command_height_ - c_.vertical.max_descent * elapsed)
                   : (c_.visual.enabled && phase_ == Phase::HeightHold
                          ? command_height_ + std::clamp(target_ - command_height_,
                                                         -c_.vertical.max_descent * elapsed,
                                                         c_.vertical.max_climb * elapsed)
                          : std::min(target_, command_height_ + c_.vertical.max_climb * elapsed));
    // 轨迹前馈为本轮目标高度变化/elapsed，单位向上m/s；接近终点时另按剩余距离限制前馈。
    // 除elapsed得到本帧轨迹速度；elapsed是否有效仍由原控制门和垂向控制器判断，
    // 此处不补一个任意最小dt来悄悄改变积分/拒绝行为。
    const double trajectory_velocity = (command_height_ - previous_height_target) / elapsed;
    const double approach_speed =
        c_.vertical.height_gain * (descending ? command_height_
                                   : c_.visual.enabled && phase_ == Phase::HeightHold
                                       ? std::abs(target_ - command_height_)
                                       : target_ - command_height_);
    // approach_speed = 高度增益(1/s) × 剩余距离(m)，越接近终点前馈越小。
    // clamp把轨迹前馈限制在此对称速度区间，真正的推力仍由下面速度反馈、积分和限制计算。
    const double velocity_feedforward =
        std::clamp(trajectory_velocity, -approach_speed, approach_speed);
    // -> 通过Core独占的指针调用控制器；control按值保存这次计算结果，包含健康位与诊断。
    // 调用可能推进内部积分/slew；输入校验早退不完成闭环推进，apply失败也不自动回滚已改状态。
    const auto control = vertical_->step(vertical_sample(s), command_height_, elapsed,
                                         velocity_feedforward, takeoff_ceiling);
    if (!apply(control, out, now))
      return false;
    out.target_height = command_height_;
    return true;
  }
  // 新任务清提前反馈标志和离地认证，不能继承上一任务的liftoff证据。
  void clear_ramp_feedback() {
    ramp_feedback_active_ = false;
    takeoff_evidence_.reset();
  }
  // 地面证明同时要求新鲜状态、着地、静止及同一z0附近高度/速度；bench仅在其专用分支采用漂移宽容。
  bool verified_ground(const Feedback& s) const {
    return s.status_fresh && s.land_fresh && s.local_fresh && s.z_valid && s.vz_valid && s.landed &&
           s.at_rest && z_set_ &&
           (bench_arm_only_active_
                ? ((!s.armed && (bench_disarmed_seen_ || external_bench_disarm_pending_))
                       ? bench_disarmed_ground_observation(c_, s)
                       : bench_vertical_observation(c_, s, z0_))
                : (std::isfinite(s.vz) && std::abs(s.vz) <= c_.speed_tolerance &&
                   std::isfinite(s.z) && std::abs(z0_ - s.z) <= c_.height_tolerance));
  }
  // 在触发帧保存bench高度与速度门的精确失败项，用日志不能稍后重采样推断。
  void capture_bench_ground_fault(const Feedback& s) {
    armed_ground_fault_evidence_ = {true,
                                    bench_ground_failed_mask(c_, s, z0_),
                                    s.armed,
                                    s.offboard,
                                    s.landed,
                                    z0_,
                                    s.z,
                                    s.vz,
                                    z0_ - s.z,
                                    -s.vz,
                                    std::min(c_.height_tolerance, c_.liftoff_height_m),
                                    c_.bench_ground_vz_tolerance_mps,
                                    s.status_stamp,
                                    s.land_stamp,
                                    s.local_stamp,
                                    c_.bench_ground_down_tolerance_m};
  }
  // 普通连续条件计时：一旦条件失败就从头；不能把间断的稳定时间加总。
  bool dwell(bool condition, double now, double duration) {
    if (!condition) {
      dwell_since_ = -1;
      return false;
    }
    if (dwell_since_ < 0)
      dwell_since_ = now;
    return now - dwell_since_ >= duration;
  }
  // 每个新land观察还需status/local各自推进；重复读取1Hz land不增加计数也不破坏正常窗口。
  bool dwell_bench_ground(bool condition, const Feedback& s, double now) {
    const bool new_land = s.land_stamp != land_dwell_last_stamp_;
    // status按既有超时可能2.5秒仍名义新鲜；但每次新land还要见status/local各自推进。
    // 20Hz重复读取有效1Hz land不该清窗，也不能当成多次独立样本。
    if (new_land && land_dwell_last_stamp_ &&
        (s.status_stamp <= bench_dwell_status_last_ || s.local_stamp <= bench_dwell_local_last_))
      condition = false;
    const bool confirmed = dwell_land(condition, now, c_.ground_seconds, s.land_stamp);
    if (new_land && land_dwell_last_stamp_ == s.land_stamp && land_dwell_first_stamp_) {
      bench_dwell_status_last_ = s.status_stamp;
      bench_dwell_local_last_ = s.local_stamp;
    }
    return confirmed;
  }
  // 20Hz读到同一着陆缓存并不等于20个独立检测器样本。
  // 只有严格新源戳能推进或完成停留；长间隔、回退、反弹重新累计。
  bool dwell_land(bool condition, double now, double duration, std::uint64_t stamp) {
    const double max_gap = std::min(c_.land_sample_max_gap, c_.land_timeout * .5);
    if (!condition || !stamp || !std::isfinite(now)) {
      // 在坏条件下出现过的检测器源戳先记入blocked水位；
      // 随后Z/VZ或landed缓存变好，也不能把这个旧样本重标为新健康观察。
      land_dwell_blocked_stamp_ = std::max(land_dwell_blocked_stamp_, stamp);
      clear_land_dwell();
      return false;
    }
    // 曾在坏条件下出现的同一检测样本永不改标为新健康样本，防缓存值变化洗掉反弹。
    if (stamp <= land_dwell_blocked_stamp_)
      return false;
    if (land_dwell_last_stamp_ && stamp < land_dwell_last_stamp_) {
      // 回退的检测器序列不能证明新的触地/离地，保留回退前水位并清窗。
      land_dwell_blocked_stamp_ = std::max(land_dwell_blocked_stamp_, land_dwell_last_stamp_);
      clear_land_dwell();
      return false;
    }
    if (stamp == land_dwell_last_stamp_) {
      // 同一源戳无论被控制循环读多少次，都不是独立新观察；
      // 尤其不能在长冻结后用旧消息重新启动一个窗口。
      return false;
    }
    if (land_dwell_last_time_ >= 0 &&
        (now < land_dwell_last_time_ || now - land_dwell_last_time_ > max_gap)) {
      clear_land_dwell(); // 只能由当前这条严格新样本重开窗口。
    }
    if (land_dwell_first_time_ < 0) {
      land_dwell_first_time_ = now;
      land_dwell_first_stamp_ = stamp;
    }
    land_dwell_last_time_ = now;
    land_dwell_last_stamp_ = stamp;
    // 20Hz的double累加在精确1Hz边界可能略低于整秒；1e-9秒只容纳数值舍入。
    // 它不会放宽至少两个不同检测器源戳的要求。
    // 左侧保证至少有首帧之后的另一帧；右侧确认两次观察之间真正跨足单调时间。
    // 例如同一stamp被读40次仍只是一帧，不能用20Hz循环次数替代1Hz检测器进度。
    return stamp > land_dwell_first_stamp_ && now - land_dwell_first_time_ + 1e-9 >= duration;
  }
  // 这是未解锁零推力Offboard诊断专用证明路径；
  // 真正Arm/TAKEOFF仍要求原连续Vz门、所有动作门和有效控制权。
  // 三动作门关闭的诊断专用地面路径；至少3个独立land跨2秒，仅允许零推力Offboard准备，不授权TAKEOFF。
  bool observe_disarmed_ground_prep(const Feedback& s, double now) {
    const bool eligible = s.status_fresh && s.land_fresh && s.local_fresh && s.z_valid &&
                          s.vz_valid && !s.armed && s.landed && s.at_rest && s.land_stamp > 0 &&
                          s.local_stamp > 0 && std::isfinite(s.z) && std::isfinite(s.vz);
    if (!eligible) {
      clear_disarmed_prep();
      return false;
    }
    auto begin = [&]() {
      prep_since_ = now;
      prep_z_ = s.z;
      prep_z_reset_ = s.z_reset;
      prep_vz_reset_ = s.vz_reset;
      prep_land_start_ = prep_land_last_ = s.land_stamp;
      prep_land_last_time_ = now;
      prep_local_start_ = s.local_stamp;
      prep_land_samples_ = 1;
    };
    if (prep_since_ < 0) {
      begin();
      return false;
    }
    if (now < prep_since_ || s.z_reset != prep_z_reset_ || s.vz_reset != prep_vz_reset_ ||
        s.land_stamp < prep_land_last_ || s.local_stamp < prep_local_start_ ||
        std::abs(s.z - prep_z_) > c_.height_tolerance ||
        now - prep_land_last_time_ > c_.land_sample_max_gap) {
      begin();
      return false;
    }
    if (s.land_stamp > prep_land_last_) {
      prep_land_last_ = s.land_stamp;
      prep_land_last_time_ = now;
      ++prep_land_samples_;
    }
    // 至少3个独立检测器样本跨越ground_seconds（配置要求至少2秒），且local也有真实推进。
    return now - prep_since_ + 1e-9 >= c_.ground_seconds && prep_land_samples_ >= 3 &&
           s.local_stamp > prep_local_start_;
  }
  // 清除诊断准备窗口，下次必须重新获取独立land/local进度。
  void clear_disarmed_prep() {
    prep_since_ = -1;
    prep_land_samples_ = 0;
  }
  // 清本阶段land停留起止；blocked水位由调用者按事件语义另行保留或重设。
  void clear_land_dwell() {
    land_dwell_first_time_ = land_dwell_last_time_ = -1;
    land_dwell_first_stamp_ = land_dwell_last_stamp_ = 0;
  }
  // 统一阶段进入：记录单调秒时刻，清阶段连续计时与本阶段命令计划位，保留全局任务证据。
  void enter(Phase phase, double now) {
    // 参数phase是本次要进入的阶段；成员phase_是整个对象的当前阶段，两者不是同一变量。
    // enter不是“新任务重置”：只清本阶段计时/命令计划，授权、ACK提交、离地/故障历史仍保留。
    phase_ = phase;
    entered_ = now;
    dwell_since_ = -1;
    clear_land_dwell();
    land_dwell_blocked_stamp_ = 0;
    command_sent_ = false;
    reason_ = name(phase);
  }
  // 锁存首次/最新原因及所有已见源因果下界；恢复白名单只能越来越保守，后续健康帧不能清除。
  void fault(const std::string& why, bool restart_required = true) {
    const bool first = phase_ != Phase::Safety;
    if (first)
      first_fault_reason_ = why;
    latest_fault_reason_ = why;
    // 首次按白名单确定可恢复性；后续用&&收紧：已经不可恢复的故障不能被温和原因重新放宽。
    rc_fault_recoverable_ =
        first ? rc_recoverable_fault(why) : (rc_fault_recoverable_ && rc_recoverable_fault(why));
    // 成员带_，参数不带_；||累计“曾要求重启”事实，本次false不能清除之前的true。
    restart_required_ = restart_required_ || restart_required;
    if (fault_class_ == FaultClass::None || fault_class_ == FaultClass::ControlledDescent) {
      fault_class_ = flight_started_ ? FaultClass::UnknownAirborne : FaultClass::GroundAbort;
    }
    ground_confirmed_ = false;
    phase_ = Phase::Safety;
    if (first)
      reason_ = why;
    fault_status_baseline_ = std::max(fault_status_baseline_, last_status_stamp_);
    fault_land_baseline_ = std::max(fault_land_baseline_, last_land_stamp_);
    fault_local_baseline_ = std::max(fault_local_baseline_, last_local_stamp_);
    fault_rc_switch_baseline_ = std::max(fault_rc_switch_baseline_, last_rc_switch_stamp_);
    clear_rc_recovery(rc_fault_recoverable_ ? "RC_RECOVERY_WAIT_NEW_GROUND"
                                            : "RC_RECOVERY_FAULT_REQUIRES_RESTART");
    clear_land_dwell();
    land_dwell_blocked_stamp_ = fault_land_baseline_;
  }
  // RC准入汇合到原ArmedGround：开关/真实解锁/真实Offboard证据全部成立后采用已校验rc_target_height，当前候选1.5m。
  Output rc_startup(const Feedback& s, double now, const std::string& queued_reason = {}) {
    Output out;
    last_status_stamp_ = s.status_stamp;
    last_land_stamp_ = s.land_stamp;
    // queued_reason为空才计算当前health；排队坏事件给出的原因不能被当前健康尾帧替掉。
    // 返回值publish只表示准入阶段可拟发零流；真正Joined要看下面独立的phase枚举。
    const bool publish =
        rc_start_.step(c_, s, now, queued_reason.empty() ? health(s) : queued_reason);
    reason_ = rc_start_.reason();
    if (rc_start_.phase() == RcStartPhase::Blocked) {
      if (reason_ == "RC_ARMED_GROUND_REFERENCE_NOT_STABLE") {
        const double height = rc_start_.z0() - s.z;
        const std::uint32_t mask =
            (!s.landed ? 4U : 0U) |
            ((!std::isfinite(height) || std::abs(height) > c_.height_tolerance) ? 8U : 0U) |
            ((!std::isfinite(s.vz) || std::abs(s.vz) > c_.ground_vz_tolerance) ? 16U : 0U);
        armed_ground_fault_evidence_ = {true,
                                        mask,
                                        s.armed,
                                        s.offboard,
                                        s.landed,
                                        rc_start_.z0(),
                                        s.z,
                                        s.vz,
                                        height,
                                        -s.vz,
                                        c_.height_tolerance,
                                        c_.ground_vz_tolerance,
                                        s.status_stamp,
                                        s.land_stamp,
                                        s.local_stamp};
      }
      fault(reason_, !rc_preparation_retry_reason(reason_));
      return out;
    }
    z_set_ = rc_start_.anchor_valid();
    if (z_set_) {
      // 继承准入机已证明的同一地面坐标/姿态锚点，而不是用这帧armed反馈重新定义地面。
      z0_ = rc_start_.z0();
      z_reset_ = rc_start_.z_reset();
      vz_reset_ = rc_start_.vz_reset();
      reference_.clear();
      reference_.observe(rc_start_.q0(), rc_start_.quat_reset(), false);
      authority_.mode = authority_.attempt_mode = authority::Mode::RcTakeover;
      authority_.locked = true;
    } else {
      reference_.clear();
      authority_ = {};
    }
    // 人工RC已解锁，无程序Arm；继承独立地面参考，重置垂向状态与离地证据后开始原推力流程。
    if (rc_start_.phase() == RcStartPhase::Joined) {
      rc_joined_ = true;
      flight_started_ = arm_confirmed_ = true;
      flight_since_ = last_ = now;
      // 任务目标来自配置（当前候选1.5m），轨迹从地面0开始；不把实测z直接设为目标高度。
      target_ = c_.rc_target_height;
      command_height_ = 0;
      switch_generation_ = s.operator_switch_generation;
      task_rc_switch_baseline_ = s.rc_switch_sample_stamp;
      offboard_observed_ = true;
      ground_confirmed_ = liftoff_evidenced_ = false;
      last_sample_set_ = true;
      last_sample_z_ = s.z;
      last_sample_vz_ = s.vz;
      clear_ramp_feedback();
      vertical_->reset(0);
      feedback_guard_.reset();
      enter(Phase::ArmedGround, now);
      reason_ = "RC_MANUAL_ARM_AND_NEW_OFFBOARD_CONFIRMED";
      out.publish = true;
      out.q = reference_.level();
      out.height = z0_ - s.z;
      return out;
    }
    switch (rc_start_.phase()) {
    case RcStartPhase::Prestream:
      phase_ = Phase::RcPrestream;
      break;
    case RcStartPhase::WaitArm:
      phase_ = Phase::RcWaitArm;
      break;
    case RcStartPhase::WaitNavigation:
      phase_ = Phase::RcWaitNavigation;
      break;
    default:
      phase_ = Phase::Idle;
      break;
    }
    out.publish = publish;
    out.q = reference_.level();
    out.height = z_set_ ? z0_ - s.z : 0;
    return out;
  }
  // 以下成员均由同一个Core对象跨step持有；{}值初始化不是“已收到真实硬件值”。
  // 先读每组用途，再回到写入它的入口：prepare/rc_startup清任务局部，enter只清阶段局部，
  // fault保留故障因果水位；Runtime还另持有跨任务ACK历史，不在本成员列表中。
  //
  // 成员组1：RC准入与最近观察；rc_joined_只说明已汇合，仍需独立liftoff与定高门。
  RcNavigationStart rc_start_;
  bool rc_joined_{}, rc_operator_live_{};
  double rc_observed_at_{};
  // 成员组2：配置/阶段/姿态参考；visual计划与已提交yaw分离，避免拟发送目标提前成为事实。
  Config c_;
  Phase phase_{Phase::Idle};
  AttitudeReference reference_;
  vision::Controller visual_;
  vision::Result visual_result_;
  // -1是“尚无计划时刻”的哨兵；committed_yaw只有真实本地pair提交回报后才更新。
  double visual_plan_at_{-1}, committed_yaw_{};
  bool committed_yaw_set_{};
  // 飞行监测与恢复监测各有独立积分状态，恢复不能继承旧飞行窗口。
  VerticalFeedbackGuard feedback_guard_;
  VerticalFeedbackGuard recovery_feedback_guard_;
  // 成员组3：故障恢复窗口、独立源计数、reset与因果水位；本周期坏事件会否决整轮恢复。
  bool rc_fault_recoverable_{}, rc_recovery_ready_{}, recovery_event_blocked_{};
  double rc_recovery_since_{-1}, rc_recovery_last_land_at_{-1}, rc_recovery_z_{},
      rc_recovery_observed_at_{};
  double rc_recovery_last_z_{}, rc_recovery_last_vz_{}, rc_recovery_last_deriv_{};
  unsigned rc_recovery_land_samples_{}, rc_recovery_status_samples_{}, rc_recovery_local_samples_{};
  std::uint8_t rc_recovery_z_reset_{}, rc_recovery_vz_reset_{}, rc_recovery_quat_reset_{};
  std::uint64_t rc_recovery_last_land_{}, rc_recovery_last_status_{}, rc_recovery_last_local_{},
      rc_recovery_last_switch_{}, rc_recovery_last_vertical_{}, fault_local_baseline_{},
      fault_rc_switch_baseline_{}, last_local_stamp_{}, last_rc_switch_stamp_{},
      task_rc_switch_baseline_{};
  std::string first_fault_reason_, latest_fault_reason_,
      rc_recovery_block_reason_{"RC_RECOVERY_NO_ENDED_FAULT"};
  // 成员组4：垂向状态和Arm/Disarm/释放事务意图；planned、committed和verified保持区分。
  std::unique_ptr<VerticalController> vertical_;
  // interactive是控制台租约；flight_started是任务托管；z_set是地面参考存在。
  // command_sent_名称沿旧协议保留，但语义仅是“本阶段已计划过命令”，enter会把它清为false。
  bool interactive_{}, flight_started_{}, bench_arm_only_active_{}, ground_abort_{}, z_set_{},
      command_sent_{};
  // cancel_pending是取消意图；committed仅由最终发布结果写入，不能与command_sent_混用。
  bool arm_cancel_pending_{}, arm_command_committed_{};
  // in_flight指上锁事务仍待处理，不是“飞机在飞”；outcome_uncertain禁止不明结果下盲重试。
  bool disarm_in_flight_{}, disarm_command_committed_{}, disarm_outcome_uncertain_{};
  bool operator_release_requested_{};
  // ground/liftoff/arm三类证明各自独立：曾证明Arm或离地，不等于当前已安全触地。
  bool ground_confirmed_{}, liftoff_evidenced_{}, last_sample_set_{}, arm_confirmed_{},
      ground_release_pending_{};
  FaultClass fault_class_{FaultClass::None};
  ArmedGroundFaultEvidence armed_ground_fault_evidence_{};
  AuthorityGroundEvidence authority_ground_evidence_{};
  bool external_bench_disarm_pending_{};
  bool bench_disarm_proven_{}, bench_disarmed_seen_{};
  std::uint64_t bench_arm_status_baseline_{}, bench_arm_land_baseline_{},
      bench_arm_local_baseline_{};
  std::uint64_t bench_dwell_status_last_{}, bench_dwell_local_last_{};
  std::uint64_t bench_disarm_status_floor_{}, bench_disarm_local_floor_{},
      bench_disarm_land_floor_{};
  bool offboard_command_committed_{}, release_command_committed_{};
  std::uint64_t external_bench_status_baseline_{}, external_bench_land_baseline_{};
  // 最近实际控制观察的NED z(m)/vz(m/s)，用于跨多个回调的累计跳变检查。
  double last_sample_z_{}, last_sample_vz_{};
  bool ramp_feedback_active_{};
  // 离地独立证据与推力爬坡计时分离；不能从hover值、耗时或计划高度推断真实离地。
  TakeoffEvidence takeoff_evidence_;
  std::uint64_t touchdown_baseline_{}, disarm_land_commit_stamp_{};
  std::uint8_t z_reset_{}, vz_reset_{};
  // baseline是进入动作/阶段时捕获的源戳下界；之后必须严格大于它，旧健康缓存不能证明新结果。
  // 各来源独立比较：status对status、land对land，不比较“哪个DDS话题数值更大”。
  std::uint64_t status_baseline_{}, land_baseline_{};
  // entered/last/ping/flight_since为单调秒；z0为NED地面m，target/command_height为向上相对m。
  // entered_每次enter更新；flight_since_只在本次任务开始捕获，所以切阶段不会续长任务总期限。
  // target_是最终高度，command_height_是按速度限制逐帧推进的轨迹点，z0_是NED绝对地面原点。
  double entered_{}, last_{}, ping_{-1}, z0_{}, target_{}, command_height_{}, flight_since_{};
  double dwell_since_{-1}, saturated_since_{-1};
  double prep_since_{-1}, prep_z_{}, prep_land_last_time_{-1};
  std::uint64_t prep_land_start_{}, prep_land_last_{}, prep_local_start_{};
  std::uint8_t prep_z_reset_{}, prep_vz_reset_{};
  unsigned prep_land_samples_{};
  // 地面检测窗口主机秒与后面的源微秒stamp分别保存；不得跨域直接比较。
  double land_dwell_first_time_{-1}, land_dwell_last_time_{-1};
  // deadline是恢复完成的绝对单调秒期限；它到期只披露超期，不证明已落地或允许放弃托管。
  double recovery_deadline_{-1}, recovery_entry_height_{};
  bool recovery_budget_exhausted_{}, px4_land_committed_{}, px4_land_attempted_{},
      px4_land_mode_observed_{}, recovery_progress_observed_{},
      external_ground_disarmed_confirmed_{};
  std::uint64_t recovery_last_local_stamp_{}, handoff_status_baseline_{}, fault_status_baseline_{},
      fault_land_baseline_{}, last_status_stamp_{}, last_land_stamp_{};
  std::uint64_t land_dwell_first_stamp_{}, land_dwell_last_stamp_{}, land_dwell_blocked_stamp_{};
  std::string reason_{"IDLE_NO_AUTOMATIC_ARM"}, event_;
  // 控制权策略和接管锁存独立于正常阶段；切换阶段不能隐式解锁权限。
  authority::State authority_;
  bool offboard_observed_{}, restart_required_{};
  std::uint64_t switch_generation_{};
  std::string takeover_reason_;
};
} // namespace fc01::attitude
