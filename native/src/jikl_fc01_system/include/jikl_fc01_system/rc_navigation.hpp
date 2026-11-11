#pragma once
// 人工RC起飞准入：先证明未解锁安全地面和零推力预发送，再配对新的解锁开关、真实PX4解锁及导航模式。
// 此对象只能给准入/零推力许可，不构造Arm命令、不计算非零推力；Joined后才由原Core按rc_target_height起飞（当前候选1.5m）。
#include "jikl_fc01_system/attitude_reference.hpp"
#include "jikl_fc01_system/vertical_feedback_guard.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <optional>

namespace fc01::attitude {
// 从上往下读：查询方法 → note_commit记录本地零流 → 两种短时零流桥接 → step阶段机 → 私有证据窗口。
// “准入”的意思是让Core有资格进入下一阶段；这里没有ROS发布者，更没有自动解锁电机的代码。
// WaitGround等地面，Prestream积累零输出，WaitArm等人工解锁，WaitNavigation等导航，Joined证据汇合，Blocked终止本轮。
enum class RcStartPhase { WaitGround, Prestream, WaitArm, WaitNavigation, Joined, Blocked };
// 仅列举可以重新准备的任务原因；不是清除任意发布、时钟或执行完整性故障的通用开关。
inline bool rc_preparation_retry_reason(const std::string& reason) {
  return reason == "RC_MANUAL_ARM_EVIDENCE_TIMEOUT" ||
         reason == "RC_NAVIGATION_SELECTION_TIMEOUT" ||
         reason == "RC_NAVIGATION_CONFIRMATION_TIMEOUT" ||
         reason == "RC_NAVIGATION_MODE_SLOT_MISMATCH" ||
         reason == "RC_NAVIGATION_BEFORE_MANUAL_ARM" ||
         reason == "RC_ARM_OR_NAVIGATION_BEFORE_READY" ||
         reason == "RC_NAVIGATION_SELECTION_WITHDRAWN" ||
         reason == "RC_NAVIGATION_MODE_WITHDRAWN" || reason == "RC_ARM_AUTHORIZATION_WITHDRAWN" ||
         reason == "RC_UNEXPECTED_MODE_BEFORE_NAVIGATION";
}
// 保存一次人工授权过程的地面锚点、输入边沿及源时间水位，避免乱序回调把旧开关当成新同意。
class RcNavigationStart {
public:
  // Xxx() const 只能查询当前对象；返回值为普通标量时复制，返回const T&时借用成员（不能跨对象寿命保存）。
  // 以下只读查询暴露准入阶段/拒绝原因及锚点；q0为wxyz姿态，z0为PX4 NED向下位置m。
  RcStartPhase phase() const {
    return phase_;
  }
  const std::string& reason() const {
    return reason_;
  }
  bool anchor_valid() const {
    return anchor_;
  }
  double z0() const {
    return z0_;
  }
  const Quaternion& q0() const {
    return q0_;
  }
  std::uint8_t z_reset() const {
    return z_reset_;
  }
  std::uint8_t vz_reset() const {
    return vz_reset_;
  }
  std::uint8_t quat_reset() const {
    return quat_reset_;
  }
  // 地面参考合格要求不同源各自推进并覆盖静止窗口，不是只看landed=true。
  bool ground_reference_qualified() const {
    return ground_qualified_;
  }
  unsigned ground_reference_samples() const {
    return ground_samples_;
  }
  double ground_reference_seconds() const {
    // ?: 表示窗口未开始时显示0秒，否则显示已观察窗口长度；这只是诊断，不额外推进计时。
    return ground_since_ < 0 ? 0 : ground_last_at_ - ground_since_;
  }
  // 接收调度提前识别相对本窗口的高度离开；即使随后回调回到原高度也不能吞掉这次变化。
  bool ground_reference_height_changed(double z, double tolerance) const {
    return ground_since_ >= 0 && std::abs(z - ground_window_z_) > tolerance;
  }
  unsigned commits() const {
    return commits_;
  }
  bool manual_arm_proven() const {
    return manual_arm_proven_;
  }
  std::uint64_t manual_armed_time() const {
    return manual_armed_time_;
  }
  // “当前开关OFF”不证明此前从未观察到解锁意图；全部历史证据均为空才能采用未解锁准备重试。
  bool no_arm_intent_observed() const {
    // &&全部成立才为真。它检查历史，不是仅检查当前CH5：曾观察过意图的对象不能走“从未解锁”窄重试。
    return !arm_intent_observed_ && !manual_arm_proven_ && !manual_armed_time_ && !arm_edge_ &&
           !navigation_edge_ && !pending_armed_time_ && arm_since_ < 0 && navigation_since_ < 0 &&
           !navigation_offboard_observed_;
  }
  // 只记录最终sink成功的零目标/心跳配对；计划数不算发送数，失败/大间隔会重启连续流证明。
  void note_commit(bool success, double now) {
    if (!success) {
      stream_first_ = stream_last_ = -1;
      commits_ = 0;
      return;
    }
    if (phase_ == RcStartPhase::WaitGround || phase_ == RcStartPhase::Blocked ||
        phase_ == RcStartPhase::Joined)
      return;
    if (stream_last_ < 0 || now < stream_last_ || now - stream_last_ > .3) {
      // 重新起算流窗口而非给旧窗口补时间；.3为本方法既有连续性界限，stream_ready还另查配置期限。
      stream_first_ = now;
      commits_ = 0;
    }
    stream_last_ = now;
    ++commits_;
  }
  // 同时要求最近提交未过期、持续时长和按loop_hz计算的最低提交量；不能靠空等主机时间READY。
  template <class C> bool stream_ready(const C& c, double now) const {
    // template让此方法接收具有这些字段的配置类型；编译时展开，不是运行时查字符串/动态分派。
    // ceil(Hz×秒×0.8)向上取整最低提交次数；流有持续时间还须有足够实际提交，不能仅延时假装READY。
    return stream_first_ >= 0 && stream_last_ >= 0 && now >= stream_last_ &&
           now - stream_last_ <= c.stall_seconds && now - stream_first_ >= c.prestream_seconds &&
           commits_ >= unsigned(std::ceil(c.loop_hz * c.prestream_seconds * .8));
  }
  // 选中且新鲜的RC源与开关时钟都有效，Kill必须明确OFF；遥控话题存在本身不等于可人工接管。
  template <class F> static bool rc_live(const F& s) {
    return s.rc_link_ready && s.rc_switches_ready && s.rc_source_selected &&
           s.rc_source_clock_valid && s.rc_switch_clock_valid && !s.kill && s.rc_kill_switch == 3;
  }
  // 桥接许可仍须在实际sink前按最新快照复查；它不是“人工解锁已完成”的证据。
  // 独立消息可在固定短窗内配对；只允许精确零输出维持流，不把部分armed或开关单帧升级为人工解锁证明。
  template <class C, class F> bool pending_zero_allowed(const C& c, const F& s, double now) const {
    // 返回bool只许可“精确零目标+心跳”这个候选，不写phase_，也不证明人工解锁完成。
    if (!anchor_ || manual_arm_proven_ || !ready_source_floor_ ||
        (phase_ != RcStartPhase::WaitArm && phase_ != RcStartPhase::WaitNavigation) ||
        !std::isfinite(now) || observed_at_ < 0 || now < observed_at_ ||
        now - observed_at_ > c.stall_seconds || !stream_ready(c, now))
      return false;
    // 过了状态配对期限或整段解锁到导航期限即撤销桥接，不因为新的半边回调重新续期。
    if (phase_ == RcStartPhase::WaitNavigation &&
        (arm_since_ < 0 || now < arm_since_ || now - arm_since_ > c.status_timeout ||
         now - arm_since_ > c.rc_arm_to_navigation_timeout))
      return false;
    // 这里两种零输出桥接均要求同步源、RC安全轴/辅助开关、电池预检、同一参考和地面包络。
    if (!s.source_clock_validated || !s.source_clock_valid || !s.source_host_stamp || !rc_live(s) ||
        !s.rc_axes_safe || s.rc_sticks_moving || !s.rc_aux_switches_safe || !s.battery_ready ||
        !s.preflight || !s.stabilized_selected || s.offboard || s.rc_mode_slot == 0 ||
        s.rc_mode_slot > 6 || s.rc_mode_slot == c.rc_navigation_mode_slot || !s.landed ||
        !std::isfinite(s.z) || !std::isfinite(s.vz) || std::abs(z0_ - s.z) > c.height_tolerance ||
        std::abs(s.vz) > c.ground_vz_tolerance || s.z_reset != z_reset_ ||
        s.vz_reset != vz_reset_ || s.quat_reset != quat_reset_ ||
        (s.rc_arm_switch != 1 && s.rc_arm_switch != 3))
      return false;
    const auto q = normalized(s.q);
    // q是optional<Quaternion>：!q先排除空值，||短路后才可用(*q)[i]取四元数分量。
    // R33=1-2(x²+y²)为机体下轴与世界下轴的夹角余弦；acos→弧度，×180/pi→度。
    if (!q || std::acos(std::clamp(1 - 2 * ((*q)[1] * (*q)[1] + (*q)[2] * (*q)[2]), -1., 1.)) *
                      180 / 3.141592653589793 >
                  c.max_tilt_deg)
      return false;
    // 解锁开关使用PX4值1=ON，3=OFF；桥接前必须确保该边沿晚于READY源水位。
    const bool arm_on = s.rc_arm_switch == 1;
    if (arm_on &&
        (s.rc_switch_sample_stamp <= ready_source_floor_ ||
         (!arm_edge_ && (last_arm_switch_ != 3 || s.rc_switch_sample_stamp <= last_switch_stamp_))))
      return false;
    // 真实解锁反馈还必须具有RC开关原因、新status及新的同源armed_time；不得混配其他解锁事件。
    if (s.armed) {
      return s.latest_arming_reason == 2 && s.status_stamp > ready_source_floor_ &&
             s.status_stamp > disarmed_status_ && s.armed_time > disarmed_armed_time_ &&
             (!pending_armed_time_ || s.armed_time == pending_armed_time_);
    }
    return arm_on;
  }
  // 已证明人工解锁后，允许模式与档位短时异步到达；仍只给零输出，并要求同一次armed_time。
  template <class C, class F>
  bool navigation_zero_allowed(const C& c, const F& s, double now) const {
    // 这一桥接比pending_zero更靠后：manual_arm_proven_必须已经为真，且armed_time绑定同一解锁事件。
    if (phase_ != RcStartPhase::WaitNavigation || !manual_arm_proven_ || !anchor_ ||
        !std::isfinite(now) || observed_at_ < 0 || now < observed_at_ ||
        now - observed_at_ > c.stall_seconds || arm_since_ < 0 || now < arm_since_ ||
        now - arm_since_ > c.rc_arm_to_navigation_timeout ||
        (navigation_since_ >= 0 &&
         (now < navigation_since_ || now - navigation_since_ > c.status_timeout)) ||
        !stream_ready(c, now))
      return false;
    if (!s.source_clock_validated || !s.source_clock_valid || !s.source_host_stamp || !rc_live(s) ||
        !s.rc_axes_safe || s.rc_sticks_moving || !s.rc_aux_switches_safe || !s.battery_ready ||
        !s.preflight || !s.armed || s.armed_time != manual_armed_time_ ||
        s.latest_arming_reason != 2 || s.rc_arm_switch != 1 || s.rc_mode_slot == 0 ||
        s.rc_mode_slot > 6 || (!s.stabilized_selected && !s.offboard) || !s.landed ||
        !std::isfinite(s.z) || !std::isfinite(s.vz) || std::abs(z0_ - s.z) > c.height_tolerance ||
        std::abs(s.vz) > c.ground_vz_tolerance || s.z_reset != z_reset_ ||
        s.vz_reset != vz_reset_ || s.quat_reset != quat_reset_)
      return false;
    const auto q = normalized(s.q);
    if (!q || std::acos(std::clamp(1 - 2 * ((*q)[1] * (*q)[1] + (*q)[2] * (*q)[2]), -1., 1.)) *
                      180 / 3.141592653589793 >
                  c.max_tilt_deg)
      return false;
    // 导航档位是否选中必须按候选槽位判断；它与PX4实际Offboard状态分别到达，需要两者都成立。
    const bool nav = s.rc_mode_slot == c.rc_navigation_mode_slot;
    // nav只是遥控档位匹配；s.offboard是实际飞控模式。两条独立消息可异步到达，不能互相替代。
    if ((navigation_edge_ && !nav) || (navigation_offboard_observed_ && !s.offboard))
      return false;
    if (nav &&
        (s.rc_switch_sample_stamp <= arm_edge_ || s.rc_switch_sample_stamp <= ready_source_floor_ ||
         (!navigation_edge_ && s.rc_switch_sample_stamp <= last_switch_stamp_)))
      return false;
    if (s.offboard && (!last_stabilized_status_ || s.status_stamp <= last_stabilized_status_ ||
                       s.status_stamp <= arm_edge_ || s.status_stamp <= ready_source_floor_ ||
                       s.offboard_signal_lost))
      return false;
    return true;
  }
  // 准入主步：c为原配置，s为同一当前快照，now为单调秒，health为上层已核对的健康拒绝原因。
  // 返回true仅表示本阶段允许拟发零输出；Joined返回false后外层切换到原ArmedGround，不在这里输出起飞推力。
  template <class C, class F>
  bool step(const C& c, const F& s, double now, const std::string& health) {
    // 主干顺序：终态早退→记历史意图→主机时间→首个健康原因→地面/预发→新边沿→解锁/导航配对。
    // 返回false有不同含义：等待/Blocked/Joined须配合phase()读，绝不能仅凭false断言故障。
    if (phase_ == RcStartPhase::Blocked || phase_ == RcStartPhase::Joined)
      return false;
    // 一旦看到解锁意图就永久记录在本轮，后续健康失败不能重置历史来绕过准入。
    if (s.armed || s.rc_arm_switch == 1)
      arm_intent_observed_ = true;
    // 主机非有限时间或回退先于任何计时更新拒绝，防止负窗口或错误累计。
    if (!std::isfinite(now) || (observed_at_ >= 0 && now < observed_at_))
      return block("RC_TIME_REGRESSION");
    observed_at_ = now;
    const bool nav = s.rc_mode_slot == c.rc_navigation_mode_slot;
    const bool arm_on = s.rc_arm_switch == 1;
    const bool intent = arm_since_ >= 0;
    // intent表示“已进入固定配对时间窗”，不同于arm_intent_observed_（任何历史解锁意图）。
    // 按原优先级补充来源、轴、安全辅助开关与预检失败；已有health原因不能被后面的正常数据覆盖。
    std::string why = health;
    // std::string是本次调用的副本，why.empty()守住首错优先级；后续检查只在尚无原因时补充。
    if (why.empty() && (!s.source_clock_validated || !s.source_host_stamp || !rc_live(s)))
      why = "RC_FRESH_SELECTED_SOURCE_REQUIRED";
    if (why.empty() && (!s.rc_axes_safe || s.rc_sticks_moving))
      why = "RC_MINIMUM_THROTTLE_CENTERED_STICKS_REQUIRED";
    if (why.empty() && !s.rc_aux_switches_safe)
      why = "RC_OTHER_MODE_SWITCH_ACTIVE";
    if (why.empty() && (!s.battery_ready || !s.preflight))
      why = "BATTERY_PREFLIGHT_REQUIRED";
    const auto q = normalized(s.q);
    // 从规范化姿态算地面倾角；人工RC地面准入沿用独立倾角门，不影响自主高度主链的实测姿态反馈。
    if (why.empty() && q) {
      const double tilt =
          std::acos(std::clamp(1 - 2 * ((*q)[1] * (*q)[1] + (*q)[2] * (*q)[2]), -1., 1.)) * 180 /
          3.141592653589793;
      if (tilt > c.max_tilt_deg)
        why = "RC_GROUND_TILT_EXCEEDED";
    }
    // 健康失败时，已进入解锁配对窗口（arm_since_>=0）或实际armed则Blocked；否则重回地面等待。
    // 仅开关首次ON还不等于已进入窗口；reset仍保留历史意图水位，不能误用旧边沿。
    if (!why.empty()) {
      if (intent || s.armed)
        return block(why);
      reset(why);
      return false;
    }
    // 必须未解锁、Stabilized、解锁/导航开关OFF、着地静止，才能开始零预发送及新的地面证明。
    if (phase_ == RcStartPhase::WaitGround) {
      if (s.armed || arm_on || nav || s.offboard || !s.stabilized_selected || !s.landed ||
          !s.at_rest || s.rc_arm_switch != 3 || s.rc_mode_slot == 0 || s.rc_mode_slot > 6 ||
          s.stationary_seconds < c.ground_seconds || std::abs(s.vz) > c.ground_vz_tolerance) {
        reset("RC_WAIT_DISARMED_STABILIZED_ARM_OFF_NAV_OFF_GROUND");
        return false;
      }
      // 零预发与新的源样本地面证明重叠进行；stationary_seconds单独不能证明z参考，回调可能跨估计器参考或重复旧数据。
      phase_ = RcStartPhase::Prestream;
    }
    // READY之前提前操作解锁或导航会阻止本轮；预发送不构成授权，地面变化需重新积累。
    if (phase_ == RcStartPhase::Prestream)
      return step_prestream(c, s, now, arm_on, nav);
    // 只有严格新的开关采样才更新边沿；重复缓存不能制造第二次解锁/导航动作。
    const bool new_switch = s.rc_switch_sample_stamp > last_switch_stamp_;
    if (new_switch) {
      // OFF→ON边沿必须晚于READY；记录源采样时间而非主机接收时间。
      if (last_arm_switch_ == 3 && arm_on) {
        if (s.rc_switch_sample_stamp <= ready_source_floor_)
          return block("RC_ARM_SOURCE_BEFORE_READY");
        arm_edge_ = s.rc_switch_sample_stamp;
      }
      // 导航边沿必须在解锁边沿之后，防止提前放在导航档而自动接管。
      if (last_mode_slot_ != c.rc_navigation_mode_slot && nav) {
        if (!arm_edge_ || s.rc_switch_sample_stamp <= arm_edge_)
          return block("RC_NAVIGATION_BEFORE_MANUAL_ARM");
        navigation_edge_ = s.rc_switch_sample_stamp;
      }
      last_arm_switch_ = s.rc_arm_switch;
      // 先辨认边沿，再写上次开关值；若提前更新，会把OFF→ON比较成ON→ON，吞掉真正授权。
      last_mode_slot_ = s.rc_mode_slot;
      last_switch_stamp_ = s.rc_switch_sample_stamp;
    }
    // 尚未解锁时继续独立地面观察；估计原点变动会退回预发送重新建立参考。
    if (phase_ == RcStartPhase::WaitArm) {
      if (nav || s.offboard)
        return block("RC_NAVIGATION_BEFORE_MANUAL_ARM");
      if (!s.armed && !arm_on) {
        if (!s.landed || !s.at_rest || !s.stabilized_selected ||
            std::abs(s.vz) > c.ground_vz_tolerance) {
          reset("RC_READY_GROUND_CHANGED");
          return false;
        }
        if (!observe_ground_reference(c, s, now)) {
          stream_first_ = stream_last_ = -1;
          commits_ = 0;
          ready_source_floor_ = 0;
          phase_ = RcStartPhase::Prestream;
          capture(s);
          reason_ = "RC_PRESTREAM_REBUILD_GROUND_REFERENCE";
          return true;
        }
        capture(s);
        reason_ = "RC_READY_WAIT_CH5_MANUAL_ARM";
        return true;
      }
      // 从首次观察到解锁意图开始固定配对期限；不是每收到一条消息就刷新。
      arm_since_ = now;
      phase_ = RcStartPhase::WaitNavigation;
    }
    // 先拒绝超时，再尝试拼配解锁/模式证据，迟到的健康尾帧不得复活已过期任务。
    if (phase_ == RcStartPhase::WaitNavigation)
      return step_wait_navigation(c, s, now, arm_on, nav);
    return false;
  }

private:
  // 推进预发送阶段，参数均为 step 已核对的同一快照；now 是单调秒，arm_on/nav 是该快照开关状态。
  // 返回 true 仅许可零流；可写地面证据、提交计数和 READY 水位，不能产生 Arm 或非零推力。
  // 必须在 WaitGround 当帧转入 Prestream 后直接调用，保持原同帧累积和首次拒绝原因。
  template <class C, class F>
  bool step_prestream(const C& c, const F& s, double now, bool arm_on, bool nav) {
    // 这是从step提取的同步子步骤，引用仍是同一快照；函数返回后不会保存c/s的地址。
    if (s.armed || arm_on || nav || s.offboard)
      return block("RC_ARM_OR_NAVIGATION_BEFORE_READY");
    if (!s.stabilized_selected || !s.landed || !s.at_rest ||
        std::abs(s.vz) > c.ground_vz_tolerance) {
      reset("RC_PRESTREAM_GROUND_CHANGED");
      return false;
    }
    // 地面位置/导数证据与真实输出提交并行累计；两者均满足且PX4未报丢流才READY。
    const bool qualified = observe_ground_reference(c, s, now);
    capture(s);
    if (qualified && stream_ready(c, now) && !s.offboard_signal_lost) {
      // 加上源时钟允许的100ms未来容差，保证READY前生成、后来才到达的开关不能算新授权。
      ready_source_floor_ = s.source_host_stamp + 100000;
      // 100000µs=0.1s；这是源时间下界，不是sleep，也不是再允许100ms提前解锁。
      phase_ = RcStartPhase::WaitArm;
      reason_ = "RC_READY_WAIT_CH5_MANUAL_ARM";
    } else
      reason_ = qualified ? "RC_PRESTREAM_WAIT_COMMITS_AND_PX4_SIGNAL"
                          : "RC_PRESTREAM_WAIT_NEW_GROUND_REFERENCE";
    return true;
  }
  // 在固定期限内配对人工解锁与导航，输入使用同一快照；源戳为 PX4 微秒，now 为单调秒，绝不跨域相减。
  // 返回 true 仅维持既有零流；false 表示 Joined 或 Blocked，原因/阶段仍由本对象持有。
  // 会写解锁 epoch、导航边沿及阶段；必须先检查过期再接收证据，且保持 WaitArm→WaitNavigation 的同帧调用。
  template <class C, class F>
  bool step_wait_navigation(const C& c, const F& s, double now, bool arm_on, bool nav) {
    if (now - arm_since_ > c.rc_arm_to_navigation_timeout)
      return block("RC_NAVIGATION_SELECTION_TIMEOUT");
    if (!manual_arm_proven_ && now - arm_since_ > c.status_timeout)
      return block("RC_MANUAL_ARM_EVIDENCE_TIMEOUT");
    // 人工解锁到导航之间仍绑定原地面参考；估计器重置或高度/速度变化不能在armed状态重建锚点。
    if (s.z_reset != z_reset_ || s.vz_reset != vz_reset_ || s.quat_reset != quat_reset_)
      return block("RC_ARMED_ESTIMATOR_RESET");
    if (!ground_qualified_ || !s.landed || std::abs(z0_ - s.z) > c.height_tolerance ||
        std::abs(s.vz) > c.ground_vz_tolerance)
      return block("RC_ARMED_GROUND_REFERENCE_NOT_STABLE");
    if (!s.stabilized_selected && !s.offboard)
      return block("RC_UNEXPECTED_MODE_BEFORE_NAVIGATION");
    if (s.armed && s.latest_arming_reason != 2)
      return block("RC_ARM_REASON_NOT_RC_SWITCH");
    if (s.armed && s.status_stamp <= ready_source_floor_)
      return block("RC_ARM_STATE_BEFORE_READY");
    if (s.armed && (s.status_stamp <= disarmed_status_ || s.armed_time <= disarmed_armed_time_ ||
                    (pending_armed_time_ && s.armed_time != pending_armed_time_)))
      return block("RC_ARM_STATE_NOT_NEW_MATCHING_EPISODE");
    // 首次armed帧锁定这一事件的PX4启动相对armed_time，其后帧必须仍属于同一事件。
    if (s.armed && !pending_armed_time_)
      pending_armed_time_ = s.armed_time;
    if (s.armed && arm_edge_ && arm_on && s.status_stamp > disarmed_status_ &&
        s.armed_time > disarmed_armed_time_) {
      // 开关边沿与实际armed可先后到达，但status源戳和PX4自身armed_time都必须推进，且前面已验原因=RC。
      if (!manual_arm_proven_)
        manual_armed_time_ = s.armed_time;
      manual_arm_proven_ = true;
    }
    // 已经证明的人工授权一旦撤销就阻断本轮，不能等它再次变ON而悄悄继续。
    if (manual_arm_proven_ && (!s.armed || !arm_on))
      return block("RC_ARM_AUTHORIZATION_WITHDRAWN");
    if (!manual_arm_proven_) {
      reason_ = "RC_WAIT_MATCHED_SWITCH_AND_NEW_ARMED_STATUS";
      return (!s.armed && arm_on) || pending_zero_allowed(c, s, now);
    }
    if (navigation_since_ >= 0 && now - navigation_since_ > c.status_timeout)
      return block(nav ? "RC_NAVIGATION_CONFIRMATION_TIMEOUT" : "RC_NAVIGATION_MODE_SLOT_MISMATCH");
    if (navigation_offboard_observed_ && !s.offboard)
      return block("RC_NAVIGATION_MODE_WITHDRAWN");
    if (navigation_edge_ && !nav)
      return block("RC_NAVIGATION_SELECTION_WITHDRAWN");
    // 实际Offboard反馈必须晚于先前Stabilized、解锁及READY水位，防止旧模式消息误授权。
    if (s.offboard) {
      if (!last_stabilized_status_ || s.status_stamp <= last_stabilized_status_ ||
          s.status_stamp <= arm_edge_ || s.status_stamp <= ready_source_floor_)
        return block("RC_OFFBOARD_STATE_NOT_NEW_AFTER_MANUAL_ARM");
      navigation_offboard_observed_ = true;
    } else
      last_stabilized_status_ = s.status_stamp;
    if ((nav || s.offboard) && navigation_since_ < 0)
      // 仅首次半边到来时记期限起点；反复收到同半边不会给另一半无限续期。
      navigation_since_ = now;
    // 档位、导航边沿、真实新Offboard状态和连续流全部配对后才Joined。
    if (nav && navigation_edge_ && s.offboard && s.status_stamp > navigation_edge_ &&
        stream_ready(c, now) && !s.offboard_signal_lost) {
      phase_ = RcStartPhase::Joined;
      reason_ = "RC_MANUAL_ARM_AND_NEW_OFFBOARD_CONFIRMED";
      return false;
    }
    reason_ = s.offboard && !nav ? "RC_WAIT_MATCHED_NAVIGATION_SWITCH_AND_OFFBOARD"
              : nav              ? "RC_WAIT_ACTUAL_PX4_OFFBOARD_CONFIRMATION"
                                 : "RC_ARMED_WAIT_CH6_NAVIGATION";
    return true;
  }
  // 终止本轮准入，保留原拒绝文本；输入为首个拒绝原因，返回 false 供阶段函数直接早退。
  // 只写本对象 phase_/reason_，不清地面/授权历史；是否允许新任务重建仍由外层安全恢复决定。
  bool block(const std::string& why) {
    phase_ = RcStartPhase::Blocked;
    reason_ = why;
    return false;
  }
  // 丢失地面连续性时只清本轮证据窗口/导数观察器；外层命令因果水位不在本对象中。
  void clear_ground_reference() {
    ground_qualified_ = false;
    ground_since_ = ground_last_at_ = ground_land_at_ = -1;
    ground_samples_ = ground_land_samples_ = ground_status_samples_ = 0;
    ground_local_ = ground_status_ = ground_land_ = ground_switch_ = ground_vertical_ = 0;
    ground_guard_.reset();
    // optional.reset()销毁其中的观察器；这不同于VerticalFeedbackGuard::reset()原地重建其内部状态。
  }
  // 独立地面证明：仅未解锁/静止/非Offboard的可信新样本可积累，绝不在空中重设高度零点。
  template <class C, class F> bool observe_ground_reference(const C& c, const F& s, double now) {
    // 证明范围是本次未解锁地面窗口。landed单个位不足够，还须静止、源戳推进和同一估计器参考。
    if (s.armed || s.offboard || s.rc_arm_switch != 3 || !s.stabilized_selected || !s.landed ||
        !s.at_rest || !std::isfinite(s.stationary_seconds) ||
        s.stationary_seconds < c.ground_seconds || !s.z_valid || !s.vz_valid || !s.local_fresh ||
        !s.status_fresh || !s.land_fresh || !std::isfinite(s.z) || !std::isfinite(s.vz) ||
        std::abs(s.vz) > c.ground_vz_tolerance || !s.local_stamp || !s.status_stamp ||
        !s.land_stamp || !s.rc_switch_sample_stamp ||
        (c.feedback_guard.enabled && (!s.vertical_sample_stamp || !std::isfinite(s.z_deriv)))) {
      clear_ground_reference();
      return false;
    }
    // 重置、时间回退、同stamp改数、样本大间隔及高度离开均切断连续证据，不允许正常尾帧掩盖。
    const bool discontinuity =
        ground_since_ >= 0 &&
        (s.z_reset != ground_z_reset_ || s.vz_reset != ground_vz_reset_ ||
         s.quat_reset != ground_quat_reset_ || s.local_stamp < ground_local_ ||
         s.status_stamp < ground_status_ || s.land_stamp < ground_land_ ||
         s.rc_switch_sample_stamp < ground_switch_ || s.vertical_sample_stamp < ground_vertical_ ||
         (s.local_stamp == ground_local_ && (s.z != ground_last_z_ || s.vz != ground_last_vz_)) ||
         (c.feedback_guard.enabled && s.vertical_sample_stamp == ground_vertical_ &&
          (s.z != ground_last_z_ || s.vz != ground_last_vz_ || s.z_deriv != ground_last_deriv_)) ||
         (s.local_stamp > ground_local_ &&
          double(s.local_stamp - ground_local_) * 1e-6 > c.local_timeout) ||
         (s.land_stamp > ground_land_ &&
          now - ground_land_at_ > std::min(c.land_sample_max_gap, c.land_timeout * .5)) ||
         std::abs(s.z - ground_window_z_) > c.height_tolerance);
    // 同stamp改数值是“伪装的新测量”；真正new local只靠严格源推进判定，健康尾帧不能补回中间缺口。
    if (discontinuity) {
      clear_ground_reference();
      stream_first_ = stream_last_ = -1;
      commits_ = 0;
      ready_source_floor_ = 0;
    }
    if (ground_since_ < 0) {
      ground_since_ = ground_window_since_ = now;
      ground_window_z_ = s.z;
      ground_z_reset_ = s.z_reset;
      ground_vz_reset_ = s.vz_reset;
      ground_quat_reset_ = s.quat_reset;
      ground_guard_.emplace(c.feedback_guard);
      // emplace在optional内部直接构造观察器，寿命归本对象管理；不需要new/delete，也不借用配置指针。
    }
    // 地面也用同一位置/导数合同；速度差诊断不会独立阻断，硬不一致会重新积累。
    const auto& vertical =
        ground_guard_->observe(s.vertical_sample_stamp, s.z, s.vz, s.z_deriv, c.local_timeout);
    // ->访问optional内的对象；const auto&借用它的证据。下面clear_ground_reference会销毁对象，
    // 因而失败分支清除后立即return，不能继续使用vertical这个已失去所有者的引用。
    if (c.feedback_guard.enabled && !vertical.healthy) {
      clear_ground_reference();
      stream_first_ = stream_last_ = -1;
      commits_ = 0;
      ready_source_floor_ = 0;
      return false;
    }
    // 不同DDS源独立计数，不能强求每个land回调恰好同时伴随一个status，也不能用缓存重复计数。
    if (s.local_stamp > ground_local_)
      ++ground_samples_;
    if (s.status_stamp > ground_status_)
      ++ground_status_samples_;
    if (s.land_stamp > ground_land_) {
      // 各DDS源频率独立，须分别推进；不能强求每条land回调都配一条status，否则低频状态流会永远无法READY。
      ++ground_land_samples_;
      ground_land_at_ = now;
    }
    ground_local_ = s.local_stamp;
    ground_status_ = s.status_stamp;
    ground_land_ = s.land_stamp;
    ground_switch_ = s.rc_switch_sample_stamp;
    ground_vertical_ = s.vertical_sample_stamp;
    ground_last_at_ = now;
    ground_last_z_ = s.z;
    ground_last_vz_ = s.vz;
    ground_last_deriv_ = s.z_deriv;
    // 至少3个local、3个land、2个status且跨越ground_seconds，启用导数监测时还要完成位置窗口。
    ground_qualified_ = now - ground_since_ + 1e-9 >= c.ground_seconds && ground_samples_ >= 3 &&
                        ground_land_samples_ >= 3 && ground_status_samples_ >= 2 &&
                        (!c.feedback_guard.enabled || vertical.position_window_checked);
    // 已合格地面缓慢漂移可以在未解锁窗口内更新锚点；不把上一任务气压高度当永久地面。
    if (ground_qualified_ && now - ground_window_since_ >= c.ground_seconds) {
      ground_window_since_ = now;
      ground_window_z_ = s.z;
    }
    return ground_qualified_;
  }
  // 未解锁准入重置清连续流和锚点，同时保留本对象已观察过解锁意图的历史字段。
  void reset(const std::string& why) {
    // 只清下列准备字段；manual_arm_proven_/arm_edge_/arm_intent_observed_等历史不是“忘掉即可恢复”。
    phase_ = RcStartPhase::WaitGround;
    anchor_ = false;
    stream_first_ = stream_last_ = -1;
    commits_ = 0;
    ready_source_floor_ = 0;
    clear_ground_reference();
    reason_ = why;
  }
  // 只在允许的未解锁地面分支捕获NED z、wxyz姿态、reset及开关水位。
  template <class F> void capture(const F& s) {
    // 复制参考值而非保存s的引用；回调快照结束后锚点仍存在。只由准入中的未解锁地面调用更新。
    anchor_ = true;
    z0_ = s.z;
    q0_ = s.q;
    z_reset_ = s.z_reset;
    vz_reset_ = s.vz_reset;
    quat_reset_ = s.quat_reset;
    disarmed_status_ = s.status_stamp;
    disarmed_armed_time_ = s.armed_time;
    last_arm_switch_ = s.rc_arm_switch;
    last_mode_slot_ = s.rc_mode_slot;
    last_switch_stamp_ = s.rc_switch_sample_stamp;
  }
  // 成员分为阶段/意图、姿态高度锚点、单调秒窗口、PX4源µs边沿及独立地面证据；不同时间域不可直接相减。
  RcStartPhase phase_{RcStartPhase::WaitGround};
  std::string reason_{"RC_WAIT_SAFE_GROUND"};
  bool anchor_{}, manual_arm_proven_{}, navigation_offboard_observed_{}, arm_intent_observed_{};
  Quaternion q0_{1, 0, 0, 0};
  double z0_{};
  std::uint8_t z_reset_{}, vz_reset_{}, quat_reset_{}, last_arm_switch_{}, last_mode_slot_{};
  double arm_since_{-1}, navigation_since_{-1}, stream_first_{-1}, stream_last_{-1},
      observed_at_{-1};
  unsigned commits_{};
  // 解锁前status/armed_time、已确认manual_armed_time及开关边沿均保留源时序，用来匹配唯一人工解锁事件。
  std::uint64_t disarmed_status_{}, disarmed_armed_time_{}, manual_armed_time_{}, arm_edge_{},
      navigation_edge_{}, last_switch_stamp_{};
  std::uint64_t ready_source_floor_{}, pending_armed_time_{}, last_stabilized_status_{};
  bool ground_qualified_{};
  unsigned ground_samples_{}, ground_land_samples_{}, ground_status_samples_{};
  double ground_since_{-1}, ground_last_at_{-1}, ground_land_at_{-1}, ground_window_since_{-1},
      ground_window_z_{};
  double ground_last_z_{}, ground_last_vz_{}, ground_last_deriv_{};
  std::uint64_t ground_local_{}, ground_status_{}, ground_land_{}, ground_switch_{},
      ground_vertical_{};
  std::uint8_t ground_z_reset_{}, ground_vz_reset_{}, ground_quat_reset_{};
  // 仅在地面窗口建立后构造导数监测器，reset窗口时一起销毁；不会复用上次飞行积分。
  std::optional<VerticalFeedbackGuard> ground_guard_;
};
} // namespace fc01::attitude
