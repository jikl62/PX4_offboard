#pragma once
// 阅读路线：Runtime 是“任务分派员和命令记账员”，不是 ROS 节点，也不是高度控制器。
// request 接收控制台意图；step 选定一条纯计算状态机（Core）路线并生成计划；节点负责真正发布；
// confirm_attitude_command_publication 回记本地提交；ack 记 PX4 应答；后续新状态证明物理结果。
// 四层证据要逐层阅读，不能把其中任意一层直接叫作“起飞成功”：
//   requested：Core 请求动作；published：最终命令发布调用（sink）成功返回；
//   accepted：通过关联检查的 ACK 为 ACCEPTED；state_verified：独立 PX4 新状态已被本地验证。
// sent 表示本事务可以接收 ACK；实机姿态路线启用 sink 确认后，它只在实际提交时置真。
// 旧纯 Core 测试夹具（fixture）保留隐式 sent 模型，因此 sent 与 published 必须分开存，不能合并。
// 本文件的两个时间单位：now/received/sent_at 为主机单调秒；stamp/wire/baseline 为源域微秒。
// armed_time 则来自 PX4 启动时钟，只能与同一 PX4 的旧 armed_time 比较。
// profile 选择地面测试、通用速度或姿态高度路线；生产 RC 1.5 m 起飞由姿态高度 Core 管理。
// 单线程调用约定：Runtime 不持有 ROS 节点，不自行收包，也不在 step 内等待回调。
#include "jikl_fc01_system/source_clock.hpp"
#include "jikl_fc01_system/ground_link.hpp"
#include "jikl_fc01_system/velocity_core.hpp"
#include "jikl_fc01_system/attitude_altitude_core.hpp"
#include "jikl_fc01_system/publication_guard.hpp"
#include "jikl_fc01_system/protocol.hpp"
#include <array>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace fc01 {
namespace v = jikl_velocity;
namespace g = jikl_fc01_ground_link;
// 每个数据源的严格递增源µs时间与主机接收单调秒分开记录；一次源回退锁存，不能被后续正常信封洗掉。
struct Stamp {
  // {} 是值初始化：整型为 0、布尔为 false；0 在此表示尚无合法源时间。
  std::uint64_t value{};
  // -1 是“尚未接收”的哨兵，不能把默认值当作刚收到消息。
  double received{-1};
  // 一旦发现回退就保持 true，不能用后来较大的时间戳自动洗掉历史异常。
  bool regressed{};
  // 只接受非零且严格新的源stamp；更新received只在接受成功时发生，重复旧帧不能续新鲜度。
  bool accept(std::uint64_t next, double now) {
    // && 从左到右短路：next 和旧 value 均非零时，才判断是不是严格回退。
    if (next && value && next < value)
      regressed = true;
    // 早 return 表示本次未接纳；尤其不能更新 received 给重复帧“续命”。
    if (!next || next <= value || regressed)
      return false;
    value = next;
    received = now;
    return true;
  }
  // 检查接收年龄及主机时间不回退；原始采样年龄由节点另查，不能单靠此方法证明测量新鲜。
  bool fresh(double now, double limit) const {
    return !regressed && received >= 0 && now >= received && now - received < limit;
  }
};
// 统一profile及启动身份；flight_enabled只用于通用路线，姿态高度采用独立动作门。
struct Config {
  std::string profile{"ground"};
  // 部署来源元数据只用于报告真实加载路径，不授予解锁/导航权。
  std::string runtime_mode{}, entry_version{"unknown"}, config_path{}, config_source_path{};
  bool flight_enabled{};
  attitude::Config attitude;
  // 通用路线的受限默认配置；生产1.5m目标来自attitude.rc_target_height，不从这个height推导。
  // [] 是不捕获外部变量的 lambda；末尾 () 立即执行它，得到一个 Config 值。
  // c 是 lambda 内的局部对象，返回值用于初始化 control；不会留下指向 c 的引用。
  v::Config control = []() {
    v::Config c;
    c.height = 1;
    c.max_height = 5;
    c.max_radius = 3;
    c.xy_speed = .3;
    c.z_speed = .3;
    c.climb_speed = .3;
    return c;
  }();
  // diagnostic必须关闭所有动作/确认门；RC只能走attitude_altitude，通用flight门不能跨路线偷开。
  void validate() const {
    if (!runtime_mode.empty() && runtime_mode != "diagnostic" && runtime_mode != "flight-test")
      throw std::invalid_argument("FC01_RUNTIME_MODE_INVALID");
    if (runtime_mode == "diagnostic" &&
        (flight_enabled || attitude.flight_actions_enabled || attitude.arm_enabled ||
         attitude.takeoff_enabled || attitude.rc_navigation_enabled ||
         attitude.bench_arm_only_enabled || attitude.visual.enabled ||
         attitude.parameters_confirmed || attitude.rc_takeover_confirmed ||
         attitude.autonomous_failsafe_confirmed))
      throw std::invalid_argument("DIAGNOSTIC_ACTION_GATES_MUST_BE_CLOSED");
    // 按固定顺序验证子配置，再做路线间约束；先遇到的错误就是调用者看到的原因。
    control.validate();
    attitude.validate();
    if (attitude.rc_navigation_enabled && profile != "attitude_altitude")
      throw std::invalid_argument("RC_NAVIGATION_REQUIRES_ATTITUDE_PROFILE");
    if (profile == "attitude_altitude" && flight_enabled)
      throw std::invalid_argument("attitude_altitude uses separate flight_actions_enabled gates");
    if (profile != "ground" && profile != "flight" && profile != "sim" &&
        profile != "attitude_altitude")
      throw std::invalid_argument("profile must be ground, flight, sim, or attitude_altitude");
    if (profile == "ground" && flight_enabled)
      throw std::invalid_argument("ground profile cannot enable flight");
    if (control.command_timeout != .5 || control.land_after_loss != 3.)
      throw std::invalid_argument("command timeout/lease timeout must be 0.5/3 seconds");
  }
};
// 节点同轮整合的实际反馈及来源/电池/控制循环事实；Runtime只读这一快照，不在运算中插入回调。
struct Snapshot {
  // 同一物理反馈被整理成通用 Core 与姿态 Core 的输入；它们不是两个独立传感器。
  v::Feedback feedback{};
  attitude::Feedback attitude;
  g::Traffic traffic{};
  // 新鲜度与源时间分开保存：收到新包和包内容可用于控制，是两件不同的事。
  std::uint64_t status_stamp{}, land_stamp{};
  bool status_fresh{}, land_fresh{}, local_fresh{}, at_rest{};
  // 纯Core旧fixture默认值仅为兼容；实机必须由PX4固定枚举确认，未知arming state不能当未解锁。
  bool arming_state_known{true};
  // PX4启动相对µs时钟，只与同一PX4的旧armed_time比较，禁止与ROS stamp或steady秒跨域比较。
  std::uint64_t armed_time{};
  std::uint8_t latest_arming_reason{};
  // 电源和人工接管是多项事实的合取，不能只凭“电池在线”或“有 RC 包”放行。
  bool battery_fresh{}, battery_connected{}, power_valid{}, manual_fresh{}, manual_lost{};
  // 这些是输入侧整理好的异常/环境事实；time_regressed 包括应保留的回退证据。
  bool dead_reckoning{}, time_regressed{}, calibration{}, hil{};
  bool stabilized_selectable{true};
  // nav_state 是 PX4 模式枚举，不是 RC 六档开关的槽位；两者不能互换。
  int battery_warning{}, nav_state{4}, system{1}, component{1};
  // 地面静止证据已连续维持的主机秒数；不能由一次 at_rest=true 推导出此时长。
  double stationary_seconds{};
  // 分别表示最近输出/心跳是否仍活及完整性故障；地面恢复必须证明输出停止，不只看Core阶段。
  bool attitude_output_active{}, offboard_heartbeat_active{}, publication_fault_latched{};
  // 即使本轮没有调用 sink，周期超时也会打断恢复观察；不能用“没发布”掩盖循环停顿。
  // 节点在整个周期结束后更新的时限事实；恢复窗口不能跨越重复超时周期累积。
  bool control_cycle_fresh{true};
};
// 控制台请求的接受/拒绝及原因；accepted只说明请求准入，不等于PX4已执行动作。
struct Reply {
  bool accepted{};
  std::string message;
};
// 三路线的拟发布结果；ROS节点仍须逐sink校验当前上下文并回报提交事实。
struct Output {
  v::Output flight{};
  attitude::Output attitude;
  bool send_ground{};
};
// MAV_CMD 400的本地意图：None无事务，Arm解锁，Disarm上锁；ACK编号本身不包含该意图。
enum class Command400Kind { None, Arm, Disarm };
// 审计阶段区分计划、提交、ACK、独立状态、歧义/未知结果与终态；枚举名不是新的PX4协议。
enum class CommandStage {
  // 本槽尚未建立事务。
  None,
  // 只有本地计划，还没有允许关联 ACK 的最终提交。
  Planned,
  // 已登记提交，等待 ACK；生产姿态路线要求真实 sink 成功。
  CommittedAwaitAck,
  // ACK 接受，但仍须独立状态证明。
  AckAcceptedAwaitState,
  // 可归属的 ACK 明确拒绝。
  Rejected,
  // 期限已过，不能用晚到 ACK 复活事务。
  TimedOut,
  // 最终本地边界拒绝，不能伪称 PX4 拒绝。
  FinalSinkRejected,
  // 状态证据已通过，尚无 ACK。
  StateVerifiedAwaitAck,
  // 本地要求的 ACK 与状态两层证据均已满足。
  VerifiedTerminal,
  // PX4 报 IN_PROGRESS，不能当作 ACCEPTED。
  AckInProgress,
  // 400 已复用，只能观察 ACK，不能唯一认领。
  Ambiguous400Ack,
  // 独立状态证明终态，ACK 仍不归属。
  StateOnlyTerminal,
  // 不认识的 400 result 保持不确定。
  UnknownAckResult,
  // 状态证据与 ACK 拒绝并存，如实保留矛盾。
  StateAckContradictory,
  // 观察到 ACK 和状态，但尚未构成已归属成功。
  StateAckObservedUnresolved,
  // 176 偏斜 ACK 候选，等待更新的状态证明。
  AckCandidateAwaitState
};
// 保持Arm/Disarm审计字符串稳定，不能用日志名称当作wire关联nonce。
inline const char* command400_name(Command400Kind kind) {
  switch (kind) {
  case Command400Kind::None:
    return "NONE";
  case Command400Kind::Arm:
    return "ARM";
  case Command400Kind::Disarm:
    return "DISARM";
  }
  return "UNKNOWN";
}
// 把本地事务阶段映射到诊断文本；状态确认和ACK确认在文本中明确区分。
inline const char* command_stage_name(CommandStage stage) {
  switch (stage) {
  case CommandStage::None:
    return "NONE";
  case CommandStage::Planned:
    return "PLANNED";
  case CommandStage::CommittedAwaitAck:
    return "SINK_COMMITTED_AWAIT_ACK";
  case CommandStage::AckAcceptedAwaitState:
    return "ACK_ACCEPTED_AWAIT_STATE";
  case CommandStage::Rejected:
    return "ACK_REJECTED";
  case CommandStage::TimedOut:
    return "ACK_TIMEOUT";
  case CommandStage::FinalSinkRejected:
    return "FINAL_SINK_REJECTED";
  case CommandStage::StateVerifiedAwaitAck:
    return "PX4_STATE_VERIFIED_ACK_MISSING";
  case CommandStage::VerifiedTerminal:
    return "ACK_AND_PX4_STATE_VERIFIED";
  case CommandStage::AckInProgress:
    return "ACK_IN_PROGRESS_AWAIT_STATE";
  case CommandStage::Ambiguous400Ack:
    return "AMBIGUOUS_400_ACK";
  case CommandStage::StateOnlyTerminal:
    return "PX4_STATE_ONLY_TERMINAL_ACK_UNATTRIBUTED";
  case CommandStage::UnknownAckResult:
    return "UNKNOWN_ACK_RESULT_AWAIT_STATE";
  case CommandStage::StateAckContradictory:
    return "PX4_STATE_EVIDENCED_ACK_CONTRADICTORY";
  case CommandStage::StateAckObservedUnresolved:
    return "PX4_STATE_EVIDENCED_ACK_OBSERVED_UNRESOLVED";
  case CommandStage::AckCandidateAwaitState:
    return "ACK_CANDIDATE_AWAIT_NEW_OFFBOARD_STATE";
  }
  return "UNKNOWN";
}
// 按PX4/MAV_RESULT原枚举显示：FAILED=4与DENIED=2不同，未观测=-1不等于拒绝。
inline const char* ack_result_name(int result) {
  switch (result) {
  case -1:
    return "NOT_OBSERVED";
  case 0:
    return "ACCEPTED";
  case 1:
    return "TEMPORARILY_REJECTED";
  case 2:
    return "DENIED";
  case 3:
    return "UNSUPPORTED";
  case 4:
    return "FAILED";
  case 5:
    return "IN_PROGRESS";
  case 6:
    return "CANCELLED";
  }
  return "UNKNOWN_RESULT";
}
// 一条176/400/21事务的当前证据；源时间µs与接收/提交steady秒分开保存。
struct Pending {
  // sent：允许处理 ACK；accepted：应答准入已满足；rejected：ACK 拒绝或本地 sink 拒绝；timed_out：期限已过。
  // requested：有本地计划；published：最终命令 sink 成功；sink_denied：本地边界拒绝提交。
  // 例如 requested=true 且 published=false 完全合法：可能仍待提交，也可能被最终边界拦下。
  bool sent{}, accepted{}, rejected{}, timed_out{}, requested{}, published{}, sink_denied{};
  // result=-1尚未ACK；sent_at是主机单调秒，baseline/last_ack是源µs水位。
  int result{-1};
  double sent_at{};
  std::uint64_t baseline{}, last_ack{};
  // 实际命令源域时间、提交时status/land及PX4 armed_time快照用于因果核对，不能只依赖ACK到达顺序。
  std::uint64_t wire_stamp{}, status_at_wire{}, land_at_wire{}, armed_time_at_wire{};
  // 两个 -1 均是“该事件尚未发生”；它们记录主机接收/拒绝时刻，不参与源域微秒比较。
  double rejected_at{-1}, ack_received_at{-1};
  // 同一个 MAV_CMD 400 同时承载 Arm/Disarm；本地还需单独保存 param1 所代表的意图。
  Command400Kind command400_kind{Command400Kind::None};
  CommandStage stage{CommandStage::None};
  // ACK 源戳与独立状态源戳分别留证；ACK 到了不意味着状态已改变，反过来也一样。
  std::uint64_t ack_stamp{}, verified_status_stamp{};
  // 此位只证明本地审核过的物理终态，不清除已复用 400 的 ACK 归属歧义。
  bool state_verified{};
  double verified_at{-1};
  // 即使旧 400 曾同时获得 ACK 和状态证明，旧重复 ACK 仍可能晚到；复用后也不能恢复唯一归属。
  // generation 是本进程审计序号，协议中没有对应的唯一事务号（nonce）；400 复用后 ACK 不能认领特定 Arm/Disarm。
  unsigned generation{}, ack_observations{};
  // ack_ambiguous：本进程已经复用 400，wire 无 nonce，ACK 无法唯一归属。
  // previous400_unresolved：创建这笔事务时旧 400 证据仍不完整；用于限制窄后备路径。
  bool ack_ambiguous{}, previous400_unresolved{};
  // 记录适配器是否采用已检查同步域，不因提交后遥测过期就遗忘已经发出的命令时间下界。
  bool source_clock_validated{};
  // 同步域下通常为 wire_stamp + 100 ms，饱和加法见 source_causal_floor。
  // 它要求后续状态足够晚，避免把命令前但时钟略领先的状态误当作命令结果。
  std::uint64_t causal_floor{};
  // 不合时钟预算的 ACK 只计数留证，不修改“接受/拒绝”或授予下一动作。
  unsigned unattributed_clock_acks{};
  // 仅首个正常Offboard事务可使用时钟偏斜ACK候选，还要独立新模式证明；候选不直接授权Arm。
  // candidate 只是待独立状态佐证的 ACK；confirmed_by_state 才记录候选已获佐证。
  // exclusive_at_wire 是实际提交时的唯一来源事实，不能用较晚快照替代提交时事实。
  bool ack_candidate{}, ack_confirmed_by_state{}, exclusive_at_wire{};
  // 此指针只指向静态字符串字面量，不拥有动态内存；记录 176 ACK 被保留/过滤的解释。
  const char* ack_disposition{"NOT_OBSERVED"};
};
// 同一控制线程拥有所有Core与事务状态；跨任务重建只清任务，不清进程级ACK歧义与提交代数。
// 生命周期示例一：176 第一次正常 Offboard（只说明关联流程，不代表直接授权动力）。
//   1. Core 计划 Offboard -> requested；最终 sink 成功 -> published/sent，并保存 wire 戳。
//   2. 若 ACK 源戳略早于 wire，但仍在 100 ms 偏差预算内，先保留 ACCEPTED 候选。
//   3. 后续 status 必须晚于命令/status/ACK/causal_floor，且仍满足健康、权限和地面前提。
//   4. Core 同帧再核验；只有进入允许阶段后，才把候选记为已验证终态。
//   数字例（假定旧水位均更低）：wire=1,000,000 us，causal_floor=1,100,000 us。
//   ACK=1,010,000 us 是命令后 10 ms 的快速应答，可走正常 176 过滤，不要求先等 100 ms。
//   ACK=990,000 us比wire早10ms，只能成为候选；ACK<=900,000 us已触及或越过严格下界，应丢弃。
//   即使候选保留，后续 status 仍须 >1,100,000 us 并满足其他所有条件，不能拿该 ACK 直接解锁。
// 生命周期示例二：400 Arm -> Disarm。
//   第一次真实 Arm 获得 generation=1；Disarm 提交时变成 generation=2，ACK 开始永久歧义。
//   晚到的 ACCEPTED 可能是旧 Arm 的，故只记录观察；Disarm 结果由新的安全上锁地面证据证明。
//   归档只保留最近 128 笔展示，不能把全进程提交次数/ACK 水位清零来“恢复可归属”。
class Runtime {
public:
  // 先构造各纯计算Core再执行统一配置验证；不启动飞控或发布任何消息。
  explicit Runtime(Config c = {})
      : config_(std::move(c)), core_(config_.control), attitude_core_(config_.attitude) {
    // 成员按声明顺序构造；config_ 先持有移动后的配置，两个 Core 读取该成员的子配置。
    config_.validate();
  }
  // const 成员函数不改 Runtime；此 phase() 专指通用速度 Core，显示选中路线请用 phase_name()。
  v::Phase phase() const {
    return core_.phase();
  }
  // 按所选profile回答是否仍有托管任务；恢复降落也算active，不能提前关闭控制生命周期。
  bool active() const {
    return config_.profile == "attitude_altitude" ? attitude_core_.active()
                                                  : core_.active() || recover_land_;
  }
  bool shutting_down() const {
    return shutdown_latched_;
  }
  // 停止请求锁存一次并沿原LAND入口处理，不允许关闭后开启新任务。
  void begin_shutdown(const Snapshot& s, double now) {
    if (shutdown_latched_)
      return;
    shutdown_latched_ = true;
    // 先锁关闭，再沿 LAND 请求门处理；即使 LAND 被拒，关闭锁也不反转。
    request("LAND", {}, s, now);
  }
  // 最终发布或适配器拒绝转交Core故障锁存，保持具体原因。
  void abort_attitude(const std::string& reason) {
    attitude_core_.block(reason);
    message_ = reason;
  }
  // 仅枚举过的deadline拒绝、无命令且未解锁地面准备可走窄恢复；非零/未知部分发布仍是完整性故障。
  bool abort_rc_preparation_timing(const std::string& reason, const Snapshot& s) {
    // 适配器只可为明确类型的期限拒绝调用本入口：无抛异常 sink、计划推力严格为零且没有命令。
    // 未知结果或非零部分发布仍属于执行完整性故障；下面再由 Runtime 独立检查适用范围。
    if (reason != "ROS_ADAPTER_CONTROL_LOOP_STALL" &&
        reason != "PUBLISHER_BOUNDARY_CONTROL_LOOP_STALL" &&
        reason !=
            "PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_HEARTBEAT_FINAL_CONTEXT_CHANGED_CONTROL_LOOP_STALL")
      return false;
    if (config_.profile != "attitude_altitude" || !config_.attitude.rc_navigation_enabled ||
        shutdown_latched_ || maintenance_ || ground_ || s.time_regressed || !s.arming_state_known ||
        !s.status_fresh || !s.land_fresh || s.feedback.armed || s.feedback.offboard ||
        !s.feedback.landed || !s.at_rest)
      return false;
    // const auto& 逐个只读引用事务槽，不复制；任何曾有请求的槽都排除这条“未发命令”窄路径。
    for (const auto& p : pending_)
      if (p.requested)
        return false;
    if (!attitude_core_.block_rc_preparation_timing(reason, attitude_snapshot(s)))
      return false;
    rc_preparation_timing_fault_ = true;
    last_rc_timing_observation_ = -1;
    last_attitude_output_active_ = false;
    message_ = reason;
    return true;
  }
  // 每轮扫离散安全队列前初始化本轮观察；后续任意不安全payload均保留否决。
  void begin_attitude_safety_observation() {
    queued_safety_wait_ = false;
    attitude_core_.begin_safety_observation();
  }
  // 逐事件检查而非只看最新连续缓存；queued_safety_wait_用逻辑或保留整轮拒绝。
  void observe_attitude_safety_event(const attitude::Feedback& feedback,
                                     attitude::Core::SafetyEvent source, double now) {
    if (config_.profile != "attitude_altitude")
      return;
    // 左边先调用事件观察器，再与旧值取或；不能写成旧值 || 调用，否则旧值为真会短路掉观察。
    queued_safety_wait_ =
        attitude_core_.observe_safety_event(feedback, source, now) || queued_safety_wait_;
    message_ = attitude_core_.reason();
  }
  // 实际sink前重核视觉状态/命令寿命，目标不能仅凭计算时有效就获得发布。
  bool visual_output_current(const Snapshot& s, double now) const {
    return attitude_core_.visual_current(attitude_snapshot(s), now);
  }
  // 视觉输入过期后进入原下降控制并更新原因，不直接发零推力或丢掉仍armed的托管。
  void request_visual_land(const Snapshot& s, double now, const std::string& reason) {
    attitude_core_.visual_land(attitude_snapshot(s), now, reason);
    message_ = attitude_core_.reason();
  }
  // 以下只读API供节点诊断/发布上下文使用；读取不推进任务、ACK或地面计数。
  const attitude::Core& attitude_core() const {
    return attitude_core_;
  }
  const char* phase_name() const {
    return config_.profile == "attitude_altitude" ? attitude::name(attitude_core_.phase())
                                                  : v::name(core_.phase());
  }
  g::State ground_state() const {
    // unique_ptr 可为空；条件运算符只求值被选中的一边，非空时才通过 -> 访问对象。
    return ground_ ? ground_->state() : g::State::Waiting;
  }
  // 返回 const 引用避免复制字符串；调用者只在 Runtime 及对应成员仍存活时使用，勿跨销毁保存。
  const std::string& message() const {
    return message_;
  }
  const v::Core& flight_core() const {
    return core_;
  }
  // 返回的是固定数组内元素的只读别名；以后同槽建立新事务时，此别名看到的内容也会改变。
  const Pending& pending(unsigned cmd) const {
    return pending_[index(cmd)];
  }
  const std::vector<Pending>& command400_history() const {
    return command400_history_;
  }
  const std::string& last_rc_attempt_reason() const {
    return last_rc_attempt_reason_;
  }
  std::uint64_t rc_attempts_completed() const {
    return rc_attempts_completed_;
  }
  // 生产节点启用真实sink模式；旧纯Core测试的隐式提交模型只留在未显式要求的fixture里。
  void require_attitude_sink_commit() {
    if (config_.profile == "attitude_altitude")
      attitude_sink_commit_required_ = true;
  }
  // 只有节点报告真实配对结果才计零流提交，不用out.publish计划值代替。
  void confirm_rc_output(bool allowed, double now) {
    attitude_core_.note_rc_publication(allowed, now);
  }
  bool rc_restart_allowed(const Snapshot& s) const {
    return attitude_core_.rc_restart_allowed(attitude_snapshot(s));
  }
  // 未发生任何旧事务、输出或故障的未解锁RC准备才可延期；原wire水位一个也不清。
  bool defer_rc_preparation(const Snapshot& s, double now, const std::string& reason) {
    if (config_.profile != "attitude_altitude" || !config_.attitude.rc_navigation_enabled ||
        shutdown_latched_ || maintenance_ || ground_ || s.publication_fault_latched ||
        s.time_regressed || s.attitude_output_active || s.offboard_heartbeat_active)
      return false;
    // 旧命令即使已终结，也排除这条“没有解锁动作”的窄延期路径；已有 wire 水位全部保留。
    // const auto& 逐个只读引用事务槽，不复制；任何曾有请求的槽都排除这条“未发命令”窄路径。
    for (const auto& p : pending_)
      if (p.requested)
        return false;
    if (!attitude_core_.defer_rc_preparation(attitude_snapshot(s), now, reason))
      return false;
    // 这里只撤销当前发布计划标记，不改写真正 sink 是否执行过的证据。
    last_attitude_output_active_ = false;
    message_ = reason;
    return true;
  }
  // 只读准入与实际reprepare共用同一门槛，按停止/维护/时间/输出/核心/事务给出具体阻塞原因。
  std::string rc_reprepare_block_reason(const Snapshot& s, double now) const {
    if (config_.profile != "attitude_altitude" || !config_.attitude.rc_navigation_enabled)
      return "RC_RECOVERY_DISABLED";
    if (shutdown_latched_)
      return "RC_SHUTDOWN_IN_PROGRESS";
    if (maintenance_)
      return "RC_MAINTENANCE_ACTIVE";
    if (ground_)
      return "RC_GROUND_TEST_ACTIVE";
    if (!std::isfinite(now) || now < 0)
      return "RC_INVALID_HOST_TIME";
    if (s.publication_fault_latched && !rc_preparation_timing_fault_)
      return "RC_PUBLICATION_FAULT_LATCHED";
    if (s.time_regressed)
      return "RC_TIME_REGRESSION_LATCHED";
    if (!s.control_cycle_fresh)
      return "RC_CONTROL_CYCLE_NOT_FRESH";
    if (last_attitude_output_active_ || s.attitude_output_active || s.offboard_heartbeat_active)
      return "RC_OUTPUT_STILL_ACTIVE";
    if (!rc_restart_allowed(s))
      return "RC_CORE_RESTART_CONDITIONS_NOT_MET";
    for (unsigned command : {176U, 400U, 21U}) {
      const auto& p = pending_[index(command)];
      if (!p.requested)
        continue;
      // max 取最严格下界：当前状态必须同时晚于继承的旧水位、提交前状态和命令因果水位。
      const bool newer_status =
          s.status_stamp > std::max({p.baseline, p.status_at_wire, p.causal_floor});
      // 事务可由已验证物理状态或无歧义拒绝等结束；未决400不能因开关复位被忽略。
      const bool terminal =
          p.state_verified || (!p.ack_ambiguous && p.rejected) ||
          (p.command400_kind == Command400Kind::None && p.accepted && newer_status);
      if (!terminal && !ended_rc_disarm_terminal(p, s))
        return "RC_COMMAND_" + std::to_string(command) + "_OUTCOME_UNRESOLVED";
    }
    return {};
  }
  // 只重建已结束任务的姿态Core；全局提交次数、ACK水位和历史记录继续存活，防迟到ACK污染新任务。
  bool reprepare_rc(const Snapshot& s, double now) {
    // 整周期时限包括报告和最终快照；前半轮看似健康，不能抵消后半轮真实超时。
    if (rc_preparation_timing_fault_ && !s.control_cycle_fresh)
      attitude_core_.clear_rc_timing_recovery();
    if (!rc_reprepare_block_reason(s, now).empty())
      return false;
    // 只重建任务；即使旧 Disarm 已由 ACK/独立状态终结，wire 代数和 ACK 水位仍跨任务存活。
    // 已结束任务的新上锁地面停留可证明Disarm物理结果，但不把迟到400 ACK认领为本次成功。
    for (auto& p : pending_)
      if (p.requested && !p.state_verified && ended_rc_disarm_terminal(p, s)) {
        p.state_verified = true;
        p.verified_status_stamp = s.status_stamp;
        p.verified_at = now;
        p.stage = CommandStage::StateOnlyTerminal;
        p.ack_disposition = "ENDED_TASK_GROUND_CONFIRMED_ACK_UNATTRIBUTED";
      }
    last_rc_attempt_reason_ = attitude_core_.reason();
    ++rc_attempts_completed_;
    // 只重建任务状态；不要把 pending_/command*_commits_ 也一起置零，它们跨任务防迟到 ACK。
    attitude_core_ = attitude::Core(config_.attitude);
    rc_preparation_timing_fault_ = false;
    last_rc_timing_observation_ = -1;
    message_ = "RC_REPREPARE_NEW_DISARMED_GROUND_AND_CONTROLS_RESET";
    return true;
  }
  // 节点最终动作sink回报；wire_stamp与actual取自真实提交附近，重复完成不能再提交或增加代数。
  void confirm_attitude_command_publication(attitude::Command command, bool allowed,
                                            std::uint64_t wire_stamp, const Snapshot& actual,
                                            double now) {
    if (command == attitude::Command::None)
      return;
    // 命令类别转成 MAV_CMD 编号：进入/退出 Offboard 共用 176，PX4 Land 用 21，其余为 400。
    const unsigned cmd =
        command == attitude::Command::Offboard || command == attitude::Command::ReleaseOffboard
            ? 176
        : command == attitude::Command::Px4Land ? 21
                                                : 400;
    auto& p = pending_[index(cmd)];
    if (!p.requested)
      return;
    // 一个本地完成回报只能提交一次，否则会错误增加进程 generation 并制造虚假的 wire 事实。
    if (p.published)
      return; // 重复本地完成不能再提交一次。
    // 同编号400仍要校验当前意图，不能把旧Arm追踪器当成新Disarm提交。
    if (cmd == 400 &&
        p.command400_kind !=
            (command == attitude::Command::Arm ? Command400Kind::Arm : Command400Kind::Disarm)) {
      attitude_core_.block("COMMAND_400_INTENT_MISMATCH");
      return;
    }
    // validated 记录适配器采用并检查同步域的合同；不等于本次提交后的每个遥测样本都仍新鲜。
    const bool source_clock = actual.attitude.source_clock_validated;
    // 即使提交后的遥测刚好过期，已经实际发出的命令仍保留其同步域因果下界。
    const auto causal_floor = source_clock ? source_causal_floor(wire_stamp) : 0;
    attitude_core_.note_command_publication(
        command, allowed,
        source_clock ? std::max(actual.land_stamp, causal_floor) : actual.land_stamp,
        source_clock ? std::max(actual.status_stamp, causal_floor) : actual.status_stamp);
    // 成功提交后才记录wire/因果下界及物理前状态，并分配本进程generation。
    if (allowed) {
      // 这里的成功只指节点本地 sink 完成；尚未收到 ACK，更没有宣称飞控已采用或飞机已运动。
      p.published = true;
      p.sent = true;
      p.sent_at = now;
      p.wire_stamp = wire_stamp;
      p.source_clock_validated = source_clock;
      p.causal_floor = causal_floor;
      p.status_at_wire = actual.status_stamp;
      p.land_at_wire = actual.land_stamp;
      p.armed_time_at_wire = actual.armed_time;
      p.exclusive_at_wire = actual.feedback.exclusive;
      p.baseline = std::max(p.baseline, actual.status_stamp);
      if (cmd == 400)
        record400_commit(p);
      if (cmd == 176 && !p.generation)
        p.generation = ++command176_commits_;
      p.stage = CommandStage::CommittedAwaitAck;
    } else {
      // 最终边界拒绝是本地结论：result=-1 保持“没观测到 ACK”，不伪造一个 PX4 拒绝码。
      p.published = false;
      p.sent = false;
      p.sink_denied = true;
      p.rejected = true;
      p.rejected_at = now;
      p.result = -1;
      p.stage = CommandStage::FinalSinkRejected;
    }
  }
  // 按当前快照重建最终发布合同，包含参考/reset、权限、RC证据、事务和输出类别。
  // 调用者必须在每个sink前重新采样，不能复用Core计算时的一份上下文。
  attitude::BoundaryContext publication_context(const Snapshot& s, double now, bool connected,
                                                bool control_fresh, bool shutdown = false) const {
    // c 是按值创建的新合同，不是旧快照的引用；调用者必须在每个 sink 前重新生成。
    attitude::BoundaryContext c;
    c.profile = config_.profile;
    c.phase = attitude_core_.phase();
    c.config = config_.attitude;
    c.feedback = attitude_snapshot(s);
    c.shutdown = shutdown || shutdown_latched_;
    c.session_connected = connected;
    c.control_fresh = control_fresh;
    c.operator_lease_live = attitude_core_.operator_lease_live(now);
    if (config_.attitude.rc_navigation_enabled)
      c.operator_lease_live = attitude::RcNavigationStart::rc_live(c.feedback);
    c.rc_manual_arm_proven = attitude_core_.rc_manual_arm_proven();
    c.rc_manual_armed_time = attitude_core_.rc_manual_armed_time();
    c.rc_arm_evidence_pending = attitude_core_.rc_arm_evidence_pending(c.feedback, now);
    c.rc_navigation_zero_allowed = attitude_core_.rc_navigation_zero_allowed(c.feedback, now);
    c.preparation_lease_valid = attitude_core_.preparation_lease_valid(now);
    c.flight_started = attitude_core_.flight_started();
    c.reference_ready = attitude_core_.reference_ready();
    c.reference_z = attitude_core_.reference_z();
    c.reference_z_reset = attitude_core_.reference_z_reset();
    c.reference_vz_reset = attitude_core_.reference_vz_reset();
    c.reference_quat_reset = attitude_core_.reference_quat_reset();
    c.authority = attitude_core_.authority_state();
    c.offboard_observed = attitude_core_.offboard_observed();
    // && 左侧先检查操作员是否干预；括号只豁免已明确请求的 Offboard 退出过程，Kill 不豁免。
    c.feedback.operator_takeover =
        attitude_core_.operator_intervened(c.feedback) &&
        !(attitude_core_.operator_release_requested() && c.feedback.offboard && !c.feedback.kill);
    c.ground_confirmed = attitude_core_.ground_confirmed();
    c.ground_release_pending = attitude_core_.ground_release_pending();
    c.arm_confirmed = attitude_core_.arm_confirmed();
    c.bench_arm_only_active = attitude_core_.bench_arm_only_active();
    c.bench_disarm_proven = attitude_core_.bench_disarm_proven();
    c.bench_sample_guard_ready = attitude_core_.bench_sample_guard_ready();
    c.bench_sample_z = attitude_core_.bench_sample_z();
    c.bench_sample_vz = attitude_core_.bench_sample_vz();
    c.arm_command_committed = attitude_core_.arm_command_committed();
    c.bench_arm_status_baseline = attitude_core_.bench_arm_status_baseline();
    c.bench_arm_local_baseline = attitude_core_.bench_arm_local_baseline();
    c.bench_disarm_status_floor = attitude_core_.bench_disarm_status_floor();
    c.bench_disarm_local_floor = attitude_core_.bench_disarm_local_floor();
    c.bench_disarm_land_floor = attitude_core_.bench_disarm_land_floor();
    c.arm_cancel_pending = attitude_core_.arm_cancel_pending();
    c.px4_land_command_committed = attitude_core_.px4_land_command_committed();
    // 边界重放保护只认真实pending.published，不采用旧fixture的隐式发送状态。
    // 两个条件必须同时满足：当前槽确为 Disarm，且其最终命令 sink 已成功，不能仅凭编号 400。
    c.disarm_command_committed = pending_[index(400)].command400_kind == Command400Kind::Disarm &&
                                 pending_[index(400)].published;
    c.touchdown_baseline = attitude_core_.touchdown_baseline();
    c.yaw_reference = attitude_core_.yaw_reference();
    c.visual_motion_allowed = attitude_core_.visual_current(c.feedback, now);
    c.visual_target_q = attitude_core_.visual_result().q;
    // 发布完整性锁存也关闭边界；即使普通 shutdown 参数为 false，仍不能越过故障门。
    c.shutdown = c.shutdown || s.publication_fault_latched;
    return c;
  }
  // 诊断序列化只展示权限与地面证据；无效数写null，不能把默认0当真实测量。
  std::string authority_json(const Snapshot& s) const {
    const auto& state = attitude_core_.authority_state();
    const auto a = attitude_snapshot(s);
    const auto in = attitude::authority_inputs(config_.attitude, a);
    const auto policy = authority::evaluate(state, in, false);
    const bool restart =
        shutdown_latched_ || s.publication_fault_latched || attitude_core_.restart_required() ||
        (attitude_core_.phase() == attitude::Phase::Safety && !state.takeover_latched);
    const bool takeover = state.takeover_latched || attitude_core_.takeover_cause(a);
    const bool blocked = restart || takeover;
    // 两个无捕获 lambda 仅做 JSON 标量格式转换，不读改 Core，也不保存局部对象的引用。
    auto boolean = [](bool value) {
      return value ? "true" : "false";
    };
    auto number = [](double value) {
      return std::isfinite(value) ? std::to_string(value) : std::string("null");
    };
    const auto& ground = attitude_core_.authority_ground_evidence();
    const char* why = restart    ? "AUTHORITY_SAFETY_RESTART_REQUIRED"
                      : takeover ? "OPERATOR_TAKEOVER_LATCHED"
                                 : authority::name(policy);
    return std::string("\"authority_mode\":") + quote(authority::name(state.mode)) +
           ",\"authority_attempt_mode\":" + quote(authority::name(state.attempt_mode)) +
           ",\"authority_locked\":" + boolean(state.locked) +
           ",\"authority_ready\":" + boolean(!blocked && policy == authority::Reason::Allowed) +
           ",\"authority_reason\":" + quote(why) +
           ",\"rc_link_ready\":" + boolean(in.rc_link_ready) +
           ",\"rc_source_seen\":" + boolean(a.rc_source_seen) +
           ",\"rc_source_valid\":" + boolean(a.rc_source_valid) +
           ",\"rc_source_selected\":" + boolean(a.rc_source_selected) +
           ",\"rc_data_source\":" + std::to_string(unsigned(a.rc_data_source)) +
           ",\"rc_sticks_moving\":" + boolean(a.rc_sticks_moving) +
           ",\"rc_switches_seen\":" + boolean(a.rc_switches_seen) +
           ",\"rc_switches_fresh\":" + boolean(a.rc_switches_fresh) +
           ",\"rc_switches_ready\":" + boolean(in.rc_switches_ready) +
           ",\"rc_mode_slot\":" + std::to_string(unsigned(a.rc_mode_slot)) +
           ",\"rc_arm_switch\":" + std::to_string(unsigned(a.rc_arm_switch)) +
           ",\"rc_kill_switch\":" + std::to_string(unsigned(a.rc_kill_switch)) +
           ",\"rc_kill_latched\":" + boolean(a.kill) + ",\"rc_takeover_ready\":" +
           boolean(!blocked && in.rc_link_ready && in.rc_switches_ready &&
                   in.rc_takeover_confirmed) +
           ",\"rc_takeover_confirmed\":" + boolean(in.rc_takeover_confirmed) +
           ",\"autonomous_failsafe_ready\":" +
           boolean(!blocked && in.autonomous_failsafe_confirmed) +
           ",\"autonomous_failsafe_confirmed\":" + boolean(in.autonomous_failsafe_confirmed) +
           ",\"operator_takeover_latched\":" + boolean(state.takeover_latched) +
           ",\"takeover_reason\":" + quote(attitude_core_.takeover_reason()) +
           ",\"bench_disarm_ground_proven\":" + boolean(attitude_core_.bench_disarm_proven()) +
           ",\"bench_disarm_status_baseline\":" +
           std::to_string(attitude_core_.bench_arm_status_baseline()) +
           ",\"bench_disarm_local_baseline\":" +
           std::to_string(attitude_core_.bench_arm_local_baseline()) +
           ",\"bench_ground_down_tolerance_m\":" +
           number(config_.attitude.bench_ground_down_tolerance_m) +
           ",\"bench_ground_vz_tolerance_mps\":" +
           number(config_.attitude.bench_ground_vz_tolerance_mps) +
           ",\"bench_ground_dwell_s\":" + number(attitude_core_.bench_ground_dwell_seconds()) +
           ",\"stationary_seconds\":" + number(a.stationary_seconds) +
           ",\"required_ground_seconds\":" + number(config_.attitude.ground_seconds) +
           ",\"ground_vz_tolerance\":" + number(config_.attitude.ground_vz_tolerance) +
           ",\"ground_vz_ned\":" + number(a.vz) + ",\"authority_ground_failed_mask\":" +
           std::to_string(attitude_core_.authority_ground_failed_mask(a)) +
           ",\"last_authority_ground_evidence_valid\":" + boolean(ground.valid) +
           ",\"last_authority_ground_failed_mask\":" + std::to_string(ground.failed_mask) +
           ",\"last_authority_ground_stationary_seconds\":" + number(ground.stationary_seconds) +
           ",\"last_authority_ground_vz_ned\":" + number(ground.vz) +
           ",\"last_authority_ground_status_stamp\":" + std::to_string(ground.status_stamp) +
           ",\"last_authority_ground_land_stamp\":" + std::to_string(ground.land_stamp) +
           ",\"last_authority_ground_local_stamp\":" + std::to_string(ground.local_stamp);
  }
  // 地面模式测试未创建时显示NOT_RUN，不伪造测试通过。
  std::string ground_name() const {
    return ground_ ? g::name(ground_->state()) : "NOT_RUN";
  }
  std::string ground_reason() const {
    return ground_ ? ground_->reason() : "NOT_RUN";
  }
  // IPC命令统一入口：先有限值与参数个数，再按profile/RC/关闭/维护权限分派，空参也必须匹配合同。
  // 读法：op 是操作名，a 是数字参数；const & 只借用调用者对象，本函数不会把引用存到成员。
  // 所有分支都通过早 return 结束；“第一个拒绝原因”就是协议行为的一部分，不能随意交换判断。
  Reply request(const std::string& op, const std::vector<double>& a, const Snapshot& s,
                double now) {
    // 入口第一层只校验报文本身：拒绝 NaN/Inf；随后确认参数个数，后面 a[0] 等访问才有边界保证。
    for (double n : a)
      if (!std::isfinite(n))
        return refuse("NONFINITE_ARGUMENT");
    // ?: 从右结合：TAKEOFF 要 1 个数，VEL 要 4 个，其余（含未知名称）暂按 0 个检查。
    const std::size_t arity = op == "TAKEOFF" ? 1 : op == "VEL" ? 4 : 0;
    if (a.size() != arity)
      return refuse("ARGUMENT_COUNT");
    // 两个状态查询在动作权限门之前返回，故关闭过程中仍可查看事实，不触发控制动作。
    if (op == "STATUS")
      return {true, "STATUS"};
    if (op == "AUTHORITY_STATUS")
      return config_.profile == "attitude_altitude" ? Reply{true, "AUTHORITY_STATUS"}
                                                    : refuse("AUTHORITY_REQUIRES_ATTITUDE_PROFILE");
    // 生产人工RC路线控制台只读，唯一动作请求为LAND/STOP；不能通过命令行自动Arm或TAKEOFF。
    if (config_.attitude.rc_navigation_enabled && op != "LAND" && op != "STOP")
      return refuse("RC_NAVIGATION_CONSOLE_READ_ONLY_USE_PHYSICAL_SWITCHES");
    if (shutdown_latched_ && op != "PING" && op != "LAND" && op != "STOP" && op != "UPDATE_CHECK")
      return refuse("SHUTDOWN_NO_NEW_CONTROL_TASKS");
    // 路线分派必须先于通用动作处理；同名 TAKEOFF 在姿态路线与速度路线有不同的 Core 合同。
    if (op == "INTERACTIVE" && config_.profile != "attitude_altitude")
      return {true, "LIVE_CONSOLE_SESSION"};
    if (config_.profile == "attitude_altitude" && op != "MODE" && op != "UPDATE_CHECK") {
      return request_attitude(op, a, s, now);
    }
    // PING 续控制台租约；旧速度租约已到期时只送零目标，不能把过期运动续成新运动。
    if (op == "PING") {
      last_ping_ = now;
      if (now >= velocity_until_)
        core_.receive({}, now);
      return {true, "ALIVE"};
    }
    // 维护授权要求无活动任务、唯一新鲜反馈、未解锁着地静止且非Offboard；检查成功才锁maintenance。
    if (op == "UPDATE_CHECK") {
      if (active() || (ground_ && !ground_->finished()))
        return refuse("TASK_ACTIVE");
      if (!s.status_fresh || !s.land_fresh || s.time_regressed)
        return refuse("GROUND_TELEMETRY_REQUIRED");
      if (s.feedback.armed || !s.feedback.landed || !s.at_rest || s.stationary_seconds < 2 ||
          s.feedback.offboard)
        return refuse("UPDATE_REQUIRES_DISARMED_LANDED_AT_REST_OUTSIDE_OFFBOARD");
      if (!s.feedback.exclusive)
        return refuse("UPDATE_REQUIRES_UNIQUE_STATUS");
      // 这是一次会改变本地状态的“检查”：通过后锁住维护模式，后续不能偷偷开启新任务。
      maintenance_ = true;
      return {true, "UPDATE_ALLOWED"};
    }
    if (op == "STOP") {
      // 复用同一准入顺序，且保留 UPDATE_CHECK 的维护锁副作用；STOP 不直接停止电机。
      auto permit = request("UPDATE_CHECK", {}, s, now);
      return permit.accepted ? Reply{true, "STOP_ALLOWED"} : permit;
    }
    if (maintenance_ && op != "LAND")
      return refuse("MAINTENANCE_RESTART_REQUIRED");
    // 地面模式握手与飞行互斥，必须当前未解锁地面；只创建测试Core，不直接改变飞控。
    if (op == "MODE") {
      if (active() || (ground_ && !ground_->finished()))
        return refuse("TASK_ACTIVE");
      if (s.time_regressed)
        return refuse("TIME_REGRESSION_RESTART_ON_GROUND");
      if (!s.feedback.exclusive)
        return refuse("SOURCE_OR_CONTROL_CONFLICT");
      if (!s.status_fresh || !s.land_fresh || s.feedback.armed || !s.feedback.landed || !s.at_rest)
        return refuse("MODE_REQUIRES_FRESH_DISARMED_GROUND_STATE");
      // unique_ptr 独占这次测试 Core；创建新对象会释放旧对象，Runtime 销毁时也自动释放。
      // 此处只建状态机；step_ground 返回发送意图后，仍由节点负责真正写入 ROS。
      ground_ = std::make_unique<g::Core>();
      ground_origin_ = now;
      return {true, "MODE_PENDING_ACK_AND_NEW_STATUS"};
    }
    // 通用路线请求降落先清旧速度；当前armed但无活动Core时也保留恢复托管与真实模式观察。
    if (op == "LAND") {
      if (ground_ && !ground_->finished())
        return refuse("GROUND_MODE_TEST_ACTIVE");
      if (!s.status_fresh || s.time_regressed || !s.feedback.exclusive)
        return refuse("LAND_REQUIRES_UNIQUE_FRESH_STATUS");
      // 先撤销旧运动，再决定由活动 Core 降落还是建立“无原任务”的恢复观察。
      velocity_until_ = now;
      core_.invalidate();
      if (core_.active())
        core_.land(s.feedback, now, "operator requested landing");
      else if (s.feedback.armed) {
        recover_land_ = true;
        recover_started_ = now;
        recover_nav_ = s.nav_state;
        // 这里只清通用恢复用的 Land 槽；不是清整个进程的姿态路线 400 提交历史。
        pending_[2] = {};
      }
      return {true, s.feedback.armed ? "LAND_PENDING_ACK_AND_STATE" : "ALREADY_DISARMED"};
    }
    // HOLD 只适用于已允许运动的速度 Core；接收零速度表示制动/悬停目标，不表示零推力。
    if (op == "HOLD") {
      if (!core_.accepts_motion())
        return refuse("HOLD_REQUIRES_ACTIVE_FLIGHT");
      last_ping_ = now;
      velocity_until_ = now;
      core_.receive({}, now);
      return {true, "BRAKE_THEN_HOLD"};
    }
    // 仅已稳定起飞的通用路线允许四轴目标；当前反馈与命令包络同时检查，目标租约仅0.5秒。
    if (op == "VEL") {
      if (!core_.accepts_motion())
        return refuse("VELOCITY_REQUIRES_SETTLED_TAKEOFF");
      if (!s.feedback.fresh || !s.feedback.valid || !s.feedback.heading_ok || !s.feedback.exclusive)
        return refuse("VELOCITY_STATE_INVALID");
      // XY 用二维合速度，Z 和 yaw 用绝对值；参数顺序为 vx、vy、vz、yaw_rate。
      if (std::hypot(a[0], a[1]) > config_.control.xy_speed ||
          std::abs(a[2]) > config_.control.z_speed || std::abs(a[3]) > config_.control.yaw_speed)
        return refuse("VELOCITY_EXCEEDS_CONFIGURED_LIMIT");
      last_ping_ = now;
      velocity_until_ = now + .5;
      const bool ok = core_.receive({a[0], a[1], a[2], a[3]}, now);
      return ok ? Reply{true, "VELOCITY_ACCEPTED_0.5S"} : refuse("VELOCITY_REJECTED");
    }
    // 通用/仿真路线另有XY、VXY、航向与辅助定位门，不与姿态高度生产路线混用。
    if (op == "TAKEOFF") {
      if (!config_.flight_enabled || config_.profile == "ground")
        return refuse("FLIGHT_DISABLED");
      if (active() || (ground_ && !ground_->finished()))
        return refuse("TASK_ACTIVE");
      const auto why = hardware_gate(s);
      if (!why.empty())
        return refuse(why);
      if (!s.feedback.fresh || !s.feedback.valid || !s.feedback.heading_ok || s.dead_reckoning)
        return refuse("XY_VXY_Z_VZ_HEADING_VALID_AIDED_ESTIMATE_REQUIRED");
      if (s.stationary_seconds < 2 || !s.at_rest)
        return refuse("GROUND_REST_WINDOW_2S_REQUIRED");
      if (s.time_regressed)
        return refuse("TIME_REGRESSION_RESTART_ON_GROUND");
      if (a[0] <= 0 || a[0] > config_.control.max_height - 1.)
        return refuse("HEIGHT_OUTSIDE_ENVELOPE_1M_MARGIN_REQUIRED");
      if (last_ping_ < 0 || now - last_ping_ > .5)
        return refuse("LIVE_OPERATOR_CONSOLE_REQUIRED");
      // 用局部 candidate 完成验证和 start；任何拒绝都在替换原 core_ 之前发生。
      auto next = config_.control;
      next.height = a[0];
      next.validate();
      v::Core candidate(next);
      candidate.receive({}, now);
      if (!candidate.start(s.feedback, now))
        return refuse(candidate.reason());
      // 只有候选启动成功才移动进成员；此后只是新任务计划，Offboard/Arm 还要各自握手。
      core_ = std::move(candidate);
      pending_ = {};
      velocity_until_ = now;
      message_ = "TAKEOFF_PENDING_OFFBOARD_ACK_ARM_ACK_AND_FEEDBACK";
      return {true, message_};
    }
    return refuse("UNKNOWN_COMMAND");
  }
  // ACK接收：核对目标系统/组件、真实提交、源时钟与事务水位，再按结果分类；from_external不是关联键。
  // a 是本次 ACK 的只读借用，now 是主机接收单调秒；函数返回后不会保存对 a 的引用。
  // 三层过滤依次是：目标身份/已提交 -> 时间与水位 -> 是否可归属和结果分类。
  // 过滤掉的 ACK 不能因到达顺序“看起来对”就推进任务；from_external 也不能充当唯一事务号。
  void ack(const g::Ack& a, double now) {
    // 地面测试拥有自己的 ACK 状态机，先按相对测试时间转交；其结果不替代飞行事务的过滤。
    if (ground_ && !ground_->finished())
      ground_->ack(a, now - ground_origin_);
    if (a.command != 176 && a.command != 400 && a.command != 21)
      return;
    // auto& 是固定数组槽的别名，下面写 p 就是写当前事务；不是拷贝一份待稍后提交。
    auto& p = pending_[index(a.command)];
    // [&] 按引用捕获本函数局部量，只在同步函数体内调用，不逃逸、不放进队列。
    // why 来自静态字符串字面量，写入 p.ack_disposition 后仍有效；只为 176 更新解释。
    const auto disposition = [&](const char* why) {
      if (a.command == 176)
        p.ack_disposition = why;
    };
    // 此处 system/component 是 ACK 的目标（请求者 1/191），不是 PX4 遥测来源的 1/1。
    if (a.system != 1 || a.component != 191) {
      disposition("WRONG_ACK_TARGET");
      return;
    }
    // PX4 v1.16 Commander 的 answer_command() 即使应答外部命令，也可能保持 from_external=false。
    // 所以不能把它当关联键。身份/水位过滤只能排除部分 ACK，不能证明复用 400 的具体 Arm/Disarm。
    // 尚未有可关联提交时，ACK 只能抬高水位，防止未来新请求认领旧应答；不能产生 accepted。
    if (!p.sent) {
      if (a.timestamp > p.baseline)
        p.baseline = a.timestamp;
      disposition("NO_COMMITTED_COMMAND");
      return;
    }
    // bool 只说明时间偏差可能落在“首个正常 176”的窄范围，不代表该 ACK 已被接受。
    const bool skew_candidate =
        a.command == 176 && normal_offboard_candidate_scope(p) && a.timestamp <= p.wire_stamp;
    // 已同步域里同时核对因果下界和本地接收年龄上界；偏斜仅允许首个正常176的窄候选策略。
    if (p.source_clock_validated) {
      if (!std::isfinite(now) || now < p.sent_at) {
        disposition("INVALID_LOCAL_RECEIPT_TIME");
        return;
      }
      // wire 是微秒，经过时长是秒，乘 1e6 后才相加；再加既有同步偏差预算作为未来上界。
      const double receipt_upper =
          double(p.wire_stamp) + (now - p.sent_at) * 1e6 + double(source_clock_skew_budget_us);
      // 正常 176 可能在 10 ms 内应答：ACK 不套“状态证据的 +100 ms”下界，但其独立状态仍要过该水位。
      // 拆桨 176 以及 400/21 的 ACK 继续保留原因果下界，不能借用此处正常飞行的窄例外。
      // 正常飞行 176 与拆桨/地面准备不同：正常路线还会独立审核命令后的新 Offboard 状态。
      const bool flight_mode176 = a.command == 176 && config_.profile == "attitude_altitude" &&
                                  !attitude_core_.ground_mode_state_only_allowed();
      // 使用既有 100 ms 同步偏差预算做饱和减法；预算内仅是候选，不证明跨时钟的因果先后。
      // 无符号整数不可直接减到负数；三目先比较再相减，使下界最小为 0。
      const auto skew_floor = p.wire_stamp > source_clock_skew_budget_us
                                  ? p.wire_stamp - source_clock_skew_budget_us
                                  : 0;
      // 三种下界：偏斜候选用 wire-预算；正常 176 用 wire；其余用更严格的 causal_floor。
      // 无论放宽哪一种 ACK 下界，都没有放宽随后独立状态必须晚于 causal_floor 的要求。
      const std::uint64_t ack_floor = skew_candidate   ? skew_floor
                                      : flight_mode176 ? p.wire_stamp
                                                       : p.causal_floor;
      // <= 是严格“必须更新”的反面；边界相等也不能建立新因果关系。
      if (a.timestamp <= ack_floor || double(a.timestamp) > receipt_upper) {
        ++p.unattributed_clock_acks;
        disposition(a.timestamp <= ack_floor ? "BELOW_ACK_CLOCK_FLOOR"
                                             : "ABOVE_ACK_RECEIPT_BUDGET");
        return;
      }
    }
    // 复用400的结果只记观察，不用于认定接受/拒绝，也不授权重试或新任务；物理结果由独立新状态证明。
    if (a.command == 400 && config_.profile == "attitude_altitude" && p.ack_ambiguous) {
      if (!std::isfinite(now) || now < p.sent_at || a.timestamp <= p.baseline ||
          a.timestamp <= p.last_ack || a.timestamp <= p.status_at_wire)
        return;
      // 对复用的 400，即使 result=ACCEPTED 也只保留“看到过”；不改 accepted/rejected。
      p.last_ack = a.timestamp;
      p.ack_stamp = a.timestamp;
      p.ack_received_at = now;
      p.result = a.result;
      ++p.ack_observations;
      // 超时或仅状态终结后，仍可补记观察。任何结果码都不能使复用 400 获得唯一归属，
      // 也不能据此授权事务完成或重试；不因 ACCEPTED、拒绝类或 IN_PROGRESS 而改变此规则。
      if (now - p.sent_at >= config_.attitude.command_timeout)
        p.timed_out = true;
      p.stage = p.state_verified ? CommandStage::StateOnlyTerminal : CommandStage::Ambiguous400Ack;
      return;
    }
    // 已终结/过期事务不能被迟到ACK复活；重复或更旧源stamp也拒绝。
    // 以下适用于仍可归属的事务：先挡已终结，再挡旧/重复/命令前 ACK，最后检查 5 秒期限。
    if (p.accepted || p.rejected || p.timed_out) {
      disposition("TRANSACTION_ALREADY_TERMINAL");
      return;
    }
    if (now < p.sent_at) {
      disposition("INVALID_LOCAL_RECEIPT_TIME");
      return;
    }
    if (a.timestamp <= p.baseline) {
      disposition("AT_OR_BELOW_TRANSACTION_BASELINE");
      return;
    }
    if (a.timestamp <= p.last_ack) {
      disposition("DUPLICATE_OR_OLDER_ACK");
      return;
    }
    if (a.timestamp <= p.status_at_wire) {
      disposition("AT_OR_BELOW_PRECOMMAND_STATUS");
      return;
    }
    if (now - p.sent_at >= 5.) {
      p.timed_out = true;
      p.ack_candidate = false;
      p.stage = CommandStage::TimedOut;
      disposition("ACK_RECEIVED_AFTER_TIMEOUT");
      return;
    }
    // 所有前置过滤通过后才提交这次 ACK 的事实；分类器可在保留这些事实的基础上更新阶段。
    p.last_ack = a.timestamp;
    p.ack_stamp = a.timestamp;
    p.ack_received_at = now;
    p.result = a.result;
    ++p.ack_observations;
    p.ack_candidate = false;
    disposition("ACK_OBSERVED");
    // ACCEPTED与IN_PROGRESS分别处理，1/2/3/4/6是明确拒绝类；未知400结果保留不确定，不猜测拒绝。
    classify_ack_result(a, p, skew_candidate);
  }
  // 断开控制台即撤销旧目标与租约，已有任务沿原降落路线；不能因UI退出就遗弃恢复。
  void disconnect(const Snapshot& s, double now) {
    if (config_.profile == "attitude_altitude") {
      attitude_core_.disconnect(attitude_snapshot(s), now);
      message_ = attitude_core_.reason();
      return;
    }
    last_ping_ = -1;
    velocity_until_ = now;
    core_.invalidate();
    if (core_.active())
      core_.land(s.feedback, now, "operator console disconnected");
    message_ = "CONSOLE_DISCONNECTED_OLD_MOTION_CLEARED";
  }
  // 控制周期按顺序处理时限恢复、离散安全事件、地面测试、姿态主链或通用路线；只一个路线生成计划。
  // 输入 s 是本轮固定快照，函数不自行收取后来的 ROS 消息；输出是计划，绝不是 sink 结果。
  // 优先级：准备时限恢复 -> 同轮离散危险 -> 地面握手 -> 姿态路线 -> 通用恢复 -> 通用 Core。
  // 每个互斥路线通过 return 收尾，避免一个周期同时生成两条路线的控制目标。
  Output step(const Snapshot& s, double now) {
    // out 的结构体默认初始化为空计划；下面的前置拒绝分支直接返回时，没有生成新的控制输出。
    Output out;
    if (rc_preparation_timing_fault_) {
      // 连续证据不能跨越超时、无效时间或时间回退；先撤销累计窗口，再记录本次观察时间。
      if (!s.control_cycle_fresh || !std::isfinite(now) ||
          (last_rc_timing_observation_ >= 0 &&
           (now < last_rc_timing_observation_ ||
            now - last_rc_timing_observation_ > config_.attitude.stall_seconds)))
        attitude_core_.clear_rc_timing_recovery();
      last_rc_timing_observation_ = now;
      // 当前周期本身不新鲜时，不准用尾部健康状态推进 Core；清本轮计划标记后立即返回。
      if (!s.control_cycle_fresh) {
        last_attitude_output_active_ = false;
        queued_safety_wait_ = false;
        return out;
      }
    }
    // 同轮坏地面事件阻止正常尾帧推进准备/完成；仍armed的触地反弹允许原下降控制接续。
    if (queued_safety_wait_) {
      // 只消费 Runtime 本轮标记；Core 自己保存的危险/恢复否决仍然留存，不在这里清锁存。
      queued_safety_wait_ = false;
      const auto phase = attitude_core_.phase();
      // 排队着陆变化会撤销触地证明；未解锁完成/退出和未接入准备仍须等候，健康尾帧不能越过它。
      // 括号把“Landed 或 RequestDisarm”绑为一组，并要求仍 armed；这允许触地反弹继续原下降托管。
      const bool armed_landing = s.feedback.armed && (phase == attitude::Phase::Landed ||
                                                      phase == attitude::Phase::RequestDisarm);
      if (!armed_landing) {
        // Safety 在观察终态/恢复时保持无输出；所有中间不安全事件的恢复否决由 Core 继续保留。
        attitude_core_.observe_safety_terminal(attitude_snapshot(s), now);
        last_attitude_output_active_ = false;
        return out;
      }
    }
    if (ground_ && !ground_->finished()) {
      return step_ground(s, now);
    }
    // 姿态主链先形成候选状态证据供Core判断，再等Core同轮健康/reset/权限都过才锁定事务完成。
    if (config_.profile == "attitude_altitude") {
      // A. 先算“候选状态证据”。此时还不能把 state_verified 置真，因为 Core 尚未检查本帧健康。
      const bool mode_candidate_state = offboard_candidate_state_evidence(s, now);
      auto& command400 = pending_[index(400)];
      const bool disarm_state = disarm_state_evidence(s, now);
      const bool normal_arm_state = normal_takeoff_arm_state_evidence(s, now);
      // B. 先按原期限记 400 超时；有歧义的事务仍如实显示歧义，而非猜测哪笔命令超时。
      if (command400.sent && !command400.accepted && !command400.rejected &&
          now - command400.sent_at >= config_.attitude.command_timeout) {
        command400.timed_out = true;
        if (!command400.state_verified)
          command400.stage =
              command400.ack_ambiguous ? CommandStage::Ambiguous400Ack : CommandStage::TimedOut;
      }
      // 把ACK和独立状态分别装入Transaction；Core仍需检查当前阶段及物理事实，不是这里直接起飞。
      // terminal 是本次调用内使用的 lambda：this 指向当前 Runtime，三个 bool 按值冻结本帧结论。
      // 它不延后执行、不保存引用到其他线程；返回的 Transaction 是按值传给 Core 的简化证据。
      // 字段顺序为 accepted/rejected/timed_out/ambiguous/state_evidenced；五个 bool 的含义不可互换。
      auto terminal = [this, disarm_state, normal_arm_state, mode_candidate_state](unsigned cmd) {
        const auto& p = pending_[index(cmd)];
        return attitude::Transaction{p.accepted || (cmd == 176 && mode_candidate_state), p.rejected,
                                     p.timed_out, p.ack_ambiguous,
                                     cmd == 400 && (disarm_state || normal_arm_state)};
      };
      // Land21 使用单独的配置期限，不能误套 400 的 command_timeout。
      auto& land21 = pending_[index(21)];
      if (land21.sent && !land21.accepted && !land21.rejected &&
          now - land21.sent_at >= config_.attitude.px4_land_ack_seconds)
        land21.timed_out = true;
      // C. Core 用本帧反馈、候选与 ACK 做真正状态机推进，包括权限、参考/reset、时限和安全检查。
      out.attitude = attitude_core_.step(attitude_snapshot(s), now, terminal(176), terminal(400),
                                         terminal(21));
      // D. Core 同帧通过之后，再把允许的物理状态写成终态；随后才能安全考虑新的同编号动作。
      complete_attitude_transactions(s, now, disarm_state, normal_arm_state, mode_candidate_state);
      // Core的新动作若与未决同编号事务冲突则拒绝输出；不能靠覆盖pending消失旧事务。
      track_attitude_request(out, s, now);
      // 这里记的是“本轮有发布计划”；真实 target/heartbeat 是否完成由节点单独留证，两者不可混称。
      last_attitude_output_active_ = out.attitude.publish;
      message_ = attitude_core_.reason();
      return out;
    }
    // 通用路线失去上层任务后仍观察真实降落/上锁；失联/接管/超时则明确释放给PX4/操作员。
    if (recover_land_) {
      // 通用恢复先查真实落地上锁；否则继续保留恢复，直到来源/时限/模式变化要求明确交还。
      if (s.status_fresh && s.land_fresh && !s.feedback.armed && s.feedback.landed) {
        recover_land_ = false;
        message_ = "LANDED_AND_DISARMED_CONFIRMED";
        return out;
      }
      if (!s.status_fresh || !s.feedback.exclusive || s.feedback.failsafe || s.time_regressed ||
          now - recover_started_ > 60) {
        recover_land_ = false;
        message_ = "RECOVERY_RELEASED_TO_PX4_OPERATOR";
        return out;
      }
      if (!s.feedback.landing && s.nav_state != recover_nav_) {
        recover_land_ = false;
        message_ = "OPERATOR_MODE_CHANGED_RECOVERY_RELEASED";
        return out;
      }
      const auto why = land_handshake_failure(s, now);
      if (!why.empty()) {
        recover_land_ = false;
        message_ = why;
        return out;
      }
      // 通用恢复只计划一次 Land，避免每个控制周期重发同一命令；后续周期观察 ACK 和模式。
      if (!pending_[2].sent) {
        out.flight.command = v::VehicleCommand::Land;
        track(21, s, now);
      }
      return out;
    }
    // 普通空闲可直接返回；Fault 要继续进入旧 Core 的晚到 Arm 恢复检查，不能按“不 active”忽略。
    if (!core_.active() && core_.phase() != v::Phase::Fault)
      return out;
    if (core_.phase() == v::Phase::Fault && s.time_regressed) {
      core_.release(now, "time regression; unresolved ARM released to PX4/operator");
      message_ = core_.reason();
      return out;
    }
    // actual 是反馈副本；下面可调用 land 改 Core 阶段，但不改调用者的 Snapshot。
    auto actual = s.feedback;
    const auto why = hardware_gate(s);
    if (!why.empty() || s.time_regressed || !s.feedback.heading_ok || s.dead_reckoning)
      core_.land(actual, now, why.empty() ? "estimate/time invalid" : why);
    // 多个异常门按原顺序请求降落；不是互斥 else-if，不能用重构改变后续调用是否发生。
    if (last_ping_ < 0 || now - last_ping_ > 3.)
      core_.land(actual, now, "operator heartbeat lost");
    if (s.feedback.fresh && core_.origin().z - s.feedback.position.z > config_.control.max_height)
      core_.land(actual, now, "height envelope exceeded");
    for (std::size_t i = 0; i < 2; ++i) {
      auto& p = pending_[i];
      if (p.sent && !p.accepted && (p.rejected || now - p.sent_at >= 5.)) {
        p.timed_out = !p.rejected;
        core_.land(actual, now, p.rejected ? "PX4 command rejected" : "PX4 command ACK timeout");
      }
    }
    if (core_.phase() == v::Phase::Landing) {
      const auto failed = land_handshake_failure(s, now);
      if (!failed.empty()) {
        core_.release(now, failed);
        message_ = failed;
        return out;
      }
    }
    // 另建 Core 可见副本：在等待 Offboard/Arm 阶段，把“物理已发生”与该命令已接受同时要求。
    auto feedback = actual;
    // 防止仅看到模式改变就绕过 ACK，从而过早生成 Arm。
    if (core_.phase() == v::Phase::Offboard)
      feedback.offboard = actual.offboard && pending_[0].accepted;
    if (core_.phase() == v::Phase::Arm)
      feedback.armed = actual.armed && pending_[1].accepted;
    out.flight = core_.step(feedback, now);
    // 通用路线每个编号只跟踪一次；已有 sent 时消掉重复计划，尚未发送时才登记事务。
    if (out.flight.command != v::VehicleCommand::None) {
      const unsigned cmd = out.flight.command == v::VehicleCommand::Offboard ? 176
                           : out.flight.command == v::VehicleCommand::Arm    ? 400
                                                                             : 21;
      auto& p = pending_[index(cmd)];
      if (p.sent)
        out.flight.command = v::VehicleCommand::None;
      else
        track(cmd, s, now);
    }
    if (!core_.reason().empty())
      message_ = core_.reason();
    return out;
  }

private:
  // 处理姿态高度控制台请求；参数已由 request 按原次序校验，s 是当前只读反馈，now 为 steady 秒。
  // 返回原接受/拒绝文本；只修改原 Core、租约和 message_，不发布、不清事务历史。
  // 必须在 RC 只读门、停机门及 profile 分派之后调用；分支和首个错误的优先级不可交换。
  Reply request_attitude(const std::string& op, const std::vector<double>& a, const Snapshot& s,
                         double now) {
    if (maintenance_ && op != "PING" && op != "LAND" && op != "STOP")
      return refuse("MAINTENANCE_RESTART_REQUIRED");
    if (ground_ && !ground_->finished() && op != "PING")
      return refuse("GROUND_MODE_TEST_ACTIVE");
    // 复制并整合当前快照；下面各 Core 接口拿同一份反馈，避免在一个请求内混用不同观察时刻。
    auto feedback = attitude_snapshot(s);
    std::string why;
    if (op == "PING") {
      attitude_core_.ping(now);
      return {true, "ALIVE"};
    }
    if (op == "INTERACTIVE") {
      attitude_core_.live_console(now);
      return {true, "LIVE_CONSOLE_SESSION"};
    }
    // 计划活跃或任何实际输出仍活跃，都算未停止；这是一种保守的“任一为真就阻止重置”。
    const bool outputs_active =
        last_attitude_output_active_ || s.attitude_output_active || s.offboard_heartbeat_active;
    // 独立状态终止不消除ACK歧义；当前安全地面/无输出仍由Core重新证明。
    // 无捕获 lambda 是本地纯分类器；const Pending& 不复制事务，也不会修改其历史。
    const auto outstanding = [](const Pending& p) {
      // 仅状态确认的安全 Disarm 只结束物理恢复；Core 重置仍要重查当前上锁地面、停止输出和模式。
      // 不因为分类为“非未决”就清掉下面继续保留的 ACK 歧义和历史。
      if (p.state_verified && (p.command400_kind == Command400Kind::Disarm ||
                               p.command400_kind == Command400Kind::None))
        return false;
      return p.sent && (p.ack_ambiguous || (!p.accepted && !p.rejected && !p.timed_out));
    };
    // || 短路在此没有副作用问题：outstanding 是只读分类。这里只按原合同查看 176 和 400。
    const bool transaction = outstanding(pending_[0]) || outstanding(pending_[1]);
    // 权限选择、重置、主动接管不是同一件事：选择模式不代表已经取得当前任务输出权。
    if (op == "AUTHORITY_RC" || op == "AUTHORITY_AUTONOMOUS")
      why = attitude_core_.select_authority(
          op == "AUTHORITY_RC" ? authority::Mode::RcTakeover : authority::Mode::AutonomousNoRc,
          feedback, now, outputs_active, transaction, s.publication_fault_latched);
    else if (op == "AUTHORITY_RESET")
      why = attitude_core_.reset_authority(feedback, now, outputs_active, transaction,
                                           s.publication_fault_latched);
    // 主动请求交还给操作员也要求活的交互会话和活动任务，不能由无人持有的旧命令触发。
    else if (op == "AUTHORITY_TAKEOVER") {
      if (!attitude_core_.operator_lease_live(now))
        return refuse("LIVE_INTERACTIVE_CONSOLE_REQUIRED");
      if (!attitude_core_.active())
        return refuse("NO_ACTIVE_COMPANION_ATTEMPT");
      why = attitude_core_.request_operator_takeover(feedback, now);
    } else if (op == "ATTITUDE_READY")
      why = attitude_core_.prepare(feedback, now);
    // ARM_ONLY 是独立拆桨观察入口；Core 仍会检查它的专用配置和全零推力合同。
    else if (op == "ARM_ONLY") {
      if (command400_commits_ > 0)
        return refuse("COMMAND_400_REUSE_REQUIRES_SAFE_GROUND_RESTART");
      why = attitude_core_.arm_only(feedback, now);
    }
    // 程序起飞禁止复用本进程400，避免永远不可归属的新Arm ACK；人工RC不调用此分支。
    else if (op == "TAKEOFF") {
      // 普通地面重置不能擦除协议历史；新任务不能提交一笔已知永远无法唯一关联 ACK 的 Arm。
      if (command400_commits_ > 0)
        return refuse("COMMAND_400_REUSE_REQUIRES_SAFE_GROUND_RESTART");
      why = attitude_core_.takeoff(feedback, a[0], now);
    } else if (op == "LAND")
      why = attitude_core_.descend(feedback, now);
    else if (op == "STOP") {
      if (!s.status_fresh || !s.land_fresh || s.feedback.armed || !s.feedback.landed || !s.at_rest)
        return refuse("STOP_REQUIRES_CONFIRMED_DISARMED_GROUND_STATE");
      attitude_core_.stop_disarmed(feedback, now);
      return {true, "STOP_ALLOWED"};
    } else
      return refuse("COMMAND_NOT_AVAILABLE_IN_ATTITUDE_PROFILE");
    // Core 用空字符串表达准入成功；有原因就原样拒绝，保留它自己的首拒绝优先级。
    if (!why.empty())
      return refuse(why);
    message_ = op == "ATTITUDE_READY" ? "ATTITUDE_PREPARATION_PENDING_ACK_AND_NEW_STATUS"
                                      : op + "_ACCEPTED";
    return {true, message_};
  }
  // 分类已通过目标、提交、时钟和水位过滤的 ACK；p 必须是该编号的当前事务。
  // skew_candidate 只表示首个正常 176 的偏斜候选；无返回值，只更新事务并沿原通用路线通知拒绝 Arm。
  // 不能提前到过滤前调用：候选、接受、独立物理状态是不同证据，未知 400 不能被猜成拒绝。
  void classify_ack_result(const g::Ack& a, Pending& p, bool skew_candidate) {
    // [&] 按引用捕获本函数局部量，只在同步函数体内调用，不逃逸、不放进队列。
    // why 来自静态字符串字面量，写入 p.ack_disposition 后仍有效；只为 176 更新解释。
    const auto disposition = [&](const char* why) {
      if (a.command == 176)
        p.ack_disposition = why;
    };
    // switch 只做已过滤 ACK 的分类；case 1/2/3/4/6 共用拒绝分支，case 5 刻意不落入接受分支。
    switch (a.result) {
    // ACCEPTED 仍分两种：有时钟歧义先存候选；因果可归属的 ACK 才直接记 accepted。
    case 0:
      if (skew_candidate) {
        p.ack_candidate = true;
        p.stage = CommandStage::AckCandidateAwaitState;
        disposition("SKEWED_ACK_CANDIDATE_AWAIT_STATE");
      } else {
        p.accepted = true;
        p.stage = CommandStage::AckAcceptedAwaitState;
        disposition("ACK_ACCEPTED_AWAIT_STATE");
      }
      break;
    // IN_PROGRESS 只改阶段，accepted/rejected 均保持原值；还要等后续证据。
    case 5:
      p.stage = CommandStage::AckInProgress;
      break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 6:
      p.rejected = true;
      p.stage = CommandStage::Rejected;
      // 通用速度路线保留原 Arm 拒绝处理；姿态路线由自己的阶段分别处理 Arm 和 Disarm 拒绝。
      if (a.command == 400 && config_.profile != "attitude_altitude")
        core_.rejected_arm();
      break;
    default:
      // 176/21/通用速度路线保留原未知码处理；仅姿态路线未知 400 保持不确定，不能猜成拒绝。
      if (a.command != 400 || config_.profile != "attitude_altitude") {
        p.rejected = true;
        p.stage = CommandStage::Rejected;
        if (a.command == 400)
          core_.rejected_arm();
      } else
        p.stage = CommandStage::UnknownAckResult;
      break;
    }
  }
  // 推进一步已存在的地面模式测试；s 为当前反馈，now/ground_origin_ 同属 steady 秒。
  // 返回本轮仅 send_ground 的计划，更新原 ground_ 和 message_；不触碰飞行 Core 或实际 sink。
  // 只能在排队危险事件处理后且测试未终结时调用；过期先 abort，之后才送 status、land 和 tick。
  Output step_ground(const Snapshot& s, double now) {
    Output out;
    // 早返回防止“先用坏反馈推进一次，再 abort”；地面测试和飞行主链的生命周期彼此独立。
    if (s.time_regressed || !s.status_fresh || !s.land_fresh) {
      ground_->abort(s.time_regressed  ? "TIME_REGRESSION_RESTART_ON_GROUND"
                     : !s.status_fresh ? "STATUS_STALE"
                                       : "LAND_STALE");
      message_ = ground_->reason();
      return out;
    }
    // status 与 land 分别送入，顺序保持固定；now-ground_origin_ 是本次测试开始后的秒数。
    ground_->status({s.status_stamp, !s.feedback.armed, s.feedback.failsafe, s.calibration, s.hil,
                     s.stabilized_selectable, s.system, s.component, s.nav_state},
                    now - ground_origin_);
    ground_->land({s.land_stamp, s.feedback.landed, s.at_rest}, now - ground_origin_);
    // 复制交通计数后叠加来源冲突事实；不修改调用者缓存，也不伪装其为独占链路。
    auto traffic = s.traffic;
    if (!s.feedback.exclusive)
      traffic.command_publishers = 2;
    // true 只表示本轮应发送地面模式命令，不是已发送、ACK 到达或模式切换完成。
    out.send_ground = ground_->tick(now - ground_origin_, traffic);
    message_ = ground_->reason();
    return out;
  }
  // 将候选 PX4 新状态写成事务终态；参数来自本轮 Core 调用前的独立取证，now 为 steady 秒。
  // 无返回值，只更新 pending_ 的 176/400 审计；不授权飞行、不清 ACK 歧义、不提交命令。
  // 必须紧接 Core::step 后调用：同一帧需先通过 Core 健康、参考、权限和时限检查，才可锁定完成。
  void complete_attitude_transactions(const Snapshot& s, double now, bool disarm_state,
                                      bool normal_arm_state, bool mode_candidate_state) {
    auto& command400 = pending_[index(400)];
    auto& mode176 = pending_[index(176)];
    // 候选已检查权限，但仍须等 Core 核对本帧健康、参考、租约、接管与周期时限，才可锁定接受。
    // A. 偏斜候选必须等 Core 进入 OffboardReady；准备中的 STOP 也可能直接进入 ReleaseOffboard。
    if (mode_candidate_state && (attitude_core_.phase() == attitude::Phase::OffboardReady ||
                                 attitude_core_.phase() == attitude::Phase::ReleaseOffboard)) {
      mode176.accepted = true;
      mode176.ack_candidate = false;
      mode176.ack_confirmed_by_state = true;
      mode176.state_verified = true;
      mode176.verified_status_stamp = s.status_stamp;
      mode176.verified_at = now;
      mode176.stage = CommandStage::VerifiedTerminal;
      mode176.ack_disposition = "SKEWED_ACK_CONFIRMED_BY_NEW_OFFBOARD_STATE";
    } else if (mode176.ack_candidate &&
               attitude_core_.phase() != attitude::Phase::RequestOffboard) {
      // Core 已离开请求阶段却未通过时，撤掉候选；区分真正超时与被 Core 的其他保护终止。
      mode176.ack_candidate = false;
      mode176.timed_out =
          std::isfinite(now) && now - mode176.sent_at >= config_.attitude.command_timeout;
      mode176.stage =
          mode176.timed_out ? CommandStage::TimedOut : CommandStage::StateAckObservedUnresolved;
      mode176.ack_disposition =
          mode176.timed_out ? "CANDIDATE_STATE_TIMEOUT" : "CANDIDATE_ABORTED_BY_CORE";
    }
    // B. 特定拆桨地面流程允许仅状态终结 176，但仍要求真实提交后的新状态和当前安全地面。
    // 两个模式分支分别是 OffboardReady 的 Offboard(14) 和 Idle 的 Stabilized(15)。
    if (mode176.published && attitude_core_.ground_mode_state_only_allowed() && s.status_fresh &&
        s.land_fresh && !s.feedback.armed && s.feedback.landed && s.at_rest &&
        s.status_stamp > mode176.status_at_wire &&
        (!mode176.source_clock_validated || s.status_stamp > mode176.causal_floor) &&
        ((attitude_core_.phase() == attitude::Phase::OffboardReady && s.feedback.offboard &&
          s.nav_state == 14) ||
         (attitude_core_.phase() == attitude::Phase::Idle && !s.feedback.offboard &&
          s.nav_state == 15))) {
      mode176.state_verified = true;
      mode176.verified_status_stamp = s.status_stamp;
      mode176.verified_at = now;
      mode176.stage =
          mode176.accepted ? CommandStage::VerifiedTerminal : CommandStage::StateOnlyTerminal;
    }
    // 即使 ACK 丢失也保留新物理状态，避免下一次 Disarm 被旧的未决 Arm 误挡并切断正常 Offboard 流。
    // C. 400 的物理状态独立于 ACK：Arm 要 Core 当前仍 active 且已确认解锁；Disarm 要 SafeDisarmed。
    // 因果下界和新 status 条件先过；状态不能仅靠缓存 armed=false 或 ACK=ACCEPTED 推导。
    if (command400.sent && s.arming_state_known && s.status_fresh &&
        (!command400.source_clock_validated || s.status_stamp > command400.causal_floor) &&
        s.status_stamp > command400.status_at_wire && s.status_stamp > command400.baseline &&
        ((command400.command400_kind == Command400Kind::Arm && s.feedback.armed &&
          attitude_core_.active() && attitude_core_.arm_confirmed()) ||
         (command400.command400_kind == Command400Kind::Disarm && disarm_state &&
          attitude_core_.phase() == attitude::Phase::SafeDisarmed))) {
      // 本帧参考 reset、循环停顿、估计跳变和包络仍要过 Core；进入 Safety 不能当作 Disarm 成功。
      command400.state_verified = true;
      command400.verified_at = now;
      command400.verified_status_stamp = s.status_stamp;
      // 阶段选择有优先级：歧义/允许的状态独立完成 -> 已接受 -> 矛盾拒绝 -> 未决观察 -> 缺 ACK。
      // 这是嵌套 ?:，只求值最终选中的枚举值；各条件的先后不能随整理交换。
      command400.stage =
          command400.ack_ambiguous ||
                  ((command400.command400_kind == Command400Kind::Disarm ||
                    (command400.command400_kind == Command400Kind::Arm && normal_arm_state)) &&
                   !command400.accepted)
              ? CommandStage::StateOnlyTerminal
          : command400.accepted         ? CommandStage::VerifiedTerminal
          : command400.rejected         ? CommandStage::StateAckContradictory
          : command400.ack_observations ? CommandStage::StateAckObservedUnresolved
                                        : CommandStage::StateVerifiedAwaitAck;
    }
  }
  // 为 Core 的新动作建立有界事务；out 是本轮可修改计划，s 是因果基线，now 为 steady 秒。
  // 无返回值；冲突时将 Core block 并清计划，否则继承旧水位、归档旧 400 后才 track 新事务。
  // 必须在本轮状态终态取证之后、记录输出活跃位之前执行；禁止覆盖未决同编号命令以制造可重试假象。
  void track_attitude_request(Output& out, const Snapshot& s, double now) {
    if (out.attitude.command != attitude::Command::None) {
      const unsigned cmd = (out.attitude.command == attitude::Command::Offboard ||
                            out.attitude.command == attitude::Command::ReleaseOffboard)
                               ? 176
                           : out.attitude.command == attitude::Command::Px4Land ? 21
                                                                                : 400;
      auto& p = pending_[index(cmd)];
      // 旧同编号命令尚无接受/拒绝/超时/状态终态时，不能覆盖它；否则迟到 ACK 会变得不可解释。
      if (p.sent && !p.accepted && !p.rejected && !p.timed_out && !p.state_verified) {
        // 正常情况下 Core 应延期此动作；若前提仍失守，Runtime 再次 block 并清计划，不能发布任一冲突动作。
        attitude_core_.block("INDISTINGUISHABLE_OUTSTANDING_COMMAND");
        out.attitude = {};
      } else {
        // 新事务继承旧ACK/status最大水位，旧400只进有界审计历史；新的请求不会清零旧wire因果关系。
        // 先取旧事务下界再清槽；p={} 只是重置本槽，不能把旧 ACK/status 的因果边界一并丢掉。
        const auto watermark = std::max({p.baseline, p.last_ack, p.status_at_wire});
        const bool unresolved =
            cmd == 400 && p.sent && (p.ack_ambiguous || !p.accepted || !p.state_verified);
        if (cmd == 400 && p.requested)
          archive_command400(p); // 仅归档旧事务；待处理事务和因果水位仍按原流程更新。
        // 按原次序：归档旧 400 -> 清当前槽 -> 恢复继承水位 -> 记旧不确定性 -> 登记新意图。
        p = {};
        p.baseline = watermark;
        p.previous400_unresolved = unresolved;
        track(cmd, s, now, out.attitude.command);
      }
    }
  }
  // 只保留最近128笔展示审计；当前pending、全局提交次数、generation及ACK下界不参与淘汰。
  void archive_command400(const Pending& transaction) {
    // 这里只保留最近128笔审计记录，避免常驻服务每次任务结束后无限增加内存。
    // 全进程提交次数、每笔generation及ACK因果水位独立保存，不能随展示缓存淘汰而清零。
    constexpr std::size_t retained_transactions = 128;
    if (command400_history_.size() >= retained_transactions) {
      command400_history_.erase(
          command400_history_.begin()); // 淘汰最旧展示记录，保留其余事务的先后顺序。
    }
    command400_history_.push_back(
        transaction); // 复制旧事务证据；该缓存不授予控制权，也不参与ACK匹配。
  }
  // 已结束RC任务的新安全地面可终结Disarm物理结果；仍要求实际提交且status/land晚于因果下界。
  bool ended_rc_disarm_terminal(const Pending& p, const Snapshot& s) const {
    // 此函数依赖 Core 已证明“当前安全恢复就绪”，再核对实际 Disarm 提交和更新的双源地面证据。
    // 它只结清物理结果；不会把晚到 ACK 变成可归属，也不会重置进程提交次数。
    return attitude_core_.rc_fault_recovery_ready() &&
           p.command400_kind == Command400Kind::Disarm && p.sent && p.published && !p.sink_denied &&
           s.arming_state_known &&
           s.status_stamp > std::max({p.baseline, p.status_at_wire, p.causal_floor}) &&
           s.land_stamp > std::max(p.land_at_wire, p.causal_floor);
  }
  // 仅本进程首个真实提交且唯一来源的普通176进入偏斜候选；bench/地面测试不能借用。
  bool normal_offboard_candidate_scope(const Pending& p) const {
    // 从左到右全部成立才返回 true；下面混合的是路线/阶段、提交事实与进程级唯一性，不能只留后者。
    return config_.profile == "attitude_altitude" && config_.attitude.flight_actions_enabled &&
           !config_.attitude.bench_arm_only_enabled && !ground_ &&
           attitude_core_.phase() == attitude::Phase::RequestOffboard && p.requested && p.sent &&
           p.published && !p.sink_denied && p.exclusive_at_wire && p.source_clock_validated &&
           p.wire_stamp > 0 && p.causal_floor > 0 && p.generation == 1 && command176_commits_ == 1;
  }
  // 候选ACK之后仍要当前权限、健康、新Offboard且未解锁地面，严格晚于ACK及命令水位。
  bool offboard_candidate_state_evidence(const Snapshot& s, double now) const {
    const auto& p = pending_[index(176)];
    // 三组证据合取：候选未过期且权限仍允许 -> 当前健康唯一来源 -> 命令/ACK 之后的新 Offboard 状态。
    // 所有 > 都是严格更新；仅重复同一个“好状态”不能让候选完成。
    return normal_offboard_candidate_scope(p) && p.ack_candidate && p.result == 0 && !p.accepted &&
           !p.rejected && !p.timed_out && std::isfinite(now) && now >= p.sent_at &&
           now >= p.ack_received_at && now - p.sent_at < config_.attitude.command_timeout &&
           authority::evaluate(
               attitude_core_.authority_state(),
               attitude::authority_inputs(config_.attitude, attitude_snapshot(s))) ==
               authority::Reason::Allowed &&
           // 源时钟当前仍有效，不能只凭提交那一刻 validated 就使用现在已过期的反馈。
           s.attitude.source_clock_valid && s.attitude.source_clock_validated &&
           !s.time_regressed && s.system == 1 && s.component == 1 && s.arming_state_known &&
           s.status_fresh && s.land_fresh && s.local_fresh && s.feedback.exclusive &&
           !s.feedback.failsafe && !s.feedback.armed && s.feedback.landed && s.at_rest &&
           // 同时检查模式布尔和固定枚举，并要求当前 status 晚于所有独立下界。
           s.feedback.offboard && s.nav_state == 14 && s.status_stamp > p.status_at_wire &&
           s.status_stamp > p.baseline && s.status_stamp > p.ack_stamp &&
           s.status_stamp > p.causal_floor;
  }
  // 即使取得外部解锁新事件，Core/节点仍执行原健康、权限、估计与最终发布门。
  // 旧/矛盾 ACK、未成功 sink 或期限后的解锁事件，都不能靠此后备进入成功路线。
  // 程序普通TAKEOFF首个400的窄后备：真实提交、新external解锁事件、RC解锁OFF、唯一来源及完整新状态。
  // 这不用于人工RC路线，也不声称armed reason能在wire上唯一认领发送者。
  // 这是程序 TAKEOFF 专用后备，不是人工 RC 解锁证据：首笔提交、唯一来源和新 external 事件缺一不可。
  bool normal_takeoff_arm_state_evidence(const Snapshot& s, double now) const {
    const auto& p = pending_[index(400)];
    // 从左到右全部成立才返回 true；下面混合的是路线/阶段、提交事实与进程级唯一性，不能只留后者。
    return config_.profile == "attitude_altitude" && config_.attitude.flight_actions_enabled &&
           config_.attitude.arm_enabled && config_.attitude.takeoff_enabled &&
           !config_.attitude.bench_arm_only_enabled &&
           attitude_core_.phase() == attitude::Phase::RequestArm &&
           attitude_core_.flight_started() && p.sent && p.published && !p.sink_denied &&
           p.command400_kind == Command400Kind::Arm && p.generation == 1 &&
           command400_commits_ == 1 && command400_history_.empty() && !p.previous400_unresolved &&
           !p.rejected && !p.timed_out && p.source_clock_validated &&
           s.attitude.source_clock_validated && p.wire_stamp > 0 && p.causal_floor > 0 &&
           std::isfinite(now) && now >= p.sent_at &&
           now - p.sent_at < config_.attitude.command_timeout && s.arming_state_known &&
           s.feedback.armed && s.armed_time > p.armed_time_at_wire && s.armed_time > 0 &&
           // reason=4 只说明外部命令解锁；它不含发送者 nonce，因此仍必须同时通过上面的唯一性限制。
           s.latest_arming_reason == 4 && // PX4 的外部命令解锁原因 COMMAND_EXTERNAL
           s.status_stamp > p.status_at_wire && s.status_stamp > p.baseline &&
           s.status_stamp > p.causal_floor && s.status_fresh && s.land_fresh && s.local_fresh &&
           s.feedback.offboard && s.nav_state == 14 && s.feedback.landed && s.feedback.exclusive &&
           !s.feedback.failsafe && s.attitude.rc_link_ready && s.attitude.rc_switches_ready &&
           // 固定手动开关枚举 3 为 OFF；排除人工 RC 解锁事件混入程序 Arm 的状态证据。
           s.attitude.rc_arm_switch == 3 && !s.attitude.kill && !s.attitude.operator_takeover &&
           !s.time_regressed && s.feedback.preflight;
  }
  // 上锁状态必须在真实Disarm后，仍处原健康/权限/参考包络并有新地面证据；不据当前armed=false单独完成。
  bool disarm_state_evidence(const Snapshot& s, double now) const {
    const auto& p = pending_[index(400)];
    // 先拒绝未发送/拒绝提交/状态未知，再检查所有源戳和模式；任何一项失败都不会继续构造边界。
    if (!p.sent || p.sink_denied || !s.arming_state_known ||
        p.command400_kind != Command400Kind::Disarm ||
        (p.source_clock_validated &&
         (s.status_stamp <= p.causal_floor || s.land_stamp <= p.causal_floor)) ||
        s.status_stamp <= p.status_at_wire || s.status_stamp <= p.baseline ||
        s.land_stamp <= p.land_at_wire || s.system != 1 || s.component != 1 ||
        !s.feedback.offboard || s.nav_state != 14 || s.feedback.armed)
      return false;
    // 这里只复用边界的健康/权限/参考检查，不调用发布 sink。
    // 传入 true 是该局部状态证据评估的上下文参数，真实发布前节点仍必须用真实连接/周期事实再核验。
    const auto c = publication_context(s, now, true, true);
    return attitude::boundary_health(c).allowed &&
           attitude::flight_gates(config_.attitude).allowed &&
           attitude::boundary_authority(c).allowed && c.reference_ready && c.feedback.landed &&
           c.feedback.at_rest &&
           // ?: 拆桨走专用已上锁地面包络；普通任务走原参考高度和速度容差，不能互相替代。
           (c.bench_arm_only_active && c.bench_disarm_proven
                ? attitude::bench_disarmed_ground_observation(config_.attitude, c.feedback)
                : (std::isfinite(c.reference_z) &&
                   std::abs(c.reference_z - c.feedback.z) <= config_.attitude.height_tolerance &&
                   std::abs(c.feedback.vz) <= config_.attitude.speed_tolerance));
  }
  // 只有首次提交为事务分配generation；本进程复用后永久标记ACK歧义，地面任务重建不清它。
  void record400_commit(Pending& p) {
    // 非零代数表示同一笔已分配过，重复本地完成不能二次递增。++ 前缀先增加再把新值赋给 p。
    if (p.generation)
      return;
    p.ack_ambiguous = command400_commits_ > 0;
    p.generation = ++command400_commits_;
  }
  // 把通用快照事实统一填到姿态Feedback，实际arming未知则status_fresh=false；电池还需电压/连接/警告都健康。
  attitude::Feedback attitude_snapshot(const Snapshot& s) const {
    // auto 按值复制；后续覆盖统一事实，不会回写 s.attitude，也不保存对快照的悬空引用。
    auto a = s.attitude;
    // “消息新鲜”与“解锁枚举已知”都成立，才让姿态 Core 把当前 status 当可用。
    a.status_fresh = s.status_fresh && s.arming_state_known;
    a.land_fresh = s.land_fresh;
    a.local_fresh = s.local_fresh;
    a.status_stamp = s.status_stamp;
    a.land_stamp = s.land_stamp;
    a.stationary_seconds = s.stationary_seconds;
    a.armed = s.feedback.armed;
    a.landed = s.feedback.landed;
    a.at_rest = s.at_rest;
    a.armed_time = s.armed_time;
    a.latest_arming_reason = s.latest_arming_reason;
    a.offboard = s.feedback.offboard;
    a.failsafe = s.feedback.failsafe;
    a.exclusive = s.feedback.exclusive;
    a.time_regressed = s.time_regressed;
    a.calibration = s.calibration;
    a.hil = s.hil;
    a.preflight = s.feedback.preflight;
    // 电池各条件全部与在一起；std::isfinite 防止 NaN/Inf 电压穿过普通大小比较。
    a.battery_ready = s.battery_fresh && s.battery_connected && s.power_valid &&
                      s.battery_warning == 0 && std::isfinite(a.battery_voltage) &&
                      a.battery_voltage > 0;
    a.manual_ready = s.manual_fresh && !s.manual_lost;
    a.stabilized_selectable = s.stabilized_selectable;
    // 这些 bool 由固定 PX4 nav_state 推导；不根据 RC 档位猜测飞控已经切到哪个模式。
    a.stabilized_selected = s.nav_state == 15;
    a.auto_land_selected = s.nav_state == 18;
    return a;
  }
  // 固定三事务槽：176模式→0，400解上锁→1，21降落→2；调用处已限定合法命令。
  static std::size_t index(unsigned cmd) {
    return cmd == 176 ? 0 : cmd == 400 ? 1 : 2;
  }
  // 通用LAND等待ACK和真实Land/上锁状态；有ACK无模式仍是未完成，超时明确释放而不重复命令。
  std::string land_handshake_failure(const Snapshot& s, double now) {
    auto& p = pending_[2];
    // 尚未发 Land、不再需要握手、或真实落地上锁时无失败；空字符串是“此处没有失败原因”。
    if (!p.sent || (s.status_fresh && s.feedback.landing) ||
        (s.status_fresh && s.land_fresh && !s.feedback.armed && s.feedback.landed))
      return {};
    if (p.rejected)
      return "LAND_REJECTED_RELEASED_TO_PX4_OPERATOR";
    if (now - p.sent_at >= 5.) {
      p.timed_out = !p.accepted;
      return p.accepted ? "LAND_ACK_ACCEPTED_MODE_TIMEOUT_RELEASED_TO_PX4_OPERATOR"
                        : "LAND_ACK_TIMEOUT_RELEASED_TO_PX4_OPERATOR";
    }
    return {};
  }
  // 先记录意图与基线；生产sink模式sent=false，等confirm实际提交后才允许ACK影响事务。
  void track(unsigned cmd, const Snapshot& s, double now,
             attitude::Command intent = attitude::Command::None) {
    auto& p = pending_[index(cmd)];
    // track 只登记本地意图；生产姿态路线先把 sent 保持 false，等最终 sink 回报。
    p.requested = true;
    p.sent = !(attitude_sink_commit_required_ && config_.profile == "attitude_altitude");
    p.command400_kind = intent == attitude::Command::Arm      ? Command400Kind::Arm
                        : intent == attitude::Command::Disarm ? Command400Kind::Disarm
                                                              : Command400Kind::None;
    // 旧纯 Core fixture 使用隐式发送模型；published 仍为 false，避免伪造真实节点提交事实。
    p.stage = p.sent ? CommandStage::CommittedAwaitAck : CommandStage::Planned;
    p.published = false;
    p.sent_at = now;
    // 继承旧 ACK 水位与本次状态水位中的更大者；新任务不能回到旧事务之前的时间线。
    p.baseline = std::max(p.baseline, s.status_stamp);
    if (cmd == 400 && config_.profile == "attitude_altitude") {
      p.ack_ambiguous = command400_commits_ > 0;
      if (p.sent) {
        p.status_at_wire = s.status_stamp;
        p.land_at_wire = s.land_stamp;
        p.armed_time_at_wire = s.armed_time;
        record400_commit(p);
      }
    }
    // 仅旧隐式模型即时通知 Core；真实 sink 模型在 confirm_* 中通知，不能通知两次。
    if (p.sent && intent != attitude::Command::None)
      attitude_core_.note_command_publication(intent, true, s.land_stamp);
  }
  // 统一保存拒绝原因供控制台/状态报告，不改变物理飞行状态。
  Reply refuse(const std::string& why) {
    message_ = why;
    return {false, why};
  }
  // 通用路线先看来源冲突和HIL/校准；真实flight还要求电池与人工接管信号，sim例外仅属该profile。
  std::string hardware_gate(const Snapshot& s) const {
    if (!s.feedback.exclusive)
      return "SOURCE_OR_CONTROL_CONFLICT";
    if (s.calibration || s.hil)
      return "CALIBRATION_OR_HIL_ACTIVE";
    if (config_.profile != "sim") {
      if (!s.battery_fresh || !s.battery_connected || !s.power_valid || s.battery_warning != 0)
        return "BATTERY_NOT_READY";
      if (!s.manual_fresh || s.manual_lost)
        return "MANUAL_TAKEOVER_SIGNAL_REQUIRED";
    }
    return {};
  }
  // 单实例持有互斥路线及地面握手；下面的单调秒租约、进程事务计数与任务Core生命周期分离。
  // 拥有型成员按声明顺序构造、按相反顺序销毁；两条飞行 Core 常驻，但每轮只分派一条。
  Config config_;
  v::Core core_;
  attitude::Core attitude_core_;
  // 空指针表示没有地面测试；独占所有权随 Runtime 释放，无需手动 delete。
  std::unique_ptr<g::Core> ground_;
  // 主机单调秒：控制台心跳时刻、速度租约截止、地面测试原点、通用恢复起点。
  double last_ping_{-1}, velocity_until_{-1}, ground_origin_{}, recover_started_{};
  // 通用降落恢复可结束；维护/关闭锁阻止新任务，并非一般错误提示字符串。
  bool recover_land_{}, maintenance_{}, shutdown_latched_{};
  // 分别是上一轮发布计划、生产真实提交模型开关、本轮离散安全事件的推进否决。
  bool last_attitude_output_active_{}, attitude_sink_commit_required_{}, queued_safety_wait_{};
  // 通用恢复开始时的模式，用于观察操作员后来主动切模式并交还控制。
  int recover_nav_{};
  // 固定三槽无动态扩容：176 模式、400 解/上锁、21 PX4 Land；每槽保存当前事务证据。
  std::array<Pending, 3> pending_{};
  // 全进程真实/隐式提交代数，跨 RC 任务重建保留；不能按审计缓存大小或任务次数推导。
  unsigned command400_commits_{}, command176_commits_{};
  // 最多 128 笔旧 400 的展示审计；清理这段历史不解除因果约束，不授予控制权。
  std::vector<Pending> command400_history_;
  // 已结束的 RC 尝试计数与上次原因，用于解释“新任务”与“同任务重试”的边界。
  std::uint64_t rc_attempts_completed_{};
  std::string last_rc_attempt_reason_;
  // 只标记有明确类型的未解锁地面准备时限故障；其他发布完整性故障不能借此窄路清除。
  bool rc_preparation_timing_fault_{};
  // 最近一次恢复观察的主机单调秒；-1 表示还没有观察过，用于检测窗口中断。
  double last_rc_timing_observation_{-1};
  // 诊断文本没有控制权；真正是否能发布由阶段、权限、证据与最终边界共同决定。
  std::string message_{"IDLE_NO_AUTOMATIC_ARM"};
};
} // namespace fc01
