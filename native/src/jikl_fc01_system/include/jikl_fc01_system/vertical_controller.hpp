#pragma once
// 垂向推力闭环：把 PX4 的 NED 向下位置/速度转换为相对地面向上的高度/速度，再计算归一化推力。
// 本模块不发布 ROS 消息、不解锁、不判断控制权；Core 决定何时调用，最终发布边界决定是否允许送给 PX4。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace fc01::attitude {
// 数据流：Core选择当前阶段/目标 → VerticalSample → step/ramp → VerticalOutput。
// 注意这里的“推力0.5”是归一化命令，不是0.5N、50%油门实测值或离地证明；硬件效果需另行取证。
// 必须由候选 YAML 明确提供的物理增益和包络；-1 是拒绝使用的哨兵，不是可用的飞行默认值。
struct VerticalConfig {
  // hover/minimum/maximum 为归一化推力 0..1；slew 为每秒允许改变的推力幅度。
  double hover{-1}, minimum{-1}, maximum{-1}, slew{-1};
  // height_gain 为1/s，高度误差转m/s；velocity_gain 为s/m，速度误差转归一化推力；integral_gain 为1/m。
  // 积分累积速度误差×dt（单位m）后转推力；integral_limit 是归一化推力幅值上限。
  double height_gain{-1}, velocity_gain{-1}, integral_gain{-1}, integral_limit{-1};
  // 相对地面最高高度单位 m；最大上升/下降速度为正幅值 m/s，计算时再赋方向。
  double max_height{-1}, max_climb{-1}, max_descent{-1};
  // 允许相对地面出现的有限负高度余量，单位m，用于吸收估计误差；超过余量仍拒绝闭环。
  double ground_lower_margin{.25}; // 只容许有界的负相对高度
  // 构造时先拒绝非有限值，再检查推力顺序、正增益和高度包络；异常向上传递，阻止用未确认参数启动。
  void validate() const {
    for (double value : {hover, minimum, maximum, slew, height_gain, velocity_gain, integral_gain,
                         integral_limit, max_height, max_climb, max_descent})
      if (!std::isfinite(value))
        throw std::invalid_argument("NONFINITE_VERTICAL_PARAMETER");
    if (minimum < 0 || maximum > 1 || minimum >= hover || hover >= maximum || slew <= 0 ||
        height_gain <= 0 || velocity_gain <= 0 || integral_gain < 0 || integral_limit <= 0 ||
        max_height <= 0 || max_height > 5 || max_climb <= 0 || max_descent <= 0 ||
        !std::isfinite(ground_lower_margin) || ground_lower_margin <= 0 ||
        ground_lower_margin >= max_height)
      throw std::invalid_argument("EXPLICIT_VALIDATED_VERTICAL_PARAMETERS_REQUIRED");
  }
};
// 调用者提供同一控制快照；z0/z 是 PX4 NED 向下坐标 m，vz 是向下为正的 m/s。
struct VerticalSample {
  double z0{}, z{}, vz{};
  // fresh 必须同时体现接收与原始采样新鲜度；valid 位由 PX4 和适配器检查提供，不能靠数值有限代替。
  bool fresh{}, z_valid{}, vz_valid{};
  // PX4 估计器重置计数；闭环开始后改变意味着旧地面零点不再能直接沿用。
  std::uint8_t z_reset{}, vz_reset{};
};
// 返回本轮计算及证据；healthy=false 时只有 reason 可用于故障判断，不能把默认 thrust=0 当成获准输出。
struct VerticalOutput {
  bool healthy{}, saturated{}, slew_limited{};
  // thrust 为向上的正推力幅值；height 为向上 m，speed/velocity_target 为向上 m/s。
  double thrust{}, height{}, speed{}, velocity_target{};
  std::string reason;
  bool closed_loop{}, takeoff_limited{};
  // 日志分别保留比例项、积分项、未限幅推力及当前上限，便于区分动力饱和与起飞调度限制。
  double proportional_thrust{}, integral_thrust{}, raw_thrust{}, thrust_ceiling{};
};
// 有状态 PI 速度内环加比例高度外环；last_thrust_ 和积分在起飞、制动及降落间连续使用。
class VerticalController {
public:
  // 复制并校验配置；不会读取硬件或推测机体悬停推力。
  explicit VerticalController(VerticalConfig config) : c_(config) {
    // : c_(config) 是成员初始化列表，先把参数复制进本对象，再运行函数体；校验失败时构造不完成。
    c_.validate();
  }
  // 新任务显式重置推力记忆、积分和估计器计数；initial 必须是调用者认可的起始推力。
  void reset(double initial = 0) {
    // 成员名结尾的_表示跨控制步保留的记忆；局部变量则在本次函数返回后结束寿命。
    last_thrust_ = initial;
    integral_ = 0;
    reset_set_ = false;
    integral_frozen_ = false;
  }
  // 只读返回当前积分推力，供诊断和回归检查，不改变闭环状态。
  double integral() const {
    return integral_;
  }
  // 地面推力爬坡等阶段可冻结积分，避免尚未离地时把高度误差累计为过大推力。
  void freeze_integral(bool frozen) {
    integral_frozen_ = frozen;
  }
  // 只在首次切入高度反馈时保留已建立的有界余量；随后同一slew记忆可立即用于制动，积分也可退回。
  // 切入高度反馈时只保留实际已建立、且受 integral_limit 约束的悬停以上余量；它不是推力下限。
  void retain_lift_reserve() {
    // std::clamp(x,lo,hi)只截到区间：例如已爬到0.53、hover=0.50，就最多保留0.03积分修正。
    // 后续速度反馈仍可把推力减下来，因此这条赋值不是“飞起来以后推力不得低于0.53”。
    integral_ = std::clamp(last_thrust_ - c_.hover, 0., c_.integral_limit);
  }
  // 开环起飞/停桨斜坡也使用同一 slew 状态；desired 是归一化幅值，dt 是正秒数。
  VerticalOutput ramp(double desired, double dt) {
    if (!std::isfinite(desired) || !std::isfinite(dt) || dt <= 0)
      return bad("INVALID_RAMP");
    VerticalOutput o;
    // o 是本次调用独有的结果对象；healthy在输入通过后置真，不等于发布者已发送或飞机已有响应。
    o.healthy = true;
    // 先限制物理输出范围，再限制相邻控制周期的变化；饱和和变化率受限分别记录。
    const double bounded = std::clamp(desired, 0., c_.maximum);
    o.saturated = bounded != desired;
    o.thrust = limit(bounded, dt);
    o.slew_limited = o.thrust != bounded;
    return o;
  }
  // 闭环入口：target 为向上相对高度 m，velocity_feedforward 为轨迹向上速度 m/s，takeoff_ceiling 为本轮推力上限。
  // 返回前先检查输入/重置/高度，再做高度外环、速度 PI、抗积分饱和和变化率限制，顺序不可随意调整。
  VerticalOutput step(const VerticalSample& s, double target, double dt,
                      double velocity_feedforward = 0, double takeoff_ceiling = 1.) {
    // const VerticalSample&借用同一快照，不复制/修改它；dt必须由调用者提供正的秒间隔。
    // if内声明的error是const char*错误文本指针：nullptr表示无错，非空才走return，原字符串不需释放。
    if (const auto error = input_error(s, target, dt, velocity_feedforward, takeoff_ceiling))
      return bad(error);
    // 已锁定估计器参考后出现重置立即拒绝，避免跨坐标原点计算高度。
    if (reset_set_ && (s.z_reset != z_reset_ || s.vz_reset != vz_reset_))
      return bad("VERTICAL_ESTIMATOR_RESET");
    reset_set_ = true;
    // 在高度包络检查之前记住本帧reset版本是既有副作用顺序；不能把后面的早退理解成“全步没改状态”。
    z_reset_ = s.z_reset;
    vz_reset_ = s.vz_reset;
    // PX4 向下为正：地面 z0 减当前位置 z 才是向上高度；向上速度因此等于 -vz。
    VerticalOutput o;
    o.height = s.z0 - s.z;
    o.speed = -s.vz;
    // 算例：地面z0=2m，当前位置z=0.5m，得到height=1.5m；vz=-0.2m/s对应向上0.2m/s。
    // 高度必须处于飞行包络，允许有限的地面以下余量吸收估计误差；目标本身仍不能为负。
    if (!std::isfinite(o.height) || o.height > c_.max_height ||
        o.height < -c_.ground_lower_margin || target < 0 || target > c_.max_height)
      return bad("HEIGHT_ENVELOPE_EXCEEDED");
    // 外环把高度误差变成向上速度，再叠加移动参考的前馈；下降使用负 max_descent，上升使用正 max_climb。
    o.velocity_target = std::clamp(velocity_feedforward + c_.height_gain * (target - o.height),
                                   -c_.max_descent, c_.max_climb);
    // 数学含义：v目标 = v前馈 + Kh×(h目标-h实测)。Kh的1/s乘m得到m/s。
    // 高度达到移动参考时误差虽为零，v前馈仍保持轨迹运动；不能“简化”为仅高度比例控制。
    // 速度误差为目标减实测，正误差要求增加向上推力，负误差要求制动。
    const double error = o.velocity_target - o.speed;
    // 先试算积分并按幅度限幅，尚不提交；后面用实际可达推力决定是否接受该积分。
    const double trial = integral_frozen_ ? integral_
                                          : std::clamp(integral_ + c_.integral_gain * error * dt,
                                                       -c_.integral_limit, c_.integral_limit);
    // ?: 仅计算被选中的一边：冻结时直接沿用积分；否则Ki(1/m)×误差(m/s)×dt(s)形成无量纲推力。
    // trial只是候选值，后面accept_integral决定是否写回integral_；不要在这里提前提交。
    // 推力由配置的悬停基准、速度比例修正和积分修正组成；未限幅值保留用于抗饱和判断。
    double raw = c_.hover + c_.velocity_gain * error + trial;
    if (!std::isfinite(raw))
      return bad("NONFINITE_VERTICAL_OUTPUT");
    // 起飞调度上限不能超过机体物理上限，必须在积分更新之前纳入可达输出。
    const double ceiling = std::min(takeoff_ceiling, c_.maximum);
    // 早期爬坡上限若低于空中最小推力，则允许降至零，避免上下限重合而失去制动余地。
    const double minimum = ceiling < c_.minimum ? 0. : c_.minimum;
    const double bounded = std::clamp(raw, minimum, ceiling);
    // 把物理限幅后的推力再投影到本周期 slew 可达范围；积分要对这一步的限制也保持清醒。
    const double reachable =
        last_thrust_ + std::clamp(bounded - last_thrust_, -c_.slew * dt, c_.slew * dt);
    // reachable是“本步按变化率能到哪里”的试算，尚未写last_thrust_；真正更新只能由末尾limit完成一次。
    // 不能对饱和/变化率受限方向继续积累积分；允许已有积分退零，但受限时不得穿零后反向积累。
    accept_integral(trial, raw, reachable, error);
    // 用实际接受的积分重新算推力，保证日志和最后命令反映同一状态，而不是未提交的 trial。
    raw = c_.hover + c_.velocity_gain * error + integral_;
    if (!std::isfinite(raw))
      return bad("NONFINITE_VERTICAL_OUTPUT");
    const double command = std::clamp(raw, minimum, ceiling);
    // saturated 只表示物理包络，起飞的临时 ceiling 单独记 takeoff_limited，避免误触持续饱和计时。
    o.saturated = raw < minimum || raw > c_.maximum;
    o.closed_loop = true;
    o.takeoff_limited = raw > ceiling && ceiling < c_.maximum;
    o.proportional_thrust = c_.velocity_gain * error;
    o.integral_thrust = integral_;
    o.raw_thrust = raw;
    o.thrust_ceiling = ceiling;
    o.thrust = limit(command, dt);
    o.slew_limited = o.thrust != command;
    o.healthy = true;
    // 按值返回o：调用者取得计算结果，积分/上次推力仍留在这个Controller对象里用于下一步。
    return o;
  }

private:
  // 在原可达推力已算出后决定是否接受试算积分；各参数都是本次 step 已按原公式算出的标量。
  // 仅更新 integral_；冻结时不写入。先允许不加剧约束的积分，再允许已有积分退零，不能交换判断顺序。
  // 必须在 trial→raw→reachable 之后、重新计算 raw 和调用 slew 限制之前执行，不重排浮点运算。
  void accept_integral(double trial, double raw, double reachable, double error) {
    // (raw-reachable)是当前约束挡住的推力方向；乘error<=0表示积分不会继续顶住该约束。
    // else if必须依附前一个if：先接受不加剧约束的候选，否则才考虑已积累积分退零。
    // 只有误差不会继续把输出推向受限方向时才提交 trial；否则禁止继续积累饱和方向的误差。
    if (!integral_frozen_ && (raw - reachable) * error <= 0)
      integral_ = trial;
    // 受限时允许已有积分向零退回，但夹在旧积分与零之间，防止穿零后向另一方向继续 windup。
    else if (!integral_frozen_ && integral_ * error < 0)
      integral_ = std::clamp(trial, std::min(0., integral_), std::max(0., integral_));
  }
  // 只读核验本轮闭环输入，按原优先级返回首个英文错误码，成功返回 nullptr。
  // s 使用 NED m/m/s，target 为上正 m，dt 为秒，feedforward 为上正 m/s，ceiling 为归一化推力。
  // 不更新估计器计数、积分或推力；必须先调用，再让 step 接受 reset 水位和计算外环。
  static const char* input_error(const VerticalSample& s, double target, double dt,
                                 double velocity_feedforward, double takeoff_ceiling) {
    // static辅助函数不依赖this（当前对象）；只读输入并返回字面量指针，不分配错误字符串。
    // 缺少当前可信 Z/Vz 就拒绝本轮闭环；旧样本不能靠重复调用恢复有效。
    if (!s.fresh || !s.z_valid || !s.vz_valid)
      return "Z_VZ_STALE_OR_INVALID";
    if (!std::isfinite(s.z0) || !std::isfinite(s.z) || !std::isfinite(s.vz) ||
        !std::isfinite(target) || !std::isfinite(dt) || dt <= 0 ||
        !std::isfinite(velocity_feedforward))
      return "NONFINITE_VERTICAL_INPUT_OR_DT";
    if (!std::isfinite(takeoff_ceiling) || takeoff_ceiling < 0 || takeoff_ceiling > 1)
      return "INVALID_TAKEOFF_THRUST_CEILING";
    return nullptr;
  }
  // 失败仅返回不健康结果和原始原因；调用者负责后续状态转换，不能在这里自行重建参考。
  static VerticalOutput bad(const char* reason) {
    VerticalOutput o;
    // o的bool/数字按成员默认值初始化，healthy仍为false。0推力只是无效结果的默认字段，不是停机命令。
    o.reason = reason;
    return o;
  }
  // 按每秒变化率乘 dt 更新唯一推力记忆；起飞和降落共用它，避免模式切换造成推力跳变。
  double limit(double target, double dt) {
    // +=既读取旧推力又写回新推力；起飞/制动/下降共用此记忆，所以不能给不同阶段各建一个Controller。
    last_thrust_ += std::clamp(target - last_thrust_, -c_.slew * dt, c_.slew * dt);
    return last_thrust_;
  }
  // c_ 为校验后配置；last_thrust_ 为上次实际计算的推力；integral_ 为已提交的推力积分。
  VerticalConfig c_;
  double last_thrust_{}, integral_{};
  // reset_set_ 表示已记录首样本重置计数；冻结标志只控制积分，不跳过反馈与有限值检查。
  bool reset_set_{}, integral_frozen_{};
  std::uint8_t z_reset_{}, vz_reset_{};
};
} // namespace fc01::attitude
