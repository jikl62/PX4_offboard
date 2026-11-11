#pragma once
// 已有视觉导航外环：实际 VisionState 与模式 NavigationCommand 分别校验，再生成姿态/高度目标。
// 该控制器只有 config.enabled=true 且原 Core 允许 HeightHold 运动时才使用；当前默认视觉关闭。
// 世界系 NED（北/东/下）、机体系 FRD（前/右/下），位置 m、速度 m/s、加速度 m/s²、角度 rad。
// 这里不是 SLAM、PX4 EKF 或角速度内环，也不是新 VelocityCommand 的生产适配器。
#include "jikl_fc01_system/attitude_reference.hpp"
#include "jikl_fc01_system/vision_resource_budget.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace fc01::vision {
using Vec3 = std::array<double, 3>;
using Quaternion = attitude::Quaternion;
// 缺失测量用 NaN 显式表达；有效位未置位时不能把默认 0 当作可信位置或速度。
inline double missing() {
  return std::numeric_limits<double>::quiet_NaN();
}
template <std::size_t N> inline std::array<double, N> missing_array() {
  std::array<double, N> values{};
  values.fill(missing());
  return values;
}
// Disabled=禁用，Hold=锁当前位置，Cruise=指定世界位置，Follow=归一化杆量，Land=请求原降落流程。
enum class Mode { Disabled = 0, Hold = 1, Cruise = 2, Follow = 3, Land = 4 };
// 有效位独立声明实际位置/速度/姿态/角速度/两类协方差/质量；未知位由验证器拒绝。
enum Field : std::uint16_t {
  Position = 1,
  Velocity = 2,
  Attitude = 4,
  AngularVelocity = 8,
  PoseCovariance = 16,
  TwistCovariance = 32,
  Quality = 64
};
// 外环的时间、增益与速度/加速度/倾角/半径/高度包络；validate 检查物理上可用的有限正值。
struct Config {
  bool enabled{};
  // 接收与源时效单位 s；max_measurement_delay 限制到达前传输耗时，future_tolerance 限制源时钟领先。
  double state_timeout{.25}, command_timeout{.30}, max_measurement_delay{.20},
      future_tolerance{.05};
  // 位置增益把 m 误差变为 m/s 目标，速度增益把 m/s 误差变为 m/s²，max_tilt_deg 唯独使用度。
  double position_gain{.8}, velocity_gain{1.2}, max_speed{.5}, max_acceleration{.6},
      max_tilt_deg{5};
  double max_yaw_rate{.35}, max_vertical_speed{.15}, max_radius{2}, max_height{5}, min_quality{.5};
  double max_position_jump{.75}, max_velocity_jump{1};
  void validate() const;
};
// 来自视觉估计器的实际运动，不是上层希望怎么运动；received_at 只能由本机 Inbox 写入。
struct State {
  std::uint64_t sequence{}, stamp_ns{};
  double received_at{-1};
  std::uint32_t reset_counter{};
  std::string frame_id, body_frame_id, source_id, angular_velocity_source;
  bool identity_resource_invalid{}; // ROS 映射拒绝超预算身份后保留撤销证据，不复制原始长文本。
  bool initialized{}, tracking{}, calibration_valid{};
  std::uint16_t valid_fields{};
  // position/velocity 使用世界 NED；angular_velocity 使用机体 FRD；姿态为 Hamilton w,x,y,z 的 FRD→NED。
  Vec3 position{missing_array<3>()}, velocity{missing_array<3>()},
      angular_velocity{missing_array<3>()};
  Quaternion attitude{missing_array<4>()};
  std::array<double, 36> pose_covariance{missing_array<36>()},
      twist_covariance{missing_array<36>()};
  double quality{missing()};
};
// 既有上层模式目标；reset_counter 必须属于实际估计 epoch，与新独立速度消息的生产方 reset 合同不同。
struct Command {
  std::uint64_t sequence{}, stamp_ns{};
  double received_at{-1};
  std::uint32_t reset_counter{};
  std::string source_id, frame_id;
  bool identity_resource_invalid{};
  Mode mode{Mode::Disabled};
  bool valid{}, position_valid{}, yaw_valid{}, sticks_valid{};
  Vec3 position{missing_array<3>()};
  double yaw{missing()};
  // Follow 顺序为右/前/偏航/上，均归一化到 [-1,1]；水平只按视觉 yaw 旋转，垂向单独处理。
  std::array<double, 4> sticks{missing_array<4>()};
};
// 在复制或序列化前检查全部身份；未声明的角速度来源也不能夹带无界文本。
inline bool identities_within_budget(const State& value) {
  return !value.identity_resource_invalid && identity_within_budget(value.source_id) &&
         identity_within_budget(value.frame_id) && identity_within_budget(value.body_frame_id) &&
         identity_within_budget(value.angular_velocity_source);
}
inline bool identities_within_budget(const Command& value) {
  return !value.identity_resource_invalid && identity_within_budget(value.source_id) &&
         identity_within_budget(value.frame_id);
}
// 一次控制计算用的值快照：两个独立来源数量、当前 ROS ns 与本机撤销代数一起参与资格判定。
struct Input {
  State state;
  Command command;
  std::uint64_t ros_now_ns{};
  std::size_t state_publishers{}, command_publishers{};
  // Local inbox evidence only, never populated from a ROS/wire field. This
  // preserves A->B->A source/reset changes between control-loop snapshots.
  std::uint64_t revocation_generation{};
};
// active=可向原 Core 提供本次计划；request_land=撤销运动并请求原下降，不是直接向 PX4 发布命令。
struct Result {
  bool active{}, request_land{};
  Mode mode{Mode::Disabled};
  std::string reason;
  Quaternion q{1, 0, 0, 0};
  double target_height{missing()}, yaw{missing()};
  Vec3 velocity_target{0, 0, 0}, acceleration_target{0, 0, 0};
  std::uint64_t state_sequence{}, command_sequence{};
  std::uint32_t reset_counter{};
};
namespace detail {
constexpr double pi = 3.14159265358979323846;
constexpr double gravity = 9.80665;
// 数组逐项检查有限性；计算后的二维范数还要另外检查，有限分量也可能让范数溢出。
template <std::size_t N> inline bool finite(const std::array<double, N>& values) {
  for (double value : values)
    if (!std::isfinite(value))
      return false;
  return true;
}
// 同位置 NaN 被视为相同缺失值，便于完整重复载荷比较；不因此获得测量有效性。
inline bool same(double first, double second) {
  return first == second || (std::isnan(first) && std::isnan(second));
}
template <std::size_t N>
inline bool same(const std::array<double, N>& first, const std::array<double, N>& second) {
  for (std::size_t i = 0; i < N; ++i)
    if (!same(first[i], second[i]))
      return false;
  return true;
}
// 比较完整已接受快照，阻止相同 sequence 下悄悄改字段；本机 received_at 也不能被重复包续期。
inline bool same(const State& first, const State& second) {
  return first.sequence == second.sequence && first.stamp_ns == second.stamp_ns &&
         same(first.received_at, second.received_at) &&
         first.reset_counter == second.reset_counter && first.frame_id == second.frame_id &&
         first.body_frame_id == second.body_frame_id && first.source_id == second.source_id &&
         first.angular_velocity_source == second.angular_velocity_source &&
         first.initialized == second.initialized && first.tracking == second.tracking &&
         first.calibration_valid == second.calibration_valid &&
         first.valid_fields == second.valid_fields && same(first.position, second.position) &&
         same(first.velocity, second.velocity) &&
         same(first.angular_velocity, second.angular_velocity) &&
         same(first.attitude, second.attitude) &&
         same(first.pose_covariance, second.pose_covariance) &&
         same(first.twist_covariance, second.twist_covariance) &&
         same(first.quality, second.quality);
}
// 目标与实际状态分别比较；来源/reset/有效标志和全部目标量都属于重复载荷合同。
inline bool same(const Command& first, const Command& second) {
  return first.sequence == second.sequence && first.stamp_ns == second.stamp_ns &&
         same(first.received_at, second.received_at) &&
         first.reset_counter == second.reset_counter && first.source_id == second.source_id &&
         first.frame_id == second.frame_id && first.mode == second.mode &&
         first.valid == second.valid && first.position_valid == second.position_valid &&
         first.yaw_valid == second.yaw_valid && first.sticks_valid == second.sticks_valid &&
         same(first.position, second.position) && same(first.yaw, second.yaw) &&
         same(first.sticks, second.sticks);
}
// stamp/ros_now 单位 ns，received/now 单调 s；源 TTL、接收 TTL、未来容差及到达前延迟同时约束。
inline bool clock_valid(std::uint64_t stamp, double received, std::uint64_t ros_now, double now,
                        double timeout, const Config& c) {
  if (!stamp || !ros_now || !std::isfinite(now) || !std::isfinite(received) || received < 0 ||
      now < received || now - received >= timeout)
    return false;
  const long double age =
      (static_cast<long double>(ros_now) - static_cast<long double>(stamp)) / 1e9L;
  // The arrival transport cap is not another TTL. Extrapolate ROS at receipt
  // using the independent steady elapsed age; delay consumes source lifetime.
  const long double arrival_age = age - static_cast<long double>(now - received);
  return age >= -static_cast<long double>(c.future_tolerance) &&
         age < static_cast<long double>(timeout) &&
         arrival_age >= -static_cast<long double>(c.future_tolerance) &&
         arrival_age <= static_cast<long double>(c.max_measurement_delay);
}
// 这里只校验所有项有限及对角线非负，不证明矩阵对称/PSD；上游必须另提供协方差语义及转换。
inline bool covariance_valid(const std::array<double, 36>& covariance) {
  if (!finite(covariance))
    return false;
  for (std::size_t i = 0; i < 6; ++i)
    if (covariance[i * 6 + i] < 0)
      return false;
  return true;
}
inline double distance(const Vec3& first, const Vec3& second) {
  return std::hypot(std::hypot(first[0] - second[0], first[1] - second[1]), first[2] - second[2]);
}
inline double wrap(double angle) {
  return std::remainder(angle, 2 * pi);
}
inline double yaw(const Quaternion& q) {
  return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}
// 只沿原水平向量方向缩放，不分别夹紧轴导致改变方向；调用方必须先拒绝非有限范数。
inline void bound_xy(Vec3& vector, double limit) {
  const double length = std::hypot(vector[0], vector[1]);
  if (length > limit) {
    vector[0] *= limit / length;
    vector[1] *= limit / length;
  }
}
// The requested body-down axis is [-a_N,-a_E,g], with an exact ZYX yaw.
// Positive right acceleration is positive roll; forward acceleration is negative pitch.
// 期望机体下轴与 [-a_N,-a_E,g] 对齐；向右加速需正 roll，向前加速需负 pitch，yaw 保持指定 ZYX。
inline Quaternion attitude_target(const Vec3& acceleration, double yaw_target) {
  const double forward =
      std::cos(yaw_target) * acceleration[0] + std::sin(yaw_target) * acceleration[1];
  const double right =
      -std::sin(yaw_target) * acceleration[0] + std::cos(yaw_target) * acceleration[1];
  const double roll = std::atan2(right, std::hypot(gravity, forward));
  const double pitch = std::atan2(-forward, gravity);
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2), cp = std::cos(pitch / 2),
               sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw_target / 2), sy = std::sin(yaw_target / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy};
}
} // namespace detail
// 一轮视觉任务的状态机：保存原点/模式/位置及航向目标；运行中拒绝会锁存 Land，后续正常包不清除。
class Controller {
public:
  explicit Controller(Config config = {}) : c_(config) {
    c_.validate();
  }
  // 输入实际状态与目标快照，now/dt 为单调 s；px4_height 为上正 m，px4_yaw 为当前可信航向 rad。
  Result step(const Input& in, double now, double px4_height, double px4_yaw, double dt) {
    // stop 与 Land 锁存优先返回旧终态，不能靠再次调用 step 自动重新取得运动权限。
    if (stopped_)
      return last_;
    if (land_latched_)
      return last_;
    if (!c_.enabled) {
      last_ = {};
      last_.reason = "VISION_NAVIGATION_DISABLED";
      return last_;
    }
    // 先查本次控制样本、dt 和 PX4 高度/航向，避免异常时间与参考值污染后续积分目标。
    if (!std::isfinite(now) || now < 0 || !std::isfinite(dt) || dt <= 0 ||
        dt > std::min(c_.state_timeout, c_.command_timeout) || !std::isfinite(px4_height) ||
        px4_height < 0 || px4_height > c_.max_height || !std::isfinite(px4_yaw))
      return reject("NAVIGATION_CONTROL_SAMPLE_INVALID");
    // Inbox 撤销代数保存同周期 A→B→A 的变化，即使最终字段恢复原值，也不能掩盖中间撤销。
    if (accepted_ && in.revocation_generation != accepted_input_.revocation_generation)
      return land("NAVIGATION_INPUT_REVOKED");
    if (accepted_ && (now < last_now_ || in.ros_now_ns < accepted_input_.ros_now_ns))
      return reject("NAVIGATION_CLOCK_REGRESSION");
    // 先校验目标：显式新鲜 Land 可在视觉丢失时请求原降落，不强迫先有完整米制视觉状态。
    const auto command_error = validate_command(in, now);
    if (!command_error.empty())
      return reject(command_error);
    // An explicit, fresh Land request needs no metric vision estimate.
    if (in.command.mode == Mode::Land)
      return land("NAVIGATION_LAND_REQUESTED");
    if (in.command.mode == Mode::Disabled)
      return reject("NAVIGATION_COMMAND_DISABLED");
    const auto state_error = validate_state(in, now);
    if (!state_error.empty())
      return reject(state_error);
    if (in.state.reset_counter != in.command.reset_counter)
      return reject("VISION_COMMAND_EPOCH_MISMATCH");
    // 已接收过有效计划后必须维持同源、同坐标、同估计 epoch；序号/源戳/接收时间不可倒退。
    if (accepted_) {
      const auto continuity_error = accepted_input_error(in);
      if (!continuity_error.empty())
        return reject(continuity_error);
    }
    // 只在首次进入或模式变化时锁新 Hold 点/Follow 高度；普通新帧不能不停重写保持目标。
    const bool entering = !accepted_ || in.command.mode != mode_;
    // 第一帧锁世界视觉原点和可信 PX4 yaw，后续半径限制相对该任务原点。
    if (!accepted_) {
      origin_ = in.state.position;
      yaw_target_ = yaw_goal_ = detail::wrap(px4_yaw);
    }
    if (std::hypot(in.state.position[0] - origin_[0], in.state.position[1] - origin_[1]) >
        c_.max_radius)
      return reject("NAVIGATION_RADIUS_EXCEEDED");
    if (entering && in.command.mode == Mode::Hold) {
      position_target_ = in.state.position;
      yaw_goal_ = detail::wrap(px4_yaw);
    }
    if (entering && in.command.mode == Mode::Follow)
      follow_height_ = px4_height;
    // Cruise 采用上层明确有效的位置；目标和实际位置都需留在原点半径内，不能用目标跳出包络。
    if (in.command.mode == Mode::Cruise) {
      position_target_ = in.command.position;
      if (std::hypot(position_target_[0] - origin_[0], position_target_[1] - origin_[1]) >
          c_.max_radius)
        return reject("NAVIGATION_POINT_OUTSIDE_RADIUS");
      if (in.command.yaw_valid)
        yaw_goal_ = detail::wrap(in.command.yaw);
    }
    Result output;
    output.mode = in.command.mode;
    output.reason = "VISION_NAVIGATION_ACTIVE";
    output.state_sequence = in.state.sequence;
    output.command_sequence = in.command.sequence;
    output.reset_counter = in.state.reset_counter;
    // Follow 水平目标用视觉 yaw 将右/前杆量转 N/E；上杆量增加上正高度，NED 目标 vz 因而为负。
    if (in.command.mode == Mode::Follow) {
      const double vision_yaw = detail::yaw(*attitude::normalized(in.state.attitude));
      const double forward = in.command.sticks[1] * c_.max_speed,
                   right = in.command.sticks[0] * c_.max_speed;
      output.velocity_target = {std::cos(vision_yaw) * forward - std::sin(vision_yaw) * right,
                                std::sin(vision_yaw) * forward + std::cos(vision_yaw) * right,
                                -in.command.sticks[3] * c_.max_vertical_speed};
      follow_height_ = std::clamp(
          follow_height_ + in.command.sticks[3] * c_.max_vertical_speed * dt, 0., c_.max_height);
      output.target_height = follow_height_;
      yaw_target_ = detail::wrap(yaw_target_ + in.command.sticks[2] * c_.max_yaw_rate * dt);
      yaw_goal_ = yaw_target_;
    } else {
      // 先保留三轴位置误差乘增益的原始结果；垂向限幅不能掩盖计算溢出。
      const Vec3 raw_velocity_target{
          c_.position_gain * (position_target_[0] - in.state.position[0]),
          c_.position_gain * (position_target_[1] - in.state.position[1]),
          c_.position_gain * (position_target_[2] - in.state.position[2])};
      if (!detail::finite(raw_velocity_target))
        return reject("NAVIGATION_NONFINITE_PLAN");
      output.velocity_target = raw_velocity_target;
      output.velocity_target[2] =
          std::clamp(raw_velocity_target[2], -c_.max_vertical_speed, c_.max_vertical_speed);
      // Only the target is translated; PX4 Z/Vz remains the vertical feedback.
      // 只把视觉垂向位置误差转成 PX4 高度目标；真正垂向反馈仍使用原 PX4 Z/Vz。
      output.target_height = px4_height + in.state.position[2] - position_target_[2];
      yaw_target_ =
          detail::wrap(yaw_target_ + std::clamp(detail::wrap(yaw_goal_ - yaw_target_),
                                                -c_.max_yaw_rate * dt, c_.max_yaw_rate * dt));
    }
    if (!std::isfinite(output.target_height) || output.target_height < 0 ||
        output.target_height > c_.max_height)
      return reject("NAVIGATION_HEIGHT_TARGET_OUTSIDE_ENVELOPE");
    // 有限分量的合速度仍可能溢出；必须在限幅前拒绝，避免乘以零后伪装成正常目标。
    if (!detail::finite(output.velocity_target) ||
        !std::isfinite(std::hypot(output.velocity_target[0], output.velocity_target[1])))
      return reject("NAVIGATION_NONFINITE_PLAN");
    detail::bound_xy(output.velocity_target, c_.max_speed);
    output.acceleration_target = {
        c_.velocity_gain * (output.velocity_target[0] - in.state.velocity[0]),
        c_.velocity_gain * (output.velocity_target[1] - in.state.velocity[1]), 0};
    // 加速度使用世界 NED 的 m/s²；先检查误差增益结果及二维模长，再执行原有倾角/加速度限幅。
    if (!detail::finite(output.acceleration_target) ||
        !std::isfinite(std::hypot(output.acceleration_target[0], output.acceleration_target[1])))
      return reject("NAVIGATION_NONFINITE_PLAN");
    // 水平加速度同时受配置上限和 g*tan(最大倾角) 限制，取更小值后再形成姿态。
    const double acceleration_limit = std::min(
        c_.max_acceleration, detail::gravity * std::tan(c_.max_tilt_deg * detail::pi / 180));
    detail::bound_xy(output.acceleration_target, acceleration_limit);
    output.yaw = yaw_target_;
    output.q = detail::attitude_target(output.acceleration_target, yaw_target_);
    if (!detail::finite(output.velocity_target) || !detail::finite(output.acceleration_target) ||
        !attitude::normalized(output.q))
      return reject("NAVIGATION_NONFINITE_PLAN");
    // 所有中间量/四元数通过后才原子式替换本对象的已接受输入和最后结果，拒绝路径不提交新计划。
    output.active = true;
    mode_ = in.command.mode;
    accepted_input_ = in;
    accepted_ = true;
    last_now_ = now;
    last_ = output;
    return last_;
  }
  // 最终 sink 用它证明输入仍与已计算计划完全相同且未过期；只看最后 active 不足以继续发布。
  bool current(const Input& in, double now) const {
    return c_.enabled && accepted_ && !stopped_ && !land_latched_ && last_.active &&
           std::isfinite(now) && now >= last_now_ && in.ros_now_ns >= accepted_input_.ros_now_ns &&
           in.revocation_generation == accepted_input_.revocation_generation &&
           validate_state(in, now).empty() && validate_command(in, now).empty() &&
           detail::same(in.state, accepted_input_.state) &&
           detail::same(in.command, accepted_input_.command) &&
           in.state.reset_counter == last_.reset_counter &&
           in.state.sequence == last_.state_sequence &&
           in.command.sequence == last_.command_sequence;
  }
  const Result& last() const {
    return last_;
  }
  // Stop is an ownership release, not a landing command and not a reset API.
  void stop() {
    stopped_ = true;
    last_ = {};
    last_.reason = "VISION_NAVIGATION_STOPPED";
  }

private:
  // 对照已接受计划核验输入连续性；输入为已通过本帧状态/目标验证的 NED 快照，返回首个拒绝码或空串。
  // 本函数只读 accepted_input_，不提交新计划、不清撤销/下降锁存。source ns、接收 steady s 各自比较，不互换。
  // 必须在撤销代数/时钟/目标/实际状态校验之后、捕获原点与计算目标之前；保持所有拒绝优先级。
  std::string accepted_input_error(const Input& in) const {
    const auto& previous = accepted_input_;
    if (in.state.source_id != previous.state.source_id ||
        in.command.source_id != previous.command.source_id)
      return "NAVIGATION_SOURCE_CHANGED";
    if (in.state.frame_id != previous.state.frame_id ||
        in.state.body_frame_id != previous.state.body_frame_id ||
        in.command.frame_id != previous.command.frame_id)
      return "NAVIGATION_FRAME_CHANGED";
    if (in.state.reset_counter != previous.state.reset_counter ||
        in.command.reset_counter != previous.command.reset_counter)
      return "NAVIGATION_ESTIMATOR_RESET";
    if (in.state.sequence < previous.state.sequence ||
        in.command.sequence < previous.command.sequence)
      return "NAVIGATION_SEQUENCE_REGRESSION";
    if ((in.state.sequence == previous.state.sequence && !detail::same(in.state, previous.state)) ||
        (in.command.sequence == previous.command.sequence &&
         !detail::same(in.command, previous.command)))
      return "NAVIGATION_REPLAY_PAYLOAD_CHANGED";
    if ((in.state.sequence > previous.state.sequence &&
         (in.state.stamp_ns <= previous.state.stamp_ns ||
          in.state.received_at < previous.state.received_at)) ||
        (in.command.sequence > previous.command.sequence &&
         (in.command.stamp_ns <= previous.command.stamp_ns ||
          in.command.received_at < previous.command.received_at)))
      return "NAVIGATION_SAMPLE_NOT_ADVANCING";
    if (detail::distance(in.state.position, previous.state.position) > c_.max_position_jump ||
        detail::distance(in.state.velocity, previous.state.velocity) > c_.max_velocity_jump)
      return "NAVIGATION_ESTIMATOR_JUMP";
    return {};
  }
  // 实际状态同时要求单源、身份、明确 NED/FRD、时间、有效位、跟踪/标定、质量与有限测量。
  std::string validate_state(const Input& in, double now) const {
    const auto& state = in.state;
    if (in.state_publishers != 1)
      return "VISION_STATE_SOURCE_COUNT";
    if (!identities_within_budget(state))
      return "VISION_STATE_IDENTITY_RESOURCE_LIMIT";
    if (!state.sequence || state.source_id.empty())
      return "VISION_STATE_IDENTITY_INVALID";
    if (state.frame_id != "fc01_vision_ned" || state.body_frame_id != "base_link_frd")
      return "VISION_COORDINATE_FRAME_INVALID";
    if (!detail::clock_valid(state.stamp_ns, state.received_at, in.ros_now_ns, now,
                             c_.state_timeout, c_))
      return "VISION_STATE_TIME_INVALID";
    constexpr std::uint16_t required = Position | Velocity | Attitude | Quality;
    constexpr std::uint16_t allowed = required | AngularVelocity | PoseCovariance | TwistCovariance;
    if ((state.valid_fields & required) != required || (state.valid_fields & ~allowed) != 0)
      return "VISION_REQUIRED_FIELDS_INVALID";
    if (!state.initialized || !state.tracking || !state.calibration_valid)
      return "VISION_TRACKING_OR_CALIBRATION_INVALID";
    if (!detail::finite(state.position) || !detail::finite(state.velocity) ||
        !attitude::normalized(state.attitude))
      return "VISION_METRIC_STATE_INVALID";
    if (!std::isfinite(state.quality) || state.quality < c_.min_quality || state.quality > 1)
      return "VISION_QUALITY_INVALID";
    if ((state.valid_fields & AngularVelocity) &&
        (!detail::finite(state.angular_velocity) || state.angular_velocity_source.empty()))
      return "VISION_ANGULAR_VELOCITY_SOURCE_INVALID";
    if (((state.valid_fields & PoseCovariance) &&
         !detail::covariance_valid(state.pose_covariance)) ||
        ((state.valid_fields & TwistCovariance) &&
         !detail::covariance_valid(state.twist_covariance)))
      return "VISION_COVARIANCE_INVALID";
    return {};
  }
  // 目标有独立来源/时间/坐标合同；Cruise 必须有位置，Follow 必须有归一化杆量。
  std::string validate_command(const Input& in, double now) const {
    const auto& command = in.command;
    if (in.command_publishers != 1)
      return "NAVIGATION_COMMAND_SOURCE_COUNT";
    if (!identities_within_budget(command))
      return "NAVIGATION_COMMAND_IDENTITY_RESOURCE_LIMIT";
    if (!command.valid || !command.sequence || command.source_id.empty())
      return "NAVIGATION_COMMAND_INVALID";
    if (command.frame_id != "fc01_vision_ned")
      return "NAVIGATION_COMMAND_FRAME_INVALID";
    if (!detail::clock_valid(command.stamp_ns, command.received_at, in.ros_now_ns, now,
                             c_.command_timeout, c_))
      return "NAVIGATION_COMMAND_TIME_INVALID";
    if (command.mode != Mode::Disabled && command.mode != Mode::Hold &&
        command.mode != Mode::Cruise && command.mode != Mode::Follow && command.mode != Mode::Land)
      return "NAVIGATION_COMMAND_MODE_INVALID";
    if (command.position_valid && !detail::finite(command.position))
      return "NAVIGATION_COMMAND_POSITION_INVALID";
    if (command.yaw_valid && !std::isfinite(command.yaw))
      return "NAVIGATION_COMMAND_YAW_INVALID";
    if (command.sticks_valid) {
      for (double value : command.sticks)
        if (!std::isfinite(value) || std::abs(value) > 1)
          return "NAVIGATION_COMMAND_AXES_INVALID";
    }
    if (command.mode == Mode::Cruise && !command.position_valid)
      return "NAVIGATION_CRUISE_POINT_REQUIRED";
    if (command.mode == Mode::Follow && !command.sticks_valid)
      return "NAVIGATION_FOLLOW_AXES_REQUIRED";
    return {};
  }
  // 首次有效计划前拒绝只等待；已经运动后任一拒绝进入原 Land 锁存，避免保留最后非零速度。
  Result reject(const std::string& reason) {
    if (accepted_)
      return land(reason);
    last_ = {};
    last_.reason = reason;
    return last_;
  }
  // 构造空结果再置 request_land，撤销旧 active/姿态目标；锁存只由新对象/上层任务生命周期重建。
  Result land(const std::string& reason) {
    land_latched_ = true;
    last_ = {};
    last_.request_land = true;
    last_.mode = Mode::Land;
    last_.reason = reason;
    return last_;
  }
  // accepted_input_ 是计划依赖的完整快照；origin_ 不随视觉重定位自动漂移，yaw/follow_height 仅按验证后 dt 推进。
  Config c_;
  Result last_;
  Input accepted_input_;
  bool accepted_{}, land_latched_{}, stopped_{};
  Mode mode_{Mode::Disabled};
  Vec3 origin_{0, 0, 0}, position_target_{0, 0, 0};
  double yaw_target_{}, yaw_goal_{}, follow_height_{}, last_now_{};
};
// 配置验证不会夹紧不合理值为合法值；非有限/非正上限、>=90°倾角或>5 m 高度直接抛错。
inline void Config::validate() const {
  for (double value :
       {state_timeout, command_timeout, max_measurement_delay, future_tolerance, position_gain,
        velocity_gain, max_speed, max_acceleration, max_tilt_deg, max_yaw_rate, max_vertical_speed,
        max_radius, max_height, max_position_jump, max_velocity_jump})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("VISION_NAVIGATION_CONFIG_REQUIRES_FINITE_POSITIVE_LIMITS");
  if (max_tilt_deg >= 90 || max_height > 5 || !std::isfinite(min_quality) || min_quality < 0 ||
      min_quality > 1)
    throw std::invalid_argument("VISION_NAVIGATION_CONFIG_OUTSIDE_ENVELOPE");
}
// ROS 回调缓存与控制计算分层；保存最新值，同时用 revocation_generation 记录中间身份/reset/撤销。
class Inbox {
public:
  // 状态身份/坐标/reset 变化和健康→撤销即使序号未增也必须保留，避免被旧序号过滤隐藏失效。
  bool state(const State& incoming, double received_at) {
    // 先看借用输入，避免按值形参先复制长文本。即使是迟到/零序号的异常包也撤销当前资格，
    // 不推进可信序号；后续健康消息不能抹掉 revocation_generation 的中途撤销证据。
    if (!identities_within_budget(incoming)) {
      // 保留先前有界身份及高水位；清空身份会把同源旧 seq 错当新来源而允许水位回退。
      // 仅移动旧缓存的字符串，绝不保存本次异常输入的原文；所有测量默认恢复为未知。
      State rejected;
      rejected.source_id = std::move(state_.source_id);
      rejected.frame_id = std::move(state_.frame_id);
      rejected.body_frame_id = std::move(state_.body_frame_id);
      rejected.reset_counter = state_.reset_counter;
      rejected.sequence = state_sequence_;
      rejected.stamp_ns = state_.stamp_ns;
      rejected.identity_resource_invalid = true;
      rejected.received_at = received_at;
      state_ = std::move(rejected);
      state_seen_ = true;
      ++revocation_generation_;
      return true;
    }
    State value = incoming;
    const bool changed =
        state_seen_ &&
        (value.source_id != state_.source_id || value.reset_counter != state_.reset_counter ||
         value.frame_id != state_.frame_id || value.body_frame_id != state_.body_frame_id);
    const bool withdrawn = state_seen_ && state_retracted(value) && !state_retracted(state_) &&
                           value.stamp_ns && value.stamp_ns >= state_.stamp_ns;
    if (!changed && !withdrawn && (!value.sequence || value.sequence <= state_sequence_))
      return false;
    if (changed || withdrawn)
      ++revocation_generation_;
    state_sequence_ = changed ? value.sequence : std::max(state_sequence_, value.sequence);
    state_seen_ = true;
    value.received_at = received_at;
    // 按有效位清空所有未声明数据，防止旧/默认测量在快照中看似仍可用。
    if (!(value.valid_fields & Position))
      value.position = missing_array<3>();
    if (!(value.valid_fields & Velocity))
      value.velocity = missing_array<3>();
    if (!(value.valid_fields & Attitude))
      value.attitude = missing_array<4>();
    if (!(value.valid_fields & AngularVelocity)) {
      value.angular_velocity = missing_array<3>();
      value.angular_velocity_source.clear();
    }
    if (!(value.valid_fields & PoseCovariance))
      value.pose_covariance = missing_array<36>();
    if (!(value.valid_fields & TwistCovariance))
      value.twist_covariance = missing_array<36>();
    if (!(value.valid_fields & Quality))
      value.quality = missing();
    state_ = std::move(value);
    return true;
  }
  // 目标也单独保留身份和显式撤销；正常重复序号不更新接收时间，不能续期旧命令。
  bool command(const Command& incoming, double received_at) {
    // 与状态门相同：拒绝不复制文本，保持中途撤销可见；这不授予任何新飞行权限。
    if (!identities_within_budget(incoming)) {
      Command rejected;
      rejected.source_id = std::move(command_.source_id);
      rejected.frame_id = std::move(command_.frame_id);
      rejected.reset_counter = command_.reset_counter;
      rejected.sequence = command_sequence_;
      rejected.stamp_ns = command_.stamp_ns;
      rejected.identity_resource_invalid = true;
      rejected.received_at = received_at;
      command_ = std::move(rejected);
      command_seen_ = true;
      ++revocation_generation_;
      return true;
    }
    Command value = incoming;
    const bool changed = command_seen_ && (value.source_id != command_.source_id ||
                                           value.reset_counter != command_.reset_counter ||
                                           value.frame_id != command_.frame_id);
    const bool withdrawn = command_seen_ && command_retracted(value) &&
                           !command_retracted(command_) && value.stamp_ns &&
                           value.stamp_ns >= command_.stamp_ns;
    if (!changed && !withdrawn && (!value.sequence || value.sequence <= command_sequence_))
      return false;
    if (changed || withdrawn)
      ++revocation_generation_;
    command_sequence_ = changed ? value.sequence : std::max(command_sequence_, value.sequence);
    command_seen_ = true;
    value.received_at = received_at;
    if (!value.position_valid)
      value.position = missing_array<3>();
    if (!value.yaw_valid)
      value.yaw = missing();
    if (!value.sticks_valid)
      value.sticks = missing_array<4>();
    command_ = std::move(value);
    return true;
  }
  // 快照复制两个缓存，附上当前图来源数量与 ROS 时间；不从消息接收任何“本机撤销代数”。
  Input snapshot(std::uint64_t now, std::size_t states, std::size_t commands) const {
    Input in;
    in.state = state_;
    in.command = command_;
    in.ros_now_ns = now;
    in.state_publishers = states;
    in.command_publishers = commands;
    in.revocation_generation = revocation_generation_;
    return in;
  }

private:
  // 判断载荷是否撤回实际测量合同；不是完整控制批准，完整质量/时间门仍在 Controller。
  static bool state_retracted(const State& value) {
    constexpr std::uint16_t required = Position | Velocity | Attitude | Quality;
    if (!value.initialized || !value.tracking || !value.calibration_valid ||
        (value.valid_fields & required) != required || !detail::finite(value.position) ||
        !detail::finite(value.velocity) || !attitude::normalized(value.attitude) ||
        !std::isfinite(value.quality) || value.quality < 0 || value.quality > 1)
      return true;
    if ((value.valid_fields & AngularVelocity) &&
        (!detail::finite(value.angular_velocity) || value.angular_velocity_source.empty()))
      return true;
    return ((value.valid_fields & PoseCovariance) &&
            !detail::covariance_valid(value.pose_covariance)) ||
           ((value.valid_fields & TwistCovariance) &&
            !detail::covariance_valid(value.twist_covariance));
  }
  // Disabled/Land/非法模式或缺必要目标都撤销运动，不能因最后一次值恢复就丢掉中途撤销。
  static bool command_retracted(const Command& value) {
    if (!value.valid || value.mode == Mode::Disabled || value.mode == Mode::Land)
      return true;
    if (value.mode != Mode::Hold && value.mode != Mode::Cruise && value.mode != Mode::Follow)
      return true;
    if ((value.position_valid && !detail::finite(value.position)) ||
        (value.yaw_valid && !std::isfinite(value.yaw)))
      return true;
    if (value.mode == Mode::Cruise && !value.position_valid)
      return true;
    if (value.mode == Mode::Follow && !value.sticks_valid)
      return true;
    if (value.sticks_valid)
      for (double axis : value.sticks)
        if (!std::isfinite(axis) || std::abs(axis) > 1)
          return true;
    return false;
  }
  State state_;
  Command command_;
  bool state_seen_{}, command_seen_{};
  std::uint64_t state_sequence_{}, command_sequence_{}, revocation_generation_{};
};
} // namespace fc01::vision
