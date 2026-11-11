#pragma once
#include "jikl_fc01_system/vision_navigation.hpp"
#include "jikl_fc01_system/vision_ros_adapter.hpp"
#include <string>

// 视觉速度控制的离线预览：实际状态和目标速度分别检查、分别缓存。
// 读取顺序：state()/command() 接收输入 -> preview() 计算 -> Preview 查看结果。
// 本文件不创建 Runtime 输出、PX4 消息或控制权租约，不改变飞机的飞行权限。
namespace fc01::velocity_input {
using Vec3 = vision::Vec3;

inline bool identity_valid(const std::string& value) {
  // 与独立诊断接收器采用相同的文本长度限制，避免无限增长的来源标识。
  return !value.empty() && value.size() <= 256 &&
         value.find_first_not_of(" \t\r\n") != std::string::npos;
}

enum class Mode {Disabled, VelocityPreview};

// 上层给出的“希望怎么运动”，不是飞机的实际速度反馈。
struct Target {
  std::uint16_t schema_version{};
  std::uint64_t sequence{};
  std::uint64_t stamp_ns{}; // 指令测量/计算时间，使用已约定的 ROS 时钟，单位 ns。
  double received_at{-1}; // 本机 monotonic 接收时间，单位 s，由 command() 写入。
  std::uint32_t reset_counter{}; // 指令发送方的周期，与状态估计器的 reset 独立。
  std::string source_id;
  std::string frame_id;
  bool valid{};
  Vec3 body_velocity{vision::missing_array<3>()}; // FRD：前、右、下，单位 m/s。
};

// 只做消息字段映射，不以到达时刻替代原始 stamp，不授予控制权。
template<class Message>
Target from_message(const Message& message) {
  Target target;
  target.schema_version = message.schema_version;
  target.sequence = message.sequence;
  target.stamp_ns = vision::message_stamp_ns(message.stamp);
  target.reset_counter = message.reset_counter;
  target.source_id = message.source_id;
  target.frame_id = message.frame_id;
  target.valid = message.valid;
  target.body_velocity = {message.vx, message.vy, message.vz};
  return target;
}

struct Policy {
  // 默认零值表示尚未配置。离线试验必须显式提供已审查的时间与速度限制。
  double state_timeout{}; // 实际状态接收超时，s。
  double command_timeout{}; // 目标指令接收超时，s。
  double max_measurement_delay{}; // 源测量时间允许的最大年龄，s。
  double future_tolerance{}; // 源时间允许领先本机的容差，s，可显式为零。
  double max_command_speed_mps{}; // 机体系三维目标速度的模长上限，m/s。
  double max_attitude_sample_skew_sec{}; // 状态姿态与目标指令的最大时间差，s。
  double max_attitude_step_rad{}; // 相邻实际姿态允许的最大夹角，rad。

  bool configured() const {
    for (double value : {state_timeout, command_timeout, max_measurement_delay,
                         max_command_speed_mps, max_attitude_sample_skew_sec,
                         max_attitude_step_rad}) {
      if (!std::isfinite(value) || value <= 0) {
        return false;
      }
    }
    return std::isfinite(future_tolerance) && future_tolerance >= 0 &&
           max_attitude_step_rad <= vision::detail::pi;
  }
};

// 计算过程的可读结果。valid 只表示本次离线计算通过，不表示允许运动。
struct Preview {
  bool valid{};
  bool control_authority{false};
  std::string reason;
  Vec3 body_target{vision::missing_array<3>()}; // FRD 目标速度，m/s。
  Vec3 world_target{vision::missing_array<3>()}; // NED 目标速度，m/s。
  Vec3 measured_world_velocity{vision::missing_array<3>()}; // NED 实测速度，m/s。
  Vec3 velocity_error{vision::missing_array<3>()}; // 目标减实测，m/s。
  Vec3 horizontal_acceleration{vision::missing_array<3>()}; // NED 水平加速度，m/s²。
  attitude::Quaternion attitude_preview{vision::missing_array<4>()};
  // 垂向目标仅展示，没有传给原高度控制器。
  double world_vertical_velocity_preview{vision::missing()};
  double yaw_reference{vision::missing()};
  std::uint64_t state_sequence{};
  std::uint64_t command_sequence{};
  std::uint32_t state_reset{};
  std::uint32_t command_reset{};
};

// q 表示机体 FRD 相对世界 NED 的姿态，必须由调用方先归一化。
// 使用完整三维旋转：飞机倾斜时，机体“前进”也可能包含世界系垂向速度。
inline Vec3 rotate_body_to_world(const attitude::Quaternion& q, const Vec3& vector) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {
      (1-2*(y*y+z*z))*vector[0]+2*(x*y-w*z)*vector[1]+2*(x*z+w*y)*vector[2],
      2*(x*y+w*z)*vector[0]+(1-2*(x*x+z*z))*vector[1]+2*(y*z-w*x)*vector[2],
      2*(x*z-w*y)*vector[0]+2*(y*z+w*x)*vector[1]+(1-2*(x*x+y*y))*vector[2]};
}

class Session {
 public:
  explicit Session(Policy policy = {}, vision::Config candidate = {})
      : policy_(policy), control_config_(candidate) {
    control_config_.validate();
    time_validation_config_ = control_config_;
    time_validation_config_.max_measurement_delay = policy_.max_measurement_delay;
    time_validation_config_.future_tolerance = policy_.future_tolerance;
  }

  // 只重开一次离线试验，清空离线输入和首个错误。
  // 这不是飞机恢复接口，不能清除飞机故障、RC 接管或最终发布权限。
  void begin_attempt() {
    actual_state_ = {};
    target_ = {};
    has_state_ = has_target_ = false;
    state_publisher_count_ = command_publisher_count_ = 0;
    first_error_.clear();
    last_observed_at_ = -1;
    last_ros_now_ns_ = 0;
  }

  const std::string& first_error() const {
    return first_error_;
  }

  void observe_publishers(std::size_t states, std::size_t commands) {
    state_publisher_count_ = states;
    command_publisher_count_ = commands;
    if (states > 1 || (has_state_ && states != 1)) {
      latch("STATE_SOURCE_COUNT");
    }
    if (commands > 1 || (has_target_ && commands != 1)) {
      latch("COMMAND_SOURCE_COUNT");
    }
  }

  // 接收飞机实际状态。上游应已完成坐标、标定和原始协方差质量检查。
  // 此处的可选 covariance 检查只复用有限值/非负对角线检查，不证明 PSD。
  bool state(vision::State value, double received_at, std::uint64_t ros_now_ns,
             std::size_t publishers = 1) {
    state_publisher_count_ = publishers;
    if (!first_error_.empty() || !clock_observation(received_at, ros_now_ns)) {
      return false;
    }
    if (publishers != 1) {
      return latch("STATE_SOURCE_COUNT");
    }

    // 1. 检查当前输入的身份、坐标、必需字段以及可选字段。
    value.received_at = received_at;
    if (value.sequence == 0 || !identity_valid(value.source_id)) {
      return latch("STATE_IDENTITY_INVALID");
    }
    if (value.frame_id != "fc01_vision_ned" || value.body_frame_id != "base_link_frd") {
      return latch("STATE_FRAME_INVALID");
    }
    if (!required_state_fields_valid(value)) {
      return latch("STATE_MEASUREMENT_INVALID");
    }
    if (!optional_state_fields_valid(value)) {
      return latch("STATE_OPTIONAL_FIELD_INVALID");
    }
    if (!time_valid(value.stamp_ns, received_at, ros_now_ns, received_at,
                    policy_.state_timeout)) {
      return latch("STATE_TIME_INVALID");
    }

    // 2. 已有状态时，与最后通过检查的状态比较，防止重放或无声明跳变。
    if (has_state_) {
      if (value.source_id != actual_state_.source_id) {
        return latch("STATE_SOURCE_CHANGED");
      }
      if (value.reset_counter != actual_state_.reset_counter) {
        return latch("STATE_ESTIMATOR_RESET");
      }
      if (value.sequence < actual_state_.sequence) {
        return latch("STATE_SEQUENCE_REGRESSION");
      }
      if (value.sequence == actual_state_.sequence) {
        value.received_at = actual_state_.received_at;
        if (!vision::detail::same(value, actual_state_)) {
          return latch("STATE_DUPLICATE_CHANGED");
        }
        return false; // 完全相同的重复包也不能延长缓存有效期。
      }
      if (value.stamp_ns <= actual_state_.stamp_ns) {
        return latch("STATE_STAMP_NOT_ADVANCING");
      }
      if (vision::detail::distance(value.velocity, actual_state_.velocity) >
              control_config_.max_velocity_jump ||
          ((value.valid_fields & actual_state_.valid_fields & vision::Position) &&
           vision::detail::distance(value.position, actual_state_.position) >
               control_config_.max_position_jump)) {
        return latch("STATE_DISCONTINUITY_REQUIRES_NEW_ATTEMPT");
      }
      const auto previous_attitude = *attitude::normalized(actual_state_.attitude);
      const auto current_attitude = *attitude::normalized(value.attitude);
      double attitude_dot_product = 0;
      for (std::size_t axis = 0; axis < 4; ++axis) {
        attitude_dot_product += previous_attitude[axis] * current_attitude[axis];
      }
      // q 和 -q 表示同一姿态，因此先取点积绝对值，再计算真实夹角。
      if (2*std::acos(std::clamp(std::abs(attitude_dot_product), 0., 1.)) >
          policy_.max_attitude_step_rad) {
        return latch("STATE_ATTITUDE_DISCONTINUITY_REQUIRES_NEW_ATTEMPT");
      }
    }

    // 3. 所有检查完成后才替换实际状态缓存。
    actual_state_ = std::move(value);
    has_state_ = true;
    return true;
  }

  // 接收上层目标速度；它使用自己的来源、序号和 reset，不能混入实际状态。
  bool command(Target value, double received_at, std::uint64_t ros_now_ns,
               std::size_t publishers = 1) {
    command_publisher_count_ = publishers;
    if (!first_error_.empty() || !clock_observation(received_at, ros_now_ns)) {
      return false;
    }
    if (publishers != 1) {
      return latch("COMMAND_SOURCE_COUNT");
    }

    value.received_at = received_at;
    if (value.schema_version != 1 || !value.valid || !value.sequence ||
        !identity_valid(value.source_id)) {
      return latch("COMMAND_IDENTITY_OR_VALIDITY_INVALID");
    }
    if (value.frame_id != "body_frd") {
      return latch("COMMAND_FRAME_INVALID");
    }
    if (!vision::detail::finite(value.body_velocity)) {
      return latch("COMMAND_NONFINITE_VELOCITY");
    }
    if (std::hypot(std::hypot(value.body_velocity[0], value.body_velocity[1]),
                   value.body_velocity[2]) > policy_.max_command_speed_mps) {
      return latch("COMMAND_SPEED_OUTSIDE_POLICY");
    }
    if (!time_valid(value.stamp_ns, received_at, ros_now_ns, received_at,
                    policy_.command_timeout)) {
      return latch("COMMAND_TIME_INVALID");
    }

    if (has_target_) {
      if (value.source_id != target_.source_id) {
        return latch("COMMAND_SOURCE_CHANGED");
      }
      if (value.reset_counter != target_.reset_counter) {
        return latch("COMMAND_PRODUCER_RESET");
      }
      if (value.sequence < target_.sequence) {
        return latch("COMMAND_SEQUENCE_REGRESSION");
      }
      if (value.sequence == target_.sequence) {
        if (value.stamp_ns != target_.stamp_ns ||
            value.schema_version != target_.schema_version || value.valid != target_.valid ||
            value.frame_id != target_.frame_id || value.body_velocity != target_.body_velocity) {
          return latch("COMMAND_DUPLICATE_CHANGED");
        }
        return false; // 重复目标同样不能续期。
      }
      if (value.stamp_ns <= target_.stamp_ns) {
        return latch("COMMAND_STAMP_NOT_ADVANCING");
      }
    }
    target_ = std::move(value);
    has_target_ = true;
    return true;
  }

  Preview preview(Mode mode, double now, std::uint64_t ros_now_ns,
                  double trusted_yaw_reference) {
    Preview result;
    if (mode == Mode::Disabled) {
      result.reason = "VELOCITY_PREVIEW_DISABLED";
      return result;
    }
    if (mode != Mode::VelocityPreview) {
      return rejected("VELOCITY_PREVIEW_MODE_INVALID");
    }
    if (!first_error_.empty()) {
      return rejected(first_error_);
    }
    if (!policy_.configured()) {
      return rejected("VELOCITY_PREVIEW_POLICY_UNCONFIGURED");
    }
    if (!clock_observation(now, ros_now_ns)) {
      return rejected(first_error_);
    }
    if (!has_state_ || !has_target_) {
      result.reason = !has_state_ ? "VELOCITY_PREVIEW_AWAIT_STATE"
                                  : "VELOCITY_PREVIEW_AWAIT_COMMAND";
      return result;
    }
    if (state_publisher_count_ != 1 || command_publisher_count_ != 1) {
      return rejected("VELOCITY_PREVIEW_SOURCE_COUNT");
    }
    if (!time_valid(actual_state_.stamp_ns, actual_state_.received_at, ros_now_ns,
                    now, policy_.state_timeout)) {
      return rejected("STATE_TIME_INVALID");
    }
    if (!time_valid(target_.stamp_ns, target_.received_at, ros_now_ns,
                    now, policy_.command_timeout)) {
      return rejected("COMMAND_TIME_INVALID");
    }

    // 姿态决定 FRD -> NED 旋转，姿态与目标不能来自相隔过远的时刻。
    const long double attitude_command_skew_sec =
        std::abs(static_cast<long double>(actual_state_.stamp_ns) -
                 static_cast<long double>(target_.stamp_ns)) / 1e9L;
    if (attitude_command_skew_sec > policy_.max_attitude_sample_skew_sec) {
      return rejected("STATE_COMMAND_ATTITUDE_TIME_SKEW");
    }
    if (!std::isfinite(trusted_yaw_reference)) {
      return rejected("YAW_REFERENCE_INVALID");
    }

    // 1. 将三维机体目标旋转到世界 NED，再限制水平/垂向目标速度。
    result.body_target = target_.body_velocity;
    result.world_target = rotate_body_to_world(
        *attitude::normalized(actual_state_.attitude), result.body_target);
    if (!vision::detail::finite(result.world_target) ||
        !std::isfinite(std::hypot(result.world_target[0], result.world_target[1]))) {
      return rejected("VELOCITY_PREVIEW_NONFINITE_WORLD_TARGET");
    }
    vision::detail::bound_xy(result.world_target, control_config_.max_speed);
    result.world_target[2] = std::clamp(result.world_target[2],
                                       -control_config_.max_vertical_speed,
                                       control_config_.max_vertical_speed);

    // 2. 用“目标速度 - 实测速度”得到误差，反馈不能由目标值代替。
    result.measured_world_velocity = actual_state_.velocity;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      result.velocity_error[axis] = result.world_target[axis] - actual_state_.velocity[axis];
    }
    if (!vision::detail::finite(result.velocity_error) ||
        !std::isfinite(std::hypot(std::hypot(result.velocity_error[0],
                                            result.velocity_error[1]),
                                 result.velocity_error[2]))) {
      return rejected("VELOCITY_PREVIEW_NONFINITE_VELOCITY_ERROR");
    }

    // 3. 水平加速度 = 速度误差 × 原有外环增益，再同时遵守加速度和倾角限制。
    result.horizontal_acceleration = {
        control_config_.velocity_gain * result.velocity_error[0],
        control_config_.velocity_gain * result.velocity_error[1], 0};
    if (!vision::detail::finite(result.horizontal_acceleration) ||
        !std::isfinite(std::hypot(result.horizontal_acceleration[0],
                                  result.horizontal_acceleration[1]))) {
      return rejected("VELOCITY_PREVIEW_NONFINITE_ACCELERATION");
    }
    // 上面每一步先拒绝非有限中间量，避免 inf 被限幅“洗成”有限值或零。
    const double acceleration_limit = std::min(
        control_config_.max_acceleration,
        vision::detail::gravity * std::tan(control_config_.max_tilt_deg *
                                           vision::detail::pi / 180));
    vision::detail::bound_xy(result.horizontal_acceleration, acceleration_limit);

    // 4. 只生成姿态预览；消息没有 yaw 字段，沿用显式传入的可信航向参考。
    result.yaw_reference = vision::detail::wrap(trusted_yaw_reference);
    result.attitude_preview = vision::detail::attitude_target(
        result.horizontal_acceleration, result.yaw_reference);
    result.world_vertical_velocity_preview = result.world_target[2];
    if (!vision::detail::finite(result.world_target) ||
        !vision::detail::finite(result.velocity_error) ||
        !vision::detail::finite(result.horizontal_acceleration) ||
        !attitude::normalized(result.attitude_preview)) {
      return rejected("VELOCITY_PREVIEW_NONFINITE_RESULT");
    }
    result.state_sequence = actual_state_.sequence;
    result.command_sequence = target_.sequence;
    result.state_reset = actual_state_.reset_counter;
    result.command_reset = target_.reset_counter;
    result.valid = true;
    result.reason = "VELOCITY_PREVIEW_ONLY_NOT_DISPATCHED";
    return result;
  }

 private:
  bool required_state_fields_valid(const vision::State& value) const {
    constexpr auto required_fields = vision::Velocity | vision::Attitude | vision::Quality;
    if ((value.valid_fields & required_fields) != required_fields ||
        (value.valid_fields & ~std::uint16_t(127))) {
      return false;
    }
    if (!value.initialized || !value.tracking || !value.calibration_valid) {
      return false;
    }
    if (!vision::detail::finite(value.velocity) || !attitude::normalized(value.attitude)) {
      return false;
    }
    if (!std::isfinite(value.quality) || value.quality < control_config_.min_quality ||
        value.quality > 1) {
      return false;
    }
    return true;
  }

  bool optional_state_fields_valid(const vision::State& value) const {
    if ((value.valid_fields & vision::Position) && !vision::detail::finite(value.position)) {
      return false;
    }
    if ((value.valid_fields & vision::AngularVelocity) &&
        (!vision::detail::finite(value.angular_velocity) || value.angular_velocity_source.empty())) {
      return false;
    }
    if ((value.valid_fields & vision::PoseCovariance) &&
        !vision::detail::covariance_valid(value.pose_covariance)) {
      return false;
    }
    if ((value.valid_fields & vision::TwistCovariance) &&
        !vision::detail::covariance_valid(value.twist_covariance)) {
      return false;
    }
    return true;
  }

  bool latch(const std::string& reason) {
    if (first_error_.empty()) {
      first_error_ = reason;
    }
    return false;
  }

  Preview rejected(const std::string& reason) {
    latch(reason);
    Preview result;
    result.reason = first_error_;
    return result;
  }

  bool clock_observation(double now, std::uint64_t ros_now_ns) {
    if (!policy_.configured()) {
      return latch("VELOCITY_PREVIEW_POLICY_UNCONFIGURED");
    }
    if (!std::isfinite(now) || now < 0 || !ros_now_ns ||
        now < last_observed_at_ || ros_now_ns < last_ros_now_ns_) {
      return latch("RECEIVER_CLOCK_INVALID_OR_REGRESSED");
    }
    // 每次接收/预览先检查旧缓存。即使两个源随后都补来正常包，已观察到的
    // 超时也不能被新包覆盖；只有显式 begin_attempt() 重开离线试验。
    if (has_state_ && !time_valid(actual_state_.stamp_ns, actual_state_.received_at,
                                  ros_now_ns, now, policy_.state_timeout)) {
      return latch("STATE_TIME_INVALID");
    }
    if (has_target_ && !time_valid(target_.stamp_ns, target_.received_at,
                                   ros_now_ns, now, policy_.command_timeout)) {
      return latch("COMMAND_TIME_INVALID");
    }
    last_observed_at_ = now;
    last_ros_now_ns_ = ros_now_ns;
    return true;
  }

  bool time_valid(std::uint64_t stamp, double received, std::uint64_t ros_now,
                  double now, double timeout) const {
    return vision::detail::clock_valid(
        stamp, received, ros_now, now, timeout, time_validation_config_);
  }

  Policy policy_;
  vision::Config control_config_;
  vision::Config time_validation_config_;
  vision::State actual_state_;
  Target target_;
  bool has_state_{};
  bool has_target_{};
  std::size_t state_publisher_count_{};
  std::size_t command_publisher_count_{};
  std::string first_error_; // 本轮离线试验的第一个错误，后续健康包不覆盖。
  double last_observed_at_{-1};
  std::uint64_t last_ros_now_ns_{};
};
} // namespace fc01::velocity_input
