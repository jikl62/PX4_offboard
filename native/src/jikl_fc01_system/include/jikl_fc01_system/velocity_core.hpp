#pragma once
// 保留的通用位置/速度控制路线，使用PX4 NED实际状态与航向相对的前/左/上目标。
// 它不是当前人工RC姿态高度1.5m主链，也不是新VelocityCommand诊断入口；Runtime按profile选择，不能据这里的默认高度推断生产目标。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace jikl_velocity {
constexpr double pi = 3.14159265358979323846;
inline double nan() {
  return std::numeric_limits<double>::quiet_NaN();
}
// 世界NED向量：x北/y东/z下；位置单位m，速度单位m/s，具体由所在字段决定。
struct Vec {
  double x = 0, y = 0, z = 0;
};
// 控制输入为航向相对的前/左/上m/s与逆时针偏航rad/s；它与NED反馈的正方向不同。
struct Velocity {
  double forward = 0, left = 0, up = 0, ccw = 0;
};
// 每轴有限才可进入控制数学；NaN既可能表示未选通字段，也可能是非法实际反馈，需按用途区分。
inline bool finite(Vec p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
// 目标四轴全部有限才允许缓存，避免某一未检查轴混入输出。
inline bool finite(Velocity v) {
  return std::isfinite(v.forward) && std::isfinite(v.left) && std::isfinite(v.up) &&
         std::isfinite(v.ccw);
}
// 零命令容差用于准备及停车判定；先检查有限值，不能把NaN当成“没有运动”。
inline bool zero(Velocity v) {
  return finite(v) &&
         std::abs(v.forward) + std::abs(v.left) + std::abs(v.up) + std::abs(v.ccw) < 1e-5;
}
// 三轴欧氏速度/位移幅值，单位沿用输入；保持原公式，中文化不替换运算以免改变边界行为。
inline double norm(Vec v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
// 角度归一到remainder定义的2π区间，单位rad；不改变原浮点表达式。
inline double wrap(double x) {
  return std::remainder(x, 2 * pi);
}
// 按实际偏航yaw把前/左目标旋转到北/东，向上速度取负成为NED向下速度；yaw单位rad。
inline Vec heading_to_ned(Velocity v, double yaw) {
  return {std::cos(yaw) * v.forward + std::sin(yaw) * v.left,
          std::sin(yaw) * v.forward - std::cos(yaw) * v.left, -v.up};
}
// 仅接受严格递增、未超过age秒、未来偏差最多0.1秒的源时间；新接收不能续期旧指令。
inline bool stamp_valid(double stamp, double now, double previous, double age) {
  return std::isfinite(stamp) && std::isfinite(now) && stamp > 0 && stamp > previous &&
         now - stamp <= age && stamp - now <= .1;
}
// 一次更新最多改变delta幅值，用于垂速与偏航速度的加速度约束。
inline double approach(double old, double wanted, double delta) {
  return old + std::clamp(wanted - old, -delta, delta);
}

// 通用/仿真路线的米/秒包络，不是姿态高度生产参数；构造Core时验证所有值有限且为正。
struct Config {
  // 高度目标m，上升参考速度m/s，reference_lead限制目标最多领先实际高度m。
  double height = 6., climb_speed = .7, reference_lead = .6;
  // 水平/垂向速度幅值m/s，偏航速度幅值rad/s；下方accel分别为m/s²或rad/s²。
  double xy_speed = .8, z_speed = .5, yaw_speed = .5;
  double xy_accel = .5, z_accel = .3, yaw_accel = .6;
  // 短期失令先制动，超过land_after_loss秒再请求降落，两种期限不应混淆。
  double command_timeout = .5, land_after_loss = 3.;
  double prestream = 2., takeoff_timeout = 60., settle_time = 3.;
  double height_tolerance = .15, speed_tolerance = .2;
  double mode_timeout = 8., land_timeout = 60.;
  double max_height = 8., max_radius = 20.;
  // 拒绝非法增益、准备时间及包络关系；不从实际飞机估计这些配置。
  void validate() const {
    const double values[] = {height,          climb_speed,     reference_lead,   xy_speed,
                             z_speed,         yaw_speed,       xy_accel,         z_accel,
                             yaw_accel,       command_timeout, land_after_loss,  prestream,
                             takeoff_timeout, settle_time,     height_tolerance, speed_tolerance,
                             mode_timeout,    land_timeout,    max_height,       max_radius};
    for (double value : values)
      if (!std::isfinite(value) || value <= 0.)
        throw std::invalid_argument("all control parameters must be finite and positive");
    if (prestream < 1.1 || command_timeout < .1 || land_after_loss <= command_timeout ||
        height > max_height - 1.)
      throw std::invalid_argument("invalid prestream, timeout or height limits");
  }
};
// 调用者提供NED位置/速度、偏航rad及PX4状态；fresh与valid必须独立成立，reset_id标记估计参考变化。
struct Feedback {
  Vec position, velocity;
  double yaw = 0;
  bool fresh = false, valid = false, heading_ok = false, preflight = false;
  bool armed = false, landed = false, offboard = false, landing = false, failsafe = false,
       exclusive = false;
  uint32_t reset_id = 0;
};
// 通用阶段：空闲/预发送/切外控/解锁/起飞/保持/速度/制动/降落/完成/释放/故障；顺序参与active区间判断。
enum class Phase {
  Idle,
  Prestream,
  Offboard,
  Arm,
  Takeoff,
  Hold,
  Velocity,
  Brake,
  Landing,
  Done,
  Released,
  Fault
};
// 协议阶段名保持英文原值，供现有日志与控制台识别。
inline const char* name(Phase s) {
  switch (s) {
  case Phase::Idle:
    return "IDLE";
  case Phase::Prestream:
    return "PRESTREAM";
  case Phase::Offboard:
    return "REQUEST_OFFBOARD";
  case Phase::Arm:
    return "REQUEST_ARM";
  case Phase::Takeoff:
    return "TAKEOFF";
  case Phase::Hold:
    return "HOLD";
  case Phase::Velocity:
    return "VELOCITY";
  case Phase::Brake:
    return "BRAKE";
  case Phase::Landing:
    return "LANDING";
  case Phase::Done:
    return "DONE";
  case Phase::Released:
    return "RELEASED";
  case Phase::Fault:
    return "FAULT";
  }
  return "UNKNOWN";
}
// None不发布运动目标，Position使用位置目标，Velocity使用速度目标；未选字段保留NaN。
enum class Control { None, Position, Velocity };
// 动作意图由外层最终发布；这里没有强制Disarm命令，降落需PX4独立确认。
enum class VehicleCommand { None, Offboard, Arm, Land };
// 返回计划中的模式、动作和目标；NaN表示未控制字段，不能回填为零改变PX4语义。
struct Output {
  Control control = Control::None;
  VehicleCommand command = VehicleCommand::None;
  Vec position{nan(), nan(), nan()}, velocity{nan(), nan(), nan()};
  double yaw = nan(), yaw_rate = nan();
};

// 纯计算状态机，不拥有ROS、串口或线程；调用顺序是receive/start/step，状态与滤波记忆属于单个Core实例。
class Core {
public:
  // 复制并验证通用配置；默认值仅用于该历史/仿真路线。
  explicit Core(Config c = {}) : c_(c) {
    c_.validate();
  }
  // 只读暴露阶段/原因与任务原点；不会触发状态转移。
  Phase phase() const {
    return phase_;
  }
  const std::string& reason() const {
    return reason_;
  }
  Vec origin() const {
    return origin_;
  }
  // 区间只包含预发送到降落；Fault和Released不会继续发运动目标。
  bool active() const {
    return phase_ >= Phase::Prestream && phase_ <= Phase::Landing;
  }
  // 仅保持/速度/制动阶段接受非零输入，起飞过程中不能提前插入水平速度。
  bool accepts_motion() const {
    return phase_ == Phase::Hold || phase_ == Phase::Velocity || phase_ == Phase::Brake;
  }
  // 年龄按缓存接收时间计算；无命令使用巨大哨兵值，确保失令保护生效。
  double command_age(double t) const {
    return received_ >= 0 ? t - received_ : 1e9;
  }
  // 目标必须显式有效且年龄在0到command_timeout之间；时间回退也不视为新鲜。
  bool command_fresh(double t) const {
    return command_valid_ && command_age(t) >= 0 && command_age(t) <= c_.command_timeout;
  }
  // 启动前按故障、任务占用、遥测、发布者冲突、地面/预检和零指令流顺序拒绝，空串才表示可准备。
  std::string not_ready(const Feedback& f, double t) const {
    if (phase_ == Phase::Fault)
      return "FAULT: inspect cause, then restart this controller on safe ground";
    if (active())
      return "flight already active";
    if (!f.fresh)
      return "waiting for fresh PX4 position/status/land telemetry";
    if (!f.valid || !finite(f.position) || !finite(f.velocity) || !std::isfinite(f.yaw))
      return "local state estimate invalid";
    if (!f.exclusive)
      return "another PX4 setpoint publisher or not exactly one command source";
    if (f.armed || !f.landed || f.offboard)
      return "must be landed, disarmed and out of Offboard";
    if (!f.preflight || f.failsafe)
      return "PX4 preflight checks have not passed";
    if (norm(f.velocity) > .15)
      return "vehicle is not stationary";
    if (!command_fresh(t) || !zero(input_))
      return "fresh zero command stream required";
    return {};
  }
  // t为主机单调秒时刻，source_age为传输前已经消耗的寿命；非法或阶段不允许的目标会清空旧指令。
  bool receive(Velocity v, double t, double source_age = 0.) {
    if (!finite(v) || !std::isfinite(t) || !std::isfinite(source_age) || source_age < 0. ||
        source_age > c_.command_timeout || (!accepts_motion() && !zero(v))) {
      invalidate();
      return false;
    }
    // Transport delay consumes the existing lifetime; reception must not renew it.
    // 回溯缓存基准到生成时刻，不允许把迟到目标重新赋予完整command_timeout。
    input_ = v;
    received_ = t - source_age;
    command_valid_ = true;
    return true;
  }
  // 撤销缓存及有效位，后续step自然生成制动/降落，不能继续沿用上一条非零目标。
  void invalidate() {
    input_ = {};
    command_valid_ = false;
  }
  // 仅在not_ready为空时锁定原点、偏航及reset_id，清滤波/事务记忆后进入预发送。
  bool start(const Feedback& f, double t) {
    reason_ = not_ready(f, t);
    if (!reason_.empty())
      return false;
    origin_ = f.position;
    hold_ = f.position;
    reference_z_ = f.position.z;
    yaw_target_ = f.yaw;
    reset_id_ = f.reset_id;
    last_ = t;
    arm_sent_ = false;
    unresolved_arm_ = false;
    seen_armed_ = false;
    last_command_ = -1e9;
    filtered_ = {};
    filtered_yaw_ = 0;
    enter(Phase::Prestream, t);
    return true;
  }
  // 请求降落会撤销输入；已发Arm结果不明时仍进入降落等待，不能因当前armed=false就判定已安全停止。
  void land(const Feedback& f, double t, const std::string& reason) {
    if (!active() || phase_ == Phase::Landing)
      return;
    reason_ = reason;
    invalidate();
    last_command_ = -1e9;
    if (!f.armed && !unresolved_arm_ && phase_ != Phase::Landing) {
      enter(Phase::Done, t);
      return;
    }
    enter(Phase::Landing, t);
  }
  // 外层确认Arm被拒绝时才清除未决Arm记忆，不靠超时假设命令未执行。
  void rejected_arm() {
    unresolved_arm_ = false;
  }
  // 明确放弃本路线托管并撤销目标；不再自动抢回PX4模式。
  void release(double t, const std::string& reason) {
    reason_ = reason;
    invalidate();
    unresolved_arm_ = false;
    enter(Phase::Released, t);
  }

  // 每次使用一个完整实际快照；先检查权限/健康/时钟和失令，再推进当前阶段，返回尚待最终sink校验的计划。
  Output step(const Feedback& f, double t) {
    Output o;
    const double elapsed = t - last_;
    last_ = t;
    // 滤波积分步长最多0.1秒；原始elapsed另用于stall判断，不能用截断dt掩盖控制循环停顿。
    const double dt = std::clamp(elapsed, 0., .1);
    // 故障后唯一特殊入口是迟到的新armed+Offboard反馈：需要请求降落而非忘记已提交Arm。
    if (phase_ == Phase::Fault) {
      if (unresolved_arm_ && f.fresh && f.armed && f.offboard) {
        seen_armed_ = true;
        unresolved_arm_ = false;
        enter(Phase::Landing, t);
        last_command_ = -1e9;
        reason_ = "late ARM feedback; requesting landing";
      } else
        return o;
    }
    if (!active())
      return o;
    // 发现控制发布者冲突立即释放，不与其他来源竞争运动目标。
    if (!f.exclusive) {
      reason_ = "publisher conflict; controller released";
      enter(Phase::Released, t);
      return o;
    }
    if (f.fresh && f.armed) {
      seen_armed_ = true;
      unresolved_arm_ = false;
    }
    if (phase_ != Phase::Landing) {
      // PX4已换模式或进入failsafe时释放；不能把自动降落当成夺回控制权的理由。
      if ((phase_ >= Phase::Arm && f.fresh && !f.offboard) || f.failsafe) {
        reason_ = "PX4 changed mode or entered failsafe; no automatic takeover";
        enter(Phase::Released, t);
        return o;
      }
      if (!f.fresh || !f.valid || !finite(f.position) || !finite(f.velocity) ||
          !std::isfinite(f.yaw))
        land(f, t, "telemetry or local estimate invalid");
      // 停顿或时钟回退走原降落路径；此处阈值不随格式化修改。
      else if (elapsed > .3 || elapsed < 0)
        land(f, t, "control timer stalled or moved backwards");
      else if (f.reset_id != reset_id_)
        land(f, t, "estimator reference reset");
      else if (seen_armed_ && !f.armed) {
        reason_ = "unexpected disarm";
        enter(Phase::Fault, t);
        return o;
      } else if (command_age(t) > c_.land_after_loss)
        land(f, t, "command stream lost");
      else if (accepts_motion() && !f.heading_ok)
        land(f, t, "heading estimate invalid");
      else if (accepts_motion() &&
               (origin_.z - f.position.z > c_.max_height || origin_.z - f.position.z < .5 ||
                std::hypot(f.position.x - origin_.x, f.position.y - origin_.y) > c_.max_radius))
        land(f, t, "simulation flight envelope exceeded");
    }
    if (!active())
      return o;
    if (phase_ == Phase::Landing)
      return landing(f, t);
    // 预发送先保持当前坐标，满足持续时长后才计划切Offboard。
    if (phase_ == Phase::Prestream) {
      o = position_output(hold_);
      if (t - entered_ >= c_.prestream)
        enter(Phase::Offboard, t);
      // 位置目标持续发送；模式未确认时有界等待，按原0.5秒间隔重试模式命令。
    } else if (phase_ == Phase::Offboard) {
      o = position_output(hold_);
      if (f.offboard)
        enter(Phase::Arm, t);
      else if (t - entered_ > c_.mode_timeout) {
        reason_ = "Offboard not confirmed";
        enter(Phase::Fault, t);
        return {};
      } else if (t - last_command_ >= .5) {
        o.command = VehicleCommand::Offboard;
        last_command_ = t;
      }
      // 解锁仅发送一次；结果未确认超时后保留unresolved_arm_以处理迟到反馈。
    } else if (phase_ == Phase::Arm) {
      o = position_output(hold_);
      if (f.armed)
        enter(Phase::Takeoff, t);
      else if (t - entered_ > c_.mode_timeout) {
        reason_ = "ARM not confirmed; do not automatically retry";
        enter(Phase::Fault, t);
        return {};
      } else if (!arm_sent_) {
        o.command = VehicleCommand::Arm;
        arm_sent_ = unresolved_arm_ = true;
      }
      // NED上升意味着z下降；轨迹既限制爬升速度，又限制领先实测位置，防止目标提前冲到最终高度。
    } else if (phase_ == Phase::Takeoff) {
      const double final_z = origin_.z - c_.height;
      reference_z_ = std::max(
          final_z, std::max(reference_z_ - c_.climb_speed * dt, f.position.z - c_.reference_lead));
      o = position_output({origin_.x, origin_.y, reference_z_});
      // 完成起飞需未着陆、航向可用、位置和三轴速度在容差内持续settle_time，时间到了并不等于起飞成功。
      const bool arrived = !f.landed && f.heading_ok &&
                           std::abs(f.position.z - final_z) <= c_.height_tolerance &&
                           std::hypot(f.position.x - origin_.x, f.position.y - origin_.y) < .3 &&
                           norm(f.velocity) <= c_.speed_tolerance;
      if (dwell(arrived, t, c_.settle_time)) {
        hold_ = f.position;
        filtered_ = {};
        filtered_yaw_ = 0;
        enter(Phase::Hold, t);
        reason_ = "takeoff settled; velocity commands enabled";
      } else if (t - entered_ > c_.takeoff_timeout) {
        land(f, t, "takeoff feedback gate timed out");
        return landing(f, t);
      }
    } else {
      // 超时目标立即替换为零；制动完成后重新捕获当前保持点，避免跳回旧坐标。
      const Velocity cmd = command_fresh(t) ? input_ : Velocity{};
      if (phase_ == Phase::Hold && zero(cmd))
        return position_output(hold_);
      const bool stopping = zero(cmd);
      if (!stopping) {
        if (phase_ != Phase::Velocity)
          enter(Phase::Velocity, t);
      } else if (phase_ != Phase::Brake)
        enter(Phase::Brake, t);
      // 先转换坐标再限制XY合速度、垂速和逐步加速度，保持原运算顺序。
      Vec desired = heading_to_ned(cmd, f.yaw);
      const double xy = std::hypot(desired.x, desired.y);
      if (xy > c_.xy_speed) {
        desired.x *= c_.xy_speed / xy;
        desired.y *= c_.xy_speed / xy;
      }
      desired.z = std::clamp(desired.z, -c_.z_speed, c_.z_speed);
      // XY采用向量增量限幅，保持加速方向；垂向与偏航分别用approach约束。
      const double dx = desired.x - filtered_.x, dy = desired.y - filtered_.y;
      const double delta = std::hypot(dx, dy),
                   scale = delta > 0 ? std::min(1., c_.xy_accel * dt / delta) : 0.;
      filtered_.x += dx * scale;
      filtered_.y += dy * scale;
      filtered_.z = approach(filtered_.z, desired.z, c_.z_accel * dt);
      filtered_yaw_ = approach(filtered_yaw_, std::clamp(-cmd.ccw, -c_.yaw_speed, c_.yaw_speed),
                               c_.yaw_accel * dt);
      yaw_target_ = wrap(yaw_target_ + filtered_yaw_ * dt);
      // 偏航目标相对实测航向也限制最大误差，避免长时间积分后命令机头瞬间追赶。
      yaw_target_ = wrap(f.yaw + std::clamp(wrap(yaw_target_ - f.yaw), -.35, .35));
      o.control = Control::Velocity;
      o.velocity = filtered_;
      o.yaw = yaw_target_;
      o.yaw_rate = filtered_yaw_;
      if (stopping &&
          dwell(norm(f.velocity) < .15 && norm(filtered_) < .01 && std::abs(filtered_yaw_) < .01, t,
                .3)) {
        hold_ = f.position;
        enter(Phase::Hold, t);
        o = position_output(hold_);
      }
    }
    return o;
  }

private:
  // 阶段转移统一记录进入时刻并重置连续稳定计时；进入降落时重新要求模式确认。
  void enter(Phase s, double t) {
    if (s == Phase::Landing && phase_ != Phase::Landing)
      landing_confirmed_ = false;
    phase_ = s;
    entered_ = t;
    settled_ = -1;
  }
  // 条件中断立即清计时；只有连续成立才满足duration，不累计碎片时间。
  bool dwell(bool valid, double t, double duration) {
    if (!valid) {
      settled_ = -1;
      return false;
    }
    if (settled_ < 0)
      settled_ = t;
    return t - settled_ >= duration;
  }
  // 生成位置模式计划及当前偏航，速度字段保持NaN，不同时声明两种控制模式。
  Output position_output(Vec p) const {
    Output o;
    o.control = Control::Position;
    o.position = p;
    o.yaw = yaw_target_;
    return o;
  }
  // 持续观察真实Land/上锁状态；若PX4已有其他模式接管则释放，缺少着陆证据不能强行上锁。
  Output landing(const Feedback& f, double t) {
    Output o;
    if (f.fresh && f.landing)
      landing_confirmed_ = true;
    // 完成要求新鲜、已着陆且上锁并无未决Arm，再连续观察1秒；ACK或经过时间都不单独证明落地。
    if (f.fresh && f.landed && !f.armed && !unresolved_arm_) {
      if (dwell(true, t, 1.)) {
        enter(Phase::Done, t);
        reason_ = "landing and disarm confirmed";
      }
      return o;
    }
    settled_ = -1;
    if (t - entered_ > c_.land_timeout) {
      reason_ = "landing not confirmed; PX4/operator must finish";
      enter(Phase::Fault, t);
      return o;
    }
    if (f.fresh && f.armed && !f.landing && (landing_confirmed_ || !f.offboard || f.failsafe)) {
      reason_ = "other mode took over during landing";
      enter(Phase::Released, t);
      return o;
    }
    // 仅在健康且仍归本路线Offboard托管时给零速度保护，其他模式不继续干预。
    if (f.fresh && f.offboard && f.valid && finite(f.position) && finite(f.velocity) &&
        std::isfinite(f.yaw)) {
      o.control = Control::Velocity;
      o.velocity = {};
      o.yaw = f.yaw;
      o.yaw_rate = 0;
    }
    // Request LAND until telemetry confirms PX4 Land. Never force disarm.
    if (!f.landing && t - last_command_ >= 1.) {
      o.command = VehicleCommand::Land;
      last_command_ = t;
    }
    return o;
  }
  // 以下是该实例的配置、阶段、输入与滤波记忆；不与姿态高度主链共享隐式全局状态。
  Config c_;
  Phase phase_ = Phase::Idle;
  Velocity input_;
  Vec origin_, hold_, filtered_;
  double filtered_yaw_ = 0, yaw_target_ = 0, reference_z_ = 0;
  // received/last/entered/settled/last_command均为同一主机单调秒域；负值为尚未观测的哨兵。
  double received_ = -1, last_ = 0, entered_ = 0, settled_ = -1, last_command_ = -1e9;
  uint32_t reset_id_ = 0;
  bool command_valid_ = false, arm_sent_ = false, unresolved_arm_ = false, seen_armed_ = false;
  bool landing_confirmed_ = false;
  std::string reason_;
};
} // namespace jikl_velocity
