// 视觉目标速度只读预览回归：自身状态与机体目标各自独立合同，预览始终没有真实控制权限。
// 状态NED，目标body_frd前/右/下m/s；使用完整FRD→NED四元数旋转，不用仅yaw近似代替。
// ROS源戳ns、接收寿命单调s同时有效；state/command reset分别表示估计器和制导重置，不能混淆。
#include "jikl_fc01_system/velocity_command_input.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace vi = fc01::velocity_input;
namespace v = fc01::vision;
constexpr double pi = 3.14159265358979323846;
constexpr std::uint64_t epoch = 100000000000ULL;
static unsigned checks = 0;
static void need(bool value, const char* reason) {
  ++checks;
  if (!value)
    throw std::runtime_error(reason);
}
static bool near(double first, double second, double epsilon = 1e-9) {
  return std::abs(first - second) < epsilon;
}
static vi::Policy policy() {
  // Synthetic offline fixtures only; these do not configure production flight.
  return {.25, .30, .20, .05, 2., .08, .6};
}
static v::State state() {
  v::State out;
  out.sequence = 1;
  out.stamp_ns = epoch;
  out.reset_counter = 7;
  out.source_id = "synthetic-state";
  out.frame_id = "fc01_vision_ned";
  out.body_frame_id = "base_link_frd";
  out.initialized = out.tracking = out.calibration_valid = true;
  out.valid_fields = v::Position | v::Velocity | v::Attitude | v::Quality;
  out.position = {0, 0, 0};
  out.velocity = {0, 0, 0};
  out.attitude = {1, 0, 0, 0};
  out.quality = 1;
  return out;
}
static vi::Target target() {
  vi::Target out;
  out.schema_version = 1;
  out.sequence = 1;
  out.stamp_ns = epoch;
  out.reset_counter = 13;
  out.source_id = "synthetic-command";
  out.frame_id = "body_frd";
  out.valid = true;
  out.body_velocity = {.3, -.2, -.1};
  return out;
}
static void feed(vi::Session& session, v::State measured = state(), vi::Target wanted = target()) {
  need(session.state(measured, 10, epoch), "valid measured state must cache");
  need(session.command(wanted, 10, epoch), "valid independent target must cache");
}
static vi::Preview preview(vi::Session& session, double now = 10, std::uint64_t ros_now = epoch,
                           double yaw = 0) {
  const auto out = session.preview(vi::Mode::VelocityPreview, now, ros_now, yaw);
  need(!out.control_authority, "offline preview must never carry aircraft authority");
  return out;
}
// 检查wire字段/纳秒边界与显式策略；未配置速度和时限不得输出默认可用预览，缺首帧仅等待。
static void wire_contract_and_unconfigured_policy() {
  struct Stamp {
    std::int32_t sec{100};
    std::uint32_t nanosec{};
  };
  struct Wire {
    std::uint16_t schema_version{1};
    std::uint64_t sequence{42};
    Stamp stamp;
    std::uint32_t reset_counter{19};
    std::string source_id{"wire"}, frame_id{"body_frd"};
    bool valid{true};
    double vx{-.2}, vy{.1}, vz{.05};
  } wire;
  auto mapped = vi::from_message(wire);
  need(mapped.sequence == 42 && mapped.stamp_ns == epoch && mapped.reset_counter == 19,
       "wire identity maps exactly");
  need(mapped.body_velocity == vi::Vec3({-.2, .1, .05}) && mapped.received_at == -1,
       "wire target stays independent of receive time");
  wire.stamp.nanosec = 1000000000U;
  need(vi::from_message(wire).stamp_ns == 0, "invalid wire nanoseconds rejected");
  wire.stamp.nanosec = 0;
  wire.stamp.sec = -1;
  need(vi::from_message(wire).stamp_ns == 0, "negative wire seconds rejected");
  vi::Session unconfigured;
  need(!preview(unconfigured).valid, "default policy may not qualify preview");
  need(!unconfigured.command(target(), 10, epoch), "default limits may not qualify target cache");
  auto invalid = policy();
  invalid.max_command_speed_mps = 0;
  vi::Session no_speed(invalid);
  need(!preview(no_speed).valid, "missing explicit speed limit disables preview");
  invalid = policy();
  invalid.command_timeout = std::numeric_limits<double>::quiet_NaN();
  vi::Session no_time(invalid);
  need(!preview(no_time).valid, "nonfinite policy disables preview");
  vi::Session waiting(policy());
  need(!preview(waiting).valid, "missing state/target waits");
  need(waiting.first_error().empty(), "startup without data does not latch an input fault");
  vi::Session whitespace(policy());
  auto blank = target();
  blank.source_id = " \t\r\n";
  need(!whitespace.command(blank, 10, epoch), "whitespace is not a source identity");
  vi::Session oversized(policy());
  auto huge = state();
  huge.source_id = std::string(257, 's');
  need(!oversized.state(huge, 10, epoch), "oversize source identity is not cached");
}
// 误差=目标减实际；不同reset合法保留，Disabled无可用姿态且不清离线缓存，消息无yaw则保持可信参考。
static void actual_target_separation_and_modes() {
  vi::Session session(policy());
  auto measured = state();
  measured.velocity = {.1, -.05, .02};
  feed(session, measured);
  auto out = preview(session);
  need(out.valid, "independent command reset 13 and estimator reset 7 are accepted");
  need(out.state_reset == 7 && out.command_reset == 13, "both reset meanings are preserved");
  need(out.body_target == target().body_velocity, "target cache is not the measured state");
  need(out.measured_world_velocity == measured.velocity,
       "actual velocity remains measured NED velocity");
  need(near(out.velocity_error[0], .2) && near(out.velocity_error[1], -.15) &&
           near(out.velocity_error[2], -.12),
       "target minus actual forms velocity error");
  need(near(out.horizontal_acceleration[0], .24) && near(out.horizontal_acceleration[1], -.18),
       "existing velocity gain computes horizontal acceleration");
  auto disabled = session.preview(vi::Mode::Disabled, 10, epoch, 0);
  need(!disabled.valid && !disabled.control_authority && std::isnan(disabled.attitude_preview[0]),
       "Disabled has no usable attitude preview");
  need(preview(session).valid, "Disabled does not erase cached offline session");
  out = preview(session, 10, epoch, pi / 2);
  need(near(v::detail::yaw(out.attitude_preview), pi / 2) && near(out.yaw_reference, pi / 2),
       "message has no yaw: trusted reference is preserved");
}
// 分别用yaw/pitch/roll验证完整三维旋转，再检查XY向量速度、垂向、加速度和倾角上限与零速度制动。
static void full_three_dimensional_body_rotation_and_limits() {
  const double half = std::sqrt(.5);
  for (const auto& q :
       {fc01::attitude::Quaternion{half, 0, 0, half}, fc01::attitude::Quaternion{half, 0, half, 0},
        fc01::attitude::Quaternion{half, half, 0, 0}}) {
    vi::Session session(policy());
    auto measured = state();
    measured.attitude = q;
    auto wanted = target();
    wanted.body_velocity = {.2, 0, 0};
    if (q[1] != 0)
      wanted.body_velocity = {0, .2, 0};
    feed(session, measured, wanted);
    const auto out = preview(session);
    need(out.valid, "valid nonlevel attitude permits a pure coordinate preview");
    if (q[3] != 0)
      need(near(out.world_target[0], 0) && near(out.world_target[1], .2),
           "yaw rotates forward into east");
    if (q[2] != 0)
      need(near(out.world_target[0], 0) && near(out.world_target[2], -.15),
           "pitch rotates forward into world vertical then limits it");
    if (q[1] != 0)
      need(near(out.world_target[1], 0) && near(out.world_target[2], .15),
           "roll rotates right into world vertical then limits it");
  }
  vi::Session vertical_rotation(policy());
  auto measured = state();
  measured.attitude = {half, 0, half, 0};
  auto wanted = target();
  wanted.body_velocity = {0, 0, .2};
  feed(vertical_rotation, measured, wanted);
  auto out = preview(vertical_rotation);
  need(near(out.world_target[0], .2) && near(out.world_target[2], 0),
       "body down may become horizontal: no yaw-only substitution");
  vi::Session limited(policy());
  wanted = target();
  wanted.body_velocity = {1, 1, 1};
  feed(limited, state(), wanted);
  out = preview(limited);
  need(out.valid && near(std::hypot(out.world_target[0], out.world_target[1]), .5),
       "existing candidate XY vector speed limit applies");
  need(near(out.world_vertical_velocity_preview, .15),
       "vertical target is bounded and explicitly preview only");
  need(std::hypot(out.horizontal_acceleration[0], out.horizontal_acceleration[1]) <= .6 + 1e-9,
       "existing candidate horizontal acceleration limit applies");
  const double tilt =
      std::acos(std::clamp(1 - 2 * (out.attitude_preview[1] * out.attitude_preview[1] +
                                    out.attitude_preview[2] * out.attitude_preview[2]),
                           -1., 1.));
  need(tilt <= 5 * pi / 180 + 1e-9, "existing candidate tilt envelope applies");
  vi::Session zero(policy());
  wanted = target();
  wanted.body_velocity = {0, 0, 0};
  measured = state();
  measured.velocity = {.2, 0, 0};
  feed(zero, measured, wanted);
  out = preview(zero);
  need(out.valid && out.horizontal_acceleration[0] < 0,
       "zero desired speed brakes actual movement");
  vi::Session changing(policy());
  wanted = target();
  wanted.body_velocity = {.2, 0, 0};
  feed(changing, state(), wanted);
  measured = state();
  measured.sequence = 2;
  measured.stamp_ns += 50000000;
  measured.attitude = {std::cos(.2), 0, 0, std::sin(.2)};
  need(changing.state(measured, 10.05, epoch + 50000000),
       "continuous attitude changes are accepted");
  out = preview(changing, 10.05, epoch + 50000000);
  need(out.valid && near(out.world_target[0], .2 * std::cos(.4)) &&
           near(out.world_target[1], .2 * std::sin(.4)),
       "fresh attitude changes full body-vector conversion");
  measured.sequence = 3;
  measured.stamp_ns += 50000000;
  for (auto& component : measured.attitude)
    component = -component;
  need(changing.state(measured, 10.1, epoch + 100000000),
       "quaternion sign flip is the same physical attitude");
  measured.sequence = 4;
  measured.stamp_ns += 50000000;
  measured.attitude = {0, 1, 0, 0};
  need(!changing.state(measured, 10.15, epoch + 150000000) &&
           !preview(changing, 10.15, epoch + 150000000).valid,
       "unannounced orientation jump invalidates preview");
}
// 十二类错误目标锁存首因，正常尾帧不能掩盖；只有显式新离线attempt清全部旧缓存。
static void bad_command_never_recovers_through_good_tail() {
  for (int fault = 0; fault < 12; ++fault) {
    vi::Session session(policy());
    feed(session);
    need(preview(session).valid, "fault fixture starts valid");
    auto bad = target();
    bad.sequence = 2;
    bad.stamp_ns += 50000000;
    if (fault == 0)
      bad.schema_version = 2;
    if (fault == 1)
      bad.frame_id = "base_link_frd";
    if (fault == 2)
      bad.valid = false;
    if (fault == 3)
      bad.body_velocity[0] = std::numeric_limits<double>::quiet_NaN();
    if (fault == 4)
      bad.body_velocity[2] = std::numeric_limits<double>::infinity();
    if (fault == 5)
      bad.body_velocity = {2, 2, 2};
    if (fault == 6)
      bad.sequence = 0;
    if (fault == 7)
      bad.source_id = "other-producer";
    if (fault == 8)
      ++bad.reset_counter;
    if (fault == 9)
      bad.stamp_ns = epoch;
    if (fault == 10)
      bad.stamp_ns = 0;
    if (fault == 11)
      bad.source_id = "";
    need(!session.command(bad, 10.05, epoch + 50000000), "bad command must be rejected");
    const auto reason = session.first_error();
    need(!reason.empty(), "bad command retains first error");
    auto good = target();
    good.sequence = 3;
    good.stamp_ns += 100000000;
    need(!session.command(good, 10.1, epoch + 100000000), "good tail cannot hide command anomaly");
    need(!preview(session, 10.1, epoch + 100000000).valid && session.first_error() == reason,
         "preview remains invalid with original reason");
    session.begin_attempt();
    need(!preview(session).valid && session.first_error().empty(),
         "new offline attempt drops all old caches");
    feed(session);
    need(preview(session).valid, "new explicit offline attempt can recover");
  }
}
// 有限输入也可能在相减/乘增益/范数溢出；不可由限幅洗成可用零制动，拒绝预览字段维持NaN。
static void intermediate_overflow_is_not_hidden_by_limits() {
  auto wanted = target();
  wanted.body_velocity = {0, 0, 0};
  for (const auto& velocity : {vi::Vec3{1.2e308, 1.2e308, 0}, vi::Vec3{1.3e308, 1.3e308, 0},
                               vi::Vec3{std::numeric_limits<double>::max(), 0, 0}}) {
    vi::Session session(policy());
    auto measured = state();
    measured.velocity = velocity;
    feed(session, measured, wanted);
    const auto out = preview(session);
    need(!out.valid,
         "finite input with an overflowing error, product or norm must not qualify preview");
    need(out.reason == "VELOCITY_PREVIEW_NONFINITE_VELOCITY_ERROR" ||
             out.reason == "VELOCITY_PREVIEW_NONFINITE_ACCELERATION",
         "overflow retains its calculation stage");
    need(std::isnan(out.horizontal_acceleration[0]) && std::isnan(out.attitude_preview[0]),
         "rejected overflow must not expose a usable zero-braking preview");
    need(!preview(session).valid, "overflow cannot be cleared by another preview call");
  }
  // Exercise subtraction overflow separately from gain/norm overflow.
  auto large_policy = policy();
  large_policy.max_command_speed_mps = 1.5e308;
  v::Config large_config;
  large_config.max_speed = 1.5e308;
  vi::Session difference(large_policy, large_config);
  auto measured = state();
  measured.velocity = {-1.2e308, 0, 0};
  wanted.body_velocity = {1.2e308, 0, 0};
  feed(difference, measured, wanted);
  auto out = preview(difference);
  need(!out.valid && out.reason == "VELOCITY_PREVIEW_NONFINITE_VELOCITY_ERROR",
       "opposite finite velocities cannot overflow subtraction then be washed through a limiter");

  // Large representable norms still retain their direction and normal limits.
  vi::Session representable(policy());
  measured = state();
  measured.velocity = {1.e308, 1.e308, 0};
  wanted.body_velocity = {0, 0, 0};
  feed(representable, measured, wanted);
  out = preview(representable);
  need(out.valid, "representable finite acceleration norm remains computable");
  need(out.horizontal_acceleration[0] < 0 && out.horizontal_acceleration[1] < 0 &&
           near(out.horizontal_acceleration[0], out.horizontal_acceleration[1]),
       "representable large feedback preserves the braking direction");
  need(near(std::hypot(out.horizontal_acceleration[0], out.horizontal_acceleration[1]), .6),
       "representable large feedback uses the existing acceleration limit");

  for (double multiplier : {.999999, 1., 1.000001}) {
    vi::Session ordinary(policy());
    measured = state();
    measured.velocity = {-.3 * multiplier, -.4 * multiplier, 0};
    feed(ordinary, measured, wanted);
    out = preview(ordinary);
    const double expected = std::min(.6, .6 * multiplier);
    need(out.valid &&
             near(std::hypot(out.horizontal_acceleration[0], out.horizontal_acceleration[1]),
                  expected),
         "ordinary vectors on either side of the acceleration bound retain normal limiting");
    need(near(out.horizontal_acceleration[0] / out.horizontal_acceleration[1], .75),
         "ordinary limiting preserves acceleration direction");
  }
}
// 跟踪/质量/无效/帧/来源/reset/跳变/协方差与来源数量瞬变都保持异常，正常图尾帧不能清锁。
static void bad_states_and_graph_events_are_sticky() {
  for (int fault = 0; fault < 12; ++fault) {
    vi::Session session(policy());
    feed(session);
    auto bad = state();
    bad.sequence = 2;
    bad.stamp_ns += 50000000;
    if (fault == 0)
      bad.tracking = false;
    if (fault == 1)
      bad.quality = .1;
    if (fault == 2)
      bad.velocity[1] = std::numeric_limits<double>::infinity();
    if (fault == 3)
      bad.attitude = {0, 0, 0, 0};
    if (fault == 4)
      bad.frame_id = "map";
    if (fault == 5)
      bad.body_frame_id = "camera";
    if (fault == 6)
      bad.source_id = "another-estimator";
    if (fault == 7)
      ++bad.reset_counter;
    if (fault == 8)
      bad.velocity[0] = 1.1;
    if (fault == 9)
      bad.position[0] = .8;
    if (fault == 10)
      bad.valid_fields &= ~v::Velocity;
    if (fault == 11) {
      bad.valid_fields |= v::PoseCovariance;
      bad.pose_covariance.fill(0);
      bad.pose_covariance[0] = -1;
    }
    need(!session.state(bad, 10.05, epoch + 50000000), "bad state must be rejected");
    auto good = state();
    good.sequence = 3;
    good.stamp_ns += 100000000;
    need(!session.state(good, 10.1, epoch + 100000000) &&
             !preview(session, 10.1, epoch + 100000000).valid,
         "healthy state tail cannot conceal anomaly");
  }
  for (const auto& counts : {std::pair<std::size_t, std::size_t>{2, 1}, {1, 2}, {0, 1}, {1, 0}}) {
    vi::Session session(policy());
    feed(session);
    session.observe_publishers(counts.first, counts.second);
    session.observe_publishers(1, 1);
    need(!preview(session).valid,
         "temporary missing/conflicting source survives healthy graph tail");
  }
}
// 相同姿态和整体变号真转角应为零；1e-10 rad门两侧真实小转角仍分别判断，不能用固定容差放水。
static void attitude_step_remains_accurate_for_small_rotations() {
  auto precise_policy = policy();
  precise_policy.max_attitude_step_rad = 1e-10;

  // 这个四元数归一化后的自点积会略小于 1；完全相同姿态应仍然表示零转角。
  auto measured = state();
  measured.attitude = {.43301498687850332, .38595587278205279, .60808732941602328,
                       .54200173911837457};
  for (double equivalent_sign : {1., -1.}) {
    vi::Session session(precise_policy);
    feed(session, measured);
    auto unchanged = measured;
    unchanged.sequence = 2;
    unchanged.stamp_ns += 1000000;
    for (double& component : unchanged.attitude) {
      component *= equivalent_sign;
    }
    need(session.state(unchanged, 10.001, epoch + 1000000),
         "identical attitudes and their sign equivalents have zero angular step");
    need(session.first_error().empty() && preview(session, 10.001, epoch + 1000000).valid,
         "rounding of a quaternion self-dot product cannot latch a false discontinuity");
  }

  // 在 1e-10 rad 门限两侧放置真实转角，避免用固定容差把小跳变全部放行。
  // 同时验证整体变号不会改变几何角度，明显的大跳变仍须拒绝。
  for (double angle : {0.5e-10, 1.5e-10, .7}) {
    for (double equivalent_sign : {1., -1.}) {
      vi::Session session(precise_policy);
      feed(session);
      auto rotated = state();
      rotated.sequence = 2;
      rotated.stamp_ns += 1000000;
      rotated.attitude = {equivalent_sign * std::cos(angle / 2),
                          equivalent_sign * std::sin(angle / 2), 0, 0};
      const bool expected = angle < precise_policy.max_attitude_step_rad;
      need(session.state(rotated, 10.001, epoch + 1000000) == expected,
           "small true rotations preserve both sides of the configured angular gate");
      if (!expected) {
        need(session.first_error() == "STATE_ATTITUDE_DISCONTINUITY_REQUIRES_NEW_ATTEMPT",
             "true angular jumps retain the existing discontinuity latch");
        need(!preview(session, 10.001, epoch + 1000000).valid,
             "a rejected angular jump cannot produce a valid offline preview");
      }
    }
  }
}
// 同帧重放不续TTL、同序号改payload锁故障；ROS/steady倒退、源与姿态偏时、过期后新帧都要显式新attempt。
static void clocks_sequences_and_duplicate_leases() {
  for (bool measured : {false, true}) {
    vi::Session session(policy());
    feed(session);
    if (measured)
      need(!session.state(state(), 10.1, epoch + 100000000), "exact state duplicate ignored");
    else
      need(!session.command(target(), 10.1, epoch + 100000000), "exact command duplicate ignored");
    need(session.first_error().empty(),
         "exact duplicate is not a fresh sample or a payload mutation");
    need(!preview(session, 10.26, epoch + 260000000).valid,
         "duplicates never extend original source/receive TTL");
  }
  for (int fault = 0; fault < 7; ++fault) {
    vi::Session session(policy());
    feed(session);
    if (fault == 0) {
      auto bad = target();
      bad.body_velocity[0] = .31;
      session.command(bad, 10.01, epoch + 10000000);
    }
    if (fault == 1) {
      auto bad = state();
      bad.velocity[0] = .01;
      session.state(bad, 10.01, epoch + 10000000);
    }
    if (fault == 2)
      preview(session, 9.99, epoch);
    if (fault == 3)
      preview(session, 10.01, epoch - 1);
    if (fault == 4) {
      auto bad = target();
      bad.stamp_ns -= 300000000;
      session.command(bad, 10.01, epoch + 10000000);
    }
    if (fault == 5) {
      auto bad = target();
      bad.stamp_ns += 100000000;
      session.command(bad, 10.01, epoch + 10000000);
    }
    if (fault == 6)
      preview(session, 10, epoch, std::numeric_limits<double>::quiet_NaN());
    need(!preview(session, 10.02, epoch + 20000000).valid && !session.first_error().empty(),
         "clock or payload anomaly remains invalid");
  }
  vi::Session skew(policy());
  auto wanted = target();
  wanted.stamp_ns -= 100000000;
  feed(skew, state(), wanted);
  need(!preview(skew).valid && skew.first_error() == "STATE_COMMAND_ATTITUDE_TIME_SKEW",
       "body command needs temporally associated attitude");
  vi::Session arrival(policy());
  auto old = state();
  old.stamp_ns -= 210000000;
  need(!arrival.state(old, 10, epoch), "arrival transport delay is independent of source TTL");
  vi::Session source_ttl(policy());
  feed(source_ttl);
  need(!preview(source_ttl, 10.01, epoch + 260000000).valid,
       "new receipt age cannot renew old source timestamp");
  vi::Session steady_ttl(policy());
  feed(steady_ttl);
  need(!preview(steady_ttl, 10.26, epoch).valid,
       "frozen ROS clock cannot renew steady receive lifetime");
  vi::Session state_regression(policy());
  auto measured = state();
  measured.sequence = 2;
  feed(state_regression, measured);
  measured.sequence = 1;
  measured.stamp_ns += 50000000;
  need(!state_regression.state(measured, 10.05, epoch + 50000000),
       "state sequence regression rejected");
  vi::Session command_regression(policy());
  wanted = target();
  wanted.sequence = 2;
  feed(command_regression, state(), wanted);
  wanted.sequence = 1;
  wanted.stamp_ns += 50000000;
  need(!command_regression.command(wanted, 10.05, epoch + 50000000),
       "command sequence regression rejected");
  vi::Session gap(policy());
  feed(gap);
  measured = state();
  measured.sequence = 2;
  measured.stamp_ns += 260000000;
  need(!gap.state(measured, 10.26, epoch + 260000000),
       "fresh callback cannot conceal a previously expired state lease");
  wanted = target();
  wanted.sequence = 2;
  wanted.stamp_ns += 260000000;
  need(!gap.command(wanted, 10.26, epoch + 260000000) &&
           !preview(gap, 10.26, epoch + 260000000).valid,
       "two healthy tails cannot erase the intervening loss");
  for (int attempt = 0; attempt < 3; ++attempt) {
    gap.begin_attempt();
    feed(gap);
    need(preview(gap).valid,
         "multiple explicit offline attempts recover without permanent controller latch");
    auto invalid = target();
    invalid.valid = false;
    invalid.sequence = 2;
    invalid.stamp_ns += 50000000;
    gap.command(invalid, 10.05, epoch + 50000000);
    need(!preview(gap, 10.05, epoch + 50000000).valid,
         "each new anomaly remains within its offline attempt");
  }
}
int main() {
  wire_contract_and_unconfigured_policy();
  actual_target_separation_and_modes();
  full_three_dimensional_body_rotation_and_limits();
  bad_command_never_recovers_through_good_tail();
  intermediate_overflow_is_not_hidden_by_limits();
  bad_states_and_graph_events_are_sticky();
  attitude_step_remains_accurate_for_small_rotations();
  clocks_sequences_and_duplicate_leases();
  std::cout << "VELOCITY_COMMAND_INPUT_PREVIEW_ONLY_PASS checks=" << checks
            << " control_authority=false\n";
}
