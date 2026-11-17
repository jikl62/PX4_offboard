// R6.3.2 高度轨迹与速度前馈回归：检测确认不能重启已运行的高度轨迹，端点和slew须连续。
// HeightFlight输入observe(height,up_speed)为上正m、m/s，再写为NED z=2-height与vz=-up_speed。
// require_height失败立即非零退出；全部反馈合成，不含ROS或动力学。
// Offline Runtime regression: no ROS, serial port, motor, or PX4 connection.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main baseline_attitude_runtime_main_not_called
#include "test_attitude_runtime.cpp"
#undef main
#pragma GCC diagnostic pop
#include <cstdlib>
#include <string>

static void require_height(bool ok, const char* message) {
  if (!ok) {
    std::cerr << "HEIGHT_TRACKING_FAIL: " << message << '\n';
    std::exit(1);
  }
}
static bool near_height(double x, double y) {
  return std::abs(x - y) < 1e-9;
}
static f::Config height_config() {
  auto c = cfg(true);
  auto& p = c.attitude;
  p.vertical.hover = .50;
  p.vertical.minimum = .12;
  p.vertical.maximum = .80;
  p.vertical.slew = .4;
  p.vertical.height_gain = .5;
  p.vertical.velocity_gain = .15;
  p.vertical.integral_gain = .03;
  p.vertical.integral_limit = .08;
  p.vertical.max_climb = .3;
  p.vertical.max_descent = .2;
  p.ramp_seconds = 2.;
  p.liftoff_dwell_seconds = .2;
  p.takeoff_reserve_thrust = .04;
  c.validate();
  return c;
}
// 按准备、Arm、地面dwell、斜坡和联合离地证据进入真实Ascend；保存离地观测与任务目标分别核对。
struct HeightFlight {
  f::Config c{height_config()};
  f::Runtime r{c};
  f::Snapshot s{sample()};
  double t{}, requested{}, height_at_liftoff{};
  f::Output o;
  HeightFlight(double height = .42, double target = 1.)
      : requested(target), height_at_liftoff(height) {
    t = ready(r, s, true);
    require_height(r.request("INTERACTIVE", {}, s, t).accepted, "interactive admission");
    require_height(r.request("TAKEOFF", {target}, s, t).accepted, "takeoff admission");
    o = tick(r, s, t += .05);
    require_height(o.attitude.command == a::Command::Arm, "arm must use normal transaction");
    r.ack({r.pending(400).baseline + 1, 400, 0, 1, 191, false}, t + .01);
    s.feedback.armed = true;
    for (int i = 0; i < 65 && r.attitude_core().phase() != a::Phase::ThrustRamp; ++i)
      advance();
    require_height(r.attitude_core().phase() == a::Phase::ThrustRamp, "armed ground dwell");
    for (int i = 0; i < 42; ++i)
      advance();
    require_height(r.attitude_core().phase() == a::Phase::ThrustRamp,
                   "time/thrust cannot prove liftoff");
    s.feedback.landed = false;
    s.at_rest = false;
    s.attitude.z = 2. - height + .04;
    s.attitude.vz = -.1;
    for (int i = 0; i < 12 && r.attitude_core().phase() != a::Phase::Ascend; ++i) {
      s.attitude.z -= .005;
      advance();
    }
    require_height(r.attitude_core().phase() == a::Phase::Ascend, "joint liftoff evidence");
  }
  void advance(double dt = .05) {
    require_height(r.request("PING", {}, s, t).accepted, "console lease");
    o = tick(r, s, t += dt);
  }
  void observe(double height, double up_speed) {
    s.attitude.z = 2. - height;
    s.attitude.vz = -up_speed;
  }
};
// 反馈接入已观测高度但不可越过任务目标；离地时仍保留有界物理斜坡。
static void liftoff_reference() {
  HeightFlight q;
  require_height(
      q.o.attitude.target_height >= q.height_at_liftoff &&
          q.o.attitude.target_height <= q.requested,
      "early feedback must already have joined observed height before liftoff confirmation");
  require_height(-q.o.attitude.thrust[2] <= .54 + 1e-9,
                 "liftoff must retain bounded physical ramp");
}
// 目标轨迹上移且位置误差为零仍需要上升前馈；检测确认不重置轨迹，推力不能跳变。
static void moving_climb() {
  HeightFlight q(.2);
  const double prior = -q.o.attitude.thrust[2];
  const double prior_target = q.o.attitude.target_height;
  q.observe(.215, .3);
  q.advance();
  require_height(near_height(q.o.attitude.target_height, prior_target + .015),
                 "detector confirmation must not restart the active height trajectory");
  require_height(
      near_height(q.o.attitude.velocity_target, .3),
      "tracking a rising reference without position error still requires climb velocity");
  require_height(std::abs(-q.o.attitude.thrust[2] - prior) <= .4 * .05 + 1e-9,
                 "slew at feedback handover");
}
// 下降轨迹同理需要负的上正速度前馈，不能因跟踪无位置差就误给零速度。
static void moving_descent() {
  HeightFlight q;
  q.observe(.8, -.2);
  require_height(q.r.request("LAND", {}, q.s, q.t).accepted, "normal land admission");
  q.observe(.79, -.2);
  q.advance();
  require_height(q.r.attitude_core().phase() == a::Phase::Descent, "remain in guarded descent");
  require_height(near_height(q.o.attitude.velocity_target, -.2),
                 "tracking a descending reference requires descent velocity, not zero velocity");
}
// 接近端点时前馈按原位置增益渐减，到目标后必须连续为零。
static void shortened_endpoint() {
  HeightFlight q(.57, .6);
  q.observe(.585, 0);
  q.advance();
  require_height(
      near_height(q.o.attitude.velocity_target, .0075),
      "near the goal, feedforward must taper within the existing height-gain approach speed");
  q.observe(.6, 0);
  q.advance();
  require_height(near_height(q.o.attitude.target_height, .6),
                 "reference must stop at requested height");
  require_height(near_height(q.o.attitude.velocity_target, 0),
                 "endpoint must have no climb feedforward");
  q.observe(.6, 0);
  q.advance();
  require_height(near_height(q.o.attitude.velocity_target, 0),
                 "endpoint must not retain climb feedforward");
}
// 前馈与位置反馈合并后共同受同一升降速度上限，不能各自限幅后相加越界。
static void combined_speed_limits() {
  HeightFlight climb(.2);
  climb.observe(.13, 0);
  climb.advance();
  require_height(near_height(climb.o.attitude.velocity_target, .3),
                 "climb feedforward plus position feedback must share one upper speed limit");
  HeightFlight descent;
  descent.observe(.8, 0);
  require_height(descent.r.request("LAND", {}, descent.s, descent.t).accepted, "descent admission");
  descent.observe(.95, 0);
  descent.advance();
  require_height(near_height(descent.o.attitude.velocity_target, -.2),
                 "descent feedforward plus position feedback must share one lower speed limit");
}
// 近地下降目标按高度误差逐渐放慢，避免将固定下降速度一路带到地面。
static void descent_approach_speed() {
  HeightFlight q;
  q.observe(.30, 0);
  require_height(q.r.request("LAND", {}, q.s, q.t).accepted, "near-ground descent admission");
  q.observe(.29, 0);
  q.advance();
  require_height(
      near_height(q.o.attitude.velocity_target, -.145),
      "near the ground, feedforward must taper within the existing height-gain approach speed");
}
// 20/100/50/80 ms抖动均用实际dt推进轨迹和slew，不假定固定50 ms。
static void jittered_control_period() {
  HeightFlight q(.2);
  double height = q.o.attitude.target_height;
  for (double dt : {.02, .10, .05, .08}) {
    const double prior = -q.o.attitude.thrust[2];
    height += .3 * dt;
    q.observe(height, .3);
    q.advance(dt);
    require_height(near_height(q.o.attitude.target_height, height),
                   "reference must use actual control period");
    require_height(near_height(q.o.attitude.velocity_target, .3),
                   "feedforward must use actual control period");
    require_height(std::abs(-q.o.attitude.thrust[2] - prior) <= .4 * dt + 1e-9,
                   "slew must use actual control period");
  }
}
// 离地时已高于目标不能上调目标迁就，应制动再回到真实稳定HeightHold。
static void hold_and_overshoot() {
  HeightFlight q(.7, .6);
  require_height(near_height(q.o.attitude.target_height, .6), "never rebase above mission target");
  q.observe(.7, .1);
  q.advance();
  require_height(q.o.attitude.velocity_target < 0, "above-target liftoff must brake, not climb");
  q.observe(.6, 0);
  for (int i = 0; i < 8; ++i)
    q.advance();
  require_height(q.r.attitude_core().phase() == a::Phase::HeightHold, "normal settled height hold");
  require_height(near_height(q.o.attitude.velocity_target, 0),
                 "stationary hold has zero feedforward");
}
// 高度无效、reset、Kill仍停输出锁Safety，轨迹修正不能放宽原安全门。
static void safety_gates() {
  for (int failure = 0; failure < 3; ++failure) {
    HeightFlight q;
    if (failure == 0)
      q.s.attitude.z_valid = false;
    if (failure == 1)
      q.s.attitude.vz_reset++;
    if (failure == 2)
      q.s.attitude.kill = true;
    q.advance();
    require_height(!q.o.attitude.publish, "invalid feedback/reset/Kill must still revoke outputs");
    require_height(q.r.attitude_core().phase() == a::Phase::Safety,
                   "fault remains latched in SAFETY");
    require_height(!q.r.request("TAKEOFF", {1.}, q.s, q.t).accepted,
                   "no automatic rearm after fault");
  }
}
int main(int argc, char** argv) {
  const std::string which = argc > 1 ? argv[1] : "all";
  if (which == "all" || which == "liftoff")
    liftoff_reference();
  if (which == "all" || which == "climb")
    moving_climb();
  if (which == "all" || which == "descent")
    moving_descent();
  if (which == "all" || which == "endpoint")
    shortened_endpoint();
  if (which == "all" || which == "limits")
    combined_speed_limits();
  if (which == "all" || which == "approach")
    descent_approach_speed();
  if (which == "all" || which == "jitter")
    jittered_control_period();
  if (which == "all" || which == "hold")
    hold_and_overshoot();
  if (which == "all" || which == "safety")
    safety_gates();
  std::cout << "R632_HEIGHT_TRACKING_PASS " << which << '\n';
}
