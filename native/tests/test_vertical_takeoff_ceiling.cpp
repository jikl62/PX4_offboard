// 临时起飞推力上限回归：时间表限推、物理饱和与slew分别记录，抗积分饱和同时保留制动。
// still.z=-0.2对应地面参考上方0.2 m，推力为无量纲正幅值；配置仅用于纯垂向控制器测试。
#include "jikl_fc01_system/vertical_controller.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
namespace a = fc01::attitude;
static void check(bool ok, const char* message) {
  if (!ok) {
    std::cerr << "TAKEOFF_CEILING_FAIL: " << message << '\n';
    std::exit(1);
  }
}
static a::VerticalConfig config() {
  a::VerticalConfig c;
  c.hover = .475;
  c.minimum = .12;
  c.maximum = .75;
  c.slew = .4;
  c.height_gain = .5;
  c.velocity_gain = .15;
  c.integral_gain = .03;
  c.integral_limit = .08;
  c.max_height = 5;
  c.max_climb = .3;
  c.max_descent = .2;
  return c;
}
int main() {
  const a::VerticalSample still{0, -.2, 0, true, true, true, 0, 0};
  a::VerticalController limited(config());
  limited.reset(.2);
  // 不可达正误差被临时0.2上限限制时积分必须为0，但不能误报物理饱和；诊断项能重构原始推力。
  for (int i = 0; i < 80; ++i) {
    const auto o = limited.step(still, .5, .05, 0, .2);
    check(o.healthy && o.thrust <= .2 && !o.saturated && o.takeoff_limited,
          "scheduled clipping is not physical saturation");
    check(limited.integral() == 0,
          "the unreachable positive correction cannot wind up the integral");
    check(std::abs(o.raw_thrust - (.475 + o.proportional_thrust + o.integral_thrust)) < 1e-12,
          "diagnostic terms reconstruct raw request");
  }
  // 撤掉临时上限也不能重置slew状态，恢复动力按相同0.4/s逐步增加。
  const auto released = limited.step(still, .5, .05);
  check(released.healthy && !released.takeoff_limited && released.thrust > .2 &&
            released.thrust <= .22 + 1e-12,
        "removing the takeoff ceiling restores flight authority through the same slew state");
  // 上限绝不能变成制动下限；早期低于最小悬空推力的上限仍可往下制动。
  a::VerticalController brake(config());
  brake.reset(.5);
  const auto b = brake.step({0, -.5, -.8, true, true, true, 0, 0}, .5, .5, 0, .515);
  check(b.healthy && b.thrust < .475 && !b.takeoff_limited,
        "a ceiling must never become a thrust floor during braking");
  a::VerticalController early(config());
  const auto e = early.step(still, .5, .05, 0, .01);
  check(e.healthy && e.thrust <= .01, "initial schedule can be below the airborne minimum thrust");
  auto responsive = config();
  responsive.velocity_gain = 2.;
  a::VerticalController early_brake(responsive);
  early_brake.reset(.01);
  const auto eb = early_brake.step({0, -.5, -.5, true, true, true, 0, 0}, .5, .05, 0, .01);
  check(eb.healthy && eb.thrust < .01,
        "a sub-minimum takeoff ceiling must still allow downward braking");
  // 非法cap拒绝而不改积分或slew；下一正常步与未失败前一致。
  for (double cap : {-.1, 1.1, std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    a::VerticalController ctl(config());
    ctl.reset(.475);
    check(!ctl.step(still, .5, .05, 0, cap).healthy,
          "invalid ceiling must reject the control step");
    check(ctl.integral() == 0, "rejected ceiling cannot alter the integral");
    check(std::abs(ctl.step({0, -.2, 0, true, true, true, 0, 0}, .2, .05).thrust - .475) < 1e-12,
          "rejected ceiling cannot alter physical slew state");
  }
  // 不传cap与显式物理最大cap逐步一致，保护普通飞行PI行为。
  a::VerticalController implicit(config()), explicit_max(config());
  implicit.reset(.475);
  explicit_max.reset(.475);
  for (int i = 0; i < 40; ++i) {
    const auto x = implicit.step(still, .5, .05), y = explicit_max.step(still, .5, .05, 0, .75);
    check(x.thrust == y.thrust && implicit.integral() == explicit_max.integral(),
          "ordinary flight keeps the existing PI behavior");
  }
  std::cout << "VERTICAL_TAKEOFF_CEILING_ANTIWINDUP_BRAKING_COMPATIBILITY_PASS\n";
}
