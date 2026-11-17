// 垂向速度前馈回归：前馈与高度反馈合并受升降限速，无效前馈不能改变积分或slew状态。
// VerticalSample为NED位置/速度；控制输出velocity_target为上正m/s，推力为正归一化幅值。
#include "jikl_fc01_system/vertical_controller.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
namespace a = fc01::attitude;
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
  a::VerticalSample s{0, -1, 0, true, true, true, 0, 0};
  // 极端正负有限前馈受原0.3/-0.2 m/s边界且推力按0.4/s实际dt限斜率。
  for (double ff : {1000., -1000.}) {
    a::VerticalController ctl(config());
    ctl.reset(.475);
    const auto o = ctl.step(s, 1, .05, ff);
    assert(o.healthy && std::abs(o.velocity_target - (ff > 0 ? .3 : -.2)) < 1e-12);
    assert(o.thrust >= .455 - 1e-12 && o.thrust <= .495 + 1e-12);
  }
  // 非有限前馈拒绝并保留状态，随后正常步输出仍与未受污染控制器一致。
  for (double ff :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    a::VerticalController ctl(config());
    ctl.reset(.475);
    const auto o = ctl.step(s, 1, .05, ff);
    assert(!o.healthy && o.reason == "NONFINITE_VERTICAL_INPUT_OR_DT");
    assert(ctl.integral() == 0);
    const auto next = ctl.step(s, 1, .05);
    assert(next.healthy && std::abs(next.thrust - .475) < 1e-12);
  }
  // 省略前馈与显式0逐步完全相等，保护旧无前馈调用语义。
  a::VerticalController legacy(config()), explicit_zero(config());
  legacy.reset(.475);
  explicit_zero.reset(.475);
  for (int i = 0; i < 20; ++i) {
    const auto x = legacy.step(s, 1.2, .05), y = explicit_zero.step(s, 1.2, .05, 0.);
    assert(x.healthy && y.healthy && x.thrust == y.thrust &&
           x.velocity_target == y.velocity_target);
    assert(legacy.integral() == explicit_zero.integral());
  }
  std::cout << "VERTICAL_FEEDFORWARD_LIMITS_INVALID_INPUT_ZERO_COMPATIBILITY_PASS\n";
}
