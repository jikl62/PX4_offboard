// 合成反馈逐周期走人工RC和真实Core：不同NED原点须产生同一相对高度控制，目标来自配置。
// 不模拟真实动力学；偏置值和高度增量选二进制精确值，使浮点逐位比较有明确含义。
#include "vision_test_support.hpp"
#include "stage2_20261116_captured_config.hpp"
#include <iostream>
using namespace vision_test;
int main() {
  for (double target : {.75, 1.5}) {
    auto c = captured_config(); c.attitude.rc_target_height = target; c.validate();
    VisionFlight low(c), high(c);
    low.s.attitude.z = -16.; high.s.attitude.z = 32.;
    low.join(); high.join();
    need(low.r.attitude_core().reference_z() == -16. &&
             high.r.attitude_core().reference_z() == 32., "safe ground anchor not captured");
    const auto advance = [&](double height, double up) {
      low.s.attitude.z = -16. - height; high.s.attitude.z = 32. - height;
      low.s.attitude.vz = low.s.attitude.z_deriv = -up;
      high.s.attitude.vz = high.s.attitude.z_deriv = -up;
      low.tick(); high.tick();
      need(low.r.attitude_core().phase() == high.r.attitude_core().phase() &&
               low.o.attitude.height == high.o.attitude.height &&
               low.o.attitude.thrust == high.o.attitude.thrust &&
               low.o.attitude.q == high.o.attitude.q &&
               low.o.attitude.target_height == high.o.attitude.target_height,
           "NED origin changed relative-height control");
      need(low.r.attitude_core().reference_z() == -16. &&
               high.r.attitude_core().reference_z() == 32., "armed feedback rebased ground");
    };
    for (int i = 0; i < 100 && low.r.attitude_core().phase() != a::Phase::ThrustRamp; ++i)
      advance(0, 0);
    need(low.r.attitude_core().phase() == a::Phase::ThrustRamp, "ramp not reached");
    low.s.feedback.landed = high.s.feedback.landed = false;
    low.s.at_rest = high.s.at_rest = false;
    for (double height = .015625; height <= target; height += .015625)
      advance(height, .3125);
    for (int i = 0; i < 160 && low.r.attitude_core().phase() != a::Phase::HeightHold; ++i)
      advance(target, 0);
    need(low.r.attitude_core().phase() == a::Phase::HeightHold &&
             low.r.attitude_core().target_height_settled() &&
             low.o.attitude.target_height == target, "configured height did not settle");
    need(low.arm_commands == 0 && high.arm_commands == 0 &&
             low.mode_commands == 0 && high.mode_commands == 0, "automatic RC authority command");
  }
  std::cout << "HEIGHT_CONTRACT_PASS NED offsets -16/+32; targets .75/1.5; manual RC only\n";
}
