// RC 起飞地面轴与辅助开关回归：低油门/中杆/辅助开关安全必须取自真实新鲜输入。
// 归一化throttle=-1为低油门，roll/pitch/yaw接近0；sample_stamp保存原始开关µs供因果门使用。
#include "jikl_fc01_system/authority_telemetry.hpp"
#include <cassert>
#include <cmath>
struct ManualRc {
  static constexpr unsigned SOURCE_RC = 1;
  std::uint64_t timestamp{}, timestamp_sample{};
  bool valid{true};
  unsigned data_source{1};
  bool sticks_moving{};
  float roll{}, pitch{}, yaw{}, throttle{-1};
};
struct RcSwitch {
  std::uint64_t timestamp{}, timestamp_sample{};
  unsigned switch_changes{};
  std::uint8_t mode_slot{1}, arm_switch{3}, return_switch{}, loiter_switch{}, offboard_switch{},
      kill_switch{3};
};
int main() {
  fc01::AuthorityTelemetry t;
  ManualRc m;
  RcSwitch s;
  // 先以两次raw推进形成ready，之后逐个攻击油门、中杆、NaN、越界并要求axes_safe=false。
  for (int i = 1; i <= 2; ++i) {
    m.timestamp = m.timestamp_sample = s.timestamp = s.timestamp_sample = 1000000 + i * 10000;
    t.manual(m, i * .05);
    t.switches(s, i * .05, 2.5);
  }
  auto f = t.facts(.11, 2.5, true, false, 1, 1);
  assert(f.rc_axes_safe && f.rc_aux_switches_safe && f.rc_throttle == -1);
  assert(f.switch_sample_stamp == s.timestamp_sample);
  for (int k = 0; k < 5; ++k) {
    m.throttle = -1;
    m.roll = m.pitch = m.yaw = 0;
    if (k == 0)
      m.throttle = 0;
    if (k == 1)
      m.roll = .3;
    if (k == 2)
      m.throttle = NAN;
    if (k == 3)
      m.yaw = .2;
    if (k == 4)
      m.throttle = -2;
    m.timestamp += 10000;
    m.timestamp_sample = m.timestamp;
    t.manual(m, .15 + k * .05);
    assert(!t.facts(.15 + k * .05, 2.5, true, false, 1, 1).rc_axes_safe);
  }
  // 返航开关ON撤销辅助开关安全，不能在意图不明确时取得下一次起飞授权。
  s.return_switch = 1;
  s.timestamp += 10000;
  s.timestamp_sample = s.timestamp;
  t.switches(s, .5, 2.5);
  assert(!t.facts(.5, 2.5, true, false, 1, 1).rc_aux_switches_safe);
}
