// 垂向反馈守卫离线探针：逐行重放原始源戳、高度与两路速度，输出真实守卫结论。
// stdin 为 stamp(µs)、PX4 NED z(m)、vz/z_deriv(m/s)，正向均向下；不连接 ROS 或设备。
// stdout 保留健康、疑似、两类速度误差、累计分歧秒和原因，便于与历史输入逐样本比对。
// Offline stdin/stdout only; uses the production C++ guard, no ROS connection.
#include "jikl_fc01_system/vertical_feedback_guard.hpp"
#include <iomanip>
#include <iostream>
int main() {
  fc01::attitude::VerticalFeedbackConfig c;
  c.enabled = true;
  fc01::attitude::VerticalFeedbackGuard guard(c);
  std::uint64_t stamp;
  double z, vz, z_deriv;
  std::cout << std::setprecision(12);
  // 守卫按原样消费源戳与测量；0.5 s 为允许的最大采样间隔，不把新接收时刻填成原始测量戳。
  while (std::cin >> stamp >> z >> vz >> z_deriv) {
    const auto& e = guard.observe(stamp, z, vz, z_deriv, .5);
    std::cout << stamp << ',' << e.healthy << ',' << e.suspect << ',' << e.rate_error_mps << ','
              << e.position_rate_error_mps << ',' << e.disagreement_seconds << ',' << e.reason
              << '\n';
    // 数据健康性失败立即停止；诊断 suspect 与健康性判定仍由生产守卫原逻辑区分。
    if (!e.healthy)
      break;
  }
}
