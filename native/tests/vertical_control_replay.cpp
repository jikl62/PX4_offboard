// 垂向历史输入离线比较器：把固定样本送入真实 Runtime，输出阶段、推力与垂向诊断。
// 只有合成水平姿态和测试权限；不连接 ROS/发布 sink，不据此重建真实飞行轨迹。
// stdin 每行：dt(s)、相对上正高度(m)、NED vz/z_deriv(m/s)、status/land/local/raw 源戳(µs)、landed、rest。
// Fixed historical vertical inputs through the real Runtime, with synthetic
// level attitude and pre-approved test authority. No ROS or publication sink.
// This is a controller comparison, never a reconstruction of actual motion.
#include "ramp_guard_test_support.hpp"
#include <iomanip>
int main() {
  auto c = guard_config();
  c.attitude.feedback_guard.enabled = true;
  GuardFlight q(.5, c);
  double dt = 0, height = 0, vz = 0, z_deriv = 0;
  std::uint64_t status = 0, land = 0, local = 0, raw = 0;
  int landed = 0, rest = 0;
  double elapsed = 0;
  std::cout << std::setprecision(12);
  // 保留每行独立源戳/接地证据；observe 要求上正速度，因此输入 NED vz 在此取负，z_deriv 保留原符号。
  while (std::cin >> dt >> height >> vz >> z_deriv >> status >> land >> local >> raw >> landed >>
         rest) {
    q.observe(height, -vz);
    q.s.attitude.z_deriv = z_deriv;
    q.s.status_stamp = status;
    q.s.land_stamp = land;
    q.s.attitude.local_stamp = local;
    q.s.attitude.vertical_sample_stamp = raw;
    q.s.feedback.landed = landed != 0;
    q.s.at_rest = rest != 0;
    require_guard(q.r.request("PING", {}, q.s, q.t).accepted, "historical replay console lease");
    // 控制台 PING 后按 dt 推进单调模拟时刻，再运行真实状态机；不通过工具直接改内部阶段。
    q.o = q.r.step(q.s, q.t += dt);
    elapsed += dt;
    const auto& o = q.o.attitude;
    std::cout << elapsed << ',' << a::name(q.r.attitude_core().phase()) << ',' << o.publish << ','
              << -o.thrust[2] << ',' << o.target_height << ',' << o.velocity_target;
// 候选诊断构建增加闭环/推力限制/P/I/raw/cap 列；普通构建保留六个零列维持 CSV 列合同。
#ifdef REFINEMENT_DIAGNOSTICS
    std::cout << ',' << o.vertical_closed_loop << ',' << o.takeoff_thrust_limited << ','
              << o.proportional_thrust << ',' << o.integral_thrust << ',' << o.raw_thrust << ','
              << o.thrust_ceiling;
#else
    std::cout << ",0,0,0,0,0,0";
#endif
    std::cout << ',' << q.r.attitude_core().reason() << '\n';
    // 首次进入 Safety 即停止回放，保留触发样本与原因，不伪造故障之后的正常输出。
    if (q.r.attitude_core().phase() == a::Phase::Safety)
      break;
  }
}
