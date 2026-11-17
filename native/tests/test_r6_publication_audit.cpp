// 本地发布审计回归：本轮提交标记、历史计数、最近目标及心跳间隔分别维护。
// 时间全部为单调s；thrust_z为FRD归一化负升力，此记录不能证明飞控已接收。
#include "jikl_fc01_system/publication_audit.hpp"
#include <cassert>
#include <iostream>
int main() {
  fc01::attitude::PublicationAudit a;
  a.begin(1.0);
  a.target_committed(1.0, -.42);
  a.mode_committed(1.01);
  assert(a.last_target_thrust_z == -.42);
  assert(a.target_count == 1 && a.mode_count == 1 && a.target_this_tick && a.mode_this_tick);
  // 目标已提交但心跳异常时保留新目标和独立计数，下一begin只清本轮位、不清历史。
  a.begin(1.05);
  a.target_committed(1.05, -.39);
  a.failed();
  assert(a.last_target_thrust_z == -.39);
  assert(a.target_count == 2 && a.mode_count == 1 && a.target_this_tick && !a.mode_this_tick &&
         a.exception_this_tick);
  a.begin(1.10);
  assert(!a.target_this_tick && !a.mode_this_tick && !a.exception_this_tick);
  assert(a.target_count == 2 && a.mode_count == 1);
  // 超过300 ms的本地心跳间隔应累计违例和最大gap，不把审计当续租。
  a.mode_committed(1.4);
  assert(a.max_mode_gap > .3 && a.mode_gap_violations == 1);
  a.begin(2);
  a.failed();
  assert(a.target_count == 2 && a.mode_count == 2);
  std::cout
      << "R6_PUBLICATION_AUDIT independent target/mode commits, partial exception and heartbeat gap PASS\n";
}
