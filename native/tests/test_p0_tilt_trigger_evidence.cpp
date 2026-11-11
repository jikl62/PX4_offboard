// 测量倾角与数据有效性策略回归：正常单位四元数的大倾角与坏姿态数据分开处理。
// 通过重命名main复用原Runtime测试夹具；这里只验证既有软件策略，不为20.2°/90°实际飞行授予安全结论。
// Valid measured tilt is ordinary feedback. It must not stop the autonomous
// takeoff stream; invalid attitude data still follows the existing health path.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main p0_attitude_runtime_fixture_main_not_called
#include "test_attitude_runtime.cpp"
#undef main
#pragma GCC diagnostic pop
#include <cassert>
#include <cmath>
#include <iostream>

int main(){
  // 按真实准备/Arm ACK/新armed顺序进入ArmedGround，未跳过权限或故意直接改Core状态。
  f::Runtime r(cfg(true));
  auto s=sample();
  double t=ready(r,s,true);
  assert(r.request("INTERACTIVE",{},s,t).accepted);
  assert(r.request("PING",{},s,t).accepted);
  assert(r.request("TAKEOFF",{.6},s,t).accepted);
  auto o=tick(r,s,t+=.05);
  assert(o.attitude.command==a::Command::Arm);
  r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.01);
  s.feedback.armed=true;
  o=tick(r,s,t+=.05);
  assert(r.attitude_core().phase()==a::Phase::ArmedGround);

  // Both the previous 20.2 degree sample and a 90 degree valid quaternion
  // continue through Core and keep the feedback heartbeat alive. This is a
  // software-policy test only; it does not authorize those poses for flight.
  // 20.2°与90°仍是合法测量，既有策略不以测量倾角单独停流；指令姿态边界另有测试。
  for(double degrees:{20.2,90.0}){
    const double half=degrees*3.14159265358979323846/360.;
    s.attitude.q={std::cos(half),std::sin(half),0,0};
    o=tick(r,s,t+=.05);
    assert(o.attitude.publish);
    assert(r.attitude_core().phase()!=a::Phase::Safety);
  }

  // Invalid quaternion data remains an independent safety condition.
  // NaN则是独立反馈无效，仍必须进入Safety并停发布，不能由前述策略豁免。
  s.attitude.q={NAN,0,0,0};
  o=tick(r,s,t+=.05);
  assert(!o.attitude.publish);
  assert(r.attitude_core().phase()==a::Phase::Safety);
  assert(r.attitude_core().reason()=="ATTITUDE_STALE_OR_INVALID");
  std::cout<<"FC01_NORMAL_TILT_AND_INVALID_ATTITUDE_PASS\n";
}
