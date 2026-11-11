// 五个历史独立反例的正向回归：保留原始文件字节，另以断言验证修复后的安全属性。
// 覆盖地面取消、斜坡超时、land dwell、零推力姿态旁路、接管请求与真实交接，以及最终sink证据。
// C's five independent counterexamples, converted to assertions. Fully offline.
// Original C file remains unchanged in tests/c_independent_repro.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <cassert>
#include <iostream>
namespace a=fc01::attitude;
namespace f=fc01;
struct Mode {std::uint64_t timestamp{};bool attitude{};};
struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};
  std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static f::Config cfg(){auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;return c;}
// 已解锁但未静止时LAND只进入减推地面恢复；见新静止证据后才提交一次Disarm。
static void c01_cancel_ground(){
  f::Runtime r(cfg());auto s=authority_test::sample();double t=0;
  authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
  r.require_attitude_sink_commit(); // This test explicitly exercises the real final sink.
  s.at_rest=false;s.feedback.landed=true;
  assert(r.request("LAND",{},s,t).accepted);
  auto o=authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::PoweredGround&&o.attitude.publish);
  assert(r.attitude_core().fault_class()==a::FaultClass::GroundAbort);
  for(int i=0;i<22;++i){o=authority_test::tick(r,s,t);
    assert(o.attitude.publish&&o.attitude.command!=a::Command::Disarm);
  }
  s.at_rest=true;
  for(int i=0;i<35&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i)
    o=authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::RequestDisarm);
  assert(o.attitude.command==a::Command::Disarm);
  int sink_calls=0;
  auto boundary=r.publication_context(s,t,true,true);
  assert(a::publish_disarm_at_boundary(boundary,[&]{++sink_calls;}).allowed&&sink_calls==1);
  r.confirm_attitude_command_publication(a::Command::Disarm,true,880001,s,t);
  // Neither a stale ACK nor ACK without newer status completes Disarm.
  const auto watermark=r.pending(400).baseline;
  r.ack({watermark,400,0,1,191,false},t+.001);
  o=authority_test::tick(r,s,t);assert(r.attitude_core().phase()==a::Phase::RequestDisarm);
  r.ack({watermark+1,400,0,1,191,false},t+.001);
  s.feedback.armed=false;authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::SafeDisarmed);
  std::cout<<"C01 recovered powered ground; ordered Disarm ACK + new status: GREEN\n";
}
// 无离地反馈时斜坡最终超时并按slew减推，保持心跳；不把时间到当成已经离地。
static void c02_ramp_timeout(){
  f::Runtime r(cfg());auto s=authority_test::sample();double t=0;
  authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
  s.at_rest=false;
  double max_thrust=0,previous=0;bool saw_power=false,handled=false;
  for(int i=0;i<260;++i){
    auto o=authority_test::tick(r,s,t);
    double power=-o.attitude.thrust[2];
    if(r.attitude_core().phase()==a::Phase::ThrustRamp||r.attitude_core().phase()==a::Phase::PoweredGround){
      assert(o.attitude.publish);
      assert(std::abs(power-previous)<=cfg().attitude.vertical.slew*.05+1e-7);
      previous=power;
    }
    max_thrust=std::max(max_thrust,power);saw_power|=power>1e-6;
    if(r.attitude_core().phase()==a::Phase::PoweredGround){handled=true;break;}
  }
  assert(saw_power&&max_thrust>.1&&handled);
  for(int i=0;i<30;++i){auto o=authority_test::tick(r,s,t);
    assert(o.attitude.publish&&o.attitude.command!=a::Command::Disarm);
    assert(-o.attitude.thrust[2]<=previous+1e-8);
    previous=-o.attitude.thrust[2];
  }
  assert(previous<1e-9);
  s.at_rest=true;
  a::Output last;
  for(int i=0;i<35&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i)
    last=authority_test::tick(r,s,t).attitude;
  assert(r.attitude_core().phase()==a::Phase::RequestDisarm&&last.command==a::Command::Disarm);
  std::cout<<"C02 timeout ramps motor safely, maintains heartbeat, then requests Disarm: GREEN\n";
}
// 冻结/回退/短丢帧不能凭控制tick累计land dwell，超过允许gap必须从新源样本重计。
static void c03_timestamp_dwell(){
  f::Runtime r(cfg());auto s=authority_test::sample();double t=0;
  authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
  const auto frozen=s.land_stamp;
  for(int i=0;i<9;++i){t+=.05;s.status_stamp+=1000;r.request("PING",{},s,t);
    const auto o=r.step(s,t);assert(o.attitude.publish);
  }
  assert(s.land_stamp==frozen&&r.attitude_core().phase()==a::Phase::ArmedGround);
  // Stamp regression resets dwell; it must NOT add duration to the old sample.
  --s.land_stamp;t+=.05;s.status_stamp+=1000;r.request("PING",{},s,t);r.step(s,t);
  assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  s.land_stamp=frozen+1000;
  t+=.05;s.status_stamp+=1000;r.request("PING",{},s,t);r.step(s,t);
  assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  // Short dropped frame is permitted as a gap but cannot itself complete dwell.
  t+=.1;s.status_stamp+=1000;r.request("PING",{},s,t);r.step(s,t);
  assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  s.land_stamp+=1000;t+=.05;s.status_stamp+=1000;r.request("PING",{},s,t);r.step(s,t);
  assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  // Beyond the valid detector observation gap the dwell restarts.
  for(int i=0;i<11;++i){t+=.05;s.status_stamp+=1000;r.request("PING",{},s,t);r.step(s,t);}
  s.land_stamp+=1000;t+=.05;s.status_stamp+=1000;r.request("PING",{},s,t);
  auto o=r.step(s,t);assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  (void)o;
  for(int i=0;i<6;++i){s.land_stamp+=1000;t+=.05;s.status_stamp+=1000;r.request("PING",{},s,t);r.step(s,t);}
  assert(r.attitude_core().phase()==a::Phase::ThrustRamp);
  std::cout<<"C03 frozen, regressed and dropped land-detector frames: GREEN\n";
}
// 即使推力为零，恶意90°姿态也不能绕过最终水平/航向参考检查。
static void c04_zero_thrust_sink(){
  f::Runtime r(cfg());auto s=authority_test::sample();auto c=r.publication_context(s,0,true,true);
  c.authority={fc01::authority::Mode::AutonomousNoRc,fc01::authority::Mode::AutonomousNoRc,true,false};
  c.flight_started=c.reference_ready=c.arm_confirmed=c.ground_confirmed=true;
  c.reference_z=s.attitude.z;c.yaw_reference=0;c.touchdown_baseline=100;
  c.feedback.armed=c.feedback.offboard=c.feedback.battery_ready=true;
  c.feedback.landed=c.feedback.at_rest=true;c.feedback.land_stamp=1000;
  a::Output out;out.publish=true;out.q={std::sqrt(.5),std::sqrt(.5),0,0};out.thrust={0,0,0};
  for(auto p:{a::Phase::RequestArm,a::Phase::ArmedGround,a::Phase::RequestDisarm,a::Phase::ReleaseOffboard,
               a::Phase::Prestream,a::Phase::ThrustRamp}){
    auto b=c;b.phase=p;if(p==a::Phase::ReleaseOffboard)b.feedback.armed=false;
    if(p==a::Phase::Prestream)b.feedback.armed=false;
    int targets=0,modes=0;
    const auto decision=a::publish_attitude_at_boundary<Mode,Target>(b,out,123,
      [&](const Target&){++targets;},[&](const Mode&){++modes;});
    assert(!decision.allowed&&decision.reason==a::BoundaryReason::CommandedAttitude);
    assert(targets==0&&modes==0);
  }
  std::cout<<"C04 zero-thrust attacked template sink for all applicable phases: GREEN\n";
}
// 接管请求与PX4已经离开Offboard是两个事实；等待交接期间继续原安全输出。
static void c05_takeover(){
  f::Runtime r(cfg());auto s=authority_test::sample();double t=0;
  authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
  for(int i=0;i<9;++i)authority_test::tick(r,s,t);
  assert(r.request("AUTHORITY_TAKEOVER",{},s,t).accepted);
  const auto o=authority_test::tick(r,s,t);
  assert(o.attitude.publish&&s.feedback.armed&&s.feedback.offboard);
  assert(!r.attitude_core().authority_state().takeover_latched);
  assert(r.attitude_core().operator_release_requested());
  s.feedback.offboard=false;s.nav_state=15;
  const auto observed=authority_test::tick(r,s,t);
  assert(!observed.attitude.publish&&r.attitude_core().authority_state().takeover_latched);
  std::cout<<"C05 request != completed PX4 handoff; no immediate heartbeat gap: GREEN\n";
}
// 着陆/减推后的上锁仍要新land样本，接触弹跳立即回下降，不能沿用旧接地证据。
static void touchdown_landed_fresh_samples(){
  f::Runtime r(cfg());auto s=authority_test::sample();double t=0;
  authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
  // Simulate loss of ground contact before a confirmed liftoff. Request LAND.
  s.feedback.landed=false;s.at_rest=false;s.attitude.z=1.85;s.attitude.vz=.02;
  assert(r.request("LAND",{},s,t).accepted);
  assert(r.attitude_core().phase()==a::Phase::Descent);
  for(int i=0;i<4;++i)authority_test::tick(r,s,t);
  s.attitude.z=2;s.attitude.vz=0;s.feedback.landed=true;
  const auto frozen=s.land_stamp;
  for(int i=0;i<10;++i){t+=.05;s.status_stamp+=1000;r.request("PING",{},s,t);
    const auto o=r.step(s,t);assert(o.attitude.publish);
    assert(r.attitude_core().phase()==a::Phase::Descent);}
  assert(s.land_stamp==frozen);
  for(int i=0;i<6&&r.attitude_core().phase()!=a::Phase::Landed;++i)authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::Landed);
  // Even after touchdown, frozen post-baseline detector data cannot validate Disarm.
  s.at_rest=true;
  for(int i=0;i<12;++i){t+=.05;s.status_stamp+=1000;r.request("PING",{},s,t);
    auto out=r.step(s,t);assert(out.attitude.publish&&out.attitude.command!=a::Command::Disarm);
    assert(r.attitude_core().phase()==a::Phase::Landed);}
  // Bounce immediately removes ground proof and must preserve setpoint output.
  s.feedback.landed=s.at_rest=false;
  auto out=authority_test::tick(r,s,t);
  assert(out.attitude.publish&&r.attitude_core().phase()==a::Phase::Descent);
  s.feedback.landed=s.at_rest=true;
  for(int i=0;i<5;++i)authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::Landed);
  for(int i=0;i<32&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i)
    authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::RequestDisarm);
  std::cout<<"Touchdown/Landed frozen stamps and bounce never bypass new-sample Disarm: GREEN\n";
}
// 计划命令不是线上提交；最终sink拒绝后迟到ACK也不能把未发Arm变为接受。
static void request_vs_sink_publication(){
  f::Runtime r(cfg());r.require_attitude_sink_commit();auto s=authority_test::sample();double t=0;
  assert(r.request("INTERACTIVE",{},s,t).accepted);
  assert(r.request("AUTHORITY_AUTONOMOUS",{},s,t).accepted);
  assert(r.request("ATTITUDE_READY",{},s,t).accepted);r.step(s,t);
  a::Output out;
  for(int i=0;i<40;++i){out=authority_test::tick(r,s,t).attitude;
    if(out.command==a::Command::Offboard)break;}
  assert(out.command==a::Command::Offboard);
  assert(r.pending(176).requested&&!r.pending(176).sent&&!r.pending(176).published);
  auto old=r.pending(176).baseline;
  r.ack({old+1,176,0,1,191,false},t+.001);
  assert(!r.pending(176).accepted); // ACK cannot validate unpublished request.
  r.confirm_attitude_command_publication(a::Command::Offboard,true,987654321,s,t);
  assert(r.pending(176).published&&r.pending(176).sent);
  r.ack({old+2,176,0,1,191,false},t+.001);
  s.feedback.offboard=true;s.nav_state=14;authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::OffboardReady);
  assert(r.request("TAKEOFF",{1},s,t).accepted);
  out=authority_test::tick(r,s,t).attitude;assert(out.command==a::Command::Arm);
  auto ctx=r.publication_context(s,t,true,true);
  ctx.feedback.landed=false; // final sink sees changed telemetry after core.
  int arm_sent=0;
  assert(!a::publish_arm_at_boundary(ctx,[&]{++arm_sent;}).allowed&&arm_sent==0);
  r.confirm_attitude_command_publication(a::Command::Arm,false,0,s,t);
  assert(r.pending(400).requested&&!r.pending(400).published&&r.pending(400).sink_denied);
  assert(!r.pending(400).sent&&r.pending(400).rejected);
  r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.001);
  assert(!r.pending(400).accepted);
  std::cout<<"Final sink rejection != wire publication; late ACK cannot authorize: GREEN\n";
}
int main(){c01_cancel_ground();c02_ramp_timeout();c03_timestamp_dwell();c04_zero_thrust_sink();c05_takeover();touchdown_landed_fresh_samples();request_vs_sink_publication();}
