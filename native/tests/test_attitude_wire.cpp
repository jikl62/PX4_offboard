#if __has_include("jikl_fc01_system/attitude_wire.hpp")
#include "jikl_fc01_system/attitude_wire.hpp"
#include <cassert>
#include <iostream>
struct Mode {
  std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};
};
struct Target {
  std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};
  std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};
};
int main(){
  namespace a=fc01::attitude;
  auto plan=a::publisher_plan("ground");assert(!plan.mode&&!plan.trajectory&&!plan.attitude);
  plan=a::publisher_plan("attitude_altitude");assert(plan.mode&&!plan.trajectory&&plan.attitude);
  plan=a::publisher_plan("flight");assert(plan.mode&&plan.trajectory&&!plan.attitude);
  plan=a::publisher_plan("sim");assert(plan.mode&&plan.trajectory&&!plan.attitude);
  a::Output o;o.publish=true;o.q={std::sqrt(.5),0,0,std::sqrt(.5)};
  auto packet=a::wire<Mode,Target>(o,123);assert(packet);
  assert(packet->first.timestamp==123&&packet->second.timestamp==123);
  auto m=packet->first;assert(m.attitude&&!m.position&&!m.velocity&&!m.acceleration&&!m.body_rate&&!m.thrust_and_torque&&!m.direct_actuator);
  auto t=packet->second;assert(t.thrust_body[0]==0&&t.thrust_body[1]==0&&t.thrust_body[2]==0);
  assert(t.q_d[1]==0&&t.q_d[2]==0&&t.yaw_sp_move_rate==0&&!t.reset_integral&&!t.fw_control_yaw_wheel);
  o.thrust={0,0,-.4};packet=a::wire<Mode,Target>(o,124);assert(std::abs(packet->second.thrust_body[2]+.4)<1e-7);
  o.thrust[2]=NAN;assert((!a::wire<Mode,Target>(o,125)));
  o.thrust[2]=0;o.q[0]=INFINITY;assert((!a::wire<Mode,Target>(o,126)));
  o.q={1,0,0,0};o.publish=false;assert((!a::wire<Mode,Target>(o,127)));
  std::cout<<"PX4 attitude wire / profile publisher plan: PASS\n";
}
#else
#include <cassert>
int main(){assert(!"PX4 attitude wire adapter is absent");}
#endif
