// R6.3.1 first-400 / normal TAKEOFF external-state fallback.
// SYNTHETIC, NO HARDWARE. Reuses the original project Runtime fixture.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main base_attitude_runtime_main_not_called
#include "test_attitude_runtime.cpp"
#undef main
#pragma GCC diagnostic pop

struct ArmTrial {
  f::Runtime r{r63_arm_config()};
  f::Snapshot s{sample()};
  double t{};
  a::Output command{};
  static f::Config r63_arm_config(){
    auto c=cfg(true);
    c.attitude.vertical.hover=.475;
    c.attitude.bench_arm_only_enabled=false;
    c.attitude.rc_takeover_confirmed=true;
    c.validate();
    return c;
  }
  ArmTrial(){
    t=ready(r,s,true);
    r.require_attitude_sink_commit();
    s.attitude.source_clock_validated=true;
    s.attitude.rc_link_ready=true;
    s.attitude.rc_switches_ready=true;
    s.attitude.rc_arm_switch=3;
    assert(r.request("INTERACTIVE",{},s,t).accepted);
    assert(r.request("PING",{},s,t).accepted);
    assert(r.request("TAKEOFF",{.6},s,t).accepted);
    command=tick(r,s,t+=.05).attitude;
    assert(command.command==a::Command::Arm);
    assert(!r.pending(400).sent);
  }
  void commit(bool allowed=true){
    r.confirm_attitude_command_publication(a::Command::Arm,allowed,2000000,s,t);
    if(allowed){
      assert(r.pending(400).published&&r.pending(400).generation==1);
      assert(r.pending(400).wire_stamp==2000000);
    }
  }
  void new_external_armed(){
    s.feedback.armed=true;
    s.armed_time=400000;
    s.latest_arming_reason=4; // COMMAND_EXTERNAL
    s.status_stamp=2200000;
    s.land_stamp=2200000;
    s.feedback.offboard=true;
    s.nav_state=14;
  }
  a::Output step(){
    assert(r.request("PING",{},s,t).accepted);
    return tick(r,s,t+=.05).attitude;
  }
};
static void approved_first_400_fallback(){
  ArmTrial h;
  h.commit();
  // A quick ACK can have ambiguous timestamps, and it is not trusted.
  const auto old_stamp=h.r.pending(400).wire_stamp+10;
  h.r.ack({old_stamp,400,0,1,191,false},h.t+.01);
  assert(!h.r.pending(400).accepted);
  h.new_external_armed();
  auto o=h.step();
  assert(o.publish&&o.command==a::Command::None&&o.thrust[2]==0);
  assert(h.r.attitude_core().phase()==a::Phase::ArmedGround);
  assert(h.r.attitude_core().reason()=="TAKEOFF_ARM_NEW_EXTERNAL_STATE_EVIDENCED_ACK_UNATTRIBUTED");
  assert(h.r.pending(400).state_verified&&!h.r.pending(400).accepted);
  assert(h.r.pending(400).stage==f::CommandStage::StateOnlyTerminal);
  std::cout<<"R631_FRESH_EXTERNAL_ARM_FIRST_400_NO_MATCHED_ACK_PASS\n";
}
static void denial_matrix(){
  for(int mutation=0;mutation<15;++mutation){
    ArmTrial h;
    if(mutation!=0)h.commit(mutation!=1);
    h.new_external_armed();
    switch(mutation){
      case 0:break; // NO FINAL SINK
      case 1:break; // SINK DENIED
      case 2:h.s.latest_arming_reason=2;break; // RC arm
      case 3:h.s.armed_time=0;break; // no new event
      // tick() advances source time by 50 ms. Keep the observed status
      // strictly behind the wire causal floor after that advance as well.
      case 4:h.s.status_stamp=h.r.pending(400).causal_floor-50001;break;
      case 5:h.s.attitude.rc_arm_switch=1;break;
      case 6:h.s.attitude.rc_link_ready=false;break;
      case 7:h.s.attitude.source_clock_validated=false;break;
      case 8:h.s.feedback.exclusive=false;break;
      case 9:h.s.feedback.offboard=false;h.s.nav_state=15;break;
      case 10:h.s.feedback.landed=false;break;
      case 11:h.s.status_fresh=false;break;
      case 12:h.s.land_fresh=false;break;
      case 13:h.s.armed_time=h.r.pending(400).armed_time_at_wire;break;
      case 14:h.t+=5.1;break; // timeout
    }
    auto o=tick(h.r,h.s,h.t+=.05).attitude;
    if(h.r.attitude_core().phase()==a::Phase::ArmedGround)std::cerr<<"UNEXPECTED_PASS mutation="<<mutation<<" p="<<h.r.phase_name()<<" reason="<<h.r.attitude_core().reason()<<"\n";
    assert(h.r.attitude_core().phase()!=a::Phase::ArmedGround);
    assert(h.r.attitude_core().phase()!=a::Phase::ThrustRamp);
    assert(o.thrust[2]==0);
  }
  std::cout<<"R631_15_NEGATIVE_GATE_CASES_PASS\n";
}
static void rejected_ack_never_authorizes_ramp(){
  ArmTrial h;h.commit();
  auto stamp=h.r.pending(400).causal_floor+1000;
  h.r.ack({stamp,400,2,1,191,false},h.t+.02);
  assert(h.r.pending(400).rejected);
  h.new_external_armed();
  auto o=h.step();
  assert(o.publish&&o.thrust[2]==0);
  assert(h.r.attitude_core().phase()!=a::Phase::ArmedGround);
  std::cout<<"R631_REJECTED_ACK_STILL_GUARDED_RECOVERY_PASS\n";
}
static void existing_matched_ack_unchanged(){
  f::Runtime r(ArmTrial::r63_arm_config());auto s=sample();double t=ready(r,s,true);
  assert(r.request("INTERACTIVE",{},s,t).accepted);
  assert(r.request("TAKEOFF",{.6},s,t).accepted);
  auto o=tick(r,s,t+=.05).attitude;assert(o.command==a::Command::Arm);
  const auto stamp=r.pending(400).baseline;
  r.ack({stamp+1,400,0,1,191,false},t+.01);
  s.feedback.armed=true;
  o=tick(r,s,t+=.05).attitude;
  assert(o.publish&&r.attitude_core().phase()==a::Phase::ArmedGround);
  std::cout<<"R631_MATCHED_ACK_COMPATIBILITY_PASS\n";
}
int main(){
  approved_first_400_fallback();
  denial_matrix();
  rejected_ack_never_authorizes_ramp();
  existing_matched_ack_unchanged();
  std::cout<<"R631_NORMAL_TAKEOFF_ACK_PROOF_REGRESSION_PASS\n";
}
