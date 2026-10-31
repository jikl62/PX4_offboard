// Offline positive properties. The unchanged C harness reaches the REAL
// Runtime and final publication boundary; none of these numbers calibrate FC01.
#define main C_S10_preserved_driver_not_called
#include "c_s10_independent/C_S10_SAFETY_ATTACKS.cpp"
#undef main

static void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
static void airborne(Harness& h){
  h.ramp();h.s.feedback.landed=false;h.s.at_rest=false;
  h.s.attitude.z=1.97;h.s.attitude.vz=-.02;
  require(!h.s.attitude.xy_valid&&!h.s.attitude.vxy_valid,"fixture needs FC01-like invalid XY/VXY");
}
static void continuous_tick(Harness& h){
  const auto targets=h.attitude_sinks,modes=h.mode_sinks;
  const auto o=h.tick();
  require(o.publish,"healthy Armed+Offboard lost output at recovery budget");
  require(h.attitude_sinks==targets+1&&h.mode_sinks==modes+1,"actual final sink missing");
  require(o.command==a::Command::None,"airborne recovery planned a new command without proof");
}
static void long_healthy_custody(int interruption){
  Harness h;airborne(h);
  if(interruption==1)h.r.begin_shutdown(h.s,h.t);
  if(interruption==2){h.connected=false;h.r.disconnect(h.s,h.t);}
  double deadline=-1,previous=0;bool first=true;
  double previous_target=INFINITY;
  for(int i=0;i<12000;++i){ // 600 simulated seconds, not wall-clock sleep.
    if(i%97==0)require(h.r.request("LAND",{},h.s,h.t).accepted,"repeat LAND refused");
    const auto targets=h.attitude_sinks,modes=h.mode_sinks;
    const auto o=h.tick();
    require(o.publish,"healthy Armed+Offboard lost output at recovery budget");
    require(h.attitude_sinks==targets+1&&h.mode_sinks==modes+1,"actual final sink missing");
    require(o.command==a::Command::None,"unverified airborne Disarm/21/176");
    const double thrust=-o.thrust[2];
    require(std::isfinite(thrust)&&thrust>=0&&thrust<=config().attitude.vertical.maximum,"thrust outside original bounds");
    if(!first)require(std::abs(thrust-previous)<=config().attitude.vertical.slew*.05+1e-9,"slew bypass at timeout");
    first=false;previous=thrust;
    const auto& core=h.r.attitude_core();
    if(core.recovery_deadline()>=0){
      if(deadline<0)deadline=core.recovery_deadline();
      require(core.recovery_deadline()==deadline,"recovery deadline renewed");
      require(o.target_height<=previous_target+1e-9,"recovery resumed ascent");
      previous_target=o.target_height;
      if(h.t>=deadline){
        require(core.reason()=="RECOVERY_BUDGET_EXHAUSTED_GUARDED_CUSTODY_ACTIVE_HANDOFF_UNVERIFIED","overdue recovery not disclosed");
        require(!core.px4_land_mode_observed(),"no-XY recovery fabricated PX4 Land handoff");
      }
    }
  }
  require(deadline>=0&&h.t>deadline+500,"observation did not cover full budget and long tail");
  require(h.arms==1&&h.disarms==0&&h.mode_commands==1,"repeat airborne command");
  std::cout<<"R6.1_600S_CUSTODY interruption="<<interruption
    <<" healthy Z/VZ, invalid XY/VXY, 1Hz land; dual sinks+limits+fixed deadline PASS\n";
}
static void reach_overdue(Harness& h){
  airborne(h);
  for(int i=0;i<900;++i)continuous_tick(h);
  require(h.r.attitude_core().recovery_deadline()>0&&h.t>h.r.attitude_core().recovery_deadline(),"not past recovery deadline");
}
static void overdue_does_not_bypass_failures(){
  for(int which=0;which<13;++which){
    Harness h;reach_overdue(h);
    if(which==0)h.s.local_fresh=false;
    if(which==1)h.s.attitude.z_valid=false;
    if(which==2)h.s.attitude.vz_valid=false;
    if(which==3)h.s.attitude.q={0,0,0,0};
    if(which==4)++h.s.attitude.quat_reset;
    if(which==5)++h.s.attitude.z_reset;
    if(which==6)++h.s.attitude.vz_reset;
    if(which==7)h.s.feedback.exclusive=false;
    if(which==8)h.s.attitude.kill=true;
    if(which==9)h.s.feedback.failsafe=true;
    if(which==10)h.s.time_regressed=true;
    if(which==11)h.s.battery_warning=1;
    if(which==12)h.t+=.5;
    const auto targets=h.attitude_sinks,modes=h.mode_sinks;
    const auto o=h.tick();
    require(!o.publish&&o.command==a::Command::None,"overdue custody bypassed an original fatal gate");
    require(h.attitude_sinks==targets&&h.mode_sinks==modes&&h.disarms==0,"unsafe failure reached a sink");
    require(h.r.attitude_core().phase()==a::Phase::Safety,"fatal failure still labelled controlled descent");
  }
  std::cout<<"R6.1_POST_BUDGET 13 original fatal gates fail closed; no routine Disarm PASS\n";
}
static void overdue_final_boundary_is_unchanged(){
  Harness h;reach_overdue(h);const auto o=h.tick();
  const auto valid=h.r.publication_context(h.s,h.t,true,true);
  for(int which=0;which<4;++which){auto c=valid;
    if(which==0)c.config.arm_enabled=false;
    if(which==1)c.control_fresh=false;
    if(which==2)c.feedback.kill=true;
    if(which==3)c.feedback.z_valid=false;
    int target=0,mode=0;
    const auto d=a::publish_attitude_at_boundary<Mode,Target>(c,o,999,
      [&](const Target&){++target;},[&](const Mode&){++mode;});
    require(!d.allowed&&target==0&&mode==0,"budget exhaustion bypassed final sink policy");
  }
  std::cout<<"R6.1_POST_BUDGET final policy still blocks gate/stall/Kill/Z failure PASS\n";
}
static void overdue_mode_exit_respected(){
  Harness h;reach_overdue(h);h.s.feedback.offboard=false;h.s.nav_state=15;
  const auto o=h.tick();require(!o.publish&&o.command==a::Command::None,"Offboard output continued after actual fresh mode exit");
  const auto fb=h.r.publication_context(h.s,h.t,true,true).feedback;
  require(std::string(h.r.attitude_core().recovery_responsibility(fb))==
    "PX4_FAILSAFE_OBSERVED_BUT_CONTROL_UNVERIFIED","mode exit fabricated working flight task");
  std::cout<<"R6.1_POST_BUDGET fresh PX4 mode exit respected, physical task unverified PASS\n";
}
static void overdue_power_removal_is_not_labelled_descent(){
  Harness h;reach_overdue(h);h.s.attitude.z=2.;h.s.attitude.vz=0.;
  h.s.feedback.landed=true;h.s.at_rest=false;
  a::Output o;
  for(int i=0;i<200;++i)o=h.tick();
  require(h.r.attitude_core().phase()==a::Phase::Landed&&o.publish&&o.thrust[2]==0,
    "fixture did not reach guarded power removal while at_rest is unconfirmed");
  require(h.r.attitude_core().reason()=="RECOVERY_BUDGET_EXHAUSTED_GUARDED_CUSTODY_ACTIVE_HANDOFF_UNVERIFIED",
    "overdue reason falsely claims descent during landed power removal");
  require(h.disarms==0,"unconfirmed at_rest allowed Disarm");
  std::cout<<"R6.1_POST_BUDGET power removal reports guarded custody, no false descent or Disarm PASS\n";
}
static void overdue_ground_proof_can_finish(){
  Harness h;reach_overdue(h);
  h.s.attitude.z=2.;h.s.attitude.vz=0.;
  // One detector pulse cannot permit Disarm even after the deadline.
  h.s.feedback.landed=true;h.s.at_rest=true;h.tick();
  require(h.disarms==0,"overdue recovery accepted one landed pulse");
  h.s.feedback.landed=false;h.s.at_rest=false;h.tick();
  h.s.feedback.landed=true;h.s.at_rest=true;
  for(int i=0;i<300&&h.disarms==0;++i){const auto o=h.tick();require(o.publish,"late ground proof lost custody");}
  require(h.disarms==1,"fresh 1Hz ground dwell did not permit one guarded Disarm");
  require(h.r.pending(400).ack_ambiguous,"late landing lost R4 Command400 ambiguity");
  h.s.feedback.armed=false;
  for(int i=0;i<60&&h.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)h.tick();
  require(h.r.attitude_core().phase()==a::Phase::SafeDisarmed,"independent safe ground status did not complete Disarm");
  require(h.r.pending(400).state_verified&&!h.r.pending(400).accepted,"state-only Disarm falsely attributed ACK");
  auto o=h.tick();require(o.command==a::Command::ReleaseOffboard,"late landing missing existing ground release");
  h.r.ack({h.r.pending(176).baseline+3000,176,0,1,191,false},h.t+.001);
  h.s.feedback.offboard=false;h.s.nav_state=15;
  o=h.tick();require(!o.publish&&h.r.attitude_core().phase()==a::Phase::Idle,"ground release did not finish");
  require(h.disarms==1&&h.arms==1,"late ground path repeated Command400");
  std::cout<<"R6.1_POST_BUDGET touchdown with NEW 1Hz samples -> one state-only Disarm -> existing ground release PASS\n";
}
int main(){
  long_healthy_custody(0);long_healthy_custody(1);long_healthy_custody(2);
  overdue_does_not_bypass_failures();overdue_final_boundary_is_unchanged();
  overdue_mode_exit_respected();overdue_ground_proof_can_finish();
  overdue_power_removal_is_not_labelled_descent();
}
