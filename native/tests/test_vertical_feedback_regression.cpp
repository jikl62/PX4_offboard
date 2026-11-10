// Exercise diagnostic rate disagreement and retained hard faults through the
// current Runtime. Synthetic feedback only; no ROS/PX4 or hardware outputs.
#include "ramp_guard_test_support.hpp"
static f::Config feedback_config(){
  auto c=guard_config();c.attitude.feedback_guard.enabled=true;return c;
}
static void sustained_rate_difference_preserves_control_and_integral(){
  GuardFlight q(.5,feedback_config());q.s.attitude.z_deriv=0;q.ground_ramp();
  q.s.feedback.landed=false;q.s.at_rest=false;
  for(int i=1;i<=50;++i){q.observe(i*.01,.2);q.advance();}
  q.observe(.5,0);for(int i=0;i<55;++i)q.advance();
  require_guard(q.r.attitude_core().phase()==a::Phase::HeightHold,"rate diagnostic starts in genuine height hold");
  const double initial_integral=q.o.attitude.integral_thrust;
  q.s.attitude.vz=-.3;q.s.attitude.z_deriv=0;
  bool warning_observed=false;
  for(int i=0;i<24;++i){
    q.advance();const auto& e=q.r.attitude_core().vertical_feedback_evidence();
    require_guard(e.healthy&&e.suspect&&e.reason.empty(),
      "a sustained vz/z_deriv difference cannot create a hard fault");
    warning_observed=warning_observed||e.rate_warning;
    require_guard(q.r.attitude_core().phase()==a::Phase::HeightHold&&q.o.attitude.publish&&
        q.o.attitude.vertical_closed_loop&&q.thrust()>0,
      "diagnostic rate disagreement retains ordinary closed-loop publication");
  }
  require_guard(warning_observed,"a sustained rate difference remains visible as a warning");
  require_guard(q.o.attitude.integral_thrust<initial_integral-.005,
    "rate disagreement alone cannot freeze normal integral adaptation");
  q.s.attitude.vz=0;q.advance();const auto& recovered=q.r.attitude_core().vertical_feedback_evidence();
  require_guard(recovered.healthy&&!recovered.suspect&&!recovered.rate_warning&&
      recovered.disagreement_seconds==0&&recovered.disagreement_samples==0&&q.o.attitude.publish,
    "a new agreeing sample clears diagnostics while control remains active");
}
static void position_contradiction_still_stops_control(){
  GuardFlight q(.5,feedback_config());q.s.attitude.vz=q.s.attitude.z_deriv=-.4;
  // Both rates agree, but their integrated displacement contradicts fixed z.
  for(int i=0;i<16&&q.r.attitude_core().active();++i)q.advance();
  require_guard(q.r.attitude_core().phase()==a::Phase::Safety&&
      q.r.attitude_core().reason()=="VERTICAL_POSITION_RATE_INCONSISTENT",
    "a genuine position/derivative contradiction must still stop autonomous control");
  require_guard(!q.o.attitude.publish,"hard feedback faults suppress publication rather than sending zero thrust");
  require_guard(!q.r.request("TAKEOFF",{.5},q.s,q.t).accepted,"hard feedback faults cannot admit another takeoff");
}
static void raw_samples_not_envelopes(){
  GuardFlight q(.5,feedback_config());q.s.attitude.z_deriv=0;q.s.attitude.vz=-.4;
  for(int i=0;i<6;++i)q.advance();
  const auto before=q.r.attitude_core().vertical_feedback_evidence();
  for(int i=0;i<6;++i){
    q.s.attitude.local_stamp+=50000; // new envelope, repeated raw estimate
    q.advance(.05,true,false);
    require_guard(q.r.attitude_core().active()&&q.o.attitude.publish,
      "repeated raw estimates cannot stop control for a diagnostic rate difference");
  }
  const auto& after=q.r.attitude_core().vertical_feedback_evidence();
  require_guard(after.disagreement_seconds==before.disagreement_seconds&&
      after.disagreement_samples==before.disagreement_samples&&!after.rate_warning,
    "new envelopes cannot manufacture raw disagreement duration, counts or warnings");
}
static void guarded_lifecycle(){
  GuardFlight q(.5,feedback_config());q.s.attitude.z_deriv=0;q.ground_ramp();
  q.s.feedback.landed=false;q.s.at_rest=false;
  for(int i=1;i<=50;++i){q.observe(i*.01,.2);q.advance();}
  q.observe(.5,0);
  for(int i=0;i<55;++i)q.advance();
  require_guard(q.r.attitude_core().phase()==a::Phase::HeightHold,"enabled guard must allow normal height hold");
  require_guard(q.r.request("LAND",{},q.s,q.t).accepted,"guarded landing admitted");
  for(int i=49;i>=0;--i){q.observe(i*.01,i?-.2:0);q.advance();}
  q.s.feedback.landed=q.s.at_rest=true;q.s.stationary_seconds=3;
  bool disarmed=false;
  for(int i=0;i<250&&q.r.attitude_core().active();++i){
    q.observe(0,0);q.advance();
    if(q.o.attitude.command==a::Command::Disarm){
      q.r.ack({q.r.pending(400).baseline+1,400,0,1,191,false},q.t+.001);
      q.s.feedback.armed=false;disarmed=true;
    }
    if(q.o.attitude.command==a::Command::ReleaseOffboard){
      q.r.ack({q.r.pending(176).baseline+1,176,0,1,191,false},q.t+.001);
      q.s.feedback.offboard=false;q.s.nav_state=15;q.s.stabilized_selectable=true;
    }
  }
  require_guard(disarmed&&!q.s.feedback.armed,"enabled guard must allow ordinary disarm");
  require_guard(q.r.attitude_core().phase()!=a::Phase::Safety,"normal guarded lifecycle must not fault");
  require_guard(!q.r.attitude_core().active(),"normal guarded lifecycle must finish release");
}
static void bad_raw_inputs_fail_closed(){
  for(int k=0;k<4;++k){
    GuardFlight q(.5,feedback_config());q.s.attitude.z_deriv=0;q.advance();
    if(k==0)q.s.attitude.vertical_sample_stamp=0;
    else if(k==1)--q.s.attitude.vertical_sample_stamp;
    else if(k==2)q.s.attitude.vertical_sample_stamp+=600000;
    else q.s.attitude.z_deriv=NAN;
    q.advance(.05,true,false);
    const char* reason=k==0||k==3?"VERTICAL_FEEDBACK_INPUT_INVALID":
      k==1?"VERTICAL_FEEDBACK_TIME_REGRESSION":"VERTICAL_FEEDBACK_SAMPLE_GAP";
    require_guard(q.r.attitude_core().phase()==a::Phase::Safety&&!q.o.attitude.publish&&
        q.r.attitude_core().reason()==reason,
      "invalid, regressed and gapped raw feedback retains the specific hard fault");
  }
}
int main(int argc,char** argv){
  if(argc>1&&std::string(argv[1])=="raw-clock"){raw_samples_not_envelopes();return 0;}
  sustained_rate_difference_preserves_control_and_integral();position_contradiction_still_stops_control();
  raw_samples_not_envelopes();guarded_lifecycle();bad_raw_inputs_fail_closed();
  std::cout<<"VERTICAL_FEEDBACK_REGRESSION_PASS\n";
}
