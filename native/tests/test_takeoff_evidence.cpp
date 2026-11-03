#include "ramp_guard_test_support.hpp"

// A detector callback may precede the local callback which crosses 0.12 m.
// Its !landed observation must remain available within the bounded join window.
static void detector_first_pairs_with_new_rising_local_samples(){
  GuardFlight q;q.ground_ramp(3.);
  q.s.feedback.landed=false;q.s.at_rest=false;
  q.observe(.10,.20);q.advance(.05,true);
  for(int i=1;i<=6;++i){q.observe(.10+.01*i,.20);q.advance(.05,i==6);}
  require_guard(q.r.attitude_core().phase()==a::Phase::Ascend,
    "two new detector observations must join independent rising local evidence despite callback order");
}

static void one_cached_local_sample_cannot_certify_liftoff(){
  GuardFlight q;q.ground_ramp(3.);
  q.s.feedback.landed=false;q.s.at_rest=false;
  q.observe(.20,.10);q.advance(.05,true);
  for(int i=0;i<6;++i)q.advance(.05,i==5,false);
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,
    "repeated detector messages cannot turn one cached local sample into sustained motion evidence");
}

static void fixed_height_with_repeated_velocity_is_not_observed_rise(){
  GuardFlight q;q.ground_ramp(3.);
  q.s.feedback.landed=false;q.s.at_rest=false;q.observe(.20,.10);
  for(int i=0;i<10;++i)q.advance(.05,i%5==0);
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,
    "a fixed height offset plus a repeated velocity cannot prove a physical rise");
}

static void local_first_and_delayed_detector_can_confirm_after_braking(){
  GuardFlight q;q.ground_ramp(3.);
  for(int i=0;i<8;++i){q.observe(.13+.01*i,.20);q.advance(.05,false);}
  q.observe(.22,0);q.s.feedback.landed=false;q.s.at_rest=false;q.advance();
  for(int i=0;i<6;++i)q.advance(.05,i==5);
  require_guard(q.r.attitude_core().phase()==a::Phase::Ascend,
    "independent sustained local rise remains valid while the late detector confirms a now hovering vehicle");
}

static void navigation_waits_for_liftoff_and_target_settling(){
  GuardFlight q;q.ground_ramp(3.);
  require_guard(!q.r.attitude_core().liftoff_confirmed()&&!q.r.attitude_core().navigation_allowed(),
    "ramp custody does not imply measured flight or navigation admission");
  q.s.feedback.landed=false;q.s.at_rest=false;
  for(int i=0;i<8;++i){q.observe(.13+.01*i,.20);q.advance();}
  require_guard(q.r.attitude_core().liftoff_confirmed()&&!q.r.attitude_core().target_height_settled()&&
    !q.r.attitude_core().navigation_allowed(),"liftoff alone does not admit navigation");
  q.observe(.5,0);q.advance();
  require_guard(!q.r.attitude_core().navigation_allowed(),"one target-height sample cannot replace settling dwell");
  for(int i=0;i<80;++i)q.advance();
  require_guard(q.r.attitude_core().target_height_settled()&&q.r.attitude_core().navigation_allowed(),
    "measured liftoff plus completed commanded trajectory and stable target dwell admit navigation");
  require_guard(q.r.request("LAND",{},q.s,q.t).accepted,"landing remains available");q.advance();
  require_guard(!q.r.attitude_core().navigation_allowed(),"landing immediately revokes navigation admission");
}

static void detector_contradiction_cannot_settle_or_prove_ground(){
  GuardFlight q;q.ground_ramp(3.);q.s.feedback.landed=false;q.s.at_rest=false;
  for(int i=0;i<8;++i){q.observe(.13+.01*i,.20);q.advance();}
  q.observe(.5,0);q.s.feedback.landed=true;
  for(int i=0;i<70;++i){q.advance();
    require_guard(!q.r.attitude_core().navigation_allowed()&&q.o.attitude.command!=a::Command::Disarm,
      "contradictory detector and high local position cannot admit navigation or infer safe ground");
  }
  require_guard(q.o.attitude.publish&&q.thrust()>0,"unresolved airborne custody retains controlled output");
}

static void transport_delay_cannot_stretch_a_short_source_sequence(){
  for(bool compressed_land:{false,true}){
    GuardFlight q;q.ground_ramp(3.);q.s.feedback.landed=false;q.s.at_rest=false;
    const auto first_land=q.s.land_stamp,first_local=q.s.attitude.local_stamp;
    for(int i=1;i<=8;++i){
      if(compressed_land)q.s.land_stamp=first_land+1000*i-50000;
      else {
        q.s.attitude.local_stamp=first_local+1000*i-50000;
        q.s.attitude.vertical_sample_stamp=q.s.attitude.local_stamp;
      }
      q.observe(.13+.01*i,.20);q.advance();
    }
    require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,
      "host delivery delay cannot turn a few source milliseconds into a sustained detector or local dwell");
  }
}

static void new_envelopes_cannot_relabel_one_raw_local_sample(){
  GuardFlight q;q.ground_ramp(3.);q.s.feedback.landed=false;q.s.at_rest=false;
  const auto raw_sample=q.s.attitude.vertical_sample_stamp;
  for(int i=0;i<8;++i){
    q.s.attitude.vertical_sample_stamp=raw_sample-50000;
    q.observe(.13+.01*i,.20);q.advance();
  }
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,
    "new publication timestamps cannot turn one repeated timestamp_sample into independent local measurements");
}

static void an_old_rise_cannot_join_an_unrelated_later_detector_episode(){
  GuardFlight q;q.ground_ramp(3.);
  for(int i=0;i<8;++i){q.observe(.13+.01*i,.20);q.advance();}
  q.observe(.22,0);
  for(int i=0;i<70;++i)q.advance();
  q.s.feedback.landed=false;q.s.at_rest=false;
  for(int i=0;i<6;++i)q.advance();
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,
    "fresh high-position samples cannot indefinitely renew old motion evidence for an unrelated detector episode");
}

int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="all"||which=="order")detector_first_pairs_with_new_rising_local_samples();
  if(which=="all"||which=="cached")one_cached_local_sample_cannot_certify_liftoff();
  if(which=="all"||which=="offset")fixed_height_with_repeated_velocity_is_not_observed_rise();
  if(which=="all"||which=="local-first")local_first_and_delayed_detector_can_confirm_after_braking();
  if(which=="all"||which=="settled")navigation_waits_for_liftoff_and_target_settling();
  if(which=="all"||which=="contradiction")detector_contradiction_cannot_settle_or_prove_ground();
  if(which=="all"||which=="compressed")transport_delay_cannot_stretch_a_short_source_sequence();
  if(which=="all"||which=="raw-replay")new_envelopes_cannot_relabel_one_raw_local_sample();
  if(which=="all"||which=="old-rise")an_old_rise_cannot_join_an_unrelated_later_detector_episode();
  std::cout<<"TAKEOFF_SOURCE_JOIN_PASS "<<which<<'\n';
}
