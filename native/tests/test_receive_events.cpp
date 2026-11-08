#include "jikl_fc01_system/receive_events.hpp"
#include "jikl_fc01_system/receive_schedule.hpp"
#include "authority_test_support.hpp"
#include "vision_test_support.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace f=fc01;
struct Status {
  std::uint64_t timestamp{1000000},armed_time{};
  std::uint8_t system_id{1},component_id{1},arming_state{1},latest_arming_reason{},nav_state{15},hil_state{};
  bool failsafe{},failsafe_and_user_took_over{},calibration_enabled{},rc_calibration_in_progress{};
  bool pre_flight_checks_pass{true},power_input_valid{true};
  std::uint32_t can_set_nav_states_mask{0xffffffff};
};
struct Land {std::uint64_t timestamp{1000000};bool landed{true},at_rest{true};};
struct Flags {
  std::uint64_t timestamp{1000000};
  bool manual_control_signal_lost{},offboard_control_signal_lost{},fd_critical_failure{};
};
struct Battery {
  std::uint64_t timestamp{1000000};bool connected{true};std::uint8_t warning{};double voltage_v{16};
};
struct Local {
  std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  double x{},y{},z{},vx{},vy{},vz{},z_deriv{},heading{};
  bool xy_valid{true},z_valid{true},v_xy_valid{true},v_z_valid{true},heading_good_for_control{true},dead_reckoning{};
  std::uint8_t xy_reset_counter{},z_reset_counter{},vxy_reset_counter{},vz_reset_counter{},heading_reset_counter{};
};
struct Attitude {
  std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  std::array<double,4> q{1,0,0,0};std::uint8_t quat_reset_counter{};
};

template<class Change>static void status_transition(Change change){
  Status before,after=before;change(after);
  assert(f::status_receive_event(before,after));
  assert(f::status_receive_event(after,before));
}
static void status_and_ground_transitions(){
  status_transition([](auto& s){s.system_id=2;});
  status_transition([](auto& s){s.component_id=2;});
  status_transition([](auto& s){s.arming_state=2;});
  status_transition([](auto& s){s.armed_time=42;});
  status_transition([](auto& s){s.latest_arming_reason=2;});
  status_transition([](auto& s){s.nav_state=14;});
  status_transition([](auto& s){s.failsafe=true;});
  status_transition([](auto& s){s.failsafe_and_user_took_over=true;});
  status_transition([](auto& s){s.calibration_enabled=true;});
  status_transition([](auto& s){s.rc_calibration_in_progress=true;});
  status_transition([](auto& s){s.hil_state=1;});
  status_transition([](auto& s){s.pre_flight_checks_pass=false;});
  status_transition([](auto& s){s.power_input_valid=false;});
  status_transition([](auto& s){s.can_set_nav_states_mask=0;});
  Status a,b;b.timestamp+=10000;assert(!f::status_receive_event(a,b));
  Land land,airborne=land;airborne.landed=false;
  assert(f::land_receive_event(land,airborne)&&f::land_receive_event(airborne,land));
  airborne=land;airborne.at_rest=false;assert(f::land_receive_event(land,airborne));
  airborne=land;airborne.timestamp+=10000;assert(!f::land_receive_event(land,airborne));
  Flags flags,changed=flags;changed.manual_control_signal_lost=true;
  assert(f::flags_receive_event(flags,changed));
  changed=flags;changed.offboard_control_signal_lost=true;assert(f::flags_receive_event(flags,changed));
  changed=flags;changed.fd_critical_failure=true;assert(f::flags_receive_event(flags,changed));
  changed=flags;changed.timestamp+=10000;assert(!f::flags_receive_event(flags,changed));
  Battery battery,other=battery;other.connected=false;assert(f::battery_receive_event(battery,other));
  other=battery;other.warning=1;assert(f::battery_receive_event(battery,other));
  other=battery;other.voltage_v-=.01;assert(f::battery_receive_event(battery,other));
  other=battery;other.voltage_v=std::numeric_limits<double>::quiet_NaN();
  assert(f::battery_receive_event(battery,other));
  other=battery;other.timestamp+=10000;assert(!f::battery_receive_event(battery,other));
}

template<class Change>static void authority_transition(Change change){
  f::AuthorityTelemetry::Facts before,after=before;change(after);
  assert(f::authority_receive_key(before)!=f::authority_receive_key(after));
}
static void authority_events_do_not_compare_numeric_stick_motion(){
  authority_transition([](auto& s){s.rc_link_ready=true;});
  authority_transition([](auto& s){s.rc_switches_ready=true;});
  authority_transition([](auto& s){s.kill=true;});
  authority_transition([](auto& s){s.manual_seen=true;});
  authority_transition([](auto& s){s.manual_valid=true;});
  authority_transition([](auto& s){s.rc_source_selected=true;});
  authority_transition([](auto& s){s.sticks_moving=true;});
  authority_transition([](auto& s){s.rc_source_clock_valid=true;});
  authority_transition([](auto& s){s.rc_switch_clock_valid=true;});
  authority_transition([](auto& s){s.data_source=1;});
  authority_transition([](auto& s){s.mode_slot=1;});
  authority_transition([](auto& s){s.arm_switch=1;});
  authority_transition([](auto& s){s.kill_switch=1;});
  authority_transition([](auto& s){s.switch_generation=1;});
  authority_transition([](auto& s){s.rc_axes_safe=true;});
  authority_transition([](auto& s){s.rc_aux_switches_safe=true;});
  f::AuthorityTelemetry::Facts before,after=before;
  before.rc_throttle=-1.;after.rc_throttle=-.99;after.switch_sample_stamp=123456;
  // Quantities used only for diagnostics do not manufacture a safety event.
  assert(f::authority_receive_key(before)==f::authority_receive_key(after));
}

template<class Change>static void local_transition(const f::attitude::Config& config,Change change){
  Local before,after=before;change(after);
  assert(f::local_receive_event(before,after,config));
}
static void reset_invalid_and_continuous_estimates(){
  f::attitude::Config config;config.z_jump_limit_m=1.;config.vz_jump_limit_mps=1.;
  config.ground_vz_tolerance=1.;
  local_transition(config,[](auto& s){s.xy_valid=false;});
  local_transition(config,[](auto& s){s.z_valid=false;});
  local_transition(config,[](auto& s){s.v_xy_valid=false;});
  local_transition(config,[](auto& s){s.v_z_valid=false;});
  local_transition(config,[](auto& s){s.heading_good_for_control=false;});
  local_transition(config,[](auto& s){s.dead_reckoning=true;});
  local_transition(config,[](auto& s){s.xy_reset_counter=1;});
  local_transition(config,[](auto& s){s.z_reset_counter=1;});
  local_transition(config,[](auto& s){s.vxy_reset_counter=1;});
  local_transition(config,[](auto& s){s.vz_reset_counter=1;});
  local_transition(config,[](auto& s){s.heading_reset_counter=1;});
  const std::array<double Local::*,8> measured{&Local::x,&Local::y,&Local::z,&Local::vx,
    &Local::vy,&Local::vz,&Local::z_deriv,&Local::heading};
  for(auto field:measured)for(double invalid:{std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}){
    local_transition(config,[&](auto& s){s.*field=invalid;});
  }
  local_transition(config,[](auto& s){s.z=1.001;});
  local_transition(config,[](auto& s){s.vz=-1.001;});
  Local before,after=before;after.z=1.;after.vz=-1.;
  assert(!f::local_receive_event(before,after,config));
  before=Local{};
  for(unsigned i=0;i<10;++i){
    after=before;after.timestamp+=50000;after.timestamp_sample+=50000;
    after.x+=.01;after.y-=.01;after.z-=.01;after.vx+=.005;after.vy-=.005;
    after.vz-=.005;after.z_deriv=after.vz;after.heading+=.01;
    assert(!f::local_receive_event(before,after,config));before=after;
  }
  // Preserve a suspect sample for the existing vertical freeze/dwell logic.
  config.feedback_guard.enabled=true;config.feedback_guard.rate_tolerance_mps=.2;
  local_transition(config,[](auto& s){s.vz=.3;s.z_deriv=0.;});
  before=Local{};after=before;after.vz=.1;
  assert(!f::local_receive_event(before,after,config));
  config.feedback_guard.enabled=false;after.vz=.3;
  assert(!f::local_receive_event(before,after,config));
}

static f::Config receive_flight_config(f::Config config){
  config.attitude.autonomous_failsafe_confirmed=true; // synthetic fixture only
  return config;
}
struct ReceiveFlight {
  f::Config c;f::Runtime runtime;f::Snapshot sample{authority_test::sample()};
  double now{};f::Output output;
  explicit ReceiveFlight(f::Config config=authority_test::config()):c(receive_flight_config(config)),runtime(c){
    sample.attitude.z_deriv=0.;
    authority_test::ready(runtime,sample,now,"AUTHORITY_AUTONOMOUS");
    const auto admitted=runtime.request(c.attitude.bench_arm_only_enabled?"ARM_ONLY":"TAKEOFF",
      c.attitude.bench_arm_only_enabled?std::vector<double>{}:std::vector<double>{1.},sample,now);
    if(!admitted.accepted)std::cerr<<"receive fixture admission: "<<admitted.message<<'\n';
    assert(admitted.accepted);
    advance();assert(output.attitude.command==f::attitude::Command::Arm);
    runtime.ack({runtime.pending(400).baseline+1,400,0,1,191,false},now+.01);
    sample.feedback.armed=true;advance();
    assert(core().phase()==(c.attitude.bench_arm_only_enabled?
      f::attitude::Phase::BenchArmedGround:f::attitude::Phase::ArmedGround));
  }
  const f::attitude::Core& core()const{return runtime.attitude_core();}
  void advance(){output=authority_test::tick(runtime,sample,now);}
  void ramp(){for(unsigned i=0;i<50&&core().phase()!=f::attitude::Phase::ThrustRamp;++i)advance();
    assert(core().phase()==f::attitude::Phase::ThrustRamp);}
  void observe(double height,double up_speed=0){
    sample.attitude.z=core().reference_z()-height;
    sample.attitude.vz=sample.attitude.z_deriv=-up_speed;
  }
  bool needs(double height,double up_speed=0,double derivative=NAN)const{
    return core().local_requires_control_step(sample.attitude.vertical_sample_stamp+50000,
      core().reference_z()-height,-up_speed,std::isnan(derivative)?-up_speed:derivative);
  }
};
static Local local_sample(const ReceiveFlight& flight){
  Local local;local.timestamp=flight.sample.attitude.local_stamp;
  local.timestamp_sample=flight.sample.attitude.vertical_sample_stamp;
  local.z=flight.sample.attitude.z;local.vz=flight.sample.attitude.vz;
  local.z_deriv=flight.sample.attitude.z_deriv;return local;
}

template<class Change,class Check>
static void drain_preserves_first_unsafe_sample(const ReceiveFlight& flight,unsigned expected,
    Change change,Check check){
  f::ReceivePassSchedule schedule;std::uint64_t callbacks=0,events=0;
  double now=1.;unsigned consumed=0;Local held=local_sample(flight);
  schedule.begin(now,.005,callbacks,events);
  while(schedule.next_pass(now,callbacks,events)){
    assert(consumed<3);const auto previous=held;change(held,consumed);
    held.timestamp+=50000;held.timestamp_sample+=50000;++callbacks;++consumed;
    if(f::local_receive_event(previous,held,flight.c.attitude)||
       flight.core().local_requires_control_step(held.timestamp_sample,held.z,held.vz,held.z_deriv))++events;
    now+=.0001;
  }
  assert(consumed==expected&&schedule.reason()==f::ReceivePassStop::Event);
  check(held); // Existing Core sees this sample before the returning callback.
}
static void absolute_flight_envelope_and_control_step_baseline(){
  auto config=authority_test::config();config.attitude.z_jump_limit_m=1.;
  config.attitude.vz_jump_limit_mps=1.;config.attitude.ground_vz_tolerance=2.;
  ReceiveFlight flight(config);flight.ramp();
  const double velocity_bound=config.attitude.vertical.max_climb+config.attitude.vertical.max_descent+
    2*config.attitude.vz_jump_limit_mps;
  for(double speed=.5;speed<velocity_bound;speed+=.5){flight.observe(0,-speed);flight.advance();}
  flight.observe(0,-(velocity_bound-.1));flight.advance();
  assert(flight.core().phase()==f::attitude::Phase::ThrustRamp);
  assert(flight.needs(0,-(velocity_bound+.1)));
  assert(!flight.needs(0,-velocity_bound));
  drain_preserves_first_unsafe_sample(flight,1,[&](auto& s,unsigned i){
    s.vz=s.z_deriv=i==0?velocity_bound+.1:velocity_bound-.1;
  },[&](const auto& s){assert(s.vz>velocity_bound);});
  ReceiveFlight height(config);height.ramp();
  for(double h=.5;h<config.attitude.vertical.max_height;h+=.5){height.observe(h);height.advance();}
  height.observe(config.attitude.vertical.max_height-.1);height.advance();
  assert(height.core().phase()==f::attitude::Phase::ThrustRamp);
  assert(height.needs(config.attitude.vertical.max_height+.1));
  drain_preserves_first_unsafe_sample(height,1,[&](auto& s,unsigned i){
    s.z=height.core().reference_z()-(i==0?config.attitude.vertical.max_height+.1:
      config.attitude.vertical.max_height-.1);
  },[&](const auto& s){assert(height.core().reference_z()-s.z>config.attitude.vertical.max_height);});
  ReceiveFlight lower(config);lower.ramp();lower.observe(-.2);lower.advance();
  assert(lower.needs(-.3));
  drain_preserves_first_unsafe_sample(lower,1,[&](auto& s,unsigned i){
    s.z=lower.core().reference_z()+(i==0?.3:.2);
  },[&](const auto& s){assert(lower.core().reference_z()-s.z< -config.attitude.height_lower_margin_m);});
  // Adjacent .6 changes cannot hide the middle 1.2 from Core's fixed 0 sample.
  ReceiveFlight jumps(config);jumps.ramp();jumps.observe(.2,.1);jumps.advance();
  jumps.observe(.2,0);jumps.advance(); // feedback remains latched; rise has been revoked
  const double consumed_height=.2;const std::array<double,3> values{.6,1.2,.6};
  drain_preserves_first_unsafe_sample(jumps,2,[&](auto& s,unsigned i){
    s.z=jumps.core().reference_z()-consumed_height-values[i];
  },[&](const auto& s){assert(jumps.core().reference_z()-s.z-consumed_height>config.attitude.z_jump_limit_m);});
  drain_preserves_first_unsafe_sample(jumps,2,[&](auto& s,unsigned i){s.vz=s.z_deriv=values[i];},
    [&](const auto& s){assert(s.vz>config.attitude.vz_jump_limit_mps);});
}
static void ground_guards_and_ramp_rise_survive_extra_passes(){
  ReceiveFlight ground;
  assert(!ground.needs(.01,.01));
  assert(ground.needs(ground.c.attitude.height_tolerance+.01));
  assert(ground.needs(0,ground.c.attitude.ground_vz_tolerance+.01));
  drain_preserves_first_unsafe_sample(ground,1,[&](auto& s,unsigned i){
    s.z=ground.core().reference_z()-(i==0?ground.c.attitude.height_tolerance+.01:0);
  },[&](const auto& s){assert(ground.core().reference_z()-s.z>ground.c.attitude.height_tolerance);});
  auto bench_config=authority_test::config();bench_config.attitude.bench_arm_only_enabled=true;
  bench_config.attitude.takeoff_enabled=false;ReceiveFlight bench(bench_config);
  assert(!bench.needs(-.3,.1)); // existing asymmetric bench drift grace
  assert(bench.needs(-.51));assert(bench.needs(.11));assert(bench.needs(0,.26));
  ReceiveFlight ramp;ramp.ramp();assert(!ramp.needs(.01));
  const double crossing=std::min(1.,ramp.c.attitude.liftoff_height_m)+.01;
  drain_preserves_first_unsafe_sample(ramp,1,[&](auto& s,unsigned i){
    s.z=ramp.core().reference_z()-(i==0?crossing:0);
  },[&](const auto& s){assert(ramp.core().reference_z()-s.z>=ramp.c.attitude.liftoff_height_m);});
  ramp.observe(crossing,.1);ramp.advance();
  assert(ramp.core().liftoff_evidence().local_samples>0);
  assert(!ramp.needs(crossing+.01,.1));
  assert(ramp.needs(crossing,.01)); // unconfirmed rise must clear on low speed
  drain_preserves_first_unsafe_sample(ramp,1,[&](auto& s,unsigned i){
    s.z=ramp.core().reference_z()-(i==0?.10:crossing);s.vz=s.z_deriv=-.1;
  },[&](const auto& s){assert(ramp.core().reference_z()-s.z<ramp.c.attitude.liftoff_height_m);});
  // An idle controller has no active reference to impose on a relocated frame.
  f::attitude::Core idle;
  assert(!idle.local_requires_control_step(50000,100.,0.,0.));
}
static void feedback_preview_preserves_position_window_and_real_state(){
  auto config=authority_test::config();config.attitude.feedback_guard.enabled=true;
  ReceiveFlight flight(config);flight.ramp();
  // Fill the actual monitor's window with ten coherent 50ms control samples.
  for(unsigned i=0;i<10;++i)flight.advance();
  const auto before=flight.core().vertical_feedback_evidence();
  assert(before.healthy&&!before.suspect);
  const double bad_height=.12; // .12m in .5s exceeds the unchanged .2m/s residual
  assert(flight.needs(bad_height));
  assert(!flight.needs(0));
  const auto after=flight.core().vertical_feedback_evidence();
  assert(before.sample_stamp==after.sample_stamp&&before.healthy==after.healthy&&
    before.position_window_checked==after.position_window_checked&&
    before.position_rate_error_mps==after.position_rate_error_mps&&before.reason==after.reason);
  drain_preserves_first_unsafe_sample(flight,1,[&](auto& s,unsigned i){
    s.z=flight.core().reference_z()-(i==0?bad_height:0);
  },[&](const auto& s){assert(flight.core().reference_z()-s.z>.1);});
  // The real step, rather than the observer, owns the fault transition.
  assert(flight.core().phase()==f::attitude::Phase::ThrustRamp);
  flight.observe(bad_height);flight.advance();
  assert(flight.core().phase()==f::attitude::Phase::Safety);
  assert(flight.core().reason()=="VERTICAL_POSITION_RATE_INCONSISTENT");
}
static void rc_anchor_and_touchdown_dwell_are_preserved(){
  vision_test::VisionFlight join;join.warm();join.manual_arm();
  assert(join.r.attitude_core().phase()==f::attitude::Phase::RcWaitNavigation);
  const auto& core=join.r.attitude_core();const double z=core.reference_z();
  assert(!core.local_requires_control_step(join.s.attitude.vertical_sample_stamp+50000,z,0,0));
  assert(core.local_requires_control_step(join.s.attitude.vertical_sample_stamp+50000,
    z-join.c.attitude.height_tolerance-.01,0,0));
  assert(core.local_requires_control_step(join.s.attitude.vertical_sample_stamp+50000,
    z,join.c.attitude.ground_vz_tolerance+.01,join.c.attitude.ground_vz_tolerance+.01));
  vision_test::VisionFlight descent;descent.reach_hold();
  assert(descent.r.request("LAND",{},descent.s,descent.t).accepted);descent.tick();
  const auto require_candidate=[&](double h,double v){return descent.r.attitude_core().local_requires_control_step(
    descent.s.attitude.vertical_sample_stamp+50000,descent.r.attitude_core().reference_z()-h,v,v);};
  assert(!require_candidate(descent.c.attitude.rc_target_height,-.1)); // high descent can catch up
  double height=descent.c.attitude.rc_target_height;
  while(height>0){height=std::max(0.,height-.01);descent.observe(height,height?-.2:0);descent.tick();}
  descent.s.feedback.landed=descent.s.at_rest=true;descent.observe(0);descent.tick();
  assert(descent.r.attitude_core().phase()==f::attitude::Phase::Descent);
  assert(require_candidate(descent.c.attitude.height_tolerance+.01,0));
  assert(require_candidate(0,descent.c.attitude.speed_tolerance+.01));
  assert(!require_candidate(0,0));
}

static void attitude_reset_invalid_and_real_tilt(){
  f::attitude::Config config;config.max_tilt_deg=10.;
  Attitude before,after=before;after.quat_reset_counter=1;
  assert(f::attitude_receive_event(before,after,config));
  after=before;after.q={0,0,0,0};assert(f::attitude_receive_event(before,after,config));
  after=before;after.q[0]=std::numeric_limits<double>::quiet_NaN();
  assert(f::attitude_receive_event(before,after,config));
  constexpr double radians=3.14159265358979323846/180.;
  after=before;after.q={std::cos(5.*radians/2),std::sin(5.*radians/2),0,0};
  assert(!f::attitude_receive_event(before,after,config));
  after.q={std::cos(15.*radians/2),std::sin(15.*radians/2),0,0};
  assert(!f::attitude_receive_event(before,after,config));
  after.q={std::cos(90.*radians/2),std::sin(90.*radians/2),0,0};
  assert(!f::attitude_receive_event(before,after,config));
  // Ordinary yaw evolution and steep valid poses are continuous feedback.
  for(unsigned i=0;i<10;++i){
    after=before;after.timestamp+=50000;after.timestamp_sample+=50000;
    const double half_yaw=double(i)*radians/2.;after.q={std::cos(half_yaw),0,0,std::sin(half_yaw)};
    assert(!f::attitude_receive_event(before,after,config));before=after;
  }
}

int main(){
  status_and_ground_transitions();
  authority_events_do_not_compare_numeric_stick_motion();
  reset_invalid_and_continuous_estimates();
  absolute_flight_envelope_and_control_step_baseline();
  ground_guards_and_ramp_rise_survive_extra_passes();
  feedback_preview_preserves_position_window_and_real_state();
  rc_anchor_and_touchdown_dwell_are_preserved();
  attitude_reset_invalid_and_real_tilt();
  std::cout<<"receive event guards preserve discrete/invalid observations and permit continuous catch-up: PASS\n";
}
