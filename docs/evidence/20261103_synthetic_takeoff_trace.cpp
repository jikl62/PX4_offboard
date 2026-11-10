/*
Synthetic Runtime/local-final-sink trace recorder, 2026-11-03.
Run from the repository root (no ROS, PX4 connection or device required):

g++ -std=c++17 -Wall -Wextra -Werror -pedantic -I native/tests -I native/src/jikl_fc01_system/include docs/evidence/20261103_synthetic_takeoff_trace.cpp -o /tmp/20261103_synthetic_takeoff_trace
/tmp/20261103_synthetic_takeoff_trace > /tmp/20261103_synthetic_takeoff_trace.raw.json

Compare every emitted runtime field with the packaged evidence JSON:

python3 - <<'PY_CHECK'
import json
from pathlib import Path
actual = json.loads(Path('/tmp/20261103_synthetic_takeoff_trace.raw.json').read_text())
evidence = json.loads(Path('docs/evidence/20261103_synthetic_takeoff_trace.json').read_text())
for key, value in actual.items():
    assert evidence[key] == value, key
print('SYNTHETIC_TRACE_RUNTIME_PAYLOAD_MATCH')
PY_CHECK

The accompanying evidence JSON adds scope, units, effective-fixture warnings,
source hashes and milestones derived from these actual runtime events. Its
runtime fields can be refreshed with this executable's emitted JSON; metadata
and milestones must describe the same run. No numerical results are supplied
by this comment or by the packaging step.

IMPORTANT: config() is vision_test::config(), using authority_test fixture
parameters, NOT native/config/attitude_altitude.yaml. No baseline test limit,
flight gate, detector threshold or timeout is relaxed. Heights, velocities,
landed/armed/Offboard states and vision inputs are prescribed feedback, NOT
a dynamics model or evidence of physical flight, DDS delivery or PX4 adoption.
*/
#include "vision_test_support.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace vision_test;
static std::string n(double value,bool valid=true){
  if(!valid||!std::isfinite(value))return "null";
  std::ostringstream out;out<<std::setprecision(12)<<value;return out.str();
}
static const char* command_name(a::Command command){
  switch(command){case a::Command::None:return "NONE";case a::Command::Arm:return "ARM";
  case a::Command::Offboard:return "OFFBOARD";case a::Command::Disarm:return "DISARM";
  case a::Command::ReleaseOffboard:return "RELEASE_OFFBOARD";case a::Command::Px4Land:return "PX4_LAND";}
  return "UNKNOWN";
}
static v::Input visual(double now,double height){
  v::Input in;in.ros_now_ns=1000000000000ULL+std::uint64_t(std::llround(now*1e9));
  in.state_publishers=in.command_publishers=1;
  auto& s=in.state;s.sequence=std::uint64_t(std::llround(now*1e6))+1;s.stamp_ns=in.ros_now_ns;s.received_at=now;
  s.frame_id="fc01_vision_ned";s.body_frame_id="base_link_frd";s.source_id="synthetic-trace-vision";
  s.initialized=s.tracking=s.calibration_valid=true;s.valid_fields=v::Position|v::Velocity|v::Attitude|v::Quality;
  s.position={0,0,-height};s.velocity={0,0,0};s.attitude={1,0,0,0};s.quality=1;
  auto& c=in.command;c.sequence=s.sequence;c.stamp_ns=in.ros_now_ns;c.received_at=now;
  c.source_id="synthetic-trace-command";c.frame_id=s.frame_id;c.valid=true;c.mode=v::Mode::Cruise;
  c.position_valid=true;c.position={.2,0,-height};c.sticks_valid=true;c.sticks={0,0,0,0};return in;
}
struct Trace:VisionFlight{
  std::string id;std::vector<std::string> rows;std::string previous;
  unsigned steps{},attempt{1},disarm_sink_commits{},denied_publish_attempts{};
  unsigned published_ticks_before_navigation{},visual_ticks_before_navigation{};
  explicit Trace(std::string name):VisionFlight(config()),id(std::move(name)){record("INITIAL",false);}
  std::string key()const{
    const auto& core=r.attitude_core();const auto& p=r.pending(400);std::ostringstream out;
    out<<r.phase_name()<<'|'<<core.takeoff_stage()<<'|'<<core.liftoff_confirmed()<<'|'<<core.navigation_allowed()
      <<'|'<<o.attitude.vertical_closed_loop<<'|'<<o.attitude.takeoff_thrust_limited<<'|'<<o.attitude.slew_limited
      <<'|'<<o.attitude.visual_motion<<'|'<<o.attitude.publish<<'|'<<a::nonzero(o.attitude.thrust)<<'|'<<p.generation<<'|'<<int(p.stage);return out.str();
  }
  void record(const std::string& event,bool output_current=true){
    const auto& core=r.attitude_core();const auto& p=r.pending(400);const auto& e=core.liftoff_evidence();
    const auto& out=o.attitude;const bool output=output_current&&steps>0;
    const bool closed=output&&out.publish&&out.vertical_closed_loop;
    std::ostringstream row;row<<std::boolalpha<<std::setprecision(12);
    row<<"{\"event\":"<<f::quote(event)<<",\"runtime_step_id\":"<<steps<<",\"attempt\":"<<attempt
       <<",\"t_s\":"<<n(t)<<",\"phase\":"<<f::quote(r.phase_name())<<",\"takeoff_stage\":"<<f::quote(core.takeoff_stage())
       <<",\"reason\":"<<f::quote(core.reason())<<",\"reference_z_ned_m\":"<<n(core.reference_z(),core.reference_ready())
       <<",\"h_m\":"<<n(core.reference_z()-s.attitude.z,core.reference_ready())
       <<",\"target_h_m\":"<<n(out.target_height,output&&core.flight_started()&&out.publish)
       <<",\"velocity_target_up_mps\":"<<n(out.velocity_target,closed)
       <<",\"thrust_body_z\":"<<n(out.thrust[2],output&&out.publish)
       <<",\"raw_thrust_magnitude\":"<<n(out.raw_thrust,closed)
       <<",\"p_thrust\":"<<n(out.proportional_thrust,closed)<<",\"i_thrust\":"<<n(out.integral_thrust,closed)
       <<",\"thrust_ceiling\":"<<n(out.thrust_ceiling,closed)
       <<",\"vertical_closed_loop\":"<<(output?(out.vertical_closed_loop?"true":"false"):"null")
       <<",\"slew_limited\":"<<(output?(out.slew_limited?"true":"false"):"null")<<",\"takeoff_thrust_limited\":"<<(output?(out.takeoff_thrust_limited?"true":"false"):"null")
       <<",\"saturated\":"<<(output?(out.saturated?"true":"false"):"null")<<",\"liftoff_confirmed\":"<<core.liftoff_confirmed()
       <<",\"target_height_settled\":"<<core.target_height_settled()<<",\"navigation_allowed\":"<<core.navigation_allowed()
       <<",\"visual_motion\":"<<(output?(out.visual_motion?"true":"false"):"null")<<",\"rc_manual_arm_proven\":"<<core.rc_manual_arm_proven()
       <<",\"rc_prestream_commits\":"<<core.rc_prestream_commits()
       <<",\"liftoff_evidence\":{\"detector_samples\":"<<e.detector_samples<<",\"local_samples\":"<<e.local_samples
       <<",\"detector_confirmed\":"<<e.detector_confirmed<<",\"rise_confirmed\":"<<e.rise_confirmed<<"}"
       <<",\"input_feedback\":{\"z_ned_m\":"<<n(s.attitude.z)<<",\"vz_ned_mps\":"<<n(s.attitude.vz)
       <<",\"z_deriv_ned_mps\":"<<n(s.attitude.z_deriv)<<",\"armed\":"<<s.feedback.armed<<",\"landed\":"<<s.feedback.landed
       <<",\"at_rest\":"<<s.at_rest<<",\"offboard\":"<<s.feedback.offboard<<",\"nav_state\":"<<s.nav_state
       <<",\"rc_mode_slot\":"<<unsigned(s.attitude.rc_mode_slot)<<",\"rc_arm_switch\":"<<unsigned(s.attitude.rc_arm_switch)
       <<",\"armed_time\":"<<n(double(s.armed_time),s.armed_time>0)<<",\"latest_arming_reason\":"<<(s.latest_arming_reason?std::to_string(unsigned(s.latest_arming_reason)):"null")
       <<",\"status_stamp_us\":"<<s.status_stamp<<",\"land_stamp_us\":"<<s.land_stamp<<",\"local_sample_stamp_us\":"<<s.attitude.vertical_sample_stamp<<"}"
       <<",\"local_publication\":{\"output_from_this_step\":"<<output
       <<",\"requested\":"<<(output&&out.publish)<<",\"boundary_allowed\":"<<(output? (last_decision.allowed?"true":"false"):"null")
       <<",\"boundary_reason\":"<<(output?f::quote(a::name(last_decision.reason)):"null")
       <<",\"target_committed_this_step\":"<<(output&&target_committed)<<",\"heartbeat_committed_this_step\":"<<(output&&heartbeat_committed)
       <<",\"wire_thrust_body_z\":"<<n(last_target.thrust_body[2],output&&target_committed)
       <<",\"wire_q_d\":";
    if(output&&target_committed)row<<'['<<n(last_target.q_d[0])<<','<<n(last_target.q_d[1])<<','<<n(last_target.q_d[2])<<','<<n(last_target.q_d[3])<<']';
    else row<<"null";
    row<<",\"published_tick_count\":"<<published<<",\"target_commit_count\":"<<target_commits
       <<",\"heartbeat_commit_count\":"<<heartbeat_commits<<",\"disarm_command_sink_commits\":"<<disarm_sink_commits<<"}"
       <<",\"command_output\":"<<(output?f::quote(command_name(out.command)):"null")
       <<",\"command400\":{\"generation\":"<<p.generation<<",\"kind\":"<<(p.generation?f::quote(f::command400_name(p.command400_kind)):"null")
       <<",\"stage\":"<<(p.generation?f::quote(f::command_stage_name(p.stage)):"null")
       <<",\"local_commit\":"<<p.published<<",\"wire_stamp_us\":"<<n(double(p.wire_stamp),p.wire_stamp>0)
       <<",\"ack_result\":"<<(p.result>=0?std::to_string(p.result):"null")
       <<",\"ack_accepted\":"<<p.accepted<<",\"ack_ambiguous\":"<<p.ack_ambiguous<<",\"state_verified\":"<<p.state_verified
       <<",\"causal_floor_us\":"<<n(double(p.causal_floor),p.causal_floor>0)
       <<",\"history_count\":"<<r.command400_history().size()<<"}}";
    rows.push_back(row.str());
  }
  void tick(const std::string& event=""){
    const double next=t+.05;
    const double height=r.attitude_core().reference_ready()?r.attitude_core().reference_z()-s.attitude.z:0;
    set_visual(visual(next,height));VisionFlight::tick();++steps;
    if(o.attitude.publish&&!last_decision.allowed)++denied_publish_attempts;
    if(!r.attitude_core().navigation_allowed()){
      published_ticks_before_navigation+=o.attitude.publish;
      visual_ticks_before_navigation+=o.attitude.visual_motion;
    }
    const auto current=key();
    if(!event.empty()||current!=previous)record(event.empty()?"STATE_OR_CONTROL_TRANSITION":event);
    previous=current;
  }
  void warm(){for(int i=0;i<75;++i)tick();need(r.attitude_core().phase()==a::Phase::RcWaitArm,"READY_MISSING");record("RC_READY");}
  void manual_arm(std::uint64_t armed_time){
    s.attitude.rc_arm_switch=1;++s.attitude.operator_switch_generation;tick("MANUAL_RC_ARM_SWITCH_INPUT");
    s.feedback.armed=true;s.armed_time=armed_time;s.latest_arming_reason=2;s.at_rest=false;tick("MANUAL_RC_ARM_STATUS_INPUT");
  }
  void navigation(){
    s.attitude.rc_mode_slot=c.attitude.rc_navigation_mode_slot;++s.attitude.operator_switch_generation;tick("RC_NAVIGATION_SWITCH_INPUT");
    s.feedback.offboard=true;s.nav_state=14;tick("SYNTHETIC_OFFBOARD_STATUS_INPUT");
    need(r.attitude_core().phase()==a::Phase::ArmedGround,"ARMED_GROUND_MISSING");
  }
  void observe(double height,double up_speed){
    s.attitude.z=r.attitude_core().reference_z()-height;s.attitude.vz=s.attitude.z_deriv=-up_speed;
  }
  void reach_hold(){
    for(int i=0;i<100&&r.attitude_core().phase()!=a::Phase::ThrustRamp;++i)tick();
    need(r.attitude_core().phase()==a::Phase::ThrustRamp,"RAMP_MISSING");
    // Keep the baseline fixture's nominal ramp wait; zero height is not liftoff.
    for(int i=0;i<8;++i)tick();
    need(!r.attitude_core().liftoff_confirmed()&&!r.attitude_core().navigation_allowed(),"TIME_CANNOT_PROVE_LIFTOFF");
    record("RAMP_WITHOUT_LIFTOFF_FEEDBACK");
    double h=0;
    for(int i=1;i<=150;++i){h=std::min(1.5,i*.01);observe(h,.2);s.feedback.landed=s.at_rest=false;
      tick(i==1?"AIRBORNE_DETECTOR_AND_RISING_LOCAL_FEEDBACK_INPUT":i%25==0?"ASCENT_HEIGHT_SAMPLE":"");
      need(last_decision.allowed,"ASCENT_SINK_DENIED");
      need(!o.attitude.visual_motion,"VISION_BEFORE_TARGET_SETTLED");
    }
    observe(1.5,0);tick("TARGET_HEIGHT_ZERO_SPEED_INPUT");
    for(int i=0;i<160&&r.attitude_core().phase()!=a::Phase::HeightHold;++i)tick();
    need(r.attitude_core().phase()==a::Phase::HeightHold,"HOLD_MISSING");
    tick("FIRST_VISUAL_PLAN_AFTER_TARGET_SETTLED");
    need(o.attitude.visual_motion&&target_committed&&heartbeat_committed,"VISUAL_FINAL_COMMITS_MISSING");
  }
  void land(unsigned generation){
    need(r.request("LAND",{},s,t).accepted,"LAND_REFUSED");record("LAND_REQUEST_ACCEPTED",false);
    for(int i=149;i>=0;--i){const double h=std::max(0.,i*.01);observe(h,-.2);
      tick(i%25==0?"DESCENT_HEIGHT_SAMPLE":"");
      need(last_decision.allowed,"DESCENT_SINK_DENIED");
    }
    observe(0,0);s.feedback.landed=s.at_rest=true;tick("TOUCHDOWN_DETECTOR_AND_ZERO_SPEED_INPUT");
    bool disarm=false;
    for(int i=0;i<100&&!disarm;++i){tick();if(o.attitude.command==a::Command::Disarm){
      const auto decision=a::publish_disarm_at_boundary(r.publication_context(s,t,true,true),[&]{++disarm_sink_commits;});
      need(decision.allowed,"DISARM_FINAL_SINK_DENIED");
      r.confirm_attitude_command_publication(a::Command::Disarm,true,s.status_stamp+1,s,t);
      record("DISARM_FINAL_LOCAL_SINK_COMMIT");
      need(r.pending(400).generation==generation,"BAD_400_GENERATION");
      if(generation==1){r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.001);record("FIRST_DISARM_SYNTHETIC_ACK_SUPPLIED");}
      else{r.ack({s.status_stamp+200001,400,0,1,191,false},t+.201);record("REUSED_DISARM_UNATTRIBUTED_ACK_OBSERVED");
        need(r.pending(400).ack_ambiguous&&!r.pending(400).accepted&&!r.pending(400).state_verified,"REUSED_ACK_FABRICATED_SUCCESS");}
      disarm=true;
    }}
    need(disarm,"DISARM_MISSING");s.feedback.armed=false;s.latest_arming_reason=4;tick("NEW_DISARMED_STATUS_INPUT");
    for(int i=0;i<10&&r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)tick();
    need(r.attitude_core().phase()==a::Phase::SafeDisarmed&&r.pending(400).state_verified,"SAFE_DISARM_MISSING");
    record("SAFE_DISARMED_INDEPENDENT_STATUS_PROVEN");
  }
  void finish_checks(){
    need(arm_commands==0&&mode_commands==0,"AUTOMATIC_ARM_OR_MODE_COMMAND");
    need(denied_publish_attempts==0,"NOMINAL_PUBLICATION_DENIAL");
    need(target_commits==heartbeat_commits&&target_commits==published,"DUAL_SINK_COUNTS_DISAGREE");
    need(visual_ticks_before_navigation==0,"EARLY_VISUAL_MOTION");
  }
  void print()const{
    std::cout<<"{\"scenario\":"<<f::quote(id)<<",\"runtime_instance_count\":1,\"runtime_steps\":"<<steps
      <<",\"summary\":{\"end_phase\":"<<f::quote(r.phase_name())<<",\"synthetic_duration_s\":"<<n(t)
      <<",\"published_tick_count\":"<<published<<",\"target_commit_count\":"<<target_commits
      <<",\"heartbeat_commit_count\":"<<heartbeat_commits<<",\"disarm_command_sink_commits\":"<<disarm_sink_commits
      <<",\"automatic_arm_requests\":"<<arm_commands<<",\"automatic_offboard_requests\":"<<mode_commands
      <<",\"denied_nominal_publication_attempts\":"<<denied_publish_attempts
      <<",\"published_ticks_without_navigation_permission\":"<<published_ticks_before_navigation
      <<",\"visual_motion_ticks_without_navigation_permission\":"<<visual_ticks_before_navigation
      <<",\"command400_generation\":"<<r.pending(400).generation<<",\"command400_history_count\":"<<r.command400_history().size()
      <<"},\"events\":[";
    for(std::size_t i=0;i<rows.size();++i){if(i)std::cout<<',';std::cout<<rows[i];}std::cout<<"]}";
  }
};
int main(){try{
  Trace main("two_complete_1p5m_rc_flights_with_same_process_reprepare");
  main.warm();main.manual_arm(10);main.navigation();main.reach_hold();main.land(1);
  need(!main.r.rc_restart_allowed(main.s),"HELD_CONTROLS_ALLOWED_RESTART");main.record("HELD_CONTROLS_REPREPARE_DENIED");
  main.s.feedback.offboard=false;main.s.nav_state=15;main.s.attitude.rc_mode_slot=6;main.s.attitude.rc_arm_switch=3;
  main.s.stationary_seconds=3;main.tick("CONTROLS_RESET_STABILIZED_DISARMED_GROUND_INPUT");
  const auto prior=main.r.pending(400);const auto history=main.r.command400_history().size();
  need(main.r.reprepare_rc(main.s,main.t),"REPREPARE_REFUSED");++main.attempt;main.record("SAME_RUNTIME_REPREPARE_ACCEPTED",false);
  need(main.r.attitude_core().phase()==a::Phase::Idle&&!main.r.attitude_core().reference_ready(),"STALE_REFERENCE");
  need(main.r.pending(400).generation==prior.generation&&main.r.pending(400).wire_stamp==prior.wire_stamp&&main.r.command400_history().size()==history,"WIRE_HISTORY_LOST");
  main.s.attitude.z=2.02;main.warm();need(std::abs(main.r.attitude_core().reference_z()-2.02)<1e-12,"SECOND_REFERENCE_WRONG");
  main.manual_arm(20);main.navigation();main.reach_hold();main.land(2);main.finish_checks();
  Trace contradiction("settled_height_landed_detector_contradiction_revokes_navigation");
  contradiction.warm();contradiction.manual_arm(10);contradiction.navigation();contradiction.reach_hold();
  auto changed=contradiction.s;changed.feedback.landed=true;unsigned probe_targets=0,probe_heartbeats=0;
  const auto probe=a::publish_attitude_at_boundary<RcMode,RcTarget>(contradiction.r.publication_context(changed,contradiction.t,true,true),contradiction.o.attitude,changed.status_stamp,
    [&](const RcTarget&){++probe_targets;},[&](const RcMode&){++probe_heartbeats;});
  need(!probe.allowed&&probe_targets==0&&probe_heartbeats==0,"POST_PLAN_CONTRADICTION_NOT_VETOED");
  contradiction.s.feedback.landed=true;contradiction.tick("CONTRADICTORY_LANDED_AT_1P5M_INPUT");
  need(contradiction.r.attitude_core().phase()==a::Phase::Descent&&!contradiction.r.attitude_core().navigation_allowed()&&
       !contradiction.o.attitude.visual_motion&&contradiction.target_committed&&contradiction.heartbeat_committed&&
       contradiction.o.attitude.thrust[2]<0&&contradiction.o.attitude.command!=a::Command::Disarm,"CONTRADICTION_UNSAFE_RESULT");
  contradiction.finish_checks();
  const auto& cfg=main.c.attitude;const auto& vertical=cfg.vertical;
  std::cout<<"{\"effective_fixture_config\":{\"rc_target_height_m\":"<<n(cfg.rc_target_height)
    <<",\"rc_navigation_mode_slot\":"<<cfg.rc_navigation_mode_slot<<",\"arm_enabled\":"<<(cfg.arm_enabled?"true":"false")
    <<",\"visual_enabled\":"<<(cfg.visual.enabled?"true":"false")
    <<",\"prestream_seconds\":"<<n(cfg.prestream_seconds)<<",\"ground_seconds\":"<<n(cfg.ground_seconds)
    <<",\"armed_dwell_seconds\":"<<n(cfg.armed_dwell_seconds)<<",\"ramp_seconds\":"<<n(cfg.ramp_seconds)
    <<",\"settle_seconds\":"<<n(cfg.settle_seconds)<<",\"landing_seconds\":"<<n(cfg.landing_seconds)
    <<",\"height_tolerance_m\":"<<n(cfg.height_tolerance)<<",\"speed_tolerance_mps\":"<<n(cfg.speed_tolerance)
    <<",\"liftoff_dwell_seconds\":"<<n(cfg.liftoff_dwell_seconds)<<",\"liftoff_height_m\":"<<n(cfg.liftoff_height_m)
    <<",\"liftoff_speed_mps\":"<<n(cfg.liftoff_speed_mps)<<",\"touchdown_dwell_seconds\":"<<n(cfg.touchdown_dwell_seconds)
    <<",\"ascent_timeout_seconds\":"<<n(cfg.ascent_timeout)<<",\"descent_timeout_seconds\":"<<n(cfg.descent_timeout)
    <<",\"max_flight_seconds\":"<<n(cfg.max_flight_seconds)<<",\"takeoff_reserve_thrust\":"<<n(cfg.takeoff_reserve_thrust)
    <<",\"thrust_hover\":"<<n(vertical.hover)<<",\"thrust_minimum\":"<<n(vertical.minimum)
    <<",\"thrust_maximum\":"<<n(vertical.maximum)<<",\"thrust_slew_per_second\":"<<n(vertical.slew)
    <<",\"height_gain\":"<<n(vertical.height_gain)<<",\"velocity_gain\":"<<n(vertical.velocity_gain)
    <<",\"integral_gain\":"<<n(vertical.integral_gain)<<",\"integral_limit\":"<<n(vertical.integral_limit)
    <<",\"max_climb_mps\":"<<n(vertical.max_climb)<<",\"max_descent_mps\":"<<n(vertical.max_descent)
    <<"},\"all_runtime_assertions_passed\":true,\"scenarios\":[";main.print();std::cout<<',';contradiction.print();
  std::cout<<"],\"post_plan_contradiction_boundary_probe\":{\"allowed\":"<<(probe.allowed?"true":"false")<<",\"reason\":"<<f::quote(a::name(probe.reason))
     <<",\"target_commits\":"<<probe_targets<<",\"heartbeat_commits\":"<<probe_heartbeats<<"}}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
