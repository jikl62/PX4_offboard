"""Run the actual SystemNode safety scan against the narrow ROS API fixture.

This exercises production adapter code without DDS, motors or ROS execution.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
import adapter_compile_check as fixture

HARNESS = r'''
#include "adapter_under_test.cpp"
#include "authority_test_support.hpp"
#define main rc_adapter_fixture_main
#include "test_rc_navigation.cpp"
#undef main
#include <cassert>
using Event=fc01::attitude::Core::SafetyEvent;
struct Fixture {
  SystemNode node;fc01::Config c{authority_test::config()};
  fc01::Snapshot s{authority_test::sample()};double now{};
  Fixture(){
    c.attitude.autonomous_failsafe_confirmed=true;
    node.config_=c;node.runtime_=std::make_unique<fc01::Runtime>(c);
    authority_test::armed(*node.runtime_,s,now,"AUTHORITY_AUTONOMOUS");
    for(unsigned i=0;i<20&&node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::ThrustRamp;++i)
      authority_test::tick(*node.runtime_,s,now);
    assert(node.runtime_->attitude_core().phase()==fc01::attitude::Phase::ThrustRamp);
  }
  fc01::Output run(){node.observe_pending_safety(s,now);auto out=authority_test::tick(*node.runtime_,s,now);
    node.end_latched_events();return out;}
  void blocked(){auto out=run();assert(!out.attitude.publish);assert(out.attitude.command==fc01::attitude::Command::None);
    assert(node.runtime_->attitude_core().phase()==fc01::attitude::Phase::Safety);}
};
// Keep the actual node's callback predicate, retained event scan, Runtime and
// final publication sinks in this regression. Source samples arrive at 40Hz,
// twice per 20Hz control scan; no ROS/DDS or flight hardware is involved.
struct FeedbackFixture {
  SystemNode node;fc01::Config c;fc01::Snapshot s;double now{};
  Local current{};double z{};unsigned queued_good{},queued_bad{},targets{},heartbeats{};
  FeedbackFixture(){
    Flight flight(production_no_vision_rc_config());flight.join();
    for(unsigned i=0;i<25&&flight.r.attitude_core().phase()!=fc01::attitude::Phase::ThrustRamp;++i)flight.tick();
    assert(flight.r.attitude_core().phase()==fc01::attitude::Phase::ThrustRamp);
    for(unsigned i=0;i<84;++i)flight.tick();
    // Establish early height feedback with a coherent position/derivative
    // history. A pending detector does not bypass the feedback protections.
    for(unsigned i=1;i<=20;++i){
      flight.s.attitude.z=2.-.01*i;flight.s.attitude.vz=flight.s.attitude.z_deriv=-.2;flight.tick();
    }
    flight.s.attitude.vz=flight.s.attitude.z_deriv=0;flight.tick();
    assert(flight.o.attitude.vertical_closed_loop&&flight.o.attitude.publish);
    c=flight.c;s=flight.s;now=flight.t;node.config_=c;
    node.runtime_=std::make_unique<fc01::Runtime>(std::move(flight.r));
    current.timestamp=s.attitude.local_stamp;current.timestamp_sample=s.attitude.vertical_sample_stamp;
    current.z=float(s.attitude.z);current.z_valid=current.v_z_valid=true;z=current.z;node.local_=current;
  }
  void receive(double vz,double derivative,bool integrate=true){
    const auto previous=current;current.timestamp+=25000;current.timestamp_sample+=25000;
    current.vz=float(vz);current.z_deriv=float(derivative);
    if(integrate)z+=(double(previous.z_deriv)+current.z_deriv)*.5*.025;
    current.z=float(z);
    const auto old=node.local_;
    node.receive_sampled(current,node.local_,node.local_stamp_,node.local_sample_,
      fc01::SourceSlot::Local,rclcpp::MessageInfo{});
    if(fc01::local_receive_event(old,current,node.config_.attitude)||
       node.runtime_->attitude_core().local_requires_control_step(
         current.timestamp_sample,current.z,current.vz,current.z_deriv)){
      node.latch_event(node.local_events_,current);++node.receive_events_;
      if(std::abs(vz-derivative)<=c.attitude.feedback_guard.rate_tolerance_mps)++queued_good;
      else ++queued_bad;
    }
  }
  fc01::Output scan(bool expect_output=true){
    now+=.05;s.status_stamp+=50000;s.land_stamp+=50000;
    s.attitude.rc_switch_sample_stamp+=50000;s.attitude.source_host_stamp=s.status_stamp;
    s.attitude.local_stamp=current.timestamp;s.attitude.vertical_sample_stamp=current.timestamp_sample;
    s.attitude.z=current.z;s.attitude.vz=current.vz;s.attitude.z_deriv=current.z_deriv;
    node.observe_pending_safety(s,now);auto out=node.runtime_->step(s,now);
    if(expect_output){
      assert(out.attitude.publish&&node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::Safety);
      const auto decision=fc01::attitude::publish_attitude_at_boundary<Mode,AttitudeTarget>(
        node.runtime_->publication_context(s,now,true,true),out.attitude,s.status_stamp,
        [&](const auto&){++targets;},[&](const auto&){++heartbeats;});
      assert(decision.allowed);node.runtime_->confirm_rc_output(true,now);
    }else{
      assert(!out.attitude.publish&&node.runtime_->attitude_core().phase()==fc01::attitude::Phase::Safety);
    }
    node.end_latched_events();return out;
  }
};
static void alternating_rate_samples_keep_separate_diagnostic_episodes(){
  FeedbackFixture f;
  for(unsigned i=0;i<20;++i){
    f.receive(0,0);f.receive(0,.3);f.scan();
    const auto& e=f.node.runtime_->attitude_core().vertical_feedback_evidence();
    assert(e.healthy&&e.suspect&&!e.rate_warning&&e.reason.empty());
    assert(e.disagreement_samples==1&&e.disagreement_seconds==0);
  }
  assert(f.queued_good>=19&&f.queued_bad==20&&f.targets==20&&f.heartbeats==20);
  f.receive(0,0);f.receive(0,0);f.scan();
  const auto& e=f.node.runtime_->attitude_core().vertical_feedback_evidence();
  assert(e.healthy&&!e.suspect&&!e.rate_warning&&e.disagreement_samples==0&&e.disagreement_seconds==0);
}
static void persistent_rate_difference_warns_without_cutting_publication(){
  FeedbackFixture f;
  for(unsigned i=0;i<20;++i){f.receive(0,.3);f.receive(0,.3);f.scan();}
  const auto& e=f.node.runtime_->attitude_core().vertical_feedback_evidence();
  assert(e.healthy&&e.suspect&&e.rate_warning&&e.reason.empty());
  assert(e.disagreement_seconds>=.9&&e.disagreement_samples==40);
  assert(f.targets==20&&f.heartbeats==20);
}
static void rate_diagnostics_do_not_freeze_vertical_integration(){
  FeedbackFixture f;
  f.receive(0,0);f.receive(0,0);const auto before=f.scan();
  assert(before.attitude.integral_thrust>0);
  for(unsigned i=0;i<15;++i){
    f.receive(-.4,0);f.receive(-.4,0);const auto out=f.scan();
    assert(out.attitude.integral_thrust<before.attitude.integral_thrust);
  }
  assert(f.node.runtime_->attitude_core().vertical_feedback_evidence().rate_warning);
}
static void rate_diagnostics_preserve_fatal_feedback_checks(){
  for(unsigned failure=0;failure<4;++failure){
    FeedbackFixture f;f.receive(0,0);f.receive(0,0);f.scan();
    Local bad=f.current;bad.timestamp+=25000;bad.timestamp_sample+=25000;
    if(failure==0)bad.z_deriv=std::numeric_limits<float>::quiet_NaN();
    if(failure==1)bad.timestamp_sample=f.current.timestamp_sample-1;
    if(failure==2)++bad.z_reset_counter;
    if(failure==3)bad.timestamp_sample=f.current.timestamp_sample+600000;
    f.node.local_events_.push(bad);
    f.node.local_events_.push(f.current); // later healthy data cannot erase the event
    f.scan(false);
    const auto& why=f.node.runtime_->attitude_core().reason();
    assert(why==(failure==0?"NONFINITE_VERTICAL_TELEMETRY":
      failure==1?"VERTICAL_FEEDBACK_TIME_REGRESSION":failure==2?
      "ACTIVE_VERTICAL_ESTIMATOR_RESET":"VERTICAL_FEEDBACK_SAMPLE_GAP"));
  }
  FeedbackFixture f;
  // Establish coherent position windows, then change position by .03m per
  // control tick with zero derivative. The independent residual must fail.
  for(unsigned i=0;i<10;++i){f.receive(0,0);f.receive(0,0);f.scan();}
  bool blocked=false;
  for(unsigned i=0;i<12&&!blocked;++i){
    f.z+=.015;f.receive(0,0,false);f.z+=.015;f.receive(0,0,false);
    // Use the real preflight scan before deciding whether output is allowed.
    f.now+=.05;f.s.status_stamp+=50000;f.s.land_stamp+=50000;
    f.s.attitude.rc_switch_sample_stamp+=50000;f.s.attitude.source_host_stamp=f.s.status_stamp;
    f.s.attitude.local_stamp=f.current.timestamp;f.s.attitude.vertical_sample_stamp=f.current.timestamp_sample;
    f.s.attitude.z=f.current.z;f.s.attitude.vz=f.s.attitude.z_deriv=0;
    f.node.observe_pending_safety(f.s,f.now);const auto out=f.node.runtime_->step(f.s,f.now);
    blocked=f.node.runtime_->attitude_core().phase()==fc01::attitude::Phase::Safety;
    if(blocked)assert(!out.attitude.publish&&f.node.runtime_->attitude_core().reason()=="VERTICAL_POSITION_RATE_INCONSISTENT");
    else assert(out.attitude.publish);
    f.node.end_latched_events();
  }
  assert(blocked);
}
int main(){
  alternating_rate_samples_keep_separate_diagnostic_episodes();
  persistent_rate_difference_warns_without_cutting_publication();
  rate_diagnostics_do_not_freeze_vertical_integration();
  rate_diagnostics_preserve_fatal_feedback_checks();
  {Fixture f;f.node.begin_cycle(steady());f.node.callbacks_done(steady());
   f.node.sampling_pair_=true;
   const auto callbacks=f.node.received_callbacks();const auto events=f.node.receive_events();
   f.node.boundary_context();const auto queries=f.node.work_.graph_queries;
   assert(queries>0);f.node.boundary_context();f.node.boundary_context();
   assert(f.node.work_.graph_queries==queries&&f.node.pair_context_count_==3);
   assert(f.node.received_callbacks()==callbacks&&f.node.receive_events()==events);
   for(const auto& sample:f.node.pair_samples_){
     assert(sample.cycle==f.node.cycle_id_&&sample.callbacks==callbacks&&sample.events==events);
     assert(sample.graph_valid);
   }
   f.node.graph_event_->set();const auto unchanged=f.node.current_graph();
   assert(!f.node.cycle_graph_invalidated_&&unchanged.traffic.status_publishers==1);
   assert(f.node.work_.graph_queries==2*queries); // Unrelated discovery rechecks counts safely.
   f.node.graph_count=[&](const std::string& topic,bool publishers){
     return publishers&&topic=="/fmu/in/offboard_control_mode"?2U:1U;};
   f.node.graph_event_->set();const auto conflict=f.node.current_graph();
   assert(conflict.ownership.mode_publishers==2);
   assert(!fc01::attitude::owns_setpoints("attitude_altitude",conflict.ownership));
   f.node.graph_count=[&](const std::string&,bool){f.node.graph_event_->set();return 1U;};
   f.node.graph_event_->set();const auto invalid=f.node.current_graph();
   assert(f.node.cycle_graph_invalidated_&&invalid.traffic.status_publishers==0);
   const auto failed_queries=f.node.work_.graph_queries;f.node.current_graph();
   assert(f.node.work_.graph_queries==failed_queries); // No retry loop on graph churn.
   f.node.graph_count={};
   f.node.begin_cycle(steady());f.node.callbacks_done(steady());
   const auto fresh=f.node.current_graph();assert(!f.node.cycle_graph_invalidated_);
   assert(fresh.traffic.status_publishers==1&&f.node.work_.graph_queries==queries);}
  {Fixture f;Attitude good{};good.q={1,0,0,0};
   for(unsigned i=0;i<33;++i)f.node.latch_event(f.node.attitude_events_,good);
   assert(f.node.receive_event_overflow_);assert(f.node.snapshot(f.now).time_regressed);}
  {Fixture f;Attitude good{},bad{};good.q={1,0,0,0};bad.q={0,0,0,0};
   f.node.attitude_events_.push(good);f.node.attitude_events_.push(bad);f.node.attitude_events_.push(good);
   f.blocked();auto out=authority_test::tick(*f.node.runtime_,f.s,f.now);assert(!out.attitude.publish);}
  {Fixture f;Local good{},bad{};good.z=f.s.attitude.z;good.z_valid=good.v_z_valid=true;
   bad=good;bad.z=std::numeric_limits<float>::quiet_NaN();
   f.node.local_events_.push(good);f.node.local_events_.push(bad);f.node.local_events_.push(good);f.blocked();}
  {Fixture f;Attitude good{},bad{};good.q={1,0,0,0};bad=good;bad.quat_reset_counter=1;
   f.node.attitude_events_.push(good);f.node.attitude_events_.push(bad);f.node.attitude_events_.push(good);f.blocked();}
  {Fixture f;Flags good{},bad{};bad.offboard_control_signal_lost=true;
   f.node.flags_events_.push(good);f.node.flags_events_.push(bad);f.node.flags_events_.push(good);f.blocked();}
  {Fixture f;Status good{},bad{};good.arming_state=Status::ARMING_STATE_ARMED;
   good.nav_state=Status::NAVIGATION_STATE_OFFBOARD;good.pre_flight_checks_pass=good.power_input_valid=true;
   bad=good;bad.failsafe=true;f.node.status_events_.push(good);f.node.status_events_.push(bad);
   f.node.status_events_.push(good);f.blocked();}
  {Fixture f;Battery good{},bad{};good.connected=true;good.voltage_v=16;bad=good;
   bad.voltage_v=std::numeric_limits<float>::quiet_NaN();f.node.battery_events_.push(good);
   f.node.battery_events_.push(bad);f.node.battery_events_.push(good);f.blocked();}
  {Fixture f;fc01::AuthorityTelemetry::Facts good{},bad{};good.rc_link_ready=good.rc_switches_ready=true;
   good.rc_axes_safe=true;good.switch_generation=f.s.attitude.operator_switch_generation;bad=good;bad.kill=true;
   f.node.authority_events_.push(good);f.node.authority_events_.push(bad);f.node.authority_events_.push(good);f.blocked();}
  {Fixture f;Attitude good{};good.q={1,0,0,0};
   for(double degrees:{20.2,90.}){const double half=degrees*3.141592653589793/360.;
     good.q={float(std::cos(half)),float(std::sin(half)),0,0};f.node.attitude_events_.push(good);}
   auto out=f.run();assert(out.attitude.publish);assert(f.node.attitude_events_.empty());
   assert(f.node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::Safety);}
  {Fixture f;fc01::AuthorityTelemetry::Facts changed{};
   changed.rc_link_ready=changed.rc_switches_ready=changed.rc_axes_safe=true;
   changed.switch_generation=f.s.attitude.operator_switch_generation+1;
   f.node.authority_events_.push(changed);auto out=f.run();assert(out.attitude.publish);
   assert(f.node.runtime_->attitude_core().operator_release_requested());
   unsigned targets=0,heartbeats=0;
   auto decision=fc01::attitude::publish_attitude_at_boundary<Mode,AttitudeTarget>(
     f.node.runtime_->publication_context(f.s,f.now,true,true),out.attitude,1000000,
     [&](const auto&){++targets;},[&](const auto&){++heartbeats;});
   assert(decision.allowed&&targets==1&&heartbeats==1);}
  {Fixture f;assert(f.node.runtime_->request("LAND",{},f.s,f.now).accepted);
   for(unsigned i=0;i<100&&f.node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::Landed;++i)
     authority_test::tick(*f.node.runtime_,f.s,f.now);
   assert(f.node.runtime_->attitude_core().phase()==fc01::attitude::Phase::Landed);
   Land good{},bad{};good.landed=good.at_rest=true;bad=good;bad.landed=false;
   f.node.land_events_.push(good);f.node.land_events_.push(bad);f.node.land_events_.push(good);
   auto out=f.run();assert(out.attitude.command==fc01::attitude::Command::None);
   assert(!f.node.runtime_->attitude_core().ground_confirmed());}
  {Fixture f;f.c.attitude.rc_navigation_enabled=true;f.c.attitude.arm_enabled=false;
   f.c.attitude.rc_takeover_confirmed=true;f.c.attitude.autonomous_failsafe_confirmed=false;
   f.c.attitude.max_flight_seconds=30;f.c.attitude.max_tilt_deg=10;f.node.config_=f.c;
   f.node.runtime_=std::make_unique<fc01::Runtime>(f.c);
   f.s=authority_test::sample();f.s.nav_state=15;auto& a=f.s.attitude;
   a.source_clock_validated=a.rc_link_ready=a.rc_switches_ready=a.rc_source_clock_valid=a.rc_switch_clock_valid=true;
   a.rc_source_selected=a.rc_axes_safe=a.rc_aux_switches_safe=true;a.rc_arm_switch=a.rc_kill_switch=3;
   a.rc_mode_slot=1;a.source_host_stamp=1000;
   Attitude invalid{};invalid.q={0,0,0,0};f.node.attitude_events_.push(invalid);
   auto out=f.run();assert(!out.attitude.publish);
   assert(f.node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::Safety);
   out=authority_test::tick(*f.node.runtime_,f.s,f.now);assert(out.attitude.publish);
   assert(f.node.runtime_->attitude_core().phase()==fc01::attitude::Phase::RcPrestream);}
  std::cout<<"actual adapter queued safety preflight: PASS\n";
}
'''


class ReceiveEventAdapterTest(unittest.TestCase):
    def test_pending_unsafe_payload_cannot_be_hidden_by_benign_head(self):
        with tempfile.TemporaryDirectory(prefix='fc01_safety_adapter_') as directory:
            root = Path(directory)
            (root/'rclcpp').mkdir()
            (root/'rclcpp/rclcpp.hpp').write_text(fixture.RCLCPP)
            msg=root/'px4_msgs/msg';msg.mkdir(parents=True)
            for filename,(name,version,fields) in fixture.MESSAGES.items():
                const='' if version is None else f'static constexpr int MESSAGE_VERSION={version};'
                (msg/(filename+'.hpp')).write_text('#pragma once\n#include <array>\n#include <cstdint>\n#include <memory>\nnamespace px4_msgs::msg {struct '+name+'{using ConstSharedPtr=std::shared_ptr<const '+name+'>;std::uint64_t timestamp{};'+const+fields+'};}\n')
            vision=root/'jikl_fc01_system/msg';vision.mkdir(parents=True)
            (vision/'stamp.hpp').write_text('#pragma once\n#include <cstdint>\nnamespace jikl_fc01_system::msg {struct Stamp{std::int32_t sec{};std::uint32_t nanosec{};};}\n')
            for filename,(name,fields) in fixture.VISION_MESSAGES.items():
                (vision/(filename+'.hpp')).write_text('#pragma once\n#include <array>\n#include <cstdint>\n#include <memory>\n#include <string>\n#include "jikl_fc01_system/msg/stamp.hpp"\nnamespace jikl_fc01_system::msg {struct '+name+'{using ConstSharedPtr=std::shared_ptr<const '+name+'>;std::uint16_t schema_version{1};'+fields+'};}\n')
            source=(fixture.ROOT/'src/jikl_fc01_system/src/fc01_system_node.cpp').read_text()
            source=source.replace(' private:', ' public:', 1).replace('int main(', 'int adapter_program_main(', 1)
            (root/'adapter_under_test.cpp').write_text(source)
            (root/'test.cpp').write_text(HARNESS)
            executable=root/'test'
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-pthread',
                '-I',str(root),'-I',str(fixture.ROOT/'src/jikl_fc01_system/include'),
                '-I',str(fixture.ROOT/'tests'),str(root/'test.cpp'),'-o',str(executable)],check=True)
            env=os.environ.copy();env['FC01_SOCKET']=str(root/'control.sock')
            result=subprocess.run([str(executable)],env=env,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr+'\n'+result.stdout)

if __name__=='__main__':unittest.main()
