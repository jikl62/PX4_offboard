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
int main(){
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
