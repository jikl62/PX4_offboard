"""Exercise safe RC recovery through the actual SystemNode.tick implementation.

The clocks and ROS API/message types are synthetic. The node's receive methods,
snapshot, safety scan, Runtime/Core, reprepare and publication path are real.
No DDS, device connection or flight command is executed.
"""
from pathlib import Path
import json
import os
import subprocess
import tempfile
import unittest

import adapter_compile_check as fixture


HARNESS = r'''
#include <cstdint>
double recovery_test_steady=0.;
std::int64_t recovery_test_wall_ns=0;
#include "adapter_under_test.cpp"
#define main rc_recovery_fixture_main
#include "test_rc_navigation.cpp"
#undef main
#include <cassert>

struct RecoveryNode {
  SystemNode node;
  fc01::Config config;
  std::uint64_t stamp{};
  unsigned retained_local_events{};
  bool armed{},offboard{},landed{true};
  fc01::attitude::Quaternion measured_q{1,0,0,0};
  double now{};
  int observer{-1};
  RecoveryNode(){
    // Start with a real RC-authorized attempt and observe actual PX4 departure.
    Flight flight(production_no_vision_rc_config());flight.join();
    flight.s.feedback.offboard=false;flight.s.nav_state=15;
    flight.s.attitude.rc_mode_slot=6;++flight.s.attitude.operator_switch_generation;flight.tick();
    assert(flight.r.attitude_core().reason()=="OPERATOR_TAKEOVER_LATCHED");
    config=flight.c;now=flight.t;stamp=flight.s.status_stamp;
    recovery_test_steady=now;recovery_test_wall_ns=std::int64_t(stamp)*1000;
    node.config_=config;node.runtime_=std::make_unique<fc01::Runtime>(std::move(flight.r));
    node.mode_pub_=node.create_publisher<Mode>("/fmu/in/offboard_control_mode",10);
    node.attitude_pub_=node.create_publisher<AttitudeTarget>("/fmu/in/vehicle_attitude_setpoint",10);
    node.graph_count=[](const std::string& topic,bool publishers){
      if(publishers&&(topic=="/fmu/in/trajectory_setpoint"||topic=="/fmu/in/vehicle_rates_setpoint"||
        topic=="/fmu/in/vehicle_thrust_setpoint"||topic=="/fmu/in/vehicle_torque_setpoint"||
        topic=="/fmu/in/actuator_motors"))return std::size_t(0);
      return std::size_t(1);
    };
    // Keep the service supervisor's ordinary read-only IPC connection alive;
    // publication independently requires an attached status consumer.
    observer=::socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    assert(observer>=0);sockaddr_un address{};address.sun_family=AF_UNIX;
    std::strncpy(address.sun_path,std::getenv("FC01_SOCKET"),sizeof(address.sun_path)-1);
    assert(::connect(observer,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0);
  }
  ~RecoveryNode(){if(observer>=0)::close(observer);}
  void local(std::uint64_t raw){
    Local value{};value.timestamp=value.timestamp_sample=raw;value.z=2;
    // A sustained velocity-output difference is diagnostic. Position stays
    // within the independent derivative residual and physical rest limits.
    value.vz=.09f;value.z_deriv=-.15f;value.z_valid=value.v_z_valid=true;
    const auto previous=node.local_;
    node.receive_sampled(value,node.local_,node.local_stamp_,node.local_sample_,
      fc01::SourceSlot::Local,rclcpp::MessageInfo{});
    if(fc01::local_receive_event(previous,value,node.config_.attitude)||
       node.runtime_->attitude_core().local_requires_control_step(
         value.timestamp_sample,value.z,value.vz,value.z_deriv)){
      node.latch_event(node.local_events_,value);++retained_local_events;
    }
  }
  void cycle(int unsafe=0){
    now+=.05;stamp+=50000;
    recovery_test_steady=now;recovery_test_wall_ns=std::int64_t(stamp)*1000;
    node.begin_cycle(now);
    Status status{};status.timestamp=stamp;status.system_id=status.component_id=1;
    status.arming_state=armed?Status::ARMING_STATE_ARMED:Status::ARMING_STATE_DISARMED;
    status.nav_state=offboard?Status::NAVIGATION_STATE_OFFBOARD:Status::NAVIGATION_STATE_STAB;
    status.pre_flight_checks_pass=status.power_input_valid=true;
    status.can_set_nav_states_mask=1U<<Status::NAVIGATION_STATE_STAB;
    node.receive_event(status,node.status_,node.status_stamp_,fc01::SourceSlot::Status,
      rclcpp::MessageInfo{},fc01::status_receive_event<Status>);
    Land land{};land.timestamp=stamp;land.landed=land.at_rest=landed;
    node.receive_event(land,node.land_,node.land_stamp_,fc01::SourceSlot::Land,
      rclcpp::MessageInfo{},fc01::land_receive_event<Land>);
    Battery battery{};battery.timestamp=stamp;battery.id=1;battery.connected=true;battery.voltage_v=7.6f;
    node.receive_event(battery,node.battery_,node.battery_stamp_,fc01::SourceSlot::Battery,
      rclcpp::MessageInfo{},fc01::battery_receive_event<Battery>);
    Flags flags{};flags.timestamp=stamp;
    node.receive_event(flags,node.flags_,node.flags_stamp_,fc01::SourceSlot::Flags,
      rclcpp::MessageInfo{},fc01::flags_receive_event<Flags>);
    Attitude attitude{};attitude.timestamp=attitude.timestamp_sample=stamp;
    for(unsigned i=0;i<4;++i)attitude.q[i]=float(measured_q[i]);
    node.receive_sampled(attitude,node.attitude_,node.attitude_stamp_,node.attitude_sample_,
      fc01::SourceSlot::Attitude,rclcpp::MessageInfo{});
    Manual manual{};manual.timestamp=manual.timestamp_sample=stamp;manual.valid=true;
    manual.data_source=Manual::SOURCE_RC;manual.throttle=-1;
    node.receive_manual(manual);
    Switches switches{};switches.timestamp=switches.timestamp_sample=stamp;
    switches.mode_slot=6;switches.arm_switch=switches.kill_switch=3;
    node.receive_switches(switches);
    // 40Hz local feedback, with both diagnostic observations retained before
    // each 20Hz tick. This previously reset the recovery proof every cycle.
    local(stamp-25000);local(stamp);
    if(unsafe==1){
      Attitude invalid=attitude;invalid.q={0,0,0,0};
      node.latch_event(node.attitude_events_,invalid);
      node.latch_event(node.attitude_events_,attitude);
    }else if(unsafe==2){
      Land moving=land;moving.landed=false;
      node.latch_event(node.land_events_,moving);node.latch_event(node.land_events_,land);
    }
    node.callbacks_done(now);
    assert(node.tick());
    char output[32768];while(::recv(observer,output,sizeof(output),0)>0){}
    // Recovery and readiness never emit automatic Arm, Offboard or Disarm.
    assert(node.command_pub_->calls==0);
    if(node.runtime_->rc_attempts_completed())
      assert(!node.runtime_->attitude_core().flight_started());
    assert(!fc01::attitude::nonzero(node.last_attitude_output_.thrust));
  }
  void remain_faulted(unsigned cycles,int unsafe=0){
    for(unsigned i=0;i<cycles;++i){cycle(unsafe);
      assert(node.runtime_->attitude_core().phase()==fc01::attitude::Phase::Safety);
      assert(node.runtime_->rc_attempts_completed()==0);
      assert(!node.last_attitude_output_.publish);
    }
  }
  void recover(){
    for(unsigned i=0;i<180&&node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::RcWaitArm;++i)
      cycle();
    assert(node.runtime_->rc_attempts_completed()==1);
    assert(node.runtime_->last_rc_attempt_reason()=="OPERATOR_TAKEOVER_LATCHED");
    assert(node.runtime_->attitude_core().phase()==fc01::attitude::Phase::RcWaitArm);
    assert(!node.runtime_->attitude_core().authority_state().takeover_latched);
    assert(node.runtime_->attitude_core().rc_ground_reference_qualified());
    assert(node.last_attitude_output_.publish);
    assert(node.command_pub_->calls==0);
  }
};
int main(){
  {RecoveryNode n;n.recover();assert(n.retained_local_events>=80);
    std::cout<<"ACTUAL_TICK_DIAGNOSTIC_EVENT_RECOVERY_PASS\n";}
  for(int unsafe:{1,2}){RecoveryNode n;n.remain_faulted(100,unsafe);
    // One healthy tick cannot reuse proof accumulated before the unsafe event.
    n.cycle();assert(n.node.runtime_->rc_attempts_completed()==0);n.recover();}
  std::cout<<"ACTUAL_TICK_QUEUED_UNSAFE_EVIDENCE_VETO_PASS\n";
  for(int condition:{1,2}){RecoveryNode n;n.armed=condition==1;n.offboard=condition==2;
    n.remain_faulted(100);n.armed=n.offboard=false;n.cycle();
    assert(n.node.runtime_->rc_attempts_completed()==0);n.recover();}
  std::cout<<"ACTUAL_TICK_NO_ARMED_OR_OFFBOARD_REACQUIRE_PASS\n";
  {RecoveryNode n;n.node.runtime_->abort_attitude("PUBLISHER_BOUNDARY_TEST_INTEGRITY");
    n.remain_faulted(3);
    std::cout<<n.node.state(n.node.snapshot(n.now),n.now)<<std::endl;}
  std::cout<<"ACTUAL_TICK_LATER_FAULT_DIAGNOSTICS_PASS\n";
  for(int condition:{1,2}){
    RecoveryNode n;n.recover();
    const auto previous=n.node.last_attitude_output_;
    const auto previous_stamp=n.node.publication_audit_.mode_last;
    assert(previous.publish);
    n.armed=n.offboard=condition==1;n.landed=condition!=1;
    if(condition==1)n.measured_q={std::sqrt(.5),std::sqrt(.5),0,0};
    n.node.runtime_->abort_attitude("STREAM_EDGE_OBSERVED_STATUS_TEST");
    n.cycle();
    assert(!n.node.last_attitude_output_.publish);
    std::cout<<"{\"kind\":\"stream_edge_expected\",\"armed\":"<<boolean(n.armed)
      <<",\"offboard\":"<<boolean(n.offboard)<<",\"landed\":"<<boolean(n.landed)
      <<",\"status_stamp\":"<<n.stamp<<",\"land_stamp\":"<<n.stamp
      <<",\"local_stamp\":"<<n.stamp
      <<",\"attitude_tilt_deg\":"<<(condition==1?90:0)
      <<",\"previous_heartbeat_commit_monotonic_s\":"<<number(previous_stamp)
      <<",\"last_committed_target_thrust_z\":"<<number(previous.thrust[2])<<"}"<<std::endl;
  }
  std::cout<<"ACTUAL_TICK_STREAM_EDGE_CURRENT_STATUS_PASS\n";
}
'''


def write_fixture(root):
    (root/'rclcpp').mkdir()
    ros=fixture.RCLCPP.replace('return 1000000000;', 'return ::recovery_test_wall_ns;')
    ros=ros.replace('void publish(const T&){};', 'unsigned calls{};void publish(const T&){++calls;};')
    (root/'rclcpp/rclcpp.hpp').write_text(ros)
    messages=root/'px4_msgs/msg';messages.mkdir(parents=True)
    for filename,(name,version,fields) in fixture.MESSAGES.items():
        constant='' if version is None else f'static constexpr int MESSAGE_VERSION={version};'
        (messages/(filename+'.hpp')).write_text(
            '#pragma once\n#include <array>\n#include <cstdint>\n#include <memory>\n'
            'namespace px4_msgs::msg {struct '+name+'{using ConstSharedPtr=std::shared_ptr<const '+name+'>;'
            'std::uint64_t timestamp{};'+constant+fields+'};}\n')
    vision=root/'jikl_fc01_system/msg';vision.mkdir(parents=True)
    (vision/'stamp.hpp').write_text(
        '#pragma once\n#include <cstdint>\nnamespace jikl_fc01_system::msg {'
        'struct Stamp{std::int32_t sec{};std::uint32_t nanosec{};};}\n')
    for filename,(name,fields) in fixture.VISION_MESSAGES.items():
        (vision/(filename+'.hpp')).write_text(
            '#pragma once\n#include <array>\n#include <cstdint>\n#include <memory>\n#include <string>\n'
            '#include "jikl_fc01_system/msg/stamp.hpp"\nnamespace jikl_fc01_system::msg {'
            'struct '+name+'{using ConstSharedPtr=std::shared_ptr<const '+name+'>;'
            'std::uint16_t schema_version{1};'+fields+'};}\n')
    source=(fixture.ROOT/'src/jikl_fc01_system/src/fc01_system_node.cpp').read_text()
    clock='double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}'
    if source.count(clock)!=1:
        raise AssertionError('Update the deterministic clock fixture for the node clock implementation')
    source=source.replace(clock,'double steady(){return recovery_test_steady;}',1)
    source=source.replace(' private:',' public:',1).replace('int main(','int adapter_program_main(',1)
    (root/'adapter_under_test.cpp').write_text(source)
    (root/'test.cpp').write_text(HARNESS)


class RcRecoveryAdapterTest(unittest.TestCase):
    def test_actual_tick_recovery_uses_current_stationary_evidence(self):
        with tempfile.TemporaryDirectory(prefix='fc01_recovery_tick_') as directory:
            root=Path(directory);write_fixture(root)
            executable=root/'test'
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-pthread',
                '-I',str(root),'-I',str(fixture.ROOT/'src/jikl_fc01_system/include'),
                '-I',str(fixture.ROOT/'tests'),str(root/'test.cpp'),'-o',str(executable)],check=True)
            env=os.environ.copy();env['FC01_SOCKET']=str(root/'control.sock')
            result=subprocess.run([str(executable)],env=env,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr+'\n'+result.stdout[-12000:])
            for marker in ('ACTUAL_TICK_DIAGNOSTIC_EVENT_RECOVERY_PASS',
                           'ACTUAL_TICK_QUEUED_UNSAFE_EVIDENCE_VETO_PASS',
                           'ACTUAL_TICK_NO_ARMED_OR_OFFBOARD_REACQUIRE_PASS',
                           'ACTUAL_TICK_LATER_FAULT_DIAGNOSTICS_PASS',
                           'ACTUAL_TICK_STREAM_EDGE_CURRENT_STATUS_PASS'):
                self.assertIn(marker,result.stdout)
            rows=[json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
            reports=[row for row in rows if row.get('kind') in ('state','phase_event')]
            self.assertTrue(any(row['kind']=='state' for row in reports))
            self.assertTrue(any(row['kind']=='phase_event' for row in reports))
            for row in reports:
                for field in ('rc_first_fault_reason','rc_latest_fault_reason',
                              'rc_recovery_block_reason','rc_reprepare_block_reason'):
                    self.assertIsInstance(row[field],str)
                self.assertIsInstance(row['rc_recovery_ready'],bool)
                self.assertIsInstance(row['rc_fault_recoverable'],bool)
            hidden=[row for row in reports if row['rc_latest_fault_reason']=='PUBLISHER_BOUNDARY_TEST_INTEGRITY']
            self.assertTrue(hidden)
            self.assertEqual(hidden[-1]['rc_first_fault_reason'],'OPERATOR_TAKEOVER_LATCHED')
            self.assertEqual(hidden[-1]['rc_recovery_block_reason'],'RC_RECOVERY_FAULT_REQUIRES_RESTART')
            self.assertEqual(hidden[-1]['rc_reprepare_block_reason'],'RC_CORE_RESTART_CONDITIONS_NOT_MET')
            self.assertFalse(hidden[-1]['rc_fault_recoverable'])
            expected=[row for row in rows if row.get('kind')=='stream_edge_expected']
            edges=[row for row in rows if row.get('kind')=='offboard_stream_edge' and
                   row.get('reason')=='STREAM_EDGE_OBSERVED_STATUS_TEST']
            self.assertEqual(len(edges),2)
            self.assertEqual(len(expected),2)
            for actual,wanted in zip(edges,expected):
                for field in ('armed','offboard','landed','status_stamp','land_stamp',
                              'local_stamp','previous_heartbeat_commit_monotonic_s','last_committed_target_thrust_z'):
                    self.assertEqual(actual[field],wanted[field],field)
                self.assertAlmostEqual(actual['attitude_tilt_deg'],wanted['attitude_tilt_deg'],places=5)
                self.assertGreater(actual['monotonic_s'],actual['previous_heartbeat_commit_monotonic_s'])
                self.assertFalse(actual['local_target_this_tick'])
                self.assertFalse(actual['local_heartbeat_this_tick'])
            startups=[row for row in rows if row.get('kind')=='startup']
            self.assertTrue(startups)
            self.assertTrue(all(row['controller_build']=='FC01_RECOVERY_DIAGNOSTIC_V4_20261110'
                                for row in startups))


if __name__=='__main__':
    unittest.main()
