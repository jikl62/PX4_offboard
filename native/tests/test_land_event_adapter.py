# 落地事件托管回归：真实SystemNode/Runtime/发布边界，ROS API、时钟和消息输入为替身。
# 稳定落地检测按1Hz到达；更快的真假翻转只作压力输入，不宣称等同真实PX4迟滞或飞行仿真。
"""Exercise land-event custody through the real node and publication boundary.

Only the ROS API, clocks and input transport are synthetic. Stable detector
samples arrive at 1 Hz. Faster changes are explicit stress inputs, not a claim
about the PX4 detector's hysteresis or a flight simulation.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

import test_safety_receive_adapter as fixture


# 复用真实恢复夹具的节点初始化，截去其main与未使用方法，保留同一控制路径。
HARNESS = fixture.recovery.HARNESS.split('\nint main(){', 1)[0]
HARNESS = HARNESS.split('\n  void remain_faulted(', 1)[0] + '\n};\n'


# 共享夹具替换项必须恰好出现一次；源码漂移时明确失败，防止静默测错路径。
def replace_once(source, old, new):
    if source.count(old) != 1:
        raise AssertionError('Update the land-event fixture for its shared node fixture')
    return source.replace(old, new, 1)


# 替换点仅构造测试输入：控制landed/at_rest、是否送达落地样本以及RC开关历史。
for old, new in (
    (
        'bool armed{},offboard{},landed{true};',
        'bool armed{},offboard{},landed{true};'
        'bool rest_input{true},deliver_land{true},bad_land_head{},bad_landed_input{};'
        'std::uint8_t mode_input{6},arm_input{1};',
    ),
    ('value.vz=.09f;value.z_deriv=-.15f;', 'value.vz=value.z_deriv=0;'),
    (
        'status.pre_flight_checks_pass=status.power_input_valid=true;',
        'status.pre_flight_checks_pass=status.power_input_valid=true;'
        'status.armed_time=10;status.latest_arming_reason=2;',
    ),
    ('land.landed=land.at_rest=landed;', 'land.landed=landed;land.at_rest=rest_input;'),
    (
        '''    node.receive_event(land,node.land_,node.land_stamp_,fc01::SourceSlot::Land,
      rclcpp::MessageInfo{},fc01::land_receive_event<Land>);''',
        '''    if(bad_land_head){
      auto bad=land;bad.timestamp-=1000;bad.landed=bad_landed_input;bad.at_rest=false;
      node.land_sub_->callback(std::make_shared<Land>(bad),rclcpp::MessageInfo{});
    }
    if(deliver_land)node.land_sub_->callback(std::make_shared<Land>(land),rclcpp::MessageInfo{});''',
    ),
    (
        'switches.mode_slot=6;switches.arm_switch=switches.kill_switch=3;',
        'switches.mode_slot=mode_input;switches.arm_switch=arm_input;switches.kill_switch=3;',
    ),
    ('assert(node.command_pub_->calls==0);', ''),
    ('assert(!fc01::attitude::nonzero(node.last_attitude_output_.thrust));', ''),
):
    HARNESS = replace_once(HARNESS, old, new)


# 以下C++字符串是既有可执行测试输入，逐字保留；中文说明放在Python边界外。
# 六组案例分别要求：等待上锁仍保留零推力双流；反跳不放弃已解锁托管；密集变化不饿死下降控制；
# 同轮坏落地样本否决完成/释放；致命姿态故障维持静默；确认新地面后才可准备下一次尝试。
HARNESS += r'''
using Phase=fc01::attitude::Phase;
struct LandNode:RecoveryNode {
  explicit LandNode(Phase desired=Phase::RequestDisarm){
    auto c=production_no_vision_rc_config();c.attitude.rc_navigation_mode_slot=6;
    Flight q(c);q.join();q.s.at_rest=true;
    assert(q.r.request("LAND",{},q.s,q.t).accepted);
    const auto first=desired==Phase::Landed?Phase::Landed:Phase::RequestDisarm;
    for(unsigned i=0;i<400&&q.r.attitude_core().phase()!=first;++i)q.tick();
    assert(q.r.attitude_core().phase()==first);
    if(first==Phase::RequestDisarm){
      q.r.confirm_attitude_command_publication(fc01::attitude::Command::Disarm,true,
        q.s.status_stamp+1,q.s,q.t);
      if(desired==Phase::SafeDisarmed||desired==Phase::ReleaseOffboard){
        q.s.feedback.armed=false;
        for(unsigned i=0;i<10&&q.r.attitude_core().phase()!=Phase::SafeDisarmed;++i)q.tick();
        assert(q.r.attitude_core().phase()==Phase::SafeDisarmed);
        if(desired==Phase::ReleaseOffboard)q.tick();
      }
    }
    assert(q.r.attitude_core().phase()==desired);
    node.config_=q.c;node.runtime_=std::make_unique<fc01::Runtime>(std::move(q.r));
    now=q.t;stamp=q.s.status_stamp;armed=q.s.feedback.armed;offboard=true;
    node.land_.timestamp=stamp;node.land_.landed=node.land_.at_rest=true;
    node.land_stamp_.accept(stamp,now);
    // Reproduce the same physical arm/navigation switch history as Flight.
    // Replacing Runtime must not fabricate a fresh operator intervention.
    Switches switches{};switches.mode_slot=1;switches.arm_switch=switches.kill_switch=3;
    for(unsigned i=0;i<3;++i){
      switches.timestamp=switches.timestamp_sample=stamp-300000+i*100000;
      if(i==1)switches.arm_switch=1;
      if(i==2)switches.mode_slot=6;
      node.authority_telemetry_.switches(switches,now-.3+i*.1,2.5,stamp);
      Manual manual{};manual.timestamp=manual.timestamp_sample=switches.timestamp;manual.valid=true;
      manual.data_source=Manual::SOURCE_RC;manual.throttle=-1;
      node.authority_telemetry_.manual(manual,now-.3+i*.1,stamp,2.5);
    }
  }
  void publishing_cycle(int unsafe=0){
    const auto targets=node.attitude_pub_->calls,heartbeats=node.mode_pub_->calls;
    const auto commands=node.command_pub_->calls;cycle(unsafe);
    assert(node.attitude_pub_->calls==targets+1&&node.mode_pub_->calls==heartbeats+1);
    assert(node.runtime_->attitude_core().phase()!=Phase::Safety&&node.last_attitude_output_.publish);
    assert(node.command_pub_->calls==commands);
  }
  void silent_cycle(int unsafe=0){
    const auto targets=node.attitude_pub_->calls,heartbeats=node.mode_pub_->calls;
    const auto commands=node.command_pub_->calls;cycle(unsafe);
    assert(node.attitude_pub_->calls==targets&&node.mode_pub_->calls==heartbeats);
    assert(!node.last_attitude_output_.publish&&node.command_pub_->calls==commands);
  }
};

static void waiting_for_disarm_keeps_its_zero_stream(){
  LandNode n;
  for(unsigned i=0;i<60;++i){
    n.deliver_land=i%20==0;n.publishing_cycle();
    assert(n.node.runtime_->attitude_core().phase()==Phase::RequestDisarm);
    assert(!fc01::attitude::nonzero(n.node.last_attitude_output_.thrust));
  }
  std::cout<<"LAND_EVENT_NORMAL_WAIT_DISARM_PASS\n";
}

static void bad_land_or_rest_keeps_armed_custody_with_both_tail_orders(){
  for(const auto phase:{Phase::Landed,Phase::RequestDisarm}){
    for(bool healthy_tail:{false,true})for(bool lost_landed:{false,true}){
      LandNode n(phase);
      if(healthy_tail){n.bad_land_head=true;n.bad_landed_input=!lost_landed;}
      else {n.landed=!lost_landed;n.rest_input=false;}
      n.publishing_cycle();
      assert(n.node.runtime_->attitude_core().phase()==
        (phase==Phase::RequestDisarm||(!healthy_tail&&lost_landed)?Phase::Descent:Phase::Landed));
      n.bad_land_head=false;n.deliver_land=false;n.publishing_cycle();
      assert(n.node.command_pub_->calls==0);
    }
  }
  std::cout<<"LAND_EVENT_ARMED_BOUNCE_BOTH_TAIL_ORDERS_PASS\n";
}

static void repeated_land_changes_cannot_starve_the_descent_controller(){
  for(unsigned count:{1U,2U,5U,6U,7U,40U}){
    LandNode n;
    for(unsigned i=0;i<60;++i){
      n.landed=i<count?i%2:(count-1)%2;n.rest_input=false;
      n.deliver_land=i<count||i==count||i%20==0;n.publishing_cycle();
      assert(n.node.runtime_->attitude_core().phase()!=Phase::RequestDisarm);
    }
    assert(n.node.attitude_pub_->calls==60&&n.node.mode_pub_->calls==60);
  }
  std::cout<<"LAND_EVENT_CONSECUTIVE_STRESS_CUSTODY_PASS\n";
}

static void healthy_tail_cannot_finish_disarmed_ground_in_an_unsafe_cycle(){
  for(const auto phase:{Phase::RequestDisarm,Phase::SafeDisarmed,Phase::ReleaseOffboard}){
    for(bool healthy_tail:{false,true}){
      LandNode n(phase);
      if(phase==Phase::RequestDisarm){
        // Make a current Disarmed status causally eligible for the committed
        // Disarm; the retained bad Land must still veto THIS cycle's terminal.
        for(unsigned i=0;i<3;++i)n.publishing_cycle();
        n.armed=false;
      }
      const bool verified=n.node.runtime_->pending(400).state_verified;
      if(healthy_tail)n.bad_land_head=true;
      else n.landed=n.rest_input=false;
      n.silent_cycle();
      assert(n.node.runtime_->attitude_core().phase()==phase);
      assert(n.node.runtime_->pending(400).state_verified==verified);
      assert(n.node.runtime_->rc_attempts_completed()==0);
    }
  }
  std::cout<<"LAND_EVENT_DISARMED_TERMINAL_AND_RELEASE_VETO_PASS\n";
}

static void a_fatal_queued_payload_still_stops_all_output(){
  LandNode n;n.bad_land_head=true;n.silent_cycle(1);
  assert(n.node.runtime_->attitude_core().phase()==Phase::Safety);
  assert(n.node.runtime_->attitude_core().reason()=="ATTITUDE_STALE_OR_INVALID");
  n.bad_land_head=false;
  for(unsigned i=0;i<20;++i)n.silent_cycle();
  assert(n.node.runtime_->attitude_core().phase()==Phase::Safety);
  std::cout<<"LAND_EVENT_FATAL_SAFETY_REMAINS_SILENT_PASS\n";
}

static void normal_landing_and_bounce_can_reprepare_after_new_ground(){
  for(bool bounce:{false,true}){
    LandNode n;
    if(bounce){n.landed=n.rest_input=false;n.publishing_cycle();}
    for(unsigned i=0;i<6;++i){n.deliver_land=i==0;n.publishing_cycle();}
    n.armed=false;n.offboard=false;n.mode_input=1;n.arm_input=3;n.landed=n.rest_input=true;
    for(unsigned i=0;i<200&&n.node.runtime_->attitude_core().phase()!=Phase::RcWaitArm;++i){
      n.deliver_land=i%20==0;n.cycle();
    }
    assert(n.node.runtime_->attitude_core().phase()==Phase::RcWaitArm);
    assert(n.node.runtime_->rc_attempts_completed()==1&&n.node.command_pub_->calls==0);
    assert(!n.node.runtime_->attitude_core().flight_started()&&n.node.last_attitude_output_.publish);
  }
  {
    LandNode n;
    for(unsigned i=0;i<6;++i)n.publishing_cycle();
    n.armed=false;
    for(unsigned i=0;i<20&&n.node.runtime_->attitude_core().phase()!=Phase::SafeDisarmed;++i)n.cycle();
    assert(n.node.runtime_->attitude_core().phase()==Phase::SafeDisarmed);
    assert(n.node.runtime_->pending(400).state_verified);
  }
  std::cout<<"LAND_EVENT_NORMAL_TERMINAL_AND_NEW_GROUND_RECOVERY_PASS\n";
}

int main(){
  waiting_for_disarm_keeps_its_zero_stream();
  bad_land_or_rest_keeps_armed_custody_with_both_tail_orders();
  repeated_land_changes_cannot_starve_the_descent_controller();
  healthy_tail_cannot_finish_disarmed_ground_in_an_unsafe_cycle();
  a_fatal_queued_payload_still_stops_all_output();
  normal_landing_and_bounce_can_reprepare_after_new_ground();
}
'''


# 编译临时C++夹具并逐项检查六组落地事件标记。
class LandEventAdapterTest(unittest.TestCase):
    # 等待Disarm、落地反跳、密集变化、终止否决、致命故障静默和新地面重准备必须全部完成。
    def test_land_events_preserve_custody_and_ground_completion_vetoes(self):
        with tempfile.TemporaryDirectory(prefix='fc01_land_event_') as directory:
            root = Path(directory)
            fixture.write_fixture(root)
            (root / 'test.cpp').write_text(HARNESS)
            subprocess.run(
                [
                    'g++',
                    '-std=c++17',
                    '-Wall',
                    '-Wextra',
                    '-Werror',
                    '-pedantic',
                    '-pthread',
                    '-I',
                    str(root),
                    '-I',
                    str(fixture.recovery.fixture.ROOT / 'src/jikl_fc01_system/include'),
                    '-I',
                    str(fixture.recovery.fixture.ROOT / 'tests'),
                    str(root / 'test.cpp'),
                    '-o',
                    str(root / 'test'),
                ],
                check=True,
            )
            result = subprocess.run(
                [str(root / 'test')],
                env=dict(os.environ, FC01_SOCKET=str(root / 'control.sock')),
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr + '\n' + result.stdout[-12000:])
            for marker in (
                'NORMAL_WAIT_DISARM',
                'ARMED_BOUNCE_BOTH_TAIL_ORDERS',
                'CONSECUTIVE_STRESS_CUSTODY',
                'DISARMED_TERMINAL_AND_RELEASE_VETO',
                'FATAL_SAFETY_REMAINS_SILENT',
                'NORMAL_TERMINAL_AND_NEW_GROUND_RECOVERY',
            ):
                self.assertIn('LAND_EVENT_' + marker + '_PASS', result.stdout)


if __name__ == '__main__':
    unittest.main()
