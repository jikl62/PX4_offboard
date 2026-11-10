#include "vision_test_support.hpp"
#include <iostream>
using namespace vision_test;
static v::Input current_visual(double now){
  v::Input in;in.ros_now_ns=1000000000000ULL+std::uint64_t(now*1e9);
  in.state_publishers=in.command_publishers=1;
  auto& s=in.state;s.sequence=std::uint64_t(now*1e6)+1;s.stamp_ns=in.ros_now_ns;s.received_at=now;
  s.frame_id="fc01_vision_ned";s.body_frame_id="base_link_frd";s.source_id="synthetic-stage-vision";
  s.initialized=s.tracking=s.calibration_valid=true;s.valid_fields=v::Position|v::Velocity|v::Attitude|v::Quality;
  s.position={0,0,-1.5};s.velocity={0,0,0};s.attitude={1,0,0,0};s.quality=1;
  auto& c=in.command;c.sequence=s.sequence;c.stamp_ns=in.ros_now_ns;c.received_at=now;
  c.source_id="synthetic-stage-command";c.frame_id=s.frame_id;c.valid=true;c.mode=v::Mode::Cruise;
  c.position_valid=true;c.position={.2,0,-1.5};c.sticks_valid=true;c.sticks={0,0,0,0};
  return in;
}
static void rc_navigation_at_one_point_five_waits_for_measured_liftoff_and_settling(){
  auto c=config();c.attitude.rc_target_height=1.5;VisionFlight q(c);
  q.set_visual(current_visual(.05));q.tick();
  need(!q.o.attitude.visual_motion&&!q.r.attitude_core().navigation_allowed(),"vision cannot initiate a task");
  q.join();const double reference=q.r.attitude_core().reference_z();
  const auto advance=[&]{q.set_visual(current_visual(q.t+.05));q.tick();};
  for(int i=0;i<50&&q.r.attitude_core().phase()!=a::Phase::ThrustRamp;++i)advance();
  need(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"real RC Runtime route reaches ramp");
  for(int i=0;i<8;++i){q.observe(.13+.01*i,.20);advance();
    need(!q.o.attitude.visual_motion&&!q.r.attitude_core().liftoff_confirmed(),
      "new height and rising samples with a landed detector cannot certify liftoff or enable vision");
  }
  const double command_before=q.o.attitude.target_height;
  for(int i=0;i<6;++i){
    q.s.attitude.rc_mode_slot=q.c.attitude.rc_navigation_mode_slot;
    need(!q.r.request("TAKEOFF",{1.5},q.s,q.t).accepted,"RC route continues rejecting console TAKEOFF");
    advance();
    need(q.r.attitude_core().reference_z()==reference&&q.o.attitude.target_height>=command_before,
      "duplicate navigation messages and rejected console input cannot rebase z0 or restart the ramp trajectory");
    need(!q.o.attitude.visual_motion&&!q.r.attitude_core().navigation_allowed(),"unconfirmed liftoff still blocks navigation");
  }
  q.s.feedback.landed=q.s.at_rest=false;
  for(int i=0;i<6;++i)advance();
  need(q.r.attitude_core().liftoff_confirmed()&&!q.r.attitude_core().navigation_allowed(),
    "two new detector samples join the sustained raw-local rise but do not bypass the 1.5 m goal");
  double height=.20;
  while(height<1.5-1e-9){height=std::min(1.5,height+.01);q.observe(height,.20);advance();
    need(!q.o.attitude.visual_motion&&!q.r.attitude_core().target_height_settled(),
      "ascending through valid vision inputs does not admit visual motion before target settling");
  }
  q.observe(1.5,0);advance();
  need(!q.r.attitude_core().navigation_allowed(),"one sample at 1.5 m cannot replace the settling dwell");
  for(int i=0;i<30&&q.r.attitude_core().phase()!=a::Phase::HeightHold;++i)advance();
  need(q.r.attitude_core().target_height_settled()&&q.r.attitude_core().navigation_allowed(),
    "the platform settles at the full 1.5 m relative goal before navigation admission");
  advance();need(q.o.attitude.visual_motion&&q.last_decision.allowed,"valid visual motion then reaches the original final sinks");
  need(q.r.attitude_core().reference_z()==reference&&q.arm_commands==0&&q.mode_commands==0,
    "the RC task preserves z0 and never emits automatic Arm or Offboard commands");
  // The final sink resamples feedback after planning. A detector contradiction
  // at that boundary must block an already planned visual target too.
  auto contradictory=q.s;contradictory.feedback.landed=true;
  const auto boundary=a::attitude_publication_policy(q.r.publication_context(contradictory,q.t,true,true),q.o.attitude);
  need(!boundary.allowed,"a post-plan landed contradiction must revoke visual publication");
  q.s.feedback.landed=true;advance();
  need(!q.r.attitude_core().navigation_allowed()&&!q.o.attitude.visual_motion,
    "a new contradictory detector after settling must revoke navigation immediately");
  need(q.r.attitude_core().phase()==a::Phase::Descent&&q.o.attitude.publish&&q.last_decision.allowed&&
    q.o.attitude.thrust[2]<0&&q.o.attitude.command!=a::Command::Disarm,
    "contradiction retains supervised descent and cannot assert safe ground or cut motor power");
}
int main(){
  rc_navigation_at_one_point_five_waits_for_measured_liftoff_and_settling();
  std::cout<<"RC_TAKEOFF_1P5_STAGE_PASS\n";
}
