#include "jikl_fc01_system/vision_navigation.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
namespace v=fc01::vision;
namespace a=fc01::attitude;
static constexpr double pi=3.14159265358979323846;
static void require(bool condition,const char* reason){if(!condition)throw std::runtime_error(reason);}
static bool near(double x,double y,double tolerance=1e-9){return std::abs(x-y)<tolerance;}
static v::Config config(){v::Config c;c.enabled=true;return c;}
static v::Input input(){
  v::Input in;in.ros_now_ns=100000000000ULL;in.state_publishers=in.command_publishers=1;
  auto& s=in.state;s.sequence=1;s.stamp_ns=in.ros_now_ns;s.received_at=10;
  s.frame_id="fc01_vision_ned";s.body_frame_id="base_link_frd";s.source_id="camera-A";
  s.initialized=s.tracking=s.calibration_valid=true;
  s.valid_fields=v::Position|v::Velocity|v::Attitude|v::Quality;
  s.position={0,0,0};s.velocity={0,0,0};s.attitude={1,0,0,0};s.quality=.9;
  auto& c=in.command;c.sequence=1;c.stamp_ns=in.ros_now_ns;c.received_at=10;
  c.source_id="navigation-client";c.frame_id=s.frame_id;c.valid=true;c.mode=v::Mode::Hold;
  return in;
}
static void advance(v::Input& in,double seconds=.05,bool refresh_command=true){
  const auto ns=static_cast<std::uint64_t>(std::llround(seconds*1e9));
  in.ros_now_ns+=ns;++in.state.sequence;in.state.stamp_ns+=ns;in.state.received_at+=seconds;
  if(refresh_command){++in.command.sequence;in.command.stamp_ns+=ns;in.command.received_at+=seconds;}
}
static v::Result step(v::Controller& c,const v::Input& in,double height=.5,double yaw=0){
  return c.step(in,in.state.received_at,height,yaw,.05);
}
static double yaw_of(a::Quaternion q){return std::atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));}
static double tilt_of(a::Quaternion q){return std::acos(std::clamp(1-2*(q[1]*q[1]+q[2]*q[2]),-1.,1.));}
// Catches accidental activation without valid metric state/command and NaN-as-zero defaults.
static void waiting_and_validity(){
  const v::State empty;require(std::isnan(empty.position[0])&&std::isnan(empty.attitude[0])&&std::isnan(empty.quality),"missing measurements must remain NaN");
  v::Controller waiting(config());v::Input absent;auto o=waiting.step(absent,10,.5,0,.05);
  require(!o.active&&!o.request_land,"missing inputs before activation must wait");
  auto in=input();o=step(waiting,in);require(o.active&&!o.request_land,"valid Hold must activate");
  require(waiting.current(in,10),"current accepted plan must be usable");
  require(near(o.target_height,.5)&&near(o.yaw,0),"Hold captures current height/yaw");
  require(std::isnan(in.state.angular_velocity[0])&&in.state.angular_velocity_source.empty(),"missing angular velocity is explicit");
  v::Controller disabled;require(!step(disabled,in).active,"default disabled feature must not activate");
  for(int field=0;field<4;++field){
    auto bad=input();bad.state.valid_fields=static_cast<std::uint16_t>(bad.state.valid_fields&~(1U<< (field==3?6:field)));
    v::Controller c(config());require(!step(c,bad).active,"missing required validity bit cannot activate");
  }
}
// Catches coordinate sign inversions and a fake controller that ignores measured velocity.
static void cruise_and_feedback(){
  auto in=input();in.command.mode=v::Mode::Cruise;in.command.position_valid=true;in.command.position={.5,0,-1};
  v::Controller c(config());const auto o=step(c,in);
  require(o.active&&near(o.velocity_target[0],.4),"position P must produce north velocity target");
  require(near(o.acceleration_target[0],.48)&&near(o.acceleration_target[1],0),"velocity P must use measured velocity");
  require(o.q[2]<0&&std::abs(o.q[1])<1e-12,"north acceleration requires negative pitch in FRD/NED");
  require(near(o.target_height,1.5),"absolute vision z maps error to PX4 relative height");
  advance(in);in.state.velocity[0]=.3;const auto braking=step(c,in);
  require(braking.acceleration_target[0]<o.acceleration_target[0],"measured north velocity must reduce north acceleration");
  require(tilt_of(braking.q)<=5*pi/180+1e-12,"motion tilt candidate limit must hold");
}
// Catches Hold recapturing its target every state/command update and wrong source of z feedback.
static void hold_is_stable(){
  auto in=input();v::Controller c(config());require(step(c,in).active,"hold activates");
  advance(in);in.state.position={.2,-.1,-.1};const auto o=step(c,in,.6);
  require(o.velocity_target[0]<0&&o.velocity_target[1]>0,"Hold must return toward original captured XY");
  require(near(o.target_height,.5),"vision z error maps to current PX4 height");
  auto modified=in;modified.command.mode=v::Mode::Follow;modified.command.sticks_valid=true;modified.command.sticks={0,0,0,0};
  require(!c.current(modified,in.state.received_at),"same-sequence mode mutation cannot authorize a new plan");
  const auto bad=step(c,modified,.6);require(bad.request_land&&!bad.active,"mode mutation without a new command must invalidate active task");
}
// Catches roll/pitch/throttle/yaw mapping errors and diagonal speed-limit bypass.
static void follow_axes(){
  for(int axis=0;axis<4;++axis){
    auto in=input();in.command.mode=v::Mode::Follow;in.command.sticks_valid=true;in.command.sticks={0,0,0,0};in.command.sticks[axis]=1;
    v::Controller c(config());const auto o=step(c,in);require(o.active,"valid Follow activates");
    if(axis==0)require(near(o.velocity_target[1],.5)&&o.q[1]>0,"positive roll requests body right/east and positive roll");
    if(axis==1)require(near(o.velocity_target[0],.5)&&o.q[2]<0,"positive pitch requests body forward/north");
    if(axis==2)require(near(o.yaw,.0175)&&near(yaw_of(o.q),.0175),"positive yaw requests right/positive NED yaw");
    if(axis==3)require(near(o.target_height,.5075)&&near(o.velocity_target[2],-.15),"positive throttle moves height target upward");
  }
  auto in=input();in.state.attitude={std::sqrt(.5),0,0,std::sqrt(.5)};
  in.command.mode=v::Mode::Follow;in.command.sticks_valid=true;in.command.sticks={0,1,0,0};
  v::Controller c(config());const auto east=step(c,in);
  require(std::abs(east.velocity_target[0])<1e-12&&near(east.velocity_target[1],.5),"body forward must rotate using measured vision yaw");
  advance(in);in.command.sticks={1,1,0,0};const auto diagonal=step(c,in);
  require(std::hypot(diagonal.velocity_target[0],diagonal.velocity_target[1])<=.5+1e-12,"diagonal request obeys vector speed bound");
  require(std::hypot(diagonal.acceleration_target[0],diagonal.acceleration_target[1])<=.6+1e-12,"diagonal acceleration obeys vector bound");
}
// Catches duplicate packets refreshing leases and malformed new measurements retaining old validity.
static void inbox_replay_and_invalidation(){
  auto in=input();v::Inbox box;require(box.state(in.state,10)&&box.command(in.command,10),"initial inbox samples accepted");
  require(!box.state(in.state,10.2)&&!box.command(in.command,10.2),"duplicates must not refresh receive times");
  auto got=box.snapshot(in.ros_now_ns,1,1);require(near(got.state.received_at,10)&&near(got.command.received_at,10),"duplicate time remains original");
  auto old=in.state;old.sequence=0;require(!box.state(old,10.2),"older sequence rejected");
  auto bad=in.state;++bad.sequence;bad.valid_fields=0;bad.position={0,0,0};require(box.state(bad,10.1),"new invalid state replaces old state");
  got=box.snapshot(in.ros_now_ns,1,1);require(got.state.valid_fields==0&&std::isnan(got.state.position[0]),"missing masked payload cleared to NaN");
  v::Controller c(config());require(step(c,in).active,"activate before invalid sample");
  require(c.step(got,10.1,.5,0,.05).request_land,"new invalid message revokes active state");
}
// Catches checking only an old active flag at the final publication boundary.
static void current_revalidates_both_clocks_and_identity(){
  auto in=input();v::Controller c(config());require(step(c,in).active,"valid plan activates");
  auto changed=in;changed.state.position[0]=.1;require(!c.current(changed,10),"same identity with mutated payload rejected");
  changed=in;++changed.state.sequence;require(!c.current(changed,10),"unplanned state generation rejected at final sink");
  changed=in;++changed.command.sequence;require(!c.current(changed,10),"unplanned command generation rejected at final sink");
  changed=in;changed.state_publishers=2;require(!c.current(changed,10),"final source conflict rejects plan");
  changed=in;changed.ros_now_ns+=220000000;require(!c.current(changed,10.01),"excess arrival transport delay rejects plan even with fresh receive age");
  require(!c.current(in,10.26),"steady state TTL expiry rejects plan even with unchanged ROS clock");
  require(!c.current(in,9.99),"steady clock regression rejects plan");
  changed=in;changed.ros_now_ns-=60000000;require(!c.current(changed,10),"future source time rejects plan");
}
// Catches automatic recovery/reacquisition after stale data, source/frame changes or estimator resets.
static void active_faults_latch_land(){
  for(int fault=0;fault<16;++fault){
    auto in=input();v::Controller c(config());require(step(c,in).active,"activate fixture");advance(in);
    if(fault==0)in.state.quality=.49;
    if(fault==1)in.state.calibration_valid=false;
    if(fault==2)in.state.tracking=false;
    if(fault==3)in.state.source_id="camera-B";
    if(fault==4)in.command.source_id="other-client";
    if(fault==5)in.state.frame_id="map";
    if(fault==6)in.state.body_frame_id="base_link";
    if(fault==7)++in.state.reset_counter;
    if(fault==8)in.state.position[0]=.751;
    if(fault==9)in.state.velocity[0]=1.001;
    if(fault==10)in.state.attitude={0,0,0,0};
    if(fault==11)in.command.mode=v::Mode::Disabled;
    if(fault==12)in.command.mode=v::Mode::Land;
    if(fault==13)in.state_publishers=0;
    if(fault==14)in.state.position[0]=std::numeric_limits<double>::quiet_NaN();
    if(fault==15)in.command.reset_counter=1;
    const auto o=step(c,in);require(o.request_land&&!o.active,"active input failure must request native landing");
    in=input();advance(in);advance(in);const auto resumed=step(c,in);
    require(resumed.request_land&&!resumed.active&&!c.current(in,in.state.received_at),"fresh data must not reacquire after landing latch");
  }
  auto in=input();v::Controller c(config());require(step(c,in).active,"activation");
  auto stale=c.step(in,10.26,.5,0,.05);require(stale.request_land&&!stale.active,"stale active state lands");
  v::Controller stopped(config());require(step(stopped,in).active,"stop fixture activates");stopped.stop();advance(in);
  require(!step(stopped,in).active,"stop permanently removes current motion permission");
}
// Catches optional angular velocity being silently accepted without provenance.
static void optional_fields_and_limits(){
  for(int issue=0;issue<4;++issue){
    auto in=input();in.state.valid_fields|=v::AngularVelocity;in.state.angular_velocity={0,0,0};in.state.angular_velocity_source="vision-derived";
    if(issue==0)in.state.angular_velocity_source.clear();
    if(issue==1)in.state.angular_velocity[1]=std::numeric_limits<double>::infinity();
    if(issue==2)in.command.mode=static_cast<v::Mode>(77);
    if(issue==3){in.command.mode=v::Mode::Follow;in.command.sticks_valid=true;in.command.sticks={1.01,0,0,0};}
    v::Controller c(config());require(!step(c,in).active,"invalid provenance/enum/axis cannot activate");
  }
  auto in=input();in.command.mode=v::Mode::Cruise;in.command.position_valid=true;in.command.position={2.01,0,0};
  v::Controller outside(config());require(!step(outside,in).active,"outside radius point target rejected");
  auto invalid=config();invalid.position_gain=0;bool threw=false;try{v::Controller c(invalid);}catch(const std::invalid_argument&){threw=true;}
  require(threw,"zero candidate controller gain rejected");
}
// Catches a controller which emits plausible setpoints without closing position feedback.
// This is a gravity/rotation ideal plant, not PX4 SITL or airframe calibration.
static void ideal_xy_plant_returns_to_point_after_disturbance(){
  auto in=input();in.command.mode=v::Mode::Cruise;in.command.position_valid=true;in.command.position={.75,-.4,0};
  v::Controller c(config());double x=0,y=0,vx=0,vy=0;
  for(int i=0;i<240;++i){
    in.state.position={x,y,0};in.state.velocity={vx,vy,0};const auto o=step(c,in);
    require(o.active&&!o.request_land,"nominal ideal XY plant must retain navigation permission");
    require(c.current(in,in.state.received_at),"plant plan is current at both simulated sinks");
    const auto& q=o.q;
    const double r13=2*(q[1]*q[3]+q[0]*q[2]);
    const double r23=2*(q[2]*q[3]-q[0]*q[1]);
    const double r33=1-2*(q[1]*q[1]+q[2]*q[2]);
    // Vertical thrust balances gravity; its horizontal component follows R(q).
    vx+=(-9.80665*r13/r33)*.05;vy+=(-9.80665*r23/r33)*.05;x+=vx*.05;y+=vy*.05;
    if(i==80){x+=.15;y-=.1;}
    advance(in);
  }
  require(std::hypot(x-.75,y+.4)<.015,"closed loop must remove position error after a real state disturbance");
  require(std::hypot(vx,vy)<.015,"closed loop must settle velocity, not just pass through the point");
}
// Catches independent command TTL being accidentally renewed by state traffic.
static void independent_leases_and_sample_progress(){
  auto in=input();v::Controller c(config());require(step(c,in).active,"lease fixture activates");
  advance(in,.31,false);const auto expired=step(c,in);
  require(expired.request_land,"fresh state cannot renew the command lease");
  in=input();v::Controller state_frozen(config());require(step(state_frozen,in).active,"state-clock fixture activates");
  ++in.state.sequence;in.state.received_at+=.05;in.command.received_at+=.05;in.ros_now_ns+=50000000;
  const auto frozen=step(state_frozen,in);require(frozen.request_land,"new sequence cannot refresh an unchanged raw measurement stamp");
  in=input();v::Controller epoch(config());require(step(epoch,in).active,"epoch fixture activates");advance(in);
  ++in.state.reset_counter;++in.command.reset_counter;
  require(step(epoch,in).request_land,"coherent state/command reset still invalidates active frame anchor");
  in=input();v::Inbox inbox;require(inbox.state(in.state,10)&&inbox.command(in.command,10),"command inbox fixture activates");
  auto cmd=in.command;++cmd.sequence;cmd.valid=false;require(inbox.command(cmd,10.05),"new invalid command replaces old valid command");
  require(!inbox.snapshot(in.ros_now_ns,1,1).command.valid,"old command validity cannot survive new invalid command");
}
// Catches treating an arrival-delay cap as a shorter replacement for the fixed TTL.
static void arrival_delay_consumes_ttl_without_replacing_it(){
  auto in=input();v::Controller c(config());require(step(c,in).active,"fixed TTL fixture activates");
  auto later=in;later.ros_now_ns+=220000000;
  require(c.current(later,10.22),"a zero-delay sample remains valid between max arrival delay and its state TTL");
  later.ros_now_ns=in.ros_now_ns+250000000;
  require(!c.current(later,10.25),"state source lifetime ends at fixed TTL, not receipt renewal");
  in=input();in.state.stamp_ns-=180000000;in.command.stamp_ns-=180000000;
  v::Controller delayed(config());require(step(delayed,in).active,"180ms transport delay is within arrival cap");
  later=in;later.ros_now_ns+=80000000;
  require(!delayed.current(later,10.08),"transport delay consumes remaining source TTL despite short receive age");
  in=input();in.state.stamp_ns-=210000000;in.command.stamp_ns-=210000000;
  v::Controller too_late(config());require(!step(too_late,in).active,"210ms arrival delay exceeds transport cap before activation");
}
// Catches sequence filtering hiding restarted source/epoch/frame revocation,
// including recovery arriving before the control loop has observed the fault.
static void inbox_identity_revocation_precedes_sequence_deduplication(){
  for(int change=0;change<5;++change){
    auto in=input();in.state.sequence=in.command.sequence=100;v::Inbox inbox;
    require(inbox.state(in.state,10)&&inbox.command(in.command,10),"identity fixture enters inbox");
    v::Controller c(config());const auto accepted=inbox.snapshot(in.ros_now_ns,1,1);require(step(c,accepted).active,"identity fixture activates");
    auto restarted=in.state;restarted.sequence=1;restarted.stamp_ns+=1000000;
    if(change==0)++restarted.reset_counter;
    if(change==1)restarted.source_id="restarted-source";
    if(change==2)restarted.frame_id="another_ned";
    if(change==3)restarted.body_frame_id="another_frd";
    if(change==4){++restarted.reset_counter;restarted.source_id="restarted-source";}
    require(inbox.state(restarted,10.001),"identity change with restarted sequence must be retained as revocation evidence");
    auto revoked=inbox.snapshot(in.ros_now_ns+1000000,1,1);
    require(!c.current(revoked,10.001),"identity change immediately invalidates current plan before any control step");
    auto old=in.state;old.sequence=101;old.stamp_ns+=2000000;
    inbox.state(old,10.002);
    auto recovered=restarted;recovered.sequence=102;recovered.stamp_ns+=2000000;
    inbox.state(recovered,10.003);
    revoked=inbox.snapshot(in.ros_now_ns+3000000,1,1);
    require(revoked.revocation_generation>accepted.revocation_generation,"revocation history must survive source restoration");
    require(!c.current(revoked,10.003),"valid traffic must not erase unobserved revocation evidence");
    const auto landing=c.step(revoked,10.003,.5,0,.003);require(landing.request_land&&!landing.active,"retained identity revocation must request native landing");
  }
  for(int change=0;change<3;++change){
    auto in=input();in.state.sequence=in.command.sequence=100;v::Inbox inbox;
    require(inbox.state(in.state,10)&&inbox.command(in.command,10),"command identity fixture enters inbox");
    v::Controller c(config());require(step(c,inbox.snapshot(in.ros_now_ns,1,1)).active,"command identity fixture activates");
    auto restarted=in.command;restarted.sequence=1;restarted.stamp_ns+=1000000;
    if(change==0)++restarted.reset_counter;
    if(change==1)restarted.source_id="restarted-client";
    if(change==2)restarted.frame_id="another_ned";
    require(inbox.command(restarted,10.001),"command identity change precedes old sequence filtering");
    require(!c.current(inbox.snapshot(in.ros_now_ns+1000000,1,1),10.001),"command identity change immediately revokes plan");
    auto old=in.command;old.sequence=101;old.stamp_ns+=2000000;inbox.command(old,10.002);
    const auto landing=c.step(inbox.snapshot(in.ros_now_ns+2000000,1,1),10.002,.5,0,.002);
    require(landing.request_land&&!landing.active,"late old command epoch cannot recover control");
  }
}
// Catches explicit fresh health withdrawal being swallowed by a reused sequence,
// while preserving normal same-stream duplicate and out-of-order behavior.
static void inbox_health_revocation_is_sticky_without_renewing_old_samples(){
  for(int change=0;change<5;++change){
    auto in=input();in.state.sequence=in.command.sequence=100;v::Inbox inbox;
    require(inbox.state(in.state,10)&&inbox.command(in.command,10),"health fixture enters inbox");
    v::Controller c(config());require(step(c,inbox.snapshot(in.ros_now_ns,1,1)).active,"health fixture activates");
    auto withdrawn=in.state;withdrawn.sequence=1;withdrawn.stamp_ns+=1000000;
    if(change==0)withdrawn.tracking=false;
    if(change==1)withdrawn.initialized=false;
    if(change==2)withdrawn.calibration_valid=false;
    if(change==3)withdrawn.valid_fields=0;
    if(change==4)withdrawn.position[0]=std::numeric_limits<double>::quiet_NaN();
    require(inbox.state(withdrawn,10.001),"fresh health withdrawal must survive repeated/restarted sequence");
    auto valid=in.state;valid.sequence=101;valid.stamp_ns+=2000000;inbox.state(valid,10.002);
    const auto revoked=inbox.snapshot(in.ros_now_ns+2000000,1,1);
    require(!c.current(revoked,10.002)&&c.step(revoked,10.002,.5,0,.002).request_land,"health recovery before step cannot cancel withdrawal");
  }
  auto in=input();in.state.sequence=in.command.sequence=100;v::Inbox inbox;
  require(inbox.state(in.state,10)&&inbox.command(in.command,10),"normal ordering fixture enters inbox");
  auto late=in.state;late.sequence=99;late.stamp_ns-=1000000;late.tracking=false;
  require(!inbox.state(late,10.001),"a genuinely older same-stream health packet remains ignored");
  require(!inbox.state(in.state,10.002)&&near(inbox.snapshot(in.ros_now_ns,1,1).state.received_at,10),"same-stream duplicate still cannot renew state TTL");
  v::Controller c(config());require(step(c,inbox.snapshot(in.ros_now_ns,1,1)).active,"ignored old health packet does not revoke current metric state");
  auto withdrawn=in.command;withdrawn.sequence=1;withdrawn.stamp_ns+=1000000;withdrawn.valid=false;
  require(inbox.command(withdrawn,10.001),"fresh command validity withdrawal precedes seq filter");
  auto valid=in.command;valid.sequence=101;valid.stamp_ns+=2000000;inbox.command(valid,10.002);
  require(c.step(inbox.snapshot(in.ros_now_ns+2000000,1,1),10.002,.5,0,.002).request_land,"command validity withdrawal is retained until landing latch observes it");
}
// Catches making revocation sticky in the Inbox itself, which would prevent
// a normal camera initialization before the controller has accepted a plan.
static void inbox_initialization_can_recover_before_first_plan(){
  for(bool changed_identity:{false,true}){
    auto in=input();auto initial=in;initial.state.initialized=false;initial.state.sequence=100;initial.command.sequence=100;
    if(changed_identity){
      initial.state.source_id="starting-camera";initial.state.frame_id="unknown";initial.state.body_frame_id="unknown";
      initial.command.valid=false;initial.command.source_id="starting-client";initial.command.frame_id="unknown";
    }
    v::Inbox inbox;require(inbox.state(initial.state,10)&&inbox.command(initial.command,10),"uninitialized samples retained as waiting state");
    v::Controller c(config());const auto waiting=c.step(inbox.snapshot(in.ros_now_ns,1,1),10,.5,0,.05);
    require(!waiting.active&&!waiting.request_land,"pre-activation invalid startup must wait without landing latch");
    in.state.sequence=in.command.sequence=changed_identity?1:101;
    in.state.stamp_ns+=1000000;in.command.stamp_ns+=1000000;in.ros_now_ns+=1000000;
    require(inbox.state(in.state,10.001)&&inbox.command(in.command,10.001),"healthy initialized input can replace startup state");
    const auto healthy=inbox.snapshot(in.ros_now_ns,1,1);const auto started=c.step(healthy,10.001,.5,0,.05);
    require(started.active&&!started.request_land&&c.current(healthy,10.001),"initialization recovery before first plan must activate normally");
  }
  auto in=input();in.state.sequence=in.command.sequence=100;v::Inbox inbox;
  require(inbox.state(in.state,10)&&inbox.command(in.command,10),"exact restoration fixture enters inbox");
  const auto original=inbox.snapshot(in.ros_now_ns,1,1);v::Controller c(config());require(step(c,original).active,"exact restoration fixture activates");
  auto another=in.state;another.source_id="another-source";another.sequence=1;inbox.state(another,10);
  require(inbox.state(in.state,10),"same source payload can be restored after a distinct stream");
  const auto restored=inbox.snapshot(in.ros_now_ns,1,1);
  require(restored.state.sequence==original.state.sequence&&restored.state.source_id==original.state.source_id,"payload and source are restored exactly");
  require(!c.current(restored,10)&&c.step(restored,10,.5,0,.05).request_land,"local generation detects exact A-to-B-to-A even at the same receipt time");
}
// Catches long-way yaw wrapping, incorrect direction outside yaw=0, and optional validity shortcuts.
static void yaw_wrap_all_quadrants_and_covariance(){
  auto in=input();in.command.mode=v::Mode::Cruise;in.command.position_valid=true;in.command.position={0,0,0};
  in.command.yaw_valid=true;in.command.yaw=-pi+.02;v::Controller wrapped(config());const auto o=step(wrapped,in,.5,pi-.01);
  require(near(o.yaw,-pi+.0075),"yaw must turn the short positive direction across pi");
  const double headings[]={0,pi/2,pi,-pi/2};
  const double want_north[]={.5,0,-.5,0},want_east[]={0,.5,0,-.5};
  for(int i=0;i<4;++i){
    in=input();in.state.attitude={std::cos(headings[i]/2),0,0,std::sin(headings[i]/2)};
    in.command.mode=v::Mode::Follow;in.command.sticks_valid=true;in.command.sticks={0,1,0,0};
    v::Controller c(config());const auto turn=step(c,in);
    require(near(turn.velocity_target[0],want_north[i])&&near(turn.velocity_target[1],want_east[i]),"all yaw quadrants must map body forward to the expected NED axis");
  }
  in=input();in.state.valid_fields|=v::PoseCovariance;in.state.pose_covariance.fill(0);in.state.pose_covariance[7]=-.1;
  v::Controller covariance(config());require(!step(covariance,in).active,"marked-valid covariance cannot contain negative variance");
  in=input();in.command.mode=v::Mode::Land;in.state={};v::Controller land(config());
  require(land.step(in,10,.5,0,.05).request_land,"explicit fresh Land needs no valid vision estimate");
}
int main(){
  waiting_and_validity();cruise_and_feedback();hold_is_stable();follow_axes();inbox_replay_and_invalidation();
  current_revalidates_both_clocks_and_identity();active_faults_latch_land();optional_fields_and_limits();
  ideal_xy_plant_returns_to_point_after_disturbance();independent_leases_and_sample_progress();yaw_wrap_all_quadrants_and_covariance();
  arrival_delay_consumes_ttl_without_replacing_it();
  inbox_identity_revocation_precedes_sequence_deduplication();inbox_health_revocation_is_sticky_without_renewing_old_samples();
  inbox_initialization_can_recover_before_first_plan();
  std::cout<<"VISION_NAVIGATION_INBOX_CLOCKS_VALIDITY_XY_FEEDBACK_FOUR_AXES_FINAL_CURRENT_LANDING_LATCH_PASS\n";
}
