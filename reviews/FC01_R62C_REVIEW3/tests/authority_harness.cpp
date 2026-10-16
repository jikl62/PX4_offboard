// G2 supplement: actual delivered Runtime/Core/final sinks, synthetic cached
// facts and fake ACK/status transport. No ROS, sockets, devices or dynamics.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <functional>
#include <iostream>
#include <sstream>
using namespace authority_test;
struct Mode {std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};};
struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
class Harness {
 public:
  explicit Harness(bool confirmed=true):c(configuration(confirmed)),r(c),s(sample()){}
  f::Config c;f::Runtime r;f::Snapshot s;double t{};bool connected{true},latched{};
  int arms{},targets{},modes{},nonzero{},offboard{},disarms{};
  std::string block_reason;std::vector<std::string> phases;
  static f::Config configuration(bool confirmed){auto c=config();
    c.attitude.autonomous_failsafe_confirmed=c.attitude.rc_takeover_confirmed=confirmed;return c;}
  void tick(){
    t+=.05;s.status_stamp+=1000;s.land_stamp+=1000;
    if(connected)r.request("PING",{},s,t);
    auto out=r.step(s,t);if(latched)out={};
    auto context=r.publication_context(s,t,connected,true);
    if(out.attitude.publish){auto d=a::attitude_publication_policy(context,out.attitude);
      if(!d.allowed){block(d);out={};}}
    if(out.attitude.command==a::Command::Offboard)++offboard;
    if(out.attitude.command==a::Command::Arm){auto d=a::publish_arm_at_boundary(context,[&]{++arms;});
      if(!d.allowed){block(d);out={};}}
    if(out.attitude.command==a::Command::Disarm)++disarms;
    if(out.attitude.publish){auto d=a::publish_attitude_at_boundary<Mode,Target>(context,out.attitude,std::uint64_t(s.status_stamp),
      [&](const Target& target){++targets;bool nz=false;for(float value:target.thrust_body)nz=nz||value!=0;
        if(nz)++nonzero;else for(float value:target.thrust_body)require(!std::signbit(value),"noncanonical zero");},
      [&](const Mode& mode){++modes;require(mode.attitude&&!mode.position&&!mode.velocity,"attitude ownership");});
      if(!d.allowed)block(d);}
    if(phases.empty()||phases.back()!=r.phase_name())phases.push_back(r.phase_name());
  }
  void ready(const char* choice=nullptr){
    require(r.request("INTERACTIVE",{},s,t).accepted,"live session");
    if(choice)require(r.request(choice,{},s,t).accepted,"selection");
    require(r.request("ATTITUDE_READY",{},s,t).accepted,"prepare");
    for(int i=0;i<40&&offboard==0;++i)tick();
    require(offboard==1,"exactly one offboard request");
    r.ack({r.pending(176).baseline+1,176,0,1,191,false},t+.001);
    s.feedback.offboard=true;s.nav_state=14;tick();
    require(r.attitude_core().phase()==a::Phase::OffboardReady,"ready ACK+status");
  }
  void arm(const char* choice){
    ready(choice);require(r.request("TAKEOFF",{1},s,t).accepted,"policy Arm admission");
    tick();require(arms==1,"final synthetic Arm sink");
    r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.001);
    s.feedback.armed=true;tick();require(r.attitude_core().phase()==a::Phase::ArmedGround,"Arm ACK+new status");
  }
  void stopped(){
    const int old_targets=targets,old_modes=modes,old_offboard=offboard;
    const int old_nonzero=nonzero,old_arms=arms;
    s=sample();s.feedback.offboard=true;s.nav_state=14;
    s.attitude.rc_link_ready=s.attitude.rc_switches_ready=true;
    for(int i=0;i<8;++i)tick();
    if(r.attitude_core().phase()==a::Phase::ReleaseOffboard){
      // During disarmed release, only canonical zero heartbeat is permitted.
      require(targets>=old_targets&&targets-old_targets==modes-old_modes,
        "release zero heartbeat must remain paired");
      require(nonzero==old_nonzero&&arms==old_arms&&offboard==old_offboard,
        "ground release cannot emit thrust, Arm or Offboard reentry");
    }else require(targets==old_targets&&modes==old_modes&&offboard==old_offboard,
      "no recovery publication or reacquisition");
    require(!r.request("ATTITUDE_READY",{},s,t).accepted,"fresh input cannot clear an attempt");
  }
  void block(a::Decision d){latched=true;block_reason=a::name(d.reason);r.abort_attitude("PUBLISHER_BOUNDARY_"+block_reason);}
};
struct Result {std::string name,error;bool pass{};int arms{},targets{},modes{},nonzero{},offboard{};std::vector<std::string> phases;};
int main(){
  std::vector<Result> results;
  auto add=[&](const std::string& name,const std::function<void(Harness&)>& body,bool confirmed=true){
    Result result;result.name=name;Harness h(confirmed);
    try{body(h);result.pass=true;}catch(const std::exception& e){result.error=e.what();}
    result.arms=h.arms;result.targets=h.targets;result.modes=h.modes;result.nonzero=h.nonzero;
    result.offboard=h.offboard;result.phases=h.phases;results.push_back(result);
  };
  add("unset_stage_one_exact_zero",[](Harness& h){h.ready();require(h.arms==0&&h.nonzero==0&&h.targets==h.modes,"stage-one paired zero");});
  add("unset_takeoff_denied",[](Harness& h){h.ready();auto reply=h.r.request("TAKEOFF",{1},h.s,h.t);
    require(!reply.accepted&&reply.message=="AUTHORITY_MODE_UNSET","UNSET upstream denial");h.tick();require(h.arms==0,"no Arm sink");});
  add("live_selection_required",[](Harness& h){require(!h.r.request("AUTHORITY_RC",{},h.s,0).accepted,"one-shot selection denied");});
  add("selection_output_busy",[](Harness& h){h.r.request("INTERACTIVE",{},h.s,0);h.s.offboard_heartbeat_active=true;
    require(!h.r.request("AUTHORITY_AUTONOMOUS",{},h.s,0).accepted,"heartbeat-active selection denied");});
  add("preparation_locks_selection",[](Harness& h){h.ready("AUTHORITY_AUTONOMOUS");
    require(h.r.request("AUTHORITY_RC",{},h.s,h.t).message=="AUTHORITY_MODE_LOCKED","locked selector");});
  add("rc_missing_link_denied",[](Harness& h){h.ready("AUTHORITY_RC");require(!h.r.request("TAKEOFF",{1},h.s,h.t).accepted,"missing RC denied");h.tick();require(h.arms==0,"no Arm");});
  add("rc_contract_unconfirmed",[](Harness& h){h.s.attitude.rc_link_ready=h.s.attitude.rc_switches_ready=true;h.ready("AUTHORITY_RC");
    require(h.r.request("TAKEOFF",{1},h.s,h.t).message=="RC_TAKEOVER_CONTRACT_UNCONFIRMED","independent RC confirmation");},false);
  add("autonomous_contract_unconfirmed",[](Harness& h){h.ready("AUTHORITY_AUTONOMOUS");
    require(h.r.request("TAKEOFF",{1},h.s,h.t).message=="AUTONOMOUS_FAILSAFE_UNCONFIRMED","independent autonomous confirmation");},false);
  add("rc_synthetic_arm_and_thrust",[](Harness& h){h.s.attitude.rc_link_ready=h.s.attitude.rc_switches_ready=true;h.arm("AUTHORITY_RC");for(int i=0;i<8;++i)h.tick();
    require(h.arms==1&&h.nonzero>0,"complete RC policy allows fake sinks");});
  add("autonomous_without_rc_synthetic_arm_and_thrust",[](Harness& h){h.arm("AUTHORITY_AUTONOMOUS");for(int i=0;i<8;++i)h.tick();
    require(!h.s.manual_fresh&&!h.s.attitude.rc_link_ready&&h.arms==1&&h.nonzero>0,"complete autonomous allows without RC");});
  for(int event=0;event<7;++event){const char* names[]={"nav_departure","operator_event","rc_loss","kill","failsafe","heartbeat_loss","switch_change"};
    add(std::string("takeover_")+names[event],[event](Harness& h){
      h.s.attitude.rc_link_ready=h.s.attitude.rc_switches_ready=true;h.arm("AUTHORITY_RC");
      switch(event){case 0:h.s.feedback.offboard=false;h.s.nav_state=15;break;case 1:h.s.attitude.operator_takeover=true;break;
        case 2:h.s.attitude.rc_link_ready=false;break;case 3:h.s.attitude.kill=true;break;case 4:h.s.feedback.failsafe=true;break;
        case 5:h.s.attitude.offboard_signal_lost=true;break;case 6:++h.s.attitude.operator_switch_generation;break;}
      h.tick();
      if(event==1||event==6){
        require(h.r.attitude_core().operator_release_requested()&&!h.r.attitude_core().authority_state().takeover_latched,
          "operator request must not masquerade as PX4 handover");
        h.s.feedback.offboard=false;h.s.nav_state=15;h.tick();
      }
      require(h.r.attitude_core().authority_state().takeover_latched,"takeover latch after evidence / Kill / failsafe");h.stopped();
    });
  }
  add("explicit_live_takeover_command",[](Harness& h){h.arm("AUTHORITY_AUTONOMOUS");
    require(h.r.request("AUTHORITY_TAKEOVER",{},h.s,h.t).accepted,"takeover request accepted");
    h.tick();require(h.r.attitude_core().operator_release_requested()&&!h.r.attitude_core().authority_state().takeover_latched,
      "pending takeover may not assert transfer");
    h.s.feedback.offboard=false;h.s.nav_state=15;h.tick();h.stopped();});
  add("pending_arm_disconnect",[](Harness& h){h.ready("AUTHORITY_AUTONOMOUS");h.r.request("TAKEOFF",{1},h.s,h.t);
    h.connected=false;h.r.disconnect(h.s,h.t);h.tick();require(h.arms==0,"disconnect before queued Arm");h.stopped();});
  add("healthy_disconnect_landing_exception",[](Harness& h){h.arm("AUTHORITY_AUTONOMOUS");h.tick();h.tick();
    h.connected=false;h.r.disconnect(h.s,h.t);for(int i=0;i<8;++i)h.tick();
    require(h.offboard==1&&h.arms==1,"landing does not reacquire or rearm");});
  add("shutdown_blocks_both_sinks",[](Harness& h){h.ready("AUTHORITY_AUTONOMOUS");h.r.begin_shutdown(h.s,h.t);h.tick();h.stopped();});
  add("loop_stall_no_reacquire",[](Harness& h){h.arm("AUTHORITY_AUTONOMOUS");h.t+=.4;h.tick();h.stopped();});
  add("takeover_ground_reset_to_unset",[](Harness& h){h.arm("AUTHORITY_AUTONOMOUS");h.s.attitude.kill=true;h.tick();h.stopped();
    require(h.r.request("AUTHORITY_RESET",{},h.s,h.t).accepted,"explicit safe reset");
    require(h.r.attitude_core().authority_state().mode==auth::Mode::Unset,"reset cannot carry flight authority");h.tick();});
  add("pending400_prevents_ground_reset",[](Harness& h){h.ready("AUTHORITY_AUTONOMOUS");h.r.request("TAKEOFF",{1},h.s,h.t);h.tick();
    h.s.attitude.operator_takeover=true;h.tick();h.s.attitude.operator_takeover=false;
    require(!h.r.request("AUTHORITY_RESET",{},h.s,h.t).accepted,"unresolved Arm blocks reset");});
  int failed=0;
  std::cout<<"{\"schema_version\":1,\"release\":\"0.4.4-offboard-flight-dev-r2\",\"hardware_authorized\":false,\"real_dds\":\"UNKNOWN\",\"scenarios\":[";
  for(std::size_t i=0;i<results.size();++i){const auto& x=results[i];if(!x.pass)++failed;if(i)std::cout<<',';
    std::cout<<"{\"name\":"<<f::quote(x.name)<<",\"passed\":"<<(x.pass?"true":"false")<<",\"error\":"<<f::quote(x.error)
      <<",\"arm_sink_count\":"<<x.arms<<",\"target_sink_count\":"<<x.targets<<",\"mode_sink_count\":"<<x.modes
      <<",\"nonzero_target_count\":"<<x.nonzero<<",\"offboard_command_count\":"<<x.offboard<<",\"phases\":[";
    for(std::size_t j=0;j<x.phases.size();++j){if(j)std::cout<<',';std::cout<<f::quote(x.phases[j]);}std::cout<<"]}";
  }
  std::cout<<"],\"summary\":{\"total\":"<<results.size()<<",\"passed\":"<<results.size()-std::size_t(failed)<<",\"failed\":"<<failed<<"}}\n";
  return failed?1:0;
}
