#include "jikl_fc01_system/control_diagnostics.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace a=fc01::attitude;
struct Mode {std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};};
struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static a::BoundaryContext safe_context(){
  a::BoundaryContext c;c.profile="attitude_altitude";c.phase=a::Phase::Prestream;
  c.session_connected=c.control_fresh=c.reference_ready=true;
  auto& s=c.feedback;s.attitude_fresh=s.local_fresh=s.status_fresh=s.land_fresh=s.failsafe_fresh=true;
  s.z_valid=s.vz_valid=s.landed=s.at_rest=s.exclusive=s.preflight=s.battery_ready=true;
  s.q={1,0,0,0};s.stationary_seconds=3;return c;
}
static void publication_context_includes_snapshot_duration(){
  for(int stall_context=0;stall_context<=3;++stall_context){
    fc01::CurrentControlCycle cycle;cycle.begin(0.);cycle.callbacks_done(.01);
    unsigned contexts=0,targets=0,heartbeats=0;double now=.02;
    a::Output output;output.publish=true;bool threw=false,allowed=false;std::string reason;
    try{
      const auto decision=a::publish_attitude_resampled<Mode,Target>([&]{
        ++contexts;auto c=safe_context();
        // Model a snapshot that starts within budget and completes overdue.
        // The guard must use its completion time, including the third context.
        const double snapshot_started=now;
        if(stall_context==int(contexts))now=.301;else now+=.01;
        if(snapshot_started>.3)throw std::runtime_error("fixture already overdue before snapshot");
        c.control_fresh=cycle.current(now,.3);return c;
      },output,123,[&](const Target&){++targets;},[&](const Mode&){++heartbeats;});
      allowed=decision.allowed;
      if(stall_context&&decision.reason!=a::BoundaryReason::ControlStall)
        throw std::runtime_error("pre-target refusal must identify control stall");
    }catch(const std::runtime_error& error){threw=true;reason=error.what();}
    if(stall_context==0){
      if(!allowed||threw||targets!=1||heartbeats!=1||contexts!=3)
        throw std::runtime_error("healthy publication must retain all three boundary checks");
    }else if(stall_context<3){
      if(allowed||threw||targets||heartbeats||contexts!=unsigned(stall_context))
        throw std::runtime_error("stall in either pre-target context refuses both sinks");
    }else if(!threw||targets!=1||heartbeats||contexts!=3||
       reason!="HEARTBEAT_FINAL_CONTEXT_CHANGED_CONTROL_LOOP_STALL"){
      throw std::runtime_error("third context stall must preserve target evidence and refuse heartbeat");
    }
  }
}

int main(){
  publication_context_includes_snapshot_duration();
  fc01::CurrentControlCycle cycle;
  if(cycle.current(100.,.3))throw std::runtime_error("no current cycle before begin and callbacks");
  cycle.begin(100.);cycle.callbacks_done(100.29);
  if(!cycle.current(100.299,.3))throw std::runtime_error("callback duration is part of the same valid cycle");
  if(cycle.current(100.301,.3))throw std::runtime_error("a final snapshot crossing the cycle budget must refuse publication");
  if(cycle.current(100.299,.3))throw std::runtime_error("a rejected cycle cannot recover without begin");
  cycle.begin(101.);cycle.callbacks_done(101.336);
  if(cycle.current(101.337,.3))throw std::runtime_error("historical 336ms callback stall cannot be hidden by fast tick");
  cycle.begin(102.);cycle.callbacks_done(102.01);
  if(!cycle.current(102.02,.3))throw std::runtime_error("next cycle establishes a new budget");
  if(cycle.current(102.015,.3))throw std::runtime_error("regressed monotonic time inside a cycle is invalid");
  cycle.begin(103.);
  if(cycle.current(103.01,.3))throw std::runtime_error("callbacks must complete before publication");
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  for(double invalid:{nan,inf,-1.}){
    cycle.begin(invalid);cycle.callbacks_done(1.);
    if(cycle.current(1.1,.3))throw std::runtime_error("invalid cycle start cannot grant publication");
    cycle.begin(1.);cycle.callbacks_done(invalid);
    if(cycle.current(1.1,.3))throw std::runtime_error("invalid callback completion cannot grant publication");
    cycle.begin(1.);cycle.callbacks_done(1.01);
    if(cycle.current(invalid,.3))throw std::runtime_error("invalid final clock cannot grant publication");
  }
  for(double invalid:{nan,inf,-1.,0.}){
    cycle.begin(1.);cycle.callbacks_done(1.01);
    if(cycle.current(1.1,invalid))throw std::runtime_error("invalid stall budget cannot grant publication");
  }
  cycle.begin(0.);cycle.callbacks_done(.01);
  if(!cycle.current(.25,.25))throw std::runtime_error("a finite cycle exactly at the configured limit is valid");
  fc01::ControlTiming timing;
  if(timing.valid)throw std::runtime_error("no cycle may be invented before observation");
  timing.observe(100.,100.01,100.03,.3);
  if(!timing.valid||std::abs(timing.callback_seconds-.01)>1e-9||
     std::abs(timing.control_seconds-.02)>1e-9||timing.period_valid)
    throw std::runtime_error("callbacks and tick must have separate measured durations");
  timing.observe(100.4,100.41,100.45,.3);
  if(!timing.period_valid||std::abs(timing.period_seconds-.4)>1e-9||timing.stall_count!=1)
    throw std::runtime_error("loop stall must include callbacks and scheduling, not only tick");
  timing.observe(99.,99.01,99.02,.3);
  if(timing.valid)throw std::runtime_error("clock regression must not appear healthy");
}
