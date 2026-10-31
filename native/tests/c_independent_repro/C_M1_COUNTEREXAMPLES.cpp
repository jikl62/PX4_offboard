// Independent offline witness. Read-only relative to the submitted candidate tree.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
namespace a=fc01::attitude;
static const char* phase(fc01::Runtime& r){return a::name(r.attitude_core().phase());}
int main(){
  auto c=authority_test::config(); c.attitude.autonomous_failsafe_confirmed=true;
  // Repro 1: armed but landed && !at_rest, common PX4 powered-ground observation.
  // LAND/console disconnect explicitly requested before detected liftoff.
  {
    fc01::Runtime r(c);auto s=authority_test::sample();double t=0;
    authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
    s.at_rest=false;s.feedback.landed=true;
    const auto reply=r.request("LAND",{},s,t);
    const auto o=authority_test::tick(r,s,t);
    std::cout<<"C01_ARMED_GROUND_LAND at_rest=false landed=true accepted="<<reply.accepted
       <<" phase="<<phase(r)<<" fault="<<a::name(r.attitude_core().fault_class())
       <<" publish="<<o.attitude.publish<<" cmd="<<int(o.attitude.command)
       <<" still_armed="<<s.feedback.armed<<" reason="<<r.message()<<"\n";
  }
  // Repro 2: power ramps up without detected liftoff. At timeout output/heartbeat
  // stops in Safety rather than controlled ramp-to-zero / post-ground disarm.
  {
    fc01::Runtime r(c);auto s=authority_test::sample();double t=0;
    authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
    double maximum=0;bool saw_thrust=false;
    for(int i=0;i<260 && r.attitude_core().phase()!=a::Phase::Safety;++i){
      const auto o=authority_test::tick(r,s,t);
      maximum=std::max(maximum,-o.attitude.thrust[2]);
      saw_thrust|=o.attitude.thrust[2]<-1e-7;
    }
    const auto o=authority_test::tick(r,s,t);
    std::cout<<"C02_LIFTOFF_TIMEOUT powered="<<saw_thrust<<" peak_thrust="<<maximum
       <<" phase="<<phase(r)<<" fault="<<a::name(r.attitude_core().fault_class())
       <<" publish_after="<<o.attitude.publish<<" cmd_after="<<int(o.attitude.command)
       <<" last_400_tracker_sent="<<r.pending(400).sent<<" (this may be prior Arm)\n";
  }
  // Repro 4: exact-zero thrust skips commanded-q envelope at FINAL SINK.
  {
    fc01::Runtime r(c);auto s=authority_test::sample();
    auto context=r.publication_context(s,0,true,true);
    context.phase=a::Phase::RequestDisarm;
    context.authority={fc01::authority::Mode::AutonomousNoRc,fc01::authority::Mode::AutonomousNoRc,true,false};
    context.flight_started=context.reference_ready=context.arm_confirmed=context.ground_confirmed=true;
    context.reference_z=s.attitude.z;context.yaw_reference=0;context.touchdown_baseline=100;
    context.feedback.armed=context.feedback.offboard=context.feedback.battery_ready=true;
    context.feedback.landed=context.feedback.at_rest=true;
    context.feedback.land_stamp=1000;
    a::Output out;out.publish=true;out.q={std::sqrt(.5),std::sqrt(.5),0,0};out.thrust={0,0,0};
    struct Mode {std::uint64_t timestamp{};bool attitude{};};
    struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
    int target_calls=0,mode_calls=0;
    const auto decision=a::publish_attitude_at_boundary<Mode,Target>(context,out,123,
       [&](const Target&){++target_calls;},[&](const Mode&){++mode_calls;});
    std::cout<<"C04_ZERO_THRUST_90ROLL_FINAL_SINK allowed="<<decision.allowed
       <<" target_publishes="<<target_calls<<" mode_publishes="<<mode_calls
       <<" reason="<<a::name(decision.reason)<<"\n";
  }
  // Repro 5: live operator's AUTHORITY_TAKEOVER is NOT a PX4 mode command.
  {
    fc01::Runtime r(c);auto s=authority_test::sample();double t=0;
    authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
    for(int k=0;k<9;++k)authority_test::tick(r,s,t);
    auto before=phase(r);
    const auto reply=r.request("AUTHORITY_TAKEOVER",{},s,t);
    const auto o=authority_test::tick(r,s,t);
    std::cout<<"C05_TTY_AUTHORITY_TAKEOVER previous_phase="<<before
      <<" accepted="<<reply.accepted<<" resulting_phase="<<phase(r)
      <<" px4_still_offboard="<<s.feedback.offboard<<" still_armed="<<s.feedback.armed
      <<" post_command="<<int(o.attitude.command)<<" heartbeat="<<o.attitude.publish<<"\n";
  }
  // Repro 3: Land detector's timestamp is frozen over the dwell. Although the
  // Boolean 'land_fresh' remains true (its actual tolerance is 2.5 s), the
  // present state machine accepts repeated reads of the same land sample.
  {
    fc01::Runtime r(c);auto s=authority_test::sample();double t=0;
    authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
    const auto frozen=s.land_stamp;
    for(int i=0;i<9;++i){
      t+=.05;s.status_stamp+=1000;
      r.request("PING",{},s,t);
      r.step(s,t); // land_stamp intentionally unchanged; local healthy per synthetic fixture
    }
    std::cout<<"C03_FROZEN_LAND_STAMP since_arm="<<frozen<<" current="<<s.land_stamp
       <<" phase="<<phase(r)<<" (ArmedGround requires continuous land evidence)\n";
  }
}
