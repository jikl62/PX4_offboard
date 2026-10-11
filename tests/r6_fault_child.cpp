// OS signals exercise an OFFLINE Runtime/boundary fixture. No ROS/FC01/PX4.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <csignal>
#include <chrono>
#include <thread>
#include <iostream>
volatile std::sig_atomic_t r6_stop=0;
static void stop(int){r6_stop=1;}
struct M {std::uint64_t timestamp{};bool attitude{},position{},velocity{};};
struct T {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};
  std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
int main(int argc,char** argv){
  auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;
  c.attitude.descent_timeout=.3;c.attitude.recovery_seconds=1;c.attitude.saturation_seconds=100;
  fc01::Runtime r(c);auto s=authority_test::sample();double now{};
  authority_test::armed(r,s,now,"AUTHORITY_AUTONOMOUS");
  s.feedback.landed=false;s.at_rest=false;s.attitude.z=1.8;s.attitude.vz=0;
  r.request("LAND",{},s,now);
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  std::cout<<"READY\n"<<std::flush;
  if(argc>1&&std::string(argv[1])=="crash")std::raise(SIGABRT);
  // Fixed OFFLINE observation window, not the production Node shutdown rule.
  // SIGINT/TERM retain healthy custody past the recovery completion deadline.
  for(int i=0;i<40;++i){
    if(r6_stop)r.begin_shutdown(s,now);
    auto o=authority_test::tick(r,s,now).attitude;
    bool target=false,mode=false;
    if(o.publish){auto d=fc01::attitude::publish_attitude_at_boundary<M,T>(
      r.publication_context(s,now,!r.shutting_down(),true),o,std::uint64_t(now*1e6),
      [&](const T&){target=true;},[&](const M&){mode=true;});assert(d.allowed);}
    std::cout<<"{\"synthetic\":true,\"target\":"<<(target?"true":"false")
      <<",\"heartbeat\":"<<(mode?"true":"false")<<",\"shutting_down\":"<<(r.shutting_down()?"true":"false")
      <<",\"phase\":\""<<r.phase_name()<<"\",\"recovery_budget_exhausted\":"
      <<(r.attitude_core().recovery_budget_exhausted()?"true":"false")
      <<",\"px4_fallback_verified\":false}"<<std::endl;
    if(!r.active())break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
