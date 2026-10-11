#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include "jikl_fc01_system/publication_audit.hpp"
#include <cassert>
#include <stdexcept>
#include <iostream>
struct M {std::uint64_t timestamp{};bool attitude{},position{},velocity{};};
struct T {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};
  std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
int main(){
  for(int throwing_sink=0;throwing_sink<2;++throwing_sink){
    auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;
    fc01::Runtime r(c);auto s=authority_test::sample();double now{};
    authority_test::armed(r,s,now,"AUTHORITY_AUTONOMOUS");
    auto o=authority_test::tick(r,s,now).attitude;
    fc01::attitude::PublicationAudit audit;audit.begin(now);
    bool completed=false;
    try{
      auto d=fc01::attitude::publish_attitude_at_boundary<M,T>(r.publication_context(s,now,true,true),o,1000,
        [&](const T&){if(throwing_sink==0)throw std::runtime_error("target failed");audit.target_committed(now);},
        [&](const M&){throw std::runtime_error("mode failed");audit.mode_committed(now);});
      completed=d.allowed;
    }catch(const std::exception&){audit.failed();r.abort_attitude("PARTIAL_SINK_FAILURE");}
    assert(!completed&&audit.mode_count==0);
    assert(audit.target_count==unsigned(throwing_sink==1));
    auto next=authority_test::tick(r,s,now).attitude;
    assert(!next.publish&&next.command==fc01::attitude::Command::None);
    assert(r.attitude_core().phase()==fc01::attitude::Phase::Safety);
  }
  std::cout<<"R6_PARTIAL_SINK real boundary exceptions retain separate commit counts; no later command PASS\n";
}
