#include "jikl_fc01_system/authority_telemetry.hpp"
#include "jikl_fc01_system/source_clock.hpp"
#include <cassert>
#include <iostream>
namespace f=fc01;
constexpr std::uint64_t epoch=1700000000000000ULL;
struct Estimate {std::uint64_t timestamp{},timestamp_sample{};double z{};};
struct Manual {
  static constexpr std::uint8_t SOURCE_RC=1;
  std::uint64_t timestamp{},timestamp_sample{};
  bool valid{true};std::uint8_t data_source{1};bool sticks_moving{};
};
struct Switches {
  std::uint64_t timestamp{},timestamp_sample{};
  std::uint8_t mode_slot{1},arm_switch{3},return_switch{3},loiter_switch{3},offboard_switch{3},kill_switch{3};
  std::uint32_t switch_changes{};
};
static void estimates(){
  f::Stamp envelope;f::SourceSampleProgress raw;Estimate held;
  Estimate input{epoch,epoch-1000,1};
  assert(f::receive_sampled_source(input,held,envelope,raw,0));
  assert(raw.fresh(epoch,0,.5));
  for(int i=1;i<=6;++i){
    input.timestamp=epoch+std::uint64_t(i)*100000;input.z=i;
    assert(f::receive_sampled_source(input,held,envelope,raw,double(i)*.1));
  }
  assert(envelope.fresh(.600001,.5));
  assert(!raw.fresh(input.timestamp,.600001,.5));
  assert(!raw.fresh(epoch,.600001,.5)); // receipt lifetime independent of wall clock
  input.timestamp+=1000;input.timestamp_sample=input.timestamp-1000;
  assert(f::receive_sampled_source(input,held,envelope,raw,.61));
  assert(raw.fresh(input.timestamp,.61,.5));
  input.timestamp+=1000;input.timestamp_sample=0;
  assert(f::receive_sampled_source(input,held,envelope,raw,.62));
  assert(!raw.fresh(input.timestamp,.62,.5)); // cannot borrow previous good sample
  input.timestamp+=1000;input.timestamp_sample=input.timestamp+1;
  assert(f::receive_sampled_source(input,held,envelope,raw,.63));
  assert(!raw.fresh(input.timestamp,.63,.5));
  input.timestamp+=1000;input.timestamp_sample=input.timestamp-1000;
  assert(f::receive_sampled_source(input,held,envelope,raw,.64));
  assert(raw.fresh(input.timestamp,.64,.5));
  input.timestamp+=1000;input.timestamp_sample-=1000;
  assert(f::receive_sampled_source(input,held,envelope,raw,.65));
  assert(raw.regressed&&!raw.fresh(input.timestamp,.65,.5));
  f::SourceSampleProgress delayed;delayed.observe(epoch,epoch-600000,0);
  assert(!delayed.fresh(epoch,0,.5)); // recently received, actually old
  f::SourceSampleProgress future;future.observe(epoch+200000,epoch+200000,0);
  assert(!future.fresh(epoch,0,.5));
  std::cout<<"REVIEW_FRESH_ENVELOPE_CANNOT_REFRESH_FROZEN_OR_INVALID_RAW_ESTIMATE PASS\n";
}
static void rc_clock(){
  f::AuthorityTelemetry observer;Manual m{1000000,1000000};
  observer.manual(m,0,epoch,.5);
  m.timestamp+=1000;m.timestamp_sample+=1000;
  observer.manual(m,.01,epoch+10000,.5);
  auto facts=observer.facts(.01,.5,true,false,1,1,epoch+10000);
  assert(!facts.rc_link_ready&&!facts.rc_source_clock_valid);
  m.timestamp=epoch+20000;m.timestamp_sample=m.timestamp-1000;
  observer.manual(m,.02,epoch+20000,.5);
  m.timestamp+=10000;m.timestamp_sample+=10000;
  observer.manual(m,.03,epoch+30000,.5);
  facts=observer.facts(.03,.5,true,false,1,1,epoch+30000);
  assert(facts.rc_link_ready&&facts.rc_source_clock_valid);
  m.timestamp+=200000;m.timestamp_sample+=200000;
  observer.manual(m,.04,epoch+40000,.5);
  assert(!observer.facts(.04,.5,true,false,1,1,epoch+40000).rc_link_ready);
  std::cout<<"REVIEW_RC_BOOT_CLOCK_AND_FUTURE_CLOCK_REVOKE_ADMISSION PASS\n";
}
static void kill_clock(){
  f::AuthorityTelemetry observer;Switches sw{epoch,epoch};sw.kill_switch=1;
  observer.switches(sw,0,.5,epoch);
  assert(observer.facts(0,.5,true,false,1,1,epoch).kill);
  sw.timestamp=epoch+200000;sw.timestamp_sample=sw.timestamp;sw.kill_switch=3;
  observer.switches(sw,.01,.5,epoch+10000);
  auto facts=observer.facts(.01,.5,true,false,1,1,epoch+10000);
  assert(facts.kill&&!facts.rc_switches_ready&&!facts.rc_switch_clock_valid);
  // The rejected future stamp must not poison the last good stamp.
  sw.timestamp=epoch+20000;sw.timestamp_sample=sw.timestamp;
  observer.switches(sw,.02,.5,epoch+20000);
  facts=observer.facts(.02,.5,true,false,1,1,epoch+20000);
  assert(!facts.kill&&facts.rc_switches_ready&&facts.rc_switch_clock_valid);
  std::cout<<"REVIEW_INVALID_CLOCK_KILL_OFF_CANNOT_CLEAR_LATCH PASS\n";
}
int main(){estimates();rc_clock();kill_clock();}
