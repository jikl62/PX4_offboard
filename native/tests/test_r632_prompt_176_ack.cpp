// 真实8.794 ms模式ACK时序回归：快速176 ACK与400更严归因分别保留。
// 使用记录中的ACK相对延迟驱动合成Runtime，不代表在本测试中连接过真实飞控。
// Targeted R6.3.2 regression for FC01 real 8.794ms Command 176 ACK.
#include "review_clock_support.hpp"
#include "jikl_fc01_system/source_clock.hpp"
#include <iostream>

int main(){
  {
    Harness h;
    for(int i=0;i<60&&h.r.attitude_core().phase()!=a::Phase::RequestOffboard;++i)h.tick();
    assert(h.r.attitude_core().phase()==a::Phase::RequestOffboard);
    const auto wire=h.r.pending(176).wire_stamp;
    assert(h.r.pending(176).source_clock_validated);
    h.r.ack({wire-1,176,0,1,191,false},h.t+.001);
    assert(!h.r.pending(176).accepted);
    // 首次入Offboard快速ACK可被记录，但仍必须等待越过因果floor的新Offboard状态才ready。
    h.r.ack({wire+8794,176,0,1,191,false},h.t+.010);
    assert(h.r.pending(176).accepted);
    h.s.feedback.offboard=true;h.s.nav_state=14;
    // Accepted ACK alone is insufficient: status must be after causal floor.
    h.t+=.05;h.s.status_stamp=wire+50000;
    h.r.request("PING",{},h.s,h.t);
    h.r.step(h.s,h.t);
    assert(h.r.attitude_core().phase()==a::Phase::RequestOffboard);
    for(int i=0;i<15&&h.r.attitude_core().phase()!=a::Phase::OffboardReady;++i)h.tick();
    assert(h.r.attitude_core().phase()==a::Phase::OffboardReady);
    assert(h.r.pending(176).accepted);
    std::cout<<"REAL_PROMPT_176_ACK_AND_FRESH_OFFBOARD_STATE PASS\n";
  }
  {
    Harness h;
    for(int i=0;i<60&&h.r.attitude_core().phase()!=a::Phase::RequestOffboard;++i)h.tick();
    assert(h.r.attitude_core().phase()==a::Phase::RequestOffboard);
    const auto wire=h.r.pending(176).wire_stamp;
    // 远未来ACK与错误component不能匹配当前176；它们不能靠随后的模式状态变为合法ACK。
    h.r.ack({wire+1000000,176,0,1,191,false},h.t+.001);
    assert(!h.r.pending(176).accepted&&h.r.pending(176).unattributed_clock_acks==1);
    h.r.ack({wire+8794,176,0,1,192,false},h.t+.010);
    assert(!h.r.pending(176).accepted);
    std::cout<<"UNATTRIBUTABLE_OR_WRONG_TARGET_176_ACK_REJECTED PASS\n";
  }
  {
    Harness h;h.ready();
    assert(h.r.request("TAKEOFF",{.6},h.s,h.t).accepted);
    h.tick();
    const auto wire=h.r.pending(400).wire_stamp;
    // 同样快速延迟不能用于普通Command400归因，原100 ms因果保护继续生效。
    h.r.ack({wire+8794,400,0,1,191,false},h.t+.010);
    assert(!h.r.pending(400).accepted&&h.r.pending(400).unattributed_clock_acks==1);
    std::cout<<"COMMAND_400_100MS_GUARD_PRESERVED PASS\n";
  }
}
