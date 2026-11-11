// 台架实测垂向记录反事实回放：只验证新代码会如何规划，不声称当时真实执行过上锁。
// 读取211行已记录CSV，NED z/vz沿用实测值；其余健康条件是夹具显式假设。
#include "review_bench_support.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

// Exact vertical/status/land/local samples from the recorded Review1 armed
// interval. Other health inputs are the explicit fixture assumptions. This
// counterfactual replay tests planning, never actual PX4 execution/Disarm.
// 保存原始相对时间s、三个独立微秒源戳、NED参考/位置/速度与landed；不插值或发明缺失值。
struct Recorded {
  double t{},z0{},z{},vz{};std::uint64_t status{},land{},local{};int landed{};
};
int main(){
  const auto path=std::filesystem::path(__FILE__).parent_path()/"data/review1_bench_armed.csv";
  std::ifstream file(path);assert(file);
  std::string line;std::getline(file,line);std::vector<Recorded> samples;
  while(std::getline(file,line)){
    std::replace(line.begin(),line.end(),',',' ');std::istringstream in(line);Recorded x;
    assert(in>>x.t>>x.status>>x.land>>x.local>>x.z0>>x.z>>x.vz>>x.landed);
    samples.push_back(x);
  }
  assert(samples.size()==211);
  B h(true);h.s.attitude.z=samples.front().z0;
  h.armed();const double start=h.t;
  // Translate all recorded source stamps by ONE common offset into this
  // fixture's clock domain; retain cross-source timing, repeats and gaps.
  // 对三路源戳加同一个偏移映射到测试域，保留跨源先后、重复和gap，不能各自重整时钟。
  const auto origin=std::min({samples.front().status,samples.front().land,samples.front().local});
  const auto base=std::max({h.s.status_stamp,h.s.land_stamp,h.s.attitude.local_stamp})+50000;
  h.s.at_rest=false;h.s.stationary_seconds=0;
  bool disarm=false;
  // 逐条实测输入送真实Runtime，要求始终零推力且不进斜坡；只记录第一次Disarm计划与假提交。
  for(const auto& x:samples){
    h.t=start+.05+x.t-samples.front().t;
    h.s.status_stamp=base+x.status-origin;h.s.land_stamp=base+x.land-origin;
    h.s.attitude.local_stamp=base+x.local-origin;
    h.s.attitude.z=x.z;h.s.attitude.vz=x.vz;h.s.feedback.landed=x.landed;
    assert(h.r.request("PING",{},h.s,h.t).accepted);
    auto o=h.r.step(h.s,h.t).attitude;
    assert(!a::nonzero(o.thrust));h.commit(o);
    assert(h.r.attitude_core().phase()!=a::Phase::ThrustRamp);
    if(o.command==a::Command::Disarm){
      disarm=true;
      std::cout<<"REVIEW2_RECORDED_VERTICAL_TRACE_DISARM_PLANNED_AT_PLUS_S="<<h.t-start<<"\n";
      break;
    }
  }
  assert(disarm&&h.arms==1&&h.disarms==1&&h.t-start<8);
  std::cout<<"REVIEW2_RECORDED_TRACE_PLANNING_ONLY_NOT_HARDWARE_EXECUTION PASS\n";
}
