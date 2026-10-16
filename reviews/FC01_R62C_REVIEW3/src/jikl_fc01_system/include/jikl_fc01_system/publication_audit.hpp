#pragma once
#include <algorithm>
#include <cstdint>
#include <cmath>
namespace fc01::attitude {
// Local publish returns only. Neither counts nor timing assert PX4 receipt.
// Target and mode are separate commits, never an atomic pair.
struct PublicationAudit {
  std::uint64_t target_count{},mode_count{},mode_gap_violations{};
  double target_last{-1},mode_last{-1},max_target_gap{},max_mode_gap{};
  double last_target_thrust_z{NAN};
  bool target_this_tick{},mode_this_tick{},exception_this_tick{};
  void begin(double){target_this_tick=mode_this_tick=exception_this_tick=false;}
  void target_committed(double now,double thrust_z=NAN){
    if(target_last>=0&&std::isfinite(now))max_target_gap=std::max(max_target_gap,now-target_last);
    target_last=now;++target_count;target_this_tick=true;
    last_target_thrust_z=thrust_z;
  }
  void mode_committed(double now){
    if(mode_last>=0&&std::isfinite(now)){
      const double gap=now-mode_last;max_mode_gap=std::max(max_mode_gap,gap);
      if(gap>.3)++mode_gap_violations;
    }
    mode_last=now;++mode_count;mode_this_tick=true;
  }
  void failed(){exception_this_tick=true;}
};
} // namespace fc01::attitude
