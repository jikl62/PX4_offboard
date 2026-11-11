#pragma once
// 本地发布审计：记录两个独立 publish 返回及其间隔，不推断 PX4 已接收或执行。
// now 和各间隔单位 s；last_target_thrust_z 是机体 FRD 归一化 z 推力，负值通常对应旋翼升力。
#include <algorithm>
#include <cstdint>
#include <cmath>
namespace fc01::attitude {
// Local publish returns only. Neither counts nor timing assert PX4 receipt.
// Target and mode are separate commits, never an atomic pair.
struct PublicationAudit {
  // 目标和心跳分别计数；mode_gap_violations 只记录超过 0.3 s 的本地心跳间隔。
  std::uint64_t target_count{},mode_count{},mode_gap_violations{};
  double target_last{-1},mode_last{-1},max_target_gap{},max_mode_gap{};
  double last_target_thrust_z{NAN};
  bool target_this_tick{},mode_this_tick{},exception_this_tick{};
  // 每轮清除“本轮已提交/异常”标志，不清历史总量与最大间隔。
  void begin(double){target_this_tick=mode_this_tick=exception_this_tick=false;}
  // 只有目标 sink 真正返回后调用；输出值用于取证，不能把它等同于实际电机推力。
  void target_committed(double now,double thrust_z=NAN){
    if(target_last>=0&&std::isfinite(now))max_target_gap=std::max(max_target_gap,now-target_last);
    target_last=now;++target_count;target_this_tick=true;
    last_target_thrust_z=thrust_z;
  }
  // 心跳单独记账，因此“目标已发但心跳失败”在报告中仍可识别。
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
