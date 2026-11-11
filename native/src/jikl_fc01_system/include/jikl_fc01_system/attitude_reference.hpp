#pragma once
// 姿态参考：验证 PX4 四元数，并在安全准备阶段锁定水平姿态所用航向。
// FRD 为机体前/右/下，NED 为世界北/东/下；yaw_ 单位 rad，按 NED 绕下轴定义。
// 估计器 reset 在尚未活动时允许重建参考，已活动时必须报告 ActiveReset，不能偷偷改变控制零点。
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace fc01::attitude {
using Quaternion=std::array<double,4>; // Hamilton w,x,y,z; body FRD -> earth NED
// 只对接近单位长度的四元数归一化，不把任意错误向量修成有效姿态。
inline std::optional<Quaternion> normalized(const Quaternion& q){
  double norm2=0;
  for(double value:q){if(!std::isfinite(value))return {};norm2+=value*value;}
  const double norm=std::sqrt(norm2);
  if(!std::isfinite(norm)||std::abs(norm-1.)>.05)return {};
  Quaternion out=q;for(double& value:out)value/=norm;return out;
}
// Healthy=参考连续；Rebased=已重建；Invalid=姿态无效；ActiveReset=活动任务中发生重置。
enum class ReferenceResult {Healthy,Rebased,Invalid,ActiveReset};
class AttitudeReference {
 public:
  // input 来自 PX4 姿态，reset 是其重置计数；返回值交给上层决定能否继续控制。
  ReferenceResult observe(const Quaternion& input,std::uint8_t reset,bool active){
    const auto q=normalized(input);if(!q)return ReferenceResult::Invalid;
    // 活动任务遇到 reset 立即返回，不更新已有航向和计数，让上层保留故障证据。
    if(set_&&reset!=reset_&&active)return ReferenceResult::ActiveReset;
    if(!set_||reset!=reset_){
      yaw_=std::atan2(2*((*q)[0]*(*q)[3]+(*q)[1]*(*q)[2]),
                     1-2*((*q)[2]*(*q)[2]+(*q)[3]*(*q)[3]));
      reset_=reset;set_=true;return ReferenceResult::Rebased;
    }
    return ReferenceResult::Healthy;
  }
  void clear(){set_=false;}
  bool ready() const{return set_;}
  double yaw() const{return yaw_;}
  std::uint8_t reset_counter() const{return reset_;}
  // 只生成横滚/俯仰为零、保持锁定 yaw 的四元数，供无视觉定高输出使用。
  Quaternion level() const{return {std::cos(yaw_/2),0,0,std::sin(yaw_/2)};}
 private:
  // set_ 表示有可信参考；reset_ 记建立参考时的估计器版本；yaw_ 不随每帧姿态漂移。
  bool set_{};std::uint8_t reset_{};double yaw_{};
};
} // namespace fc01::attitude
