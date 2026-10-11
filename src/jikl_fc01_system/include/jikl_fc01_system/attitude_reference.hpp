#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace fc01::attitude {
using Quaternion=std::array<double,4>; // Hamilton w,x,y,z; body FRD -> earth NED
inline std::optional<Quaternion> normalized(const Quaternion& q){
  double norm2=0;
  for(double value:q){if(!std::isfinite(value))return {};norm2+=value*value;}
  const double norm=std::sqrt(norm2);
  if(!std::isfinite(norm)||std::abs(norm-1.)>.05)return {};
  Quaternion out=q;for(double& value:out)value/=norm;return out;
}
enum class ReferenceResult {Healthy,Rebased,Invalid,ActiveReset};
class AttitudeReference {
 public:
  ReferenceResult observe(const Quaternion& input,std::uint8_t reset,bool active){
    const auto q=normalized(input);if(!q)return ReferenceResult::Invalid;
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
  Quaternion level() const{return {std::cos(yaw_/2),0,0,std::sin(yaw_/2)};}
 private:
  bool set_{};std::uint8_t reset_{};double yaw_{};
};
} // namespace fc01::attitude
