#pragma once
#include "jikl_fc01_system/attitude_reference.hpp"
#include <algorithm>
#include <cmath>

namespace fc01::attitude {
struct AttitudeDiagnostics {
  bool valid{};
  double roll_deg{},pitch_deg{},yaw_deg{},tilt_deg{},command_tilt_deg{},error_deg{};
};
// Observation only: NO influence on command formation or safety thresholds.
inline AttitudeDiagnostics attitude_diagnostics(const Quaternion& measured,const Quaternion& requested){
  auto m=normalized(measured),r=normalized(requested);
  if(!m||!r)return {};
  constexpr double k=180./3.141592653589793238462643383279502884;
  const double w=(*m)[0],x=(*m)[1],y=(*m)[2],z=(*m)[3];
  const double roll=std::atan2(2*(w*x+y*z),1-2*(x*x+y*y));
  const double pitch=std::asin(std::clamp(2*(w*y-z*x),-1.,1.));
  const double yaw=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
  const double cosine=std::clamp(1-2*(x*x+y*y),-1.,1.);
  const double command_cosine=std::clamp(1-2*((*r)[1]*(*r)[1]+(*r)[2]*(*r)[2]),-1.,1.);
  const double dot=std::clamp(std::abs(w*(*r)[0]+x*(*r)[1]+y*(*r)[2]+z*(*r)[3]),0.,1.);
  return {true,roll*k,pitch*k,yaw*k,std::acos(cosine)*k,
    std::acos(command_cosine)*k,2*std::acos(dot)*k};
}
} // namespace fc01::attitude
