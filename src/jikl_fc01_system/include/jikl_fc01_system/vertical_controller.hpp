#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace fc01::attitude {
struct VerticalConfig {
  // Sentinels are deliberately unusable for flight. No vehicle gains guessed.
  double hover{-1},minimum{-1},maximum{-1},slew{-1};
  double height_gain{-1},velocity_gain{-1},integral_gain{-1},integral_limit{-1};
  double max_height{-1},max_climb{-1},max_descent{-1};
  double ground_lower_margin{.25}; // permit only bounded negative relative height
  void validate() const{
    for(double value:{hover,minimum,maximum,slew,height_gain,velocity_gain,
                      integral_gain,integral_limit,max_height,max_climb,max_descent})
      if(!std::isfinite(value))throw std::invalid_argument("NONFINITE_VERTICAL_PARAMETER");
    if(minimum<0||maximum>1||minimum>=hover||hover>=maximum||slew<=0||
       height_gain<=0||velocity_gain<=0||integral_gain<0||integral_limit<=0||
       max_height<=0||max_height>5||max_climb<=0||max_descent<=0||
       !std::isfinite(ground_lower_margin)||ground_lower_margin<=0||ground_lower_margin>=max_height)
      throw std::invalid_argument("EXPLICIT_VALIDATED_VERTICAL_PARAMETERS_REQUIRED");
  }
};
struct VerticalSample {
  double z0{},z{},vz{};
  bool fresh{},z_valid{},vz_valid{};
  std::uint8_t z_reset{},vz_reset{};
};
struct VerticalOutput {
  bool healthy{},saturated{},slew_limited{};
  double thrust{},height{},speed{},velocity_target{};
  std::string reason;
};
class VerticalController {
 public:
  explicit VerticalController(VerticalConfig config):c_(config){c_.validate();}
  void reset(double initial=0){last_thrust_=initial;integral_=0;reset_set_=false;}
  double integral() const{return integral_;}
  // Landing ramp and flight ramp share the same physical slew state.
  VerticalOutput ramp(double desired,double dt){
    if(!std::isfinite(desired)||!std::isfinite(dt)||dt<=0)return bad("INVALID_RAMP");
    VerticalOutput o;o.healthy=true;
    const double bounded=std::clamp(desired,0.,c_.maximum);
    o.saturated=bounded!=desired;o.thrust=limit(bounded,dt);
    o.slew_limited=o.thrust!=bounded;return o;
  }
  VerticalOutput step(const VerticalSample& s,double target,double dt){
    if(!s.fresh||!s.z_valid||!s.vz_valid)return bad("Z_VZ_STALE_OR_INVALID");
    if(!std::isfinite(s.z0)||!std::isfinite(s.z)||!std::isfinite(s.vz)||
       !std::isfinite(target)||!std::isfinite(dt)||dt<=0)return bad("NONFINITE_VERTICAL_INPUT_OR_DT");
    if(reset_set_&&(s.z_reset!=z_reset_||s.vz_reset!=vz_reset_))return bad("VERTICAL_ESTIMATOR_RESET");
    reset_set_=true;z_reset_=s.z_reset;vz_reset_=s.vz_reset;
    VerticalOutput o;o.height=s.z0-s.z;o.speed=-s.vz;
    if(!std::isfinite(o.height)||o.height>c_.max_height||o.height< -c_.ground_lower_margin||target<0||target>c_.max_height)
      return bad("HEIGHT_ENVELOPE_EXCEEDED");
    o.velocity_target=std::clamp(c_.height_gain*(target-o.height),-c_.max_descent,c_.max_climb);
    const double error=o.velocity_target-o.speed;
    const double trial=std::clamp(integral_+c_.integral_gain*error*dt,-c_.integral_limit,c_.integral_limit);
    double raw=c_.hover+c_.velocity_gain*error+trial;
    if(!std::isfinite(raw))return bad("NONFINITE_VERTICAL_OUTPUT");
    const double bounded=std::clamp(raw,c_.minimum,c_.maximum);
    const double reachable=last_thrust_+std::clamp(bounded-last_thrust_,-c_.slew*dt,c_.slew*dt);
    // Do not accumulate against either output saturation or a slew constraint.
    // Permit an existing integral to unwind towards zero.
    if((raw-reachable)*error<=0||integral_*error<0)integral_=trial;
    raw=c_.hover+c_.velocity_gain*error+integral_;
    if(!std::isfinite(raw))return bad("NONFINITE_VERTICAL_OUTPUT");
    const double command=std::clamp(raw,c_.minimum,c_.maximum);
    o.saturated=command!=raw;o.thrust=limit(command,dt);
    o.slew_limited=o.thrust!=command;o.healthy=true;return o;
  }
 private:
  static VerticalOutput bad(const char* reason){VerticalOutput o;o.reason=reason;return o;}
  double limit(double target,double dt){
    last_thrust_+=std::clamp(target-last_thrust_,-c_.slew*dt,c_.slew*dt);
    return last_thrust_;
  }
  VerticalConfig c_;double last_thrust_{},integral_{};
  bool reset_set_{};std::uint8_t z_reset_{},vz_reset_{};
};
} // namespace fc01::attitude
