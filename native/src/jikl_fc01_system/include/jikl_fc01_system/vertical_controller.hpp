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
  bool closed_loop{},takeoff_limited{};
  double proportional_thrust{},integral_thrust{},raw_thrust{},thrust_ceiling{};
};
class VerticalController {
 public:
  explicit VerticalController(VerticalConfig config):c_(config){c_.validate();}
  void reset(double initial=0){last_thrust_=initial;integral_=0;reset_set_=false;integral_frozen_=false;}
  double integral() const{return integral_;}
  void freeze_integral(bool frozen){integral_frozen_=frozen;}
  // At the first height-feedback tick retain only the bounded reserve
  // actually built above the configured hover. This is a trim, not a floor:
  // velocity feedback can immediately command braking through the same slew
  // state, and the ordinary integrator can unwind it. Never infer lift from it.
  void retain_lift_reserve(){
    integral_=std::clamp(last_thrust_-c_.hover,0.,c_.integral_limit);
  }
  // Landing ramp and flight ramp share the same physical slew state.
  VerticalOutput ramp(double desired,double dt){
    if(!std::isfinite(desired)||!std::isfinite(dt)||dt<=0)return bad("INVALID_RAMP");
    VerticalOutput o;o.healthy=true;
    const double bounded=std::clamp(desired,0.,c_.maximum);
    o.saturated=bounded!=desired;o.thrust=limit(bounded,dt);
    o.slew_limited=o.thrust!=bounded;return o;
  }
  VerticalOutput step(const VerticalSample& s,double target,double dt,double velocity_feedforward=0,
                      double takeoff_ceiling=1.){
    if(!s.fresh||!s.z_valid||!s.vz_valid)return bad("Z_VZ_STALE_OR_INVALID");
    if(!std::isfinite(s.z0)||!std::isfinite(s.z)||!std::isfinite(s.vz)||
       !std::isfinite(target)||!std::isfinite(dt)||dt<=0||
       !std::isfinite(velocity_feedforward))return bad("NONFINITE_VERTICAL_INPUT_OR_DT");
    if(!std::isfinite(takeoff_ceiling)||takeoff_ceiling<0||takeoff_ceiling>1)
      return bad("INVALID_TAKEOFF_THRUST_CEILING");
    if(reset_set_&&(s.z_reset!=z_reset_||s.vz_reset!=vz_reset_))return bad("VERTICAL_ESTIMATOR_RESET");
    reset_set_=true;z_reset_=s.z_reset;vz_reset_=s.vz_reset;
    VerticalOutput o;o.height=s.z0-s.z;o.speed=-s.vz;
    if(!std::isfinite(o.height)||o.height>c_.max_height||o.height< -c_.ground_lower_margin||target<0||target>c_.max_height)
      return bad("HEIGHT_ENVELOPE_EXCEEDED");
    // A moving height reference requires its trajectory velocity even at
    // zero position error. Limit the combined request, including feedback.
    o.velocity_target=std::clamp(velocity_feedforward+c_.height_gain*(target-o.height),
                                 -c_.max_descent,c_.max_climb);
    const double error=o.velocity_target-o.speed;
    const double trial=integral_frozen_?integral_:
      std::clamp(integral_+c_.integral_gain*error*dt,-c_.integral_limit,c_.integral_limit);
    double raw=c_.hover+c_.velocity_gain*error+trial;
    if(!std::isfinite(raw))return bad("NONFINITE_VERTICAL_OUTPUT");
    // Before certified liftoff the scheduled ramp is an upper bound, not a
    // new thrust floor. Include it in anti-windup, rather than clamping after
    // step() has already advanced its physical slew and integral states.
    const double ceiling=std::min(takeoff_ceiling,c_.maximum);
    // Open-loop takeoff already permits zero. If this early schedule is
    // below the airborne minimum, preserve that braking range instead of
    // turning the temporary ceiling into an exact-thrust command.
    const double minimum=ceiling<c_.minimum?0.:c_.minimum;
    const double bounded=std::clamp(raw,minimum,ceiling);
    const double reachable=last_thrust_+std::clamp(bounded-last_thrust_,-c_.slew*dt,c_.slew*dt);
    // Do not accumulate against either output saturation or a slew constraint.
    // Permit an existing integral to unwind towards zero, never through it
    // into windup of the opposite sign while the constraint remains active.
    if(!integral_frozen_&&(raw-reachable)*error<=0)integral_=trial;
    else if(!integral_frozen_&&integral_*error<0)
      integral_=std::clamp(trial,std::min(0.,integral_),std::max(0.,integral_));
    raw=c_.hover+c_.velocity_gain*error+integral_;
    if(!std::isfinite(raw))return bad("NONFINITE_VERTICAL_OUTPUT");
    const double command=std::clamp(raw,minimum,ceiling);
    // Deliberate takeoff scheduling is not persistent physical saturation.
    // The existing physical-envelope flag and Safety timeout remain separate.
    o.saturated=raw<minimum||raw>c_.maximum;
    o.closed_loop=true;o.takeoff_limited=raw>ceiling&&ceiling<c_.maximum;
    o.proportional_thrust=c_.velocity_gain*error;o.integral_thrust=integral_;
    o.raw_thrust=raw;o.thrust_ceiling=ceiling;o.thrust=limit(command,dt);
    o.slew_limited=o.thrust!=command;o.healthy=true;return o;
  }
 private:
  static VerticalOutput bad(const char* reason){VerticalOutput o;o.reason=reason;return o;}
  double limit(double target,double dt){
    last_thrust_+=std::clamp(target-last_thrust_,-c_.slew*dt,c_.slew*dt);
    return last_thrust_;
  }
  VerticalConfig c_;double last_thrust_{},integral_{};
  bool reset_set_{},integral_frozen_{};std::uint8_t z_reset_{},vz_reset_{};
};
} // namespace fc01::attitude
