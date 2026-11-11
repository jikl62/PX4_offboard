#pragma once
#include "jikl_fc01_system/vision_navigation.hpp"
#include "jikl_fc01_system/vision_ros_adapter.hpp"
#include <string>

// Offline integration preparation only. Nothing here creates a Runtime output,
// a PX4 message, an ownership lease, or permission to operate an aircraft.
namespace fc01::velocity_input {
using Vec3=vision::Vec3;
inline bool identity_valid(const std::string& value){
  // Resource bound, matching the standalone diagnostic; not a flight gain.
  return !value.empty()&&value.size()<=256&&value.find_first_not_of(" \t\r\n")!=std::string::npos;
}
enum class Mode {Disabled,VelocityPreview};
struct Target {
  std::uint16_t schema_version{};
  std::uint64_t sequence{},stamp_ns{};
  double received_at{-1};
  std::uint32_t reset_counter{}; // Producer session; independent of state reset.
  std::string source_id,frame_id;
  bool valid{};
  Vec3 body_velocity{vision::missing_array<3>()}; // Full FRD vector, m/s.
};
template<class Message> Target from_message(const Message& message){
  Target out;out.schema_version=message.schema_version;out.sequence=message.sequence;
  out.stamp_ns=vision::message_stamp_ns(message.stamp);out.reset_counter=message.reset_counter;
  out.source_id=message.source_id;out.frame_id=message.frame_id;out.valid=message.valid;
  out.body_velocity={message.vx,message.vy,message.vz};return out;
}
struct Policy {
  // No new production timing/speed assumptions. Supply reviewed values to use
  // this offline preview; default construction never qualifies a target.
  double state_timeout{},command_timeout{},max_measurement_delay{},future_tolerance{};
  double max_command_speed_mps{},max_attitude_sample_skew_sec{},max_attitude_step_rad{};
  bool configured()const{
    for(double value:{state_timeout,command_timeout,max_measurement_delay,
                      max_command_speed_mps,max_attitude_sample_skew_sec,max_attitude_step_rad})
      if(!std::isfinite(value)||value<=0)return false;
    return std::isfinite(future_tolerance)&&future_tolerance>=0&&max_attitude_step_rad<=vision::detail::pi;
  }
};
struct Preview {
  bool valid{},control_authority{false};
  std::string reason;
  Vec3 body_target{vision::missing_array<3>()},world_target{vision::missing_array<3>()};
  Vec3 measured_world_velocity{vision::missing_array<3>()},velocity_error{vision::missing_array<3>()};
  Vec3 horizontal_acceleration{vision::missing_array<3>()};
  attitude::Quaternion attitude_preview{vision::missing_array<4>()};
  // This value has NOT been passed to the existing vertical controller.
  double world_vertical_velocity_preview{vision::missing()},yaw_reference{vision::missing()};
  std::uint64_t state_sequence{},command_sequence{};
  std::uint32_t state_reset{},command_reset{};
};
inline Vec3 rotate_body_to_world(const attitude::Quaternion& q,const Vec3& vector){
  const double w=q[0],x=q[1],y=q[2],z=q[3];
  return {(1-2*(y*y+z*z))*vector[0]+2*(x*y-w*z)*vector[1]+2*(x*z+w*y)*vector[2],
          2*(x*y+w*z)*vector[0]+(1-2*(x*x+z*z))*vector[1]+2*(y*z-w*x)*vector[2],
          2*(x*z-w*y)*vector[0]+2*(y*z+w*x)*vector[1]+(1-2*(x*x+y*y))*vector[2]};
}
class Session {
 public:
  explicit Session(Policy policy={},vision::Config candidate={}):p_(policy),c_(candidate){
    c_.validate();timing_=c_;timing_.max_measurement_delay=p_.max_measurement_delay;
    timing_.future_tolerance=p_.future_tolerance;
  }
  // Clears ONLY an offline preview session and all cached samples. This is not
  // a ground-admission API and cannot clear an aircraft's fault or authority.
  void begin_attempt(){
    state_={};target_={};state_seen_=target_seen_=false;state_sources_=target_sources_=0;
    error_.clear();last_now_=-1;last_ros_now_=0;
  }
  const std::string& first_error()const{return error_;}
  void observe_publishers(std::size_t states,std::size_t commands){
    state_sources_=states;target_sources_=commands;
    if(states>1||(state_seen_&&states!=1))latch("STATE_SOURCE_COUNT");
    if(commands>1||(target_seen_&&commands!=1))latch("COMMAND_SOURCE_COUNT");
  }
  // Expected input is an upstream validated/aligned state, including any
  // declared covariance. Optional covariance checks here reuse the existing
  // finite/nonnegative-diagonal guard; they do NOT prove PSD or uncertainty
  // bounds. The diagnostic state-validation layer performs those raw checks.
  bool state(vision::State value,double received_at,std::uint64_t ros_now_ns,std::size_t publishers=1){
    state_sources_=publishers;
    if(!error_.empty()||!clock_observation(received_at,ros_now_ns))return false;
    if(publishers!=1)return latch("STATE_SOURCE_COUNT");
    value.received_at=received_at;
    if(value.sequence==0||!identity_valid(value.source_id))return latch("STATE_IDENTITY_INVALID");
    if(value.frame_id!="fc01_vision_ned"||value.body_frame_id!="base_link_frd")return latch("STATE_FRAME_INVALID");
    constexpr auto required=vision::Velocity|vision::Attitude|vision::Quality;
    if((value.valid_fields&required)!=required||(value.valid_fields&~std::uint16_t(127))||
       !value.initialized||!value.tracking||!value.calibration_valid||
       !vision::detail::finite(value.velocity)||!attitude::normalized(value.attitude)||
       !std::isfinite(value.quality)||value.quality<c_.min_quality||value.quality>1)
      return latch("STATE_MEASUREMENT_INVALID");
    if(((value.valid_fields&vision::Position)&&!vision::detail::finite(value.position))||
       ((value.valid_fields&vision::AngularVelocity)&&(!vision::detail::finite(value.angular_velocity)||value.angular_velocity_source.empty()))||
       ((value.valid_fields&vision::PoseCovariance)&&!vision::detail::covariance_valid(value.pose_covariance))||
       ((value.valid_fields&vision::TwistCovariance)&&!vision::detail::covariance_valid(value.twist_covariance)))
      return latch("STATE_OPTIONAL_FIELD_INVALID");
    if(!time_valid(value.stamp_ns,received_at,ros_now_ns,received_at,p_.state_timeout))return latch("STATE_TIME_INVALID");
    if(state_seen_){
      if(value.source_id!=state_.source_id)return latch("STATE_SOURCE_CHANGED");
      if(value.reset_counter!=state_.reset_counter)return latch("STATE_ESTIMATOR_RESET");
      if(value.sequence<state_.sequence)return latch("STATE_SEQUENCE_REGRESSION");
      if(value.sequence==state_.sequence){
        value.received_at=state_.received_at;
        if(!vision::detail::same(value,state_))return latch("STATE_DUPLICATE_CHANGED");
        return false; // Exact duplicates never renew source/receive lifetime.
      }
      if(value.stamp_ns<=state_.stamp_ns)return latch("STATE_STAMP_NOT_ADVANCING");
      if(vision::detail::distance(value.velocity,state_.velocity)>c_.max_velocity_jump||
         ((value.valid_fields&state_.valid_fields&vision::Position)&&
          vision::detail::distance(value.position,state_.position)>c_.max_position_jump))
        return latch("STATE_DISCONTINUITY_REQUIRES_NEW_ATTEMPT");
      const auto previous_q=*attitude::normalized(state_.attitude),current_q=*attitude::normalized(value.attitude);
      double dot=0;for(std::size_t i=0;i<4;++i)dot+=previous_q[i]*current_q[i];
      if(2*std::acos(std::clamp(std::abs(dot),0.,1.))>p_.max_attitude_step_rad)
        return latch("STATE_ATTITUDE_DISCONTINUITY_REQUIRES_NEW_ATTEMPT");
    }
    state_=std::move(value);state_seen_=true;return true;
  }
  bool command(Target value,double received_at,std::uint64_t ros_now_ns,std::size_t publishers=1){
    target_sources_=publishers;
    if(!error_.empty()||!clock_observation(received_at,ros_now_ns))return false;
    if(publishers!=1)return latch("COMMAND_SOURCE_COUNT");
    value.received_at=received_at;
    if(value.schema_version!=1||!value.valid||!value.sequence||!identity_valid(value.source_id))return latch("COMMAND_IDENTITY_OR_VALIDITY_INVALID");
    if(value.frame_id!="body_frd")return latch("COMMAND_FRAME_INVALID");
    if(!vision::detail::finite(value.body_velocity))return latch("COMMAND_NONFINITE_VELOCITY");
    if(std::hypot(std::hypot(value.body_velocity[0],value.body_velocity[1]),value.body_velocity[2])>p_.max_command_speed_mps)
      return latch("COMMAND_SPEED_OUTSIDE_POLICY");
    if(!time_valid(value.stamp_ns,received_at,ros_now_ns,received_at,p_.command_timeout))return latch("COMMAND_TIME_INVALID");
    if(target_seen_){
      if(value.source_id!=target_.source_id)return latch("COMMAND_SOURCE_CHANGED");
      if(value.reset_counter!=target_.reset_counter)return latch("COMMAND_PRODUCER_RESET");
      if(value.sequence<target_.sequence)return latch("COMMAND_SEQUENCE_REGRESSION");
      if(value.sequence==target_.sequence){
        if(value.stamp_ns!=target_.stamp_ns||value.schema_version!=target_.schema_version||value.valid!=target_.valid||
           value.frame_id!=target_.frame_id||value.body_velocity!=target_.body_velocity)return latch("COMMAND_DUPLICATE_CHANGED");
        return false;
      }
      if(value.stamp_ns<=target_.stamp_ns)return latch("COMMAND_STAMP_NOT_ADVANCING");
    }
    target_=std::move(value);target_seen_=true;return true;
  }
  Preview preview(Mode mode,double now,std::uint64_t ros_now_ns,double trusted_yaw_reference){
    Preview out;
    if(mode==Mode::Disabled){out.reason="VELOCITY_PREVIEW_DISABLED";return out;}
    if(mode!=Mode::VelocityPreview)return rejected("VELOCITY_PREVIEW_MODE_INVALID");
    if(!error_.empty())return rejected(error_);
    if(!p_.configured())return rejected("VELOCITY_PREVIEW_POLICY_UNCONFIGURED");
    if(!clock_observation(now,ros_now_ns))return rejected(error_);
    if(!state_seen_||!target_seen_){out.reason=!state_seen_?"VELOCITY_PREVIEW_AWAIT_STATE":"VELOCITY_PREVIEW_AWAIT_COMMAND";return out;}
    if(state_sources_!=1||target_sources_!=1)return rejected("VELOCITY_PREVIEW_SOURCE_COUNT");
    if(!time_valid(state_.stamp_ns,state_.received_at,ros_now_ns,now,p_.state_timeout))return rejected("STATE_TIME_INVALID");
    if(!time_valid(target_.stamp_ns,target_.received_at,ros_now_ns,now,p_.command_timeout))return rejected("COMMAND_TIME_INVALID");
    const long double skew=std::abs(static_cast<long double>(state_.stamp_ns)-static_cast<long double>(target_.stamp_ns))/1e9L;
    if(skew>p_.max_attitude_sample_skew_sec)return rejected("STATE_COMMAND_ATTITUDE_TIME_SKEW");
    if(!std::isfinite(trusted_yaw_reference))return rejected("YAW_REFERENCE_INVALID");
    out.body_target=target_.body_velocity;
    out.world_target=rotate_body_to_world(*attitude::normalized(state_.attitude),out.body_target);
    vision::detail::bound_xy(out.world_target,c_.max_speed);
    out.world_target[2]=std::clamp(out.world_target[2],-c_.max_vertical_speed,c_.max_vertical_speed);
    out.measured_world_velocity=state_.velocity;
    for(std::size_t i=0;i<3;++i)out.velocity_error[i]=out.world_target[i]-state_.velocity[i];
    out.horizontal_acceleration={c_.velocity_gain*out.velocity_error[0],c_.velocity_gain*out.velocity_error[1],0};
    const double limit=std::min(c_.max_acceleration,vision::detail::gravity*std::tan(c_.max_tilt_deg*vision::detail::pi/180));
    vision::detail::bound_xy(out.horizontal_acceleration,limit);
    out.yaw_reference=vision::detail::wrap(trusted_yaw_reference);
    out.attitude_preview=vision::detail::attitude_target(out.horizontal_acceleration,out.yaw_reference);
    out.world_vertical_velocity_preview=out.world_target[2];
    if(!vision::detail::finite(out.world_target)||!vision::detail::finite(out.velocity_error)||
       !vision::detail::finite(out.horizontal_acceleration)||!attitude::normalized(out.attitude_preview))return rejected("VELOCITY_PREVIEW_NONFINITE_RESULT");
    out.state_sequence=state_.sequence;out.command_sequence=target_.sequence;
    out.state_reset=state_.reset_counter;out.command_reset=target_.reset_counter;
    out.valid=true;out.reason="VELOCITY_PREVIEW_ONLY_NOT_DISPATCHED";return out;
  }
 private:
  bool latch(const std::string& reason){if(error_.empty())error_=reason;return false;}
  Preview rejected(const std::string& reason){latch(reason);Preview out;out.reason=error_;return out;}
  bool clock_observation(double now,std::uint64_t ros_now_ns){
    if(!p_.configured())return latch("VELOCITY_PREVIEW_POLICY_UNCONFIGURED");
    if(!std::isfinite(now)||now<0||!ros_now_ns||now<last_now_||ros_now_ns<last_ros_now_)return latch("RECEIVER_CLOCK_INVALID_OR_REGRESSED");
    // An already observed gap cannot disappear when both streams deliver a
    // healthy tail before the next preview call.
    if(state_seen_&&!time_valid(state_.stamp_ns,state_.received_at,ros_now_ns,now,p_.state_timeout))return latch("STATE_TIME_INVALID");
    if(target_seen_&&!time_valid(target_.stamp_ns,target_.received_at,ros_now_ns,now,p_.command_timeout))return latch("COMMAND_TIME_INVALID");
    last_now_=now;last_ros_now_=ros_now_ns;return true;
  }
  bool time_valid(std::uint64_t stamp,double received,std::uint64_t ros_now,double now,double timeout)const{
    return vision::detail::clock_valid(stamp,received,ros_now,now,timeout,timing_);
  }
  Policy p_;vision::Config c_,timing_;
  vision::State state_;Target target_;
  bool state_seen_{},target_seen_{};
  std::size_t state_sources_{},target_sources_{};
  std::string error_;double last_now_{-1};std::uint64_t last_ros_now_{};
};
} // namespace fc01::velocity_input
