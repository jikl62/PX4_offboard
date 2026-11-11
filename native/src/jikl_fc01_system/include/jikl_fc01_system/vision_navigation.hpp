#pragma once
#include "jikl_fc01_system/attitude_reference.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace fc01::vision {
using Vec3=std::array<double,3>;
using Quaternion=attitude::Quaternion;
inline double missing(){return std::numeric_limits<double>::quiet_NaN();}
template<std::size_t N>inline std::array<double,N> missing_array(){std::array<double,N> values{};values.fill(missing());return values;}
enum class Mode {Disabled=0,Hold=1,Cruise=2,Follow=3,Land=4};
enum Field : std::uint16_t {Position=1,Velocity=2,Attitude=4,AngularVelocity=8,PoseCovariance=16,TwistCovariance=32,Quality=64};
struct Config {
  bool enabled{};
  double state_timeout{.25},command_timeout{.30},max_measurement_delay{.20},future_tolerance{.05};
  double position_gain{.8},velocity_gain{1.2},max_speed{.5},max_acceleration{.6},max_tilt_deg{5};
  double max_yaw_rate{.35},max_vertical_speed{.15},max_radius{2},max_height{5},min_quality{.5};
  double max_position_jump{.75},max_velocity_jump{1};
  void validate() const;
};
struct State {
  std::uint64_t sequence{},stamp_ns{};
  double received_at{-1};
  std::uint32_t reset_counter{};
  std::string frame_id,body_frame_id,source_id,angular_velocity_source;
  bool initialized{},tracking{},calibration_valid{};
  std::uint16_t valid_fields{};
  Vec3 position{missing_array<3>()},velocity{missing_array<3>()},angular_velocity{missing_array<3>()};
  Quaternion attitude{missing_array<4>()};
  std::array<double,36> pose_covariance{missing_array<36>()},twist_covariance{missing_array<36>()};
  double quality{missing()};
};
struct Command {
  std::uint64_t sequence{},stamp_ns{};
  double received_at{-1};
  std::uint32_t reset_counter{};
  std::string source_id,frame_id;
  Mode mode{Mode::Disabled};
  bool valid{},position_valid{},yaw_valid{},sticks_valid{};
  Vec3 position{missing_array<3>()};
  double yaw{missing()};
  std::array<double,4> sticks{missing_array<4>()};
};
struct Input {
  State state;Command command;std::uint64_t ros_now_ns{};std::size_t state_publishers{},command_publishers{};
  // Local inbox evidence only, never populated from a ROS/wire field. This
  // preserves A->B->A source/reset changes between control-loop snapshots.
  std::uint64_t revocation_generation{};
};
struct Result {
  bool active{},request_land{};
  Mode mode{Mode::Disabled};
  std::string reason;
  Quaternion q{1,0,0,0};
  double target_height{missing()},yaw{missing()};
  Vec3 velocity_target{0,0,0},acceleration_target{0,0,0};
  std::uint64_t state_sequence{},command_sequence{};
  std::uint32_t reset_counter{};
};
namespace detail {
constexpr double pi=3.14159265358979323846;
constexpr double gravity=9.80665;
template<std::size_t N>inline bool finite(const std::array<double,N>& values){
  for(double value:values)if(!std::isfinite(value))return false;
  return true;
}
inline bool same(double first,double second){return first==second||(std::isnan(first)&&std::isnan(second));}
template<std::size_t N>inline bool same(const std::array<double,N>& first,const std::array<double,N>& second){
  for(std::size_t i=0;i<N;++i)if(!same(first[i],second[i]))return false;
  return true;
}
inline bool same(const State& first,const State& second){
  return first.sequence==second.sequence&&first.stamp_ns==second.stamp_ns&&same(first.received_at,second.received_at)&&
    first.reset_counter==second.reset_counter&&first.frame_id==second.frame_id&&first.body_frame_id==second.body_frame_id&&
    first.source_id==second.source_id&&first.angular_velocity_source==second.angular_velocity_source&&
    first.initialized==second.initialized&&first.tracking==second.tracking&&first.calibration_valid==second.calibration_valid&&
    first.valid_fields==second.valid_fields&&same(first.position,second.position)&&same(first.velocity,second.velocity)&&
    same(first.angular_velocity,second.angular_velocity)&&same(first.attitude,second.attitude)&&
    same(first.pose_covariance,second.pose_covariance)&&same(first.twist_covariance,second.twist_covariance)&&same(first.quality,second.quality);
}
inline bool same(const Command& first,const Command& second){
  return first.sequence==second.sequence&&first.stamp_ns==second.stamp_ns&&same(first.received_at,second.received_at)&&
    first.reset_counter==second.reset_counter&&first.source_id==second.source_id&&first.frame_id==second.frame_id&&
    first.mode==second.mode&&first.valid==second.valid&&first.position_valid==second.position_valid&&
    first.yaw_valid==second.yaw_valid&&first.sticks_valid==second.sticks_valid&&same(first.position,second.position)&&
    same(first.yaw,second.yaw)&&same(first.sticks,second.sticks);
}
inline bool clock_valid(std::uint64_t stamp,double received,std::uint64_t ros_now,double now,double timeout,const Config& c){
  if(!stamp||!ros_now||!std::isfinite(now)||!std::isfinite(received)||received<0||now<received||now-received>=timeout)return false;
  const long double age=(static_cast<long double>(ros_now)-static_cast<long double>(stamp))/1e9L;
  // The arrival transport cap is not another TTL. Extrapolate ROS at receipt
  // using the independent steady elapsed age; delay consumes source lifetime.
  const long double arrival_age=age-static_cast<long double>(now-received);
  return age>=-static_cast<long double>(c.future_tolerance)&&age<static_cast<long double>(timeout)&&
    arrival_age>=-static_cast<long double>(c.future_tolerance)&&
    arrival_age<=static_cast<long double>(c.max_measurement_delay);
}
inline bool covariance_valid(const std::array<double,36>& covariance){
  if(!finite(covariance))return false;
  for(std::size_t i=0;i<6;++i)if(covariance[i*6+i]<0)return false;
  return true;
}
inline double distance(const Vec3& first,const Vec3& second){
  return std::hypot(std::hypot(first[0]-second[0],first[1]-second[1]),first[2]-second[2]);
}
inline double wrap(double angle){return std::remainder(angle,2*pi);}
inline double yaw(const Quaternion& q){return std::atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));}
inline void bound_xy(Vec3& vector,double limit){
  const double length=std::hypot(vector[0],vector[1]);
  if(length>limit){vector[0]*=limit/length;vector[1]*=limit/length;}
}
// The requested body-down axis is [-a_N,-a_E,g], with an exact ZYX yaw.
// Positive right acceleration is positive roll; forward acceleration is negative pitch.
inline Quaternion attitude_target(const Vec3& acceleration,double yaw_target){
  const double forward=std::cos(yaw_target)*acceleration[0]+std::sin(yaw_target)*acceleration[1];
  const double right=-std::sin(yaw_target)*acceleration[0]+std::cos(yaw_target)*acceleration[1];
  const double roll=std::atan2(right,std::hypot(gravity,forward));
  const double pitch=std::atan2(-forward,gravity);
  const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
  const double cy=std::cos(yaw_target/2),sy=std::sin(yaw_target/2);
  return {cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
}
} // namespace detail
class Controller {
 public:
  explicit Controller(Config config={}):c_(config){c_.validate();}
  Result step(const Input& in,double now,double px4_height,double px4_yaw,double dt){
    if(stopped_)return last_;
    if(land_latched_)return last_;
    if(!c_.enabled){last_={};last_.reason="VISION_NAVIGATION_DISABLED";return last_;}
    if(!std::isfinite(now)||now<0||!std::isfinite(dt)||dt<=0||dt>std::min(c_.state_timeout,c_.command_timeout)||
       !std::isfinite(px4_height)||px4_height<0||px4_height>c_.max_height||!std::isfinite(px4_yaw))
      return reject("NAVIGATION_CONTROL_SAMPLE_INVALID");
    if(accepted_&&in.revocation_generation!=accepted_input_.revocation_generation)return land("NAVIGATION_INPUT_REVOKED");
    if(accepted_&&(now<last_now_||in.ros_now_ns<accepted_input_.ros_now_ns))return reject("NAVIGATION_CLOCK_REGRESSION");
    const auto command_error=validate_command(in,now);
    if(!command_error.empty())return reject(command_error);
    // An explicit, fresh Land request needs no metric vision estimate.
    if(in.command.mode==Mode::Land)return land("NAVIGATION_LAND_REQUESTED");
    if(in.command.mode==Mode::Disabled)return reject("NAVIGATION_COMMAND_DISABLED");
    const auto state_error=validate_state(in,now);
    if(!state_error.empty())return reject(state_error);
    if(in.state.reset_counter!=in.command.reset_counter)return reject("VISION_COMMAND_EPOCH_MISMATCH");
    if(accepted_){
      const auto& previous=accepted_input_;
      if(in.state.source_id!=previous.state.source_id||in.command.source_id!=previous.command.source_id)
        return reject("NAVIGATION_SOURCE_CHANGED");
      if(in.state.frame_id!=previous.state.frame_id||in.state.body_frame_id!=previous.state.body_frame_id||
         in.command.frame_id!=previous.command.frame_id)return reject("NAVIGATION_FRAME_CHANGED");
      if(in.state.reset_counter!=previous.state.reset_counter||in.command.reset_counter!=previous.command.reset_counter)
        return reject("NAVIGATION_ESTIMATOR_RESET");
      if(in.state.sequence<previous.state.sequence||in.command.sequence<previous.command.sequence)
        return reject("NAVIGATION_SEQUENCE_REGRESSION");
      if((in.state.sequence==previous.state.sequence&&!detail::same(in.state,previous.state))||
         (in.command.sequence==previous.command.sequence&&!detail::same(in.command,previous.command)))
        return reject("NAVIGATION_REPLAY_PAYLOAD_CHANGED");
      if((in.state.sequence>previous.state.sequence&&(in.state.stamp_ns<=previous.state.stamp_ns||in.state.received_at<previous.state.received_at))||
         (in.command.sequence>previous.command.sequence&&(in.command.stamp_ns<=previous.command.stamp_ns||in.command.received_at<previous.command.received_at)))
        return reject("NAVIGATION_SAMPLE_NOT_ADVANCING");
      if(detail::distance(in.state.position,previous.state.position)>c_.max_position_jump||
         detail::distance(in.state.velocity,previous.state.velocity)>c_.max_velocity_jump)
        return reject("NAVIGATION_ESTIMATOR_JUMP");
    }
    const bool entering=!accepted_||in.command.mode!=mode_;
    if(!accepted_){origin_=in.state.position;yaw_target_=yaw_goal_=detail::wrap(px4_yaw);}
    if(std::hypot(in.state.position[0]-origin_[0],in.state.position[1]-origin_[1])>c_.max_radius)
      return reject("NAVIGATION_RADIUS_EXCEEDED");
    if(entering&&in.command.mode==Mode::Hold){position_target_=in.state.position;yaw_goal_=detail::wrap(px4_yaw);}
    if(entering&&in.command.mode==Mode::Follow)follow_height_=px4_height;
    if(in.command.mode==Mode::Cruise){
      position_target_=in.command.position;
      if(std::hypot(position_target_[0]-origin_[0],position_target_[1]-origin_[1])>c_.max_radius)
        return reject("NAVIGATION_POINT_OUTSIDE_RADIUS");
      if(in.command.yaw_valid)yaw_goal_=detail::wrap(in.command.yaw);
    }
    Result output;output.mode=in.command.mode;output.reason="VISION_NAVIGATION_ACTIVE";
    output.state_sequence=in.state.sequence;output.command_sequence=in.command.sequence;output.reset_counter=in.state.reset_counter;
    if(in.command.mode==Mode::Follow){
      const double vision_yaw=detail::yaw(*attitude::normalized(in.state.attitude));
      const double forward=in.command.sticks[1]*c_.max_speed,right=in.command.sticks[0]*c_.max_speed;
      output.velocity_target={std::cos(vision_yaw)*forward-std::sin(vision_yaw)*right,
        std::sin(vision_yaw)*forward+std::cos(vision_yaw)*right,-in.command.sticks[3]*c_.max_vertical_speed};
      follow_height_=std::clamp(follow_height_+in.command.sticks[3]*c_.max_vertical_speed*dt,0.,c_.max_height);
      output.target_height=follow_height_;
      yaw_target_=detail::wrap(yaw_target_+in.command.sticks[2]*c_.max_yaw_rate*dt);
      yaw_goal_=yaw_target_;
    }else{
      // 先保留三轴位置误差乘增益的原始结果；垂向限幅不能掩盖计算溢出。
      const Vec3 raw_velocity_target{c_.position_gain*(position_target_[0]-in.state.position[0]),
        c_.position_gain*(position_target_[1]-in.state.position[1]),
        c_.position_gain*(position_target_[2]-in.state.position[2])};
      if(!detail::finite(raw_velocity_target))return reject("NAVIGATION_NONFINITE_PLAN");
      output.velocity_target=raw_velocity_target;
      output.velocity_target[2]=std::clamp(raw_velocity_target[2],-c_.max_vertical_speed,c_.max_vertical_speed);
      // Only the target is translated; PX4 Z/Vz remains the vertical feedback.
      output.target_height=px4_height+in.state.position[2]-position_target_[2];
      yaw_target_=detail::wrap(yaw_target_+std::clamp(detail::wrap(yaw_goal_-yaw_target_),-c_.max_yaw_rate*dt,c_.max_yaw_rate*dt));
    }
    if(!std::isfinite(output.target_height)||output.target_height<0||output.target_height>c_.max_height)
      return reject("NAVIGATION_HEIGHT_TARGET_OUTSIDE_ENVELOPE");
    // 有限分量的合速度仍可能溢出；必须在限幅前拒绝，避免乘以零后伪装成正常目标。
    if(!detail::finite(output.velocity_target)||
       !std::isfinite(std::hypot(output.velocity_target[0],output.velocity_target[1])))
      return reject("NAVIGATION_NONFINITE_PLAN");
    detail::bound_xy(output.velocity_target,c_.max_speed);
    output.acceleration_target={c_.velocity_gain*(output.velocity_target[0]-in.state.velocity[0]),
      c_.velocity_gain*(output.velocity_target[1]-in.state.velocity[1]),0};
    // 加速度使用世界 NED 的 m/s²；先检查误差增益结果及二维模长，再执行原有倾角/加速度限幅。
    if(!detail::finite(output.acceleration_target)||
       !std::isfinite(std::hypot(output.acceleration_target[0],output.acceleration_target[1])))
      return reject("NAVIGATION_NONFINITE_PLAN");
    const double acceleration_limit=std::min(c_.max_acceleration,detail::gravity*std::tan(c_.max_tilt_deg*detail::pi/180));
    detail::bound_xy(output.acceleration_target,acceleration_limit);
    output.yaw=yaw_target_;output.q=detail::attitude_target(output.acceleration_target,yaw_target_);
    if(!detail::finite(output.velocity_target)||!detail::finite(output.acceleration_target)||!attitude::normalized(output.q))
      return reject("NAVIGATION_NONFINITE_PLAN");
    output.active=true;mode_=in.command.mode;accepted_input_=in;accepted_=true;last_now_=now;last_=output;
    return last_;
  }
  bool current(const Input& in,double now)const{
    return c_.enabled&&accepted_&&!stopped_&&!land_latched_&&last_.active&&std::isfinite(now)&&now>=last_now_&&
      in.ros_now_ns>=accepted_input_.ros_now_ns&&in.revocation_generation==accepted_input_.revocation_generation&&
      validate_state(in,now).empty()&&validate_command(in,now).empty()&&
      detail::same(in.state,accepted_input_.state)&&detail::same(in.command,accepted_input_.command)&&
      in.state.reset_counter==last_.reset_counter&&in.state.sequence==last_.state_sequence&&in.command.sequence==last_.command_sequence;
  }
  const Result& last()const{return last_;}
  // Stop is an ownership release, not a landing command and not a reset API.
  void stop(){stopped_=true;last_={};last_.reason="VISION_NAVIGATION_STOPPED";}
 private:
  std::string validate_state(const Input& in,double now)const{
    const auto& state=in.state;
    if(in.state_publishers!=1)return "VISION_STATE_SOURCE_COUNT";
    if(!state.sequence||state.source_id.empty())return "VISION_STATE_IDENTITY_INVALID";
    if(state.frame_id!="fc01_vision_ned"||state.body_frame_id!="base_link_frd")return "VISION_COORDINATE_FRAME_INVALID";
    if(!detail::clock_valid(state.stamp_ns,state.received_at,in.ros_now_ns,now,c_.state_timeout,c_))return "VISION_STATE_TIME_INVALID";
    constexpr std::uint16_t required=Position|Velocity|Attitude|Quality;
    constexpr std::uint16_t allowed=required|AngularVelocity|PoseCovariance|TwistCovariance;
    if((state.valid_fields&required)!=required||(state.valid_fields&~allowed)!=0)return "VISION_REQUIRED_FIELDS_INVALID";
    if(!state.initialized||!state.tracking||!state.calibration_valid)return "VISION_TRACKING_OR_CALIBRATION_INVALID";
    if(!detail::finite(state.position)||!detail::finite(state.velocity)||!attitude::normalized(state.attitude))return "VISION_METRIC_STATE_INVALID";
    if(!std::isfinite(state.quality)||state.quality<c_.min_quality||state.quality>1)return "VISION_QUALITY_INVALID";
    if((state.valid_fields&AngularVelocity)&&(!detail::finite(state.angular_velocity)||state.angular_velocity_source.empty()))return "VISION_ANGULAR_VELOCITY_SOURCE_INVALID";
    if(((state.valid_fields&PoseCovariance)&&!detail::covariance_valid(state.pose_covariance))||
       ((state.valid_fields&TwistCovariance)&&!detail::covariance_valid(state.twist_covariance)))return "VISION_COVARIANCE_INVALID";
    return {};
  }
  std::string validate_command(const Input& in,double now)const{
    const auto& command=in.command;
    if(in.command_publishers!=1)return "NAVIGATION_COMMAND_SOURCE_COUNT";
    if(!command.valid||!command.sequence||command.source_id.empty())return "NAVIGATION_COMMAND_INVALID";
    if(command.frame_id!="fc01_vision_ned")return "NAVIGATION_COMMAND_FRAME_INVALID";
    if(!detail::clock_valid(command.stamp_ns,command.received_at,in.ros_now_ns,now,c_.command_timeout,c_))return "NAVIGATION_COMMAND_TIME_INVALID";
    if(command.mode!=Mode::Disabled&&command.mode!=Mode::Hold&&command.mode!=Mode::Cruise&&command.mode!=Mode::Follow&&command.mode!=Mode::Land)
      return "NAVIGATION_COMMAND_MODE_INVALID";
    if(command.position_valid&&!detail::finite(command.position))return "NAVIGATION_COMMAND_POSITION_INVALID";
    if(command.yaw_valid&&!std::isfinite(command.yaw))return "NAVIGATION_COMMAND_YAW_INVALID";
    if(command.sticks_valid){for(double value:command.sticks)if(!std::isfinite(value)||std::abs(value)>1)return "NAVIGATION_COMMAND_AXES_INVALID";}
    if(command.mode==Mode::Cruise&&!command.position_valid)return "NAVIGATION_CRUISE_POINT_REQUIRED";
    if(command.mode==Mode::Follow&&!command.sticks_valid)return "NAVIGATION_FOLLOW_AXES_REQUIRED";
    return {};
  }
  Result reject(const std::string& reason){
    if(accepted_)return land(reason);
    last_={};last_.reason=reason;return last_;
  }
  Result land(const std::string& reason){land_latched_=true;last_={};last_.request_land=true;last_.mode=Mode::Land;last_.reason=reason;return last_;}
  Config c_;Result last_;Input accepted_input_;
  bool accepted_{},land_latched_{},stopped_{};
  Mode mode_{Mode::Disabled};
  Vec3 origin_{0,0,0},position_target_{0,0,0};
  double yaw_target_{},yaw_goal_{},follow_height_{},last_now_{};
};
inline void Config::validate()const{
  for(double value:{state_timeout,command_timeout,max_measurement_delay,future_tolerance,position_gain,velocity_gain,
      max_speed,max_acceleration,max_tilt_deg,max_yaw_rate,max_vertical_speed,max_radius,max_height,max_position_jump,max_velocity_jump})
    if(!std::isfinite(value)||value<=0)throw std::invalid_argument("VISION_NAVIGATION_CONFIG_REQUIRES_FINITE_POSITIVE_LIMITS");
  if(max_tilt_deg>=90||max_height>5||!std::isfinite(min_quality)||min_quality<0||min_quality>1)
    throw std::invalid_argument("VISION_NAVIGATION_CONFIG_OUTSIDE_ENVELOPE");
}
class Inbox {
 public:
  bool state(State value,double received_at){
    const bool changed=state_seen_&&(value.source_id!=state_.source_id||value.reset_counter!=state_.reset_counter||
      value.frame_id!=state_.frame_id||value.body_frame_id!=state_.body_frame_id);
    const bool withdrawn=state_seen_&&state_retracted(value)&&!state_retracted(state_)&&value.stamp_ns&&value.stamp_ns>=state_.stamp_ns;
    if(!changed&&!withdrawn&&(!value.sequence||value.sequence<=state_sequence_))return false;
    if(changed||withdrawn)++revocation_generation_;
    state_sequence_=changed?value.sequence:std::max(state_sequence_,value.sequence);
    state_seen_=true;
    value.received_at=received_at;
    if(!(value.valid_fields&Position))value.position=missing_array<3>();
    if(!(value.valid_fields&Velocity))value.velocity=missing_array<3>();
    if(!(value.valid_fields&Attitude))value.attitude=missing_array<4>();
    if(!(value.valid_fields&AngularVelocity)){value.angular_velocity=missing_array<3>();value.angular_velocity_source.clear();}
    if(!(value.valid_fields&PoseCovariance))value.pose_covariance=missing_array<36>();
    if(!(value.valid_fields&TwistCovariance))value.twist_covariance=missing_array<36>();
    if(!(value.valid_fields&Quality))value.quality=missing();
    state_=std::move(value);return true;
  }
  bool command(Command value,double received_at){
    const bool changed=command_seen_&&(value.source_id!=command_.source_id||value.reset_counter!=command_.reset_counter||value.frame_id!=command_.frame_id);
    const bool withdrawn=command_seen_&&command_retracted(value)&&!command_retracted(command_)&&value.stamp_ns&&value.stamp_ns>=command_.stamp_ns;
    if(!changed&&!withdrawn&&(!value.sequence||value.sequence<=command_sequence_))return false;
    if(changed||withdrawn)++revocation_generation_;
    command_sequence_=changed?value.sequence:std::max(command_sequence_,value.sequence);
    command_seen_=true;
    value.received_at=received_at;
    if(!value.position_valid)value.position=missing_array<3>();
    if(!value.yaw_valid)value.yaw=missing();
    if(!value.sticks_valid)value.sticks=missing_array<4>();
    command_=std::move(value);return true;
  }
  Input snapshot(std::uint64_t now,std::size_t states,std::size_t commands)const{
    Input in;in.state=state_;in.command=command_;in.ros_now_ns=now;in.state_publishers=states;in.command_publishers=commands;
    in.revocation_generation=revocation_generation_;return in;
  }
 private:
  static bool state_retracted(const State& value){
    constexpr std::uint16_t required=Position|Velocity|Attitude|Quality;
    if(!value.initialized||!value.tracking||!value.calibration_valid||(value.valid_fields&required)!=required||
       !detail::finite(value.position)||!detail::finite(value.velocity)||!attitude::normalized(value.attitude)||
       !std::isfinite(value.quality)||value.quality<0||value.quality>1)return true;
    if((value.valid_fields&AngularVelocity)&&(!detail::finite(value.angular_velocity)||value.angular_velocity_source.empty()))return true;
    return ((value.valid_fields&PoseCovariance)&&!detail::covariance_valid(value.pose_covariance))||
      ((value.valid_fields&TwistCovariance)&&!detail::covariance_valid(value.twist_covariance));
  }
  static bool command_retracted(const Command& value){
    if(!value.valid||value.mode==Mode::Disabled||value.mode==Mode::Land)return true;
    if(value.mode!=Mode::Hold&&value.mode!=Mode::Cruise&&value.mode!=Mode::Follow)return true;
    if((value.position_valid&&!detail::finite(value.position))||(value.yaw_valid&&!std::isfinite(value.yaw)))return true;
    if(value.mode==Mode::Cruise&&!value.position_valid)return true;
    if(value.mode==Mode::Follow&&!value.sticks_valid)return true;
    if(value.sticks_valid)for(double axis:value.sticks)if(!std::isfinite(axis)||std::abs(axis)>1)return true;
    return false;
  }
  State state_;Command command_;
  bool state_seen_{},command_seen_{};
  std::uint64_t state_sequence_{},command_sequence_{},revocation_generation_{};
};
} // namespace fc01::vision
