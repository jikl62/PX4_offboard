#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace jikl_velocity {
constexpr double pi=3.14159265358979323846;
inline double nan() {return std::numeric_limits<double>::quiet_NaN();}
struct Vec {double x=0,y=0,z=0;};
struct Velocity {double forward=0,left=0,up=0,ccw=0;};
inline bool finite(Vec p) {return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
inline bool finite(Velocity v) {return std::isfinite(v.forward)&&std::isfinite(v.left)&&std::isfinite(v.up)&&std::isfinite(v.ccw);}
inline bool zero(Velocity v) {return finite(v)&&std::abs(v.forward)+std::abs(v.left)+std::abs(v.up)+std::abs(v.ccw)<1e-5;}
inline double norm(Vec v) {return std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);}
inline double wrap(double x) {return std::remainder(x,2*pi);}
inline Vec heading_to_ned(Velocity v,double yaw) {
  return {std::cos(yaw)*v.forward+std::sin(yaw)*v.left,
          std::sin(yaw)*v.forward-std::cos(yaw)*v.left,-v.up};
}
inline bool stamp_valid(double stamp,double now,double previous,double age) {
  return std::isfinite(stamp)&&std::isfinite(now)&&stamp>0&&stamp>previous&&
         now-stamp<=age&&stamp-now<=.1;
}
inline double approach(double old,double wanted,double delta) {
  return old+std::clamp(wanted-old,-delta,delta);
}

struct Config {
  double height=6.,climb_speed=.7,reference_lead=.6;
  double xy_speed=.8,z_speed=.5,yaw_speed=.5;
  double xy_accel=.5,z_accel=.3,yaw_accel=.6;
  double command_timeout=.5,land_after_loss=3.;
  double prestream=2.,takeoff_timeout=60.,settle_time=3.;
  double height_tolerance=.15,speed_tolerance=.2;
  double mode_timeout=8.,land_timeout=60.;
  double max_height=8.,max_radius=20.;
  void validate() const {
    const double values[]={height,climb_speed,reference_lead,xy_speed,z_speed,yaw_speed,
      xy_accel,z_accel,yaw_accel,command_timeout,land_after_loss,prestream,takeoff_timeout,
      settle_time,height_tolerance,speed_tolerance,mode_timeout,land_timeout,max_height,max_radius};
    for(double value:values) if(!std::isfinite(value)||value<=0.)
      throw std::invalid_argument("all control parameters must be finite and positive");
    if(prestream<1.1||command_timeout<.1||land_after_loss<=command_timeout||height>max_height-1.)
      throw std::invalid_argument("invalid prestream, timeout or height limits");
  }
};
struct Feedback {
  Vec position,velocity;
  double yaw=0;
  bool fresh=false,valid=false,heading_ok=false,preflight=false;
  bool armed=false,landed=false,offboard=false,landing=false,failsafe=false,exclusive=false;
  uint32_t reset_id=0;
};
enum class Phase {Idle,Prestream,Offboard,Arm,Takeoff,Hold,Velocity,Brake,Landing,Done,Released,Fault};
inline const char* name(Phase s) {
  switch(s) {
    case Phase::Idle:return "IDLE";case Phase::Prestream:return "PRESTREAM";
    case Phase::Offboard:return "REQUEST_OFFBOARD";case Phase::Arm:return "REQUEST_ARM";
    case Phase::Takeoff:return "TAKEOFF";case Phase::Hold:return "HOLD";
    case Phase::Velocity:return "VELOCITY";case Phase::Brake:return "BRAKE";
    case Phase::Landing:return "LANDING";case Phase::Done:return "DONE";
    case Phase::Released:return "RELEASED";case Phase::Fault:return "FAULT";
  }
  return "UNKNOWN";
}
enum class Control {None,Position,Velocity};
enum class VehicleCommand {None,Offboard,Arm,Land};
struct Output {
  Control control=Control::None;
  VehicleCommand command=VehicleCommand::None;
  Vec position{nan(),nan(),nan()},velocity{nan(),nan(),nan()};
  double yaw=nan(),yaw_rate=nan();
};

class Core {
 public:
  explicit Core(Config c={}) : c_(c) {c_.validate();}
  Phase phase() const {return phase_;}
  const std::string& reason() const {return reason_;}
  Vec origin() const {return origin_;}
  bool active() const {return phase_>=Phase::Prestream&&phase_<=Phase::Landing;}
  bool accepts_motion() const {return phase_==Phase::Hold||phase_==Phase::Velocity||phase_==Phase::Brake;}
  double command_age(double t) const {return received_>=0?t-received_:1e9;}
  bool command_fresh(double t) const {return command_valid_&&command_age(t)>=0&&command_age(t)<=c_.command_timeout;}
  std::string not_ready(const Feedback& f,double t) const {
    if(phase_==Phase::Fault) return "FAULT: inspect cause, then restart this controller on safe ground";
    if(active())return "flight already active";
    if(!f.fresh)return "waiting for fresh PX4 position/status/land telemetry";
    if(!f.valid||!finite(f.position)||!finite(f.velocity)||!std::isfinite(f.yaw))return "local state estimate invalid";
    if(!f.exclusive)return "another PX4 setpoint publisher or not exactly one command source";
    if(f.armed||!f.landed||f.offboard)return "must be landed, disarmed and out of Offboard";
    if(!f.preflight||f.failsafe)return "PX4 preflight checks have not passed";
    if(norm(f.velocity)>.15)return "vehicle is not stationary";
    if(!command_fresh(t)||!zero(input_))return "fresh zero command stream required";
    return {};
  }
  bool receive(Velocity v,double t,double source_age=0.) {
    if(!finite(v)||!std::isfinite(t)||!std::isfinite(source_age)||source_age<0.||
      source_age>c_.command_timeout||(!accepts_motion()&&!zero(v))) {
      invalidate();return false;
    }
    // Transport delay consumes the existing lifetime; reception must not renew it.
    input_=v;received_=t-source_age;command_valid_=true;return true;
  }
  void invalidate() {input_={};command_valid_=false;}
  bool start(const Feedback& f,double t) {
    reason_=not_ready(f,t);if(!reason_.empty())return false;
    origin_=f.position;hold_=f.position;reference_z_=f.position.z;
    yaw_target_=f.yaw;reset_id_=f.reset_id;last_=t;arm_sent_=false;
    unresolved_arm_=false;seen_armed_=false;last_command_=-1e9;
    filtered_={};filtered_yaw_=0;enter(Phase::Prestream,t);return true;
  }
  void land(const Feedback& f,double t,const std::string& reason) {
    if(!active()||phase_==Phase::Landing)return;
    reason_=reason;invalidate();last_command_=-1e9;
    if(!f.armed&&!unresolved_arm_&&phase_!=Phase::Landing) {
      enter(Phase::Done,t);return;
    }
    enter(Phase::Landing,t);
  }
  void rejected_arm() {unresolved_arm_=false;}
  void release(double t,const std::string& reason) {
    reason_=reason;invalidate();unresolved_arm_=false;enter(Phase::Released,t);
  }

  Output step(const Feedback& f,double t) {
    Output o;
    const double elapsed=t-last_;last_=t;
    const double dt=std::clamp(elapsed,0.,.1);
    if(phase_==Phase::Fault) {
      if(unresolved_arm_&&f.fresh&&f.armed&&f.offboard) {
        seen_armed_=true;unresolved_arm_=false;
        enter(Phase::Landing,t);last_command_=-1e9;
        reason_="late ARM feedback; requesting landing";
      } else return o;
    }
    if(!active())return o;
    if(!f.exclusive) {reason_="publisher conflict; controller released";enter(Phase::Released,t);return o;}
    if(f.fresh&&f.armed){seen_armed_=true;unresolved_arm_=false;}
    if(phase_!=Phase::Landing) {
      if((phase_>=Phase::Arm&&f.fresh&&!f.offboard)||f.failsafe) {
        reason_="PX4 changed mode or entered failsafe; no automatic takeover";enter(Phase::Released,t);return o;
      }
      if(!f.fresh||!f.valid||!finite(f.position)||!finite(f.velocity)||!std::isfinite(f.yaw))
        land(f,t,"telemetry or local estimate invalid");
      else if(elapsed>.3||elapsed<0)land(f,t,"control timer stalled or moved backwards");
      else if(f.reset_id!=reset_id_)land(f,t,"estimator reference reset");
      else if(seen_armed_&&!f.armed) {reason_="unexpected disarm";enter(Phase::Fault,t);return o;}
      else if(command_age(t)>c_.land_after_loss)land(f,t,"command stream lost");
      else if(accepts_motion()&&!f.heading_ok)land(f,t,"heading estimate invalid");
      else if(accepts_motion()&&(origin_.z-f.position.z>c_.max_height||
        origin_.z-f.position.z<.5||std::hypot(f.position.x-origin_.x,f.position.y-origin_.y)>c_.max_radius))
        land(f,t,"simulation flight envelope exceeded");
    }
    if(!active())return o;
    if(phase_==Phase::Landing) return landing(f,t);
    if(phase_==Phase::Prestream) {
      o=position_output(hold_);
      if(t-entered_>=c_.prestream)enter(Phase::Offboard,t);
    } else if(phase_==Phase::Offboard) {
      o=position_output(hold_);
      if(f.offboard)enter(Phase::Arm,t);
      else if(t-entered_>c_.mode_timeout){reason_="Offboard not confirmed";enter(Phase::Fault,t);return {};}
      else if(t-last_command_>=.5){o.command=VehicleCommand::Offboard;last_command_=t;}
    } else if(phase_==Phase::Arm) {
      o=position_output(hold_);
      if(f.armed)enter(Phase::Takeoff,t);
      else if(t-entered_>c_.mode_timeout){reason_="ARM not confirmed; do not automatically retry";enter(Phase::Fault,t);return {};}
      else if(!arm_sent_){o.command=VehicleCommand::Arm;arm_sent_=unresolved_arm_=true;}
    } else if(phase_==Phase::Takeoff) {
      const double final_z=origin_.z-c_.height;
      reference_z_=std::max(final_z,std::max(reference_z_-c_.climb_speed*dt,f.position.z-c_.reference_lead));
      o=position_output({origin_.x,origin_.y,reference_z_});
      const bool arrived=!f.landed&&f.heading_ok&&std::abs(f.position.z-final_z)<=c_.height_tolerance&&
        std::hypot(f.position.x-origin_.x,f.position.y-origin_.y)<.3&&norm(f.velocity)<=c_.speed_tolerance;
      if(dwell(arrived,t,c_.settle_time)) {
        hold_=f.position;filtered_={};filtered_yaw_=0;enter(Phase::Hold,t);
        reason_="takeoff settled; velocity commands enabled";
      } else if(t-entered_>c_.takeoff_timeout) {
        land(f,t,"takeoff feedback gate timed out");return landing(f,t);
      }
    } else {
      const Velocity cmd=command_fresh(t)?input_:Velocity{};
      if(phase_==Phase::Hold&&zero(cmd))return position_output(hold_);
      const bool stopping=zero(cmd);
      if(!stopping) {if(phase_!=Phase::Velocity)enter(Phase::Velocity,t);}
      else if(phase_!=Phase::Brake)enter(Phase::Brake,t);
      Vec desired=heading_to_ned(cmd,f.yaw);
      const double xy=std::hypot(desired.x,desired.y);
      if(xy>c_.xy_speed){desired.x*=c_.xy_speed/xy;desired.y*=c_.xy_speed/xy;}
      desired.z=std::clamp(desired.z,-c_.z_speed,c_.z_speed);
      const double dx=desired.x-filtered_.x,dy=desired.y-filtered_.y;
      const double delta=std::hypot(dx,dy),scale=delta>0?std::min(1.,c_.xy_accel*dt/delta):0.;
      filtered_.x+=dx*scale;filtered_.y+=dy*scale;
      filtered_.z=approach(filtered_.z,desired.z,c_.z_accel*dt);
      filtered_yaw_=approach(filtered_yaw_,std::clamp(-cmd.ccw,-c_.yaw_speed,c_.yaw_speed),c_.yaw_accel*dt);
      yaw_target_=wrap(yaw_target_+filtered_yaw_*dt);
      yaw_target_=wrap(f.yaw+std::clamp(wrap(yaw_target_-f.yaw),-.35,.35));
      o.control=Control::Velocity;o.velocity=filtered_;o.yaw=yaw_target_;o.yaw_rate=filtered_yaw_;
      if(stopping&&dwell(norm(f.velocity)<.15&&norm(filtered_)<.01&&std::abs(filtered_yaw_)<.01,t,.3)) {
        hold_=f.position;enter(Phase::Hold,t);o=position_output(hold_);
      }
    }
    return o;
  }

 private:
  void enter(Phase s,double t){
    if(s==Phase::Landing&&phase_!=Phase::Landing)landing_confirmed_=false;
    phase_=s;entered_=t;settled_=-1;
  }
  bool dwell(bool valid,double t,double duration){
    if(!valid){settled_=-1;return false;}
    if(settled_<0)settled_=t;
    return t-settled_>=duration;
  }
  Output position_output(Vec p) const {
    Output o;o.control=Control::Position;o.position=p;o.yaw=yaw_target_;return o;
  }
  Output landing(const Feedback& f,double t) {
    Output o;
    if(f.fresh&&f.landing)landing_confirmed_=true;
    if(f.fresh&&f.landed&&!f.armed&&!unresolved_arm_) {
      if(dwell(true,t,1.)){enter(Phase::Done,t);reason_="landing and disarm confirmed";}
      return o;
    }
    settled_=-1;
    if(t-entered_>c_.land_timeout){reason_="landing not confirmed; PX4/operator must finish";enter(Phase::Fault,t);return o;}
    if(f.fresh&&f.armed&&!f.landing&&(landing_confirmed_||!f.offboard||f.failsafe)) {
      reason_="other mode took over during landing";enter(Phase::Released,t);return o;
    }
    if(f.fresh&&f.offboard&&f.valid&&finite(f.position)&&finite(f.velocity)&&std::isfinite(f.yaw)) {
      o.control=Control::Velocity;o.velocity={};o.yaw=f.yaw;o.yaw_rate=0;
    }
    // Request LAND until telemetry confirms PX4 Land. Never force disarm.
    if(!f.landing&&t-last_command_>=1.) {o.command=VehicleCommand::Land;last_command_=t;}
    return o;
  }
  Config c_;
  Phase phase_=Phase::Idle;
  Velocity input_;
  Vec origin_,hold_,filtered_;
  double filtered_yaw_=0,yaw_target_=0,reference_z_=0;
  double received_=-1,last_=0,entered_=0,settled_=-1,last_command_=-1e9;
  uint32_t reset_id_=0;
  bool command_valid_=false,arm_sent_=false,unresolved_arm_=false,seen_armed_=false;
  bool landing_confirmed_=false;
  std::string reason_;
};
} // namespace jikl_velocity
