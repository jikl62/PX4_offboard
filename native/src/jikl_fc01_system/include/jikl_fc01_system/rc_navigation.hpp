#pragma once
#include "jikl_fc01_system/attitude_reference.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace fc01::attitude {
enum class RcStartPhase {WaitGround,Prestream,WaitArm,WaitNavigation,Joined,Blocked};
inline bool rc_preparation_retry_reason(const std::string& reason){
  return reason=="RC_MANUAL_ARM_EVIDENCE_TIMEOUT"||reason=="RC_NAVIGATION_SELECTION_TIMEOUT"||
    reason=="RC_NAVIGATION_CONFIRMATION_TIMEOUT"||reason=="RC_NAVIGATION_MODE_SLOT_MISMATCH"||
    reason=="RC_NAVIGATION_BEFORE_MANUAL_ARM"||reason=="RC_ARM_OR_NAVIGATION_BEFORE_READY"||
    reason=="RC_NAVIGATION_SELECTION_WITHDRAWN"||reason=="RC_NAVIGATION_MODE_WITHDRAWN"||
    reason=="RC_ARM_AUTHORIZATION_WITHDRAWN"||reason=="RC_UNEXPECTED_MODE_BEFORE_NAVIGATION";
}
// Admission only. This object cannot construct an ARM command or nonzero thrust.
class RcNavigationStart {
 public:
  RcStartPhase phase() const{return phase_;}
  const std::string& reason() const{return reason_;}
  bool anchor_valid() const{return anchor_;}
  double z0() const{return z0_;}
  const Quaternion& q0() const{return q0_;}
  std::uint8_t z_reset()const{return z_reset_;}
  std::uint8_t vz_reset()const{return vz_reset_;}
  std::uint8_t quat_reset()const{return quat_reset_;}
  unsigned commits()const{return commits_;}
  bool manual_arm_proven()const{return manual_arm_proven_;}
  std::uint64_t manual_armed_time()const{return manual_armed_time_;}
  // A latest arm-OFF sample is not proof that this attempt never observed
  // arming intent. Keep that history even when health() resets preparation.
  bool no_arm_intent_observed()const{return !arm_intent_observed_&&!manual_arm_proven_&&
    !manual_armed_time_&&!arm_edge_&&!navigation_edge_&&!pending_armed_time_&&
    arm_since_<0&&navigation_since_<0&&!navigation_offboard_observed_;}
  void note_commit(bool success,double now){
    if(!success){stream_first_=stream_last_=-1;commits_=0;return;}
    if(phase_==RcStartPhase::WaitGround||phase_==RcStartPhase::Blocked||phase_==RcStartPhase::Joined)return;
    if(stream_last_<0||now<stream_last_||now-stream_last_>.3){stream_first_=now;commits_=0;}
    stream_last_=now;++commits_;
  }
  template<class C>bool stream_ready(const C& c,double now)const{
    return stream_first_>=0&&stream_last_>=0&&now>=stream_last_&&now-stream_last_<=c.stall_seconds&&
      now-stream_first_>=c.prestream_seconds&&commits_>=unsigned(std::ceil(c.loop_hz*c.prestream_seconds*.8));
  }
  template<class F>static bool rc_live(const F& s){
    return s.rc_link_ready&&s.rc_switches_ready&&s.rc_source_selected&&
      s.rc_source_clock_valid&&s.rc_switch_clock_valid&&!s.kill&&s.rc_kill_switch==3;
  }
  // A READY stream may bridge independently delivered arm observations. This
  // permission is zero-only, expires without matching evidence, and is checked
  // again with the actual sink snapshot; it never proves manual arming.
  template<class C,class F>bool pending_zero_allowed(const C& c,const F& s,double now)const{
    if(!anchor_||manual_arm_proven_||!ready_source_floor_||
       (phase_!=RcStartPhase::WaitArm&&phase_!=RcStartPhase::WaitNavigation)||
       !std::isfinite(now)||observed_at_<0||now<observed_at_||now-observed_at_>c.stall_seconds||
       !stream_ready(c,now))return false;
    if(phase_==RcStartPhase::WaitNavigation&&(arm_since_<0||now<arm_since_||
       now-arm_since_>c.status_timeout||now-arm_since_>c.rc_arm_to_navigation_timeout))return false;
    if(!s.source_clock_validated||!s.source_clock_valid||!s.source_host_stamp||!rc_live(s)||
       !s.rc_axes_safe||s.rc_sticks_moving||!s.rc_aux_switches_safe||!s.battery_ready||!s.preflight||
       !s.stabilized_selected||s.offboard||s.rc_mode_slot==0||s.rc_mode_slot>6||
       s.rc_mode_slot==c.rc_navigation_mode_slot||!s.landed||
       !std::isfinite(s.z)||!std::isfinite(s.vz)||std::abs(z0_-s.z)>c.height_tolerance||
       std::abs(s.vz)>c.ground_vz_tolerance||s.z_reset!=z_reset_||
       s.vz_reset!=vz_reset_||s.quat_reset!=quat_reset_||
       (s.rc_arm_switch!=1&&s.rc_arm_switch!=3))return false;
    const auto q=normalized(s.q);
    if(!q||std::acos(std::clamp(1-2*((*q)[1]*(*q)[1]+(*q)[2]*(*q)[2]),-1.,1.))*
       180/3.141592653589793>c.max_tilt_deg)return false;
    const bool arm_on=s.rc_arm_switch==1;
    if(arm_on&&(s.rc_switch_sample_stamp<=ready_source_floor_||
       (!arm_edge_&&(last_arm_switch_!=3||s.rc_switch_sample_stamp<=last_switch_stamp_))))return false;
    if(s.armed){
      return s.latest_arming_reason==2&&s.status_stamp>ready_source_floor_&&
        s.status_stamp>disarmed_status_&&s.armed_time>disarmed_armed_time_&&
        (!pending_armed_time_||s.armed_time==pending_armed_time_);
    }
    return arm_on;
  }
  // Manual arming must already be proven. Independent mode/slot deliveries
  // may pair only within one fixed navigation window, still at exact zero.
  template<class C,class F>bool navigation_zero_allowed(const C& c,const F& s,double now)const{
    if(phase_!=RcStartPhase::WaitNavigation||!manual_arm_proven_||!anchor_||
       !std::isfinite(now)||observed_at_<0||now<observed_at_||now-observed_at_>c.stall_seconds||
       arm_since_<0||now<arm_since_||now-arm_since_>c.rc_arm_to_navigation_timeout||
       (navigation_since_>=0&&(now<navigation_since_||now-navigation_since_>c.status_timeout))||
       !stream_ready(c,now))return false;
    if(!s.source_clock_validated||!s.source_clock_valid||!s.source_host_stamp||!rc_live(s)||
       !s.rc_axes_safe||s.rc_sticks_moving||!s.rc_aux_switches_safe||!s.battery_ready||!s.preflight||
       !s.armed||s.armed_time!=manual_armed_time_||s.latest_arming_reason!=2||s.rc_arm_switch!=1||
       s.rc_mode_slot==0||s.rc_mode_slot>6||(!s.stabilized_selected&&!s.offboard)||!s.landed||
       !std::isfinite(s.z)||!std::isfinite(s.vz)||std::abs(z0_-s.z)>c.height_tolerance||
       std::abs(s.vz)>c.ground_vz_tolerance||s.z_reset!=z_reset_||
       s.vz_reset!=vz_reset_||s.quat_reset!=quat_reset_)return false;
    const auto q=normalized(s.q);
    if(!q||std::acos(std::clamp(1-2*((*q)[1]*(*q)[1]+(*q)[2]*(*q)[2]),-1.,1.))*
       180/3.141592653589793>c.max_tilt_deg)return false;
    const bool nav=s.rc_mode_slot==c.rc_navigation_mode_slot;
    if((navigation_edge_&&!nav)||(navigation_offboard_observed_&&!s.offboard))return false;
    if(nav&&(s.rc_switch_sample_stamp<=arm_edge_||s.rc_switch_sample_stamp<=ready_source_floor_||
       (!navigation_edge_&&s.rc_switch_sample_stamp<=last_switch_stamp_)))return false;
    if(s.offboard&&(!last_stabilized_status_||s.status_stamp<=last_stabilized_status_||
       s.status_stamp<=arm_edge_||s.status_stamp<=ready_source_floor_||s.offboard_signal_lost))return false;
    return true;
  }
  template<class C,class F>bool step(const C& c,const F& s,double now,const std::string& health){
    if(phase_==RcStartPhase::Blocked||phase_==RcStartPhase::Joined)return false;
    if(s.armed||s.rc_arm_switch==1)arm_intent_observed_=true;
    const auto block=[this](const std::string& why){phase_=RcStartPhase::Blocked;reason_=why;return false;};
    if(!std::isfinite(now)||(observed_at_>=0&&now<observed_at_))return block("RC_TIME_REGRESSION");
    observed_at_=now;
    const bool nav=s.rc_mode_slot==c.rc_navigation_mode_slot;
    const bool arm_on=s.rc_arm_switch==1;
    const bool intent=arm_since_>=0;
    std::string why=health;
    if(why.empty()&&(!s.source_clock_validated||!s.source_host_stamp||!rc_live(s)))why="RC_FRESH_SELECTED_SOURCE_REQUIRED";
    if(why.empty()&&(!s.rc_axes_safe||s.rc_sticks_moving))why="RC_MINIMUM_THROTTLE_CENTERED_STICKS_REQUIRED";
    if(why.empty()&&!s.rc_aux_switches_safe)why="RC_OTHER_MODE_SWITCH_ACTIVE";
    if(why.empty()&&(!s.battery_ready||!s.preflight))why="BATTERY_PREFLIGHT_REQUIRED";
    const auto q=normalized(s.q);
    if(why.empty()&&q){const double tilt=std::acos(std::clamp(1-2*((*q)[1]*(*q)[1]+(*q)[2]*(*q)[2]),-1.,1.))*180/3.141592653589793;
      // Manual RC ground admission remains independently guarded; this is not
      // the autonomous measured-attitude takeoff path.
      if(tilt>c.max_tilt_deg)why="RC_GROUND_TILT_EXCEEDED";}
    if(!why.empty()){
      if(intent||s.armed)return block(why);
      reset(why);return false;
    }
    if(phase_==RcStartPhase::WaitGround){
      if(s.armed||arm_on||nav||s.offboard||!s.stabilized_selected||!s.landed||!s.at_rest||
         s.rc_arm_switch!=3||s.rc_mode_slot==0||s.rc_mode_slot>6||
         s.stationary_seconds<c.ground_seconds||std::abs(s.vz)>c.ground_vz_tolerance){
        reset("RC_WAIT_DISARMED_STABILIZED_ARM_OFF_NAV_OFF_GROUND");return false;
      }
      // stationary_seconds already proves the required continuous ground
      // rest. The following committed prestream independently verifies live
      // RC sources and safe controls; do not add another serial rest dwell.
      phase_=RcStartPhase::Prestream;
    }
    if(phase_==RcStartPhase::Prestream){
      if(s.armed||arm_on||nav||s.offboard)return block("RC_ARM_OR_NAVIGATION_BEFORE_READY");
      if(!s.stabilized_selected||!s.landed||!s.at_rest||std::abs(s.vz)>c.ground_vz_tolerance){
        reset("RC_PRESTREAM_GROUND_CHANGED");return false;}
      capture(s);
      if(stream_ready(c,now)&&!s.offboard_signal_lost){
        // Include the same 100ms future-source allowance as the clock guard:
        // an event generated before readiness cannot arrive later as consent.
        ready_source_floor_=s.source_host_stamp+100000;
        phase_=RcStartPhase::WaitArm;reason_="RC_READY_WAIT_CH5_MANUAL_ARM";
      }
      else reason_="RC_PRESTREAM_WAIT_COMMITS_AND_PX4_SIGNAL";
      return true;
    }
    const bool new_switch=s.rc_switch_sample_stamp>last_switch_stamp_;
    if(new_switch){
      if(last_arm_switch_==3&&arm_on){
        if(s.rc_switch_sample_stamp<=ready_source_floor_)return block("RC_ARM_SOURCE_BEFORE_READY");
        arm_edge_=s.rc_switch_sample_stamp;
      }
      if(last_mode_slot_!=c.rc_navigation_mode_slot&&nav){
        if(!arm_edge_||s.rc_switch_sample_stamp<=arm_edge_)return block("RC_NAVIGATION_BEFORE_MANUAL_ARM");
        navigation_edge_=s.rc_switch_sample_stamp;
      }
      last_arm_switch_=s.rc_arm_switch;last_mode_slot_=s.rc_mode_slot;last_switch_stamp_=s.rc_switch_sample_stamp;
    }
    if(phase_==RcStartPhase::WaitArm){
      if(nav||s.offboard)return block("RC_NAVIGATION_BEFORE_MANUAL_ARM");
      if(!s.armed&&!arm_on){
        if(!s.landed||!s.at_rest||!s.stabilized_selected||std::abs(s.vz)>c.ground_vz_tolerance){reset("RC_READY_GROUND_CHANGED");return false;}
        capture(s);reason_="RC_READY_WAIT_CH5_MANUAL_ARM";return true;
      }
      arm_since_=now;phase_=RcStartPhase::WaitNavigation;
    }
    if(phase_==RcStartPhase::WaitNavigation){
      if(now-arm_since_>c.rc_arm_to_navigation_timeout)return block("RC_NAVIGATION_SELECTION_TIMEOUT");
      // Expiry precedes pairing: evidence received on an overdue tick cannot
      // turn an expired attempt into a proven arm or navigation admission.
      if(!manual_arm_proven_&&now-arm_since_>c.status_timeout)return block("RC_MANUAL_ARM_EVIDENCE_TIMEOUT");
      if(!s.landed||std::abs(z0_-s.z)>c.height_tolerance||std::abs(s.vz)>c.ground_vz_tolerance)
        return block("RC_ARMED_GROUND_REFERENCE_NOT_STABLE");
      if(s.z_reset!=z_reset_||s.vz_reset!=vz_reset_||s.quat_reset!=quat_reset_)
        return block("RC_ARMED_ESTIMATOR_RESET");
      if(!s.stabilized_selected&&!s.offboard)return block("RC_UNEXPECTED_MODE_BEFORE_NAVIGATION");
      if(s.armed&&s.latest_arming_reason!=2)return block("RC_ARM_REASON_NOT_RC_SWITCH");
      if(s.armed&&s.status_stamp<=ready_source_floor_)return block("RC_ARM_STATE_BEFORE_READY");
      if(s.armed&&(s.status_stamp<=disarmed_status_||s.armed_time<=disarmed_armed_time_||
         (pending_armed_time_&&s.armed_time!=pending_armed_time_)))return block("RC_ARM_STATE_NOT_NEW_MATCHING_EPISODE");
      if(s.armed&&!pending_armed_time_)pending_armed_time_=s.armed_time;
      if(s.armed&&arm_edge_&&arm_on&&s.status_stamp>disarmed_status_&&s.armed_time>disarmed_armed_time_){
        if(!manual_arm_proven_)manual_armed_time_=s.armed_time;
        manual_arm_proven_=true;
      }
      if(manual_arm_proven_&&(!s.armed||!arm_on))return block("RC_ARM_AUTHORIZATION_WITHDRAWN");
      if(!manual_arm_proven_){
        reason_="RC_WAIT_MATCHED_SWITCH_AND_NEW_ARMED_STATUS";
        return (!s.armed&&arm_on)||pending_zero_allowed(c,s,now);
      }
      if(navigation_since_>=0&&now-navigation_since_>c.status_timeout)
        return block(nav?"RC_NAVIGATION_CONFIRMATION_TIMEOUT":"RC_NAVIGATION_MODE_SLOT_MISMATCH");
      if(navigation_offboard_observed_&&!s.offboard)return block("RC_NAVIGATION_MODE_WITHDRAWN");
      if(navigation_edge_&&!nav)return block("RC_NAVIGATION_SELECTION_WITHDRAWN");
      if(s.offboard){
        if(!last_stabilized_status_||s.status_stamp<=last_stabilized_status_||
           s.status_stamp<=arm_edge_||s.status_stamp<=ready_source_floor_)
          return block("RC_OFFBOARD_STATE_NOT_NEW_AFTER_MANUAL_ARM");
        navigation_offboard_observed_=true;
      }else last_stabilized_status_=s.status_stamp;
      if((nav||s.offboard)&&navigation_since_<0)navigation_since_=now;
      if(nav&&navigation_edge_&&s.offboard&&s.status_stamp>navigation_edge_&&
         stream_ready(c,now)&&!s.offboard_signal_lost){
        phase_=RcStartPhase::Joined;reason_="RC_MANUAL_ARM_AND_NEW_OFFBOARD_CONFIRMED";return false;
      }
      reason_=s.offboard&&!nav?"RC_WAIT_MATCHED_NAVIGATION_SWITCH_AND_OFFBOARD":
        nav?"RC_WAIT_ACTUAL_PX4_OFFBOARD_CONFIRMATION":"RC_ARMED_WAIT_CH6_NAVIGATION";
      return true;
    }
    return false;
  }
 private:
  void reset(const std::string& why){phase_=RcStartPhase::WaitGround;anchor_=false;
    stream_first_=stream_last_=-1;commits_=0;ready_source_floor_=0;reason_=why;}
  template<class F>void capture(const F& s){
    anchor_=true;z0_=s.z;q0_=s.q;z_reset_=s.z_reset;vz_reset_=s.vz_reset;quat_reset_=s.quat_reset;
    disarmed_status_=s.status_stamp;disarmed_armed_time_=s.armed_time;
    last_arm_switch_=s.rc_arm_switch;last_mode_slot_=s.rc_mode_slot;last_switch_stamp_=s.rc_switch_sample_stamp;
  }
  RcStartPhase phase_{RcStartPhase::WaitGround};std::string reason_{"RC_WAIT_SAFE_GROUND"};
  bool anchor_{},manual_arm_proven_{},navigation_offboard_observed_{},arm_intent_observed_{};
  Quaternion q0_{1,0,0,0};double z0_{};
  std::uint8_t z_reset_{},vz_reset_{},quat_reset_{},last_arm_switch_{},last_mode_slot_{};
  double arm_since_{-1},navigation_since_{-1},stream_first_{-1},stream_last_{-1},observed_at_{-1};unsigned commits_{};
  std::uint64_t disarmed_status_{},disarmed_armed_time_{},manual_armed_time_{},arm_edge_{},navigation_edge_{},last_switch_stamp_{};
  std::uint64_t ready_source_floor_{},pending_armed_time_{},last_stabilized_status_{};
};
} // namespace fc01::attitude
