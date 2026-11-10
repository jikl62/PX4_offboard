#pragma once
#include "jikl_fc01_system/authority_telemetry.hpp"
#include "jikl_fc01_system/attitude_diagnostics.hpp"
#include <tuple>

namespace fc01 {
// These predicates stop additional receive passes, then give the existing
// controller the observed event. They neither authorize output nor clear a
// fault. Ordinary advancing numeric measurements may replace one another;
// discrete changes and invalid feedback must reach a control step first.
inline auto authority_receive_key(const AuthorityTelemetry::Facts& f){
  return std::make_tuple(f.rc_link_ready,f.rc_switches_ready,f.kill,
    f.manual_seen,f.manual_valid,f.rc_source_selected,f.sticks_moving,
    f.rc_source_clock_valid,f.rc_switch_clock_valid,f.data_source,f.mode_slot,
    f.arm_switch,f.kill_switch,f.switch_generation,f.rc_axes_safe,f.rc_aux_switches_safe);
}
template<class M>bool status_receive_event(const M& a,const M& b){
  const auto key=[](const M& m){return std::make_tuple(m.system_id,m.component_id,
    m.arming_state,m.armed_time,m.latest_arming_reason,m.nav_state,m.failsafe,
    m.failsafe_and_user_took_over,m.calibration_enabled,m.rc_calibration_in_progress,
    m.hil_state,m.pre_flight_checks_pass,m.power_input_valid,m.can_set_nav_states_mask);};
  return key(a)!=key(b);
}
template<class M>bool land_receive_event(const M& a,const M& b){
  return a.landed!=b.landed||a.at_rest!=b.at_rest;
}
template<class M>bool flags_receive_event(const M& a,const M& b){
  return a.manual_control_signal_lost!=b.manual_control_signal_lost||
    a.offboard_control_signal_lost!=b.offboard_control_signal_lost||
    a.fd_critical_failure!=b.fd_critical_failure;
}
template<class M>bool battery_receive_event(const M& a,const M& b){
  // Preserve every voltage change too: battery is low rate, and its readiness
  // thresholds belong to the controller rather than this receive scheduler.
  return a.connected!=b.connected||a.warning!=b.warning||a.voltage_v!=b.voltage_v;
}
template<class M>bool local_receive_event(const M& a,const M& b,const attitude::Config& c,
    bool allow_unused_horizontal=false){
  const auto key=[](const M& m){return std::make_tuple(m.xy_valid,m.z_valid,
    m.v_xy_valid,m.v_z_valid,m.heading_good_for_control,m.dead_reckoning,
    m.xy_reset_counter,m.z_reset_counter,m.vxy_reset_counter,m.vz_reset_counter,m.heading_reset_counter);};
  if(key(a)!=key(b))return true;
  for(double value:{double(b.z),double(b.vz),double(b.z_deriv)})
    if(!std::isfinite(value))return true;
  // A vertical-only consumer does not use estimates explicitly marked
  // unavailable. Repeated NaNs in those fields must not stop every receive
  // pass and strand healthy Z/Vz behind a DDS backlog. Validity/reset edges
  // above and nonfinite fields claimed valid still remain discrete events.
  if((!allow_unused_horizontal||b.xy_valid)&&
      (!std::isfinite(b.x)||!std::isfinite(b.y)))return true;
  if((!allow_unused_horizontal||b.v_xy_valid)&&
      (!std::isfinite(b.vx)||!std::isfinite(b.vy)))return true;
  if((!allow_unused_horizontal||b.heading_good_for_control)&&!std::isfinite(b.heading))return true;
  // Preserve disagreement observations AND the return to agreement. Dropping
  // that return would join separate episodes into a false continuous warning.
  // This schedules diagnostic observations; it grants no output permission.
  if(c.feedback_guard.enabled){
    const bool before=std::abs(double(a.vz)-a.z_deriv)>c.feedback_guard.rate_tolerance_mps;
    const bool after=std::abs(double(b.vz)-b.z_deriv)>c.feedback_guard.rate_tolerance_mps;
    if(after||before!=after)return true;
  }
  // The node's disarmed stationary dwell also consumes Vz. Preserve a
  // departure from (or return to) its existing rest band before another
  // callback can hide it. No ground reference is created here.
  if(c.ground_vz_tolerance>0&&
     (std::abs(double(a.vz))<=c.ground_vz_tolerance)!=
     (std::abs(double(b.vz))<=c.ground_vz_tolerance))return true;
  // Reuse configured estimator jump limits only when available. A jump gets
  // a controller step before any subsequent sample can replace it.
  return (c.z_jump_limit_m>0&&std::abs(double(b.z)-a.z)>c.z_jump_limit_m)||
    (c.vz_jump_limit_mps>0&&std::abs(double(b.vz)-a.vz)>c.vz_jump_limit_mps);
}
template<class M>bool attitude_receive_event(const M& a,const M& b,const attitude::Config& c){
  if(a.quat_reset_counter!=b.quat_reset_counter)return true;
  const attitude::Quaternion q{b.q[0],b.q[1],b.q[2],b.q[3]};
  const auto diagnostic=attitude::attitude_diagnostics(q,{1,0,0,0});
  // A valid measured attitude is ordinary feedback, even when it is outside
  // the historical navigation envelope.  Do not stop the receive drain for
  // it: the autonomous controller must continue consuming status, estimator
  // and authority events.  Invalid quaternion data remains an event so the
  // existing independent health checks see it before a later sample replaces
  // it.  The configuration argument is retained for the shared predicate API.
  (void)c;
  return !diagnostic.valid;
}
}
