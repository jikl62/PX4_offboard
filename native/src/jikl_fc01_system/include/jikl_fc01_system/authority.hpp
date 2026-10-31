#pragma once
namespace fc01::authority {
// Companion policy; these are not PX4 navigation modes or parameter values.
enum class Mode {Unset,RcTakeover,AutonomousNoRc};
inline const char* name(Mode mode){
  switch(mode){case Mode::Unset:return "UNSET";case Mode::RcTakeover:return "RC_TAKEOVER";
    case Mode::AutonomousNoRc:return "AUTONOMOUS_NO_RC";}return "INVALID";
}
struct State {
  Mode mode{Mode::Unset},attempt_mode{Mode::Unset};
  bool locked{},takeover_latched{};
};
struct Inputs {
  bool rc_link_ready{},rc_switches_ready{},rc_takeover_confirmed{},autonomous_failsafe_confirmed{};
};
enum class Reason {Allowed,Unset,NotLocked,Changed,Takeover,RcLink,RcUnconfirmed,RcTelemetry,AutonomousUnconfirmed,Invalid};
inline const char* name(Reason reason){
  switch(reason){
    case Reason::Allowed:return "AUTHORITY_READY";
    case Reason::Unset:return "AUTHORITY_MODE_UNSET";
    case Reason::NotLocked:return "AUTHORITY_MODE_NOT_LOCKED";
    case Reason::Changed:return "AUTHORITY_CHANGED_DURING_ATTEMPT";
    case Reason::Takeover:return "OPERATOR_TAKEOVER_LATCHED";
    case Reason::RcLink:return "RC_LINK_REQUIRED";
    case Reason::RcUnconfirmed:return "RC_TAKEOVER_CONTRACT_UNCONFIRMED";
    case Reason::RcTelemetry:return "RC_TAKEOVER_TELEMETRY_REQUIRED";
    case Reason::AutonomousUnconfirmed:return "AUTONOMOUS_FAILSAFE_UNCONFIRMED";
    case Reason::Invalid:return "INVALID_AUTHORITY_MODE";
  }return "INVALID_AUTHORITY_MODE";
}
inline Reason evaluate(const State& state,const Inputs& in,bool require_lock=true){
  if(state.takeover_latched)return Reason::Takeover;
  if(state.mode==Mode::Unset)return Reason::Unset;
  if(require_lock&&!state.locked)return Reason::NotLocked;
  if(state.locked&&state.mode!=state.attempt_mode)return Reason::Changed;
  switch(state.mode){
    case Mode::RcTakeover:
      if(!in.rc_link_ready)return Reason::RcLink;
      if(!in.rc_takeover_confirmed)return Reason::RcUnconfirmed;
      if(!in.rc_switches_ready)return Reason::RcTelemetry;
      return Reason::Allowed;
    case Mode::AutonomousNoRc:
      return in.autonomous_failsafe_confirmed?Reason::Allowed:Reason::AutonomousUnconfirmed;
    default:return Reason::Invalid;
  }
}
} // namespace fc01::authority
