#pragma once
// 控制权合同：区分有人 RC 接管与无 RC 自主运行，不把 PX4 模式号当成授权。
// State 保存选定模式与本次尝试锁定的模式；Inputs 来自遥控器/配置证据。
// evaluate 是无副作用判定，Allowed 仍需叠加健康、状态阶段和最终发布门。
namespace fc01::authority {
// Companion policy; these are not PX4 navigation modes or parameter values.
// Unset=未选择，RcTakeover=依赖可接管 RC，AutonomousNoRc=依赖明确确认的自主失效处置。
enum class Mode {Unset,RcTakeover,AutonomousNoRc};
inline const char* name(Mode mode){
  switch(mode){case Mode::Unset:return "UNSET";case Mode::RcTakeover:return "RC_TAKEOVER";
    case Mode::AutonomousNoRc:return "AUTONOMOUS_NO_RC";}return "INVALID";
}
// locked 后 mode 不能与 attempt_mode 不同；takeover_latched 优先否决，后来的正常反馈不能绕过它。
struct State {
  Mode mode{Mode::Unset},attempt_mode{Mode::Unset};
  bool locked{},takeover_latched{};
};
struct Inputs {
  bool rc_link_ready{},rc_switches_ready{},rc_takeover_confirmed{},autonomous_failsafe_confirmed{};
};
// 每个拒绝原因保留独立枚举，便于状态窗口解释缺少的是选择、锁定、接管还是实际 RC 遥测。
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
  // 按原优先级先检查不可绕过的接管，再检查模式选择/锁定，保证错误信息与实际拒绝一致。
  if(state.takeover_latched)return Reason::Takeover;
  if(state.mode==Mode::Unset)return Reason::Unset;
  if(require_lock&&!state.locked)return Reason::NotLocked;
  if(state.locked&&state.mode!=state.attempt_mode)return Reason::Changed;
  switch(state.mode){
    // 有人接管路线同时要求链路、已确认接管合同和开关遥测；不能只凭“有遥控器”通过。
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
