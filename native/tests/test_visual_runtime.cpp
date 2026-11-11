// 视觉与既有起飞链集成回归：验证人工起飞前无权加动力、保持后才可运动以及故障时原生下降。
// 所有反馈与双 sink 均为本机合成；覆盖真实 Runtime/RC/Core 的调用路径，不声称已做 PX4 SITL 或实机验收。
#include "vision_test_support.hpp"
#include <iostream>
using namespace vision_test;
// 构造明确健康的 NED/FRD 实际状态与独立目标来源；源戳为模拟 ROS 纳秒，received_at 为模拟单调秒。
static v::Input input(double now,v::Mode mode=v::Mode::Cruise,double goal_x=.2,double goal_z=-.5){
  static std::uint64_t sequence=0;
  v::Input in;in.ros_now_ns=1000000000000ULL+std::uint64_t(now*1e9);
  in.state_publishers=in.command_publishers=1;
  auto& s=in.state;s.sequence=++sequence;s.stamp_ns=in.ros_now_ns;s.received_at=now;
  s.frame_id="fc01_vision_ned";s.body_frame_id="base_link_frd";s.source_id="synthetic-vision";
  s.initialized=s.tracking=s.calibration_valid=true;
  s.valid_fields=v::Position|v::Velocity|v::Attitude|v::Quality;
  s.position={0,0,-.5};s.velocity={0,0,0};s.attitude={1,0,0,0};s.quality=1;
  auto& c=in.command;c.sequence=sequence;c.stamp_ns=in.ros_now_ns;c.received_at=now;
  c.source_id="synthetic-command";c.frame_id=s.frame_id;c.valid=true;c.mode=mode;
  c.position_valid=true;c.position={goal_x,0,goal_z};c.sticks_valid=true;c.sticks={0,0,0,0};
  return in;
}
static double yaw(const a::Quaternion& q){return std::atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));}
// 有效视觉输入不能启动飞机；必须先完成原人工 RC 起飞，到保持后再验证前倾输出与双 sink。
static void motion_runs_only_after_existing_manual_start(){
  VisionFlight q;q.set_visual(input(.05));q.tick();
  need(!q.o.attitude.visual_motion&&!a::nonzero(q.o.attitude.thrust),"visual input cannot start or power vehicle");
  q.reach_hold();q.set_visual(input(q.t+.05));q.tick();
  need(q.o.attitude.visual_motion,"valid visual point enters actual Runtime control");
  need(q.o.attitude.q[2]<0,"north position error produces forward pitch");
  need(q.target_committed&&q.heartbeat_committed,"visual output uses original two final sinks");
  need(q.arm_commands==0&&q.mode_commands==0,"visual input never creates arm or Offboard entry");
}
// 视觉开关关闭时，即使提供完整视觉输入，也继续原水平定高输出。
static void disabled_visual_preserves_original_level_output(){
  auto cfg=config();cfg.attitude.visual.enabled=false;VisionFlight q(cfg);q.reach_hold();
  q.set_visual(input(q.t+.05));q.tick();
  need(!q.o.attitude.visual_motion&&std::abs(q.o.attitude.q[2])<1e-9,"default-off route remains level");
}
// 实测水平速度必须改变制动姿态；更高的视觉 z 目标还要进入既有垂向推力计算。
static void state_and_target_change_control_calculation(){
  VisionFlight q;q.reach_hold();q.set_visual(input(q.t+.05));q.tick();const double first=q.o.attitude.q[2];
  auto in=input(q.t+.05);in.state.velocity[0]=.3;q.set_visual(in);q.tick();
  need(q.o.attitude.q[2]>first,"measured horizontal velocity changes braking attitude");
  const double thrust=-q.o.attitude.thrust[2];
  in=input(q.t+.05,v::Mode::Cruise,.2,-.7);q.set_visual(in);q.tick();
  need(-q.o.attitude.thrust[2]>thrust,"visual vertical point error reaches original thrust computation");
}
// 降低视觉高度目标仍受原 max_descent×dt 轨迹速度约束，不能把目标直接跳到较低高度。
static void lowering_target_preserves_original_descent_rate(){
  VisionFlight q;q.reach_hold();const double before=q.o.attitude.target_height;
  q.set_visual(input(q.t+.05,v::Mode::Cruise,0,-.3));q.tick();
  need(q.o.attitude.target_height>=before-q.c.attitude.vertical.max_descent*.05-1e-9,
       "downward target changes must not jump original trajectory");
}
// 视觉过期进入原 Descent 并继续成对输出；随后恢复视觉不能重新接管已开始的降落。
static void expiration_requests_native_descent_and_cannot_resume(){
  VisionFlight q;q.reach_hold();q.set_visual(input(q.t+.05));q.tick();
  for(int i=0;i<7;++i)q.tick();
  need(q.r.attitude_core().phase()==a::Phase::Descent,"visual timeout enters native descent, not hard Safety");
  need(q.target_committed&&q.heartbeat_committed&&!q.o.attitude.visual_motion,"native recovery keeps original sinks");
  q.set_visual(input(q.t+.05));q.tick();
  need(q.r.attitude_core().phase()==a::Phase::Descent,"reappearing vision cannot resume motion after landing starts");
}
// 最终 sink 同时复核视觉 TTL、姿态计划绑定及 Kill 优先级，不能只相信早先计算的 active 标志。
static void final_sink_refuses_expired_or_changed_motion(){
  VisionFlight q;q.reach_hold();q.set_visual(input(q.t+.05));q.tick();
  auto c=q.r.publication_context(q.s,q.t+.31,true,true);
  unsigned sinks=0;
  auto decision=a::publish_attitude_at_boundary<RcMode,RcTarget>(c,q.o.attitude,q.s.status_stamp,
      [&](const RcTarget&){++sinks;},[&](const RcMode&){++sinks;});
  need(!decision.allowed&&sinks==0,"expired visual plan cannot reach either final sink");
  c=q.r.publication_context(q.s,q.t,true,true);auto changed=q.o.attitude;
  changed.q={1,0,0,0};
  need(!a::attitude_publication_policy(c,changed).allowed,"final sink binds movement attitude to actual computed plan");
  c.feedback.kill=true;
  need(!a::attitude_publication_policy(c,q.o.attitude).allowed,"visual permission cannot override kill");
}
// Follow 连续偏航后过期，下降须冻结最后真正提交的 yaw，不回跳任务初始机头方向。
static void native_landing_keeps_last_committed_yaw(){
  VisionFlight q;q.reach_hold();
  for(int i=0;i<12;++i){auto in=input(q.t+.05,v::Mode::Follow);in.command.sticks[2]=1;q.set_visual(in);q.tick();}
  double last=yaw(q.o.attitude.q);need(last>.05,"follow yaw moves actual attitude target");
  for(int i=0;i<7;++i){q.tick();if(q.o.attitude.visual_motion&&q.target_committed)last=yaw(q.o.attitude.q);}
  need(q.r.attitude_core().phase()==a::Phase::Descent,"yaw exercise expires to native descent");
  need(std::abs(yaw(q.o.attitude.q)-last)<1e-6,"descent freezes last committed yaw rather than initial heading");
}
// 模拟 249→251 ms 跨越 TTL：首个 sink 前过期可零提交回退下降，目标已发后过期仍保留部分发布硬故障。
static void first_sink_expiry_is_recoverable_and_partial_expiry_is_hard(){
  for(bool after_target:{false,true}){
    VisionFlight q;q.reach_hold();q.set_visual(input(q.t+.05));q.tick();
    unsigned reads=0,sinks=0;bool threw=false;
    const auto resample=[&]{
      const bool expired=++reads>=(after_target?3U:2U);
      auto actual=q.s;actual.attitude.visual_input.ros_now_ns+=expired?251000000:249000000;
      return q.r.publication_context(actual,q.t+(expired?.251:.249),true,true);
    };
    a::Decision decision;
    try{decision=a::publish_attitude_resampled<RcMode,RcTarget>(resample,q.o.attitude,q.s.status_stamp,
      [&](const RcTarget&){++sinks;},[&](const RcMode&){++sinks;});}
    catch(const std::exception&){threw=true;}
    if(after_target){need(threw&&sinks==1,"TTL after committed target preserves partial-sink hard failure");}
    else{
      need(!threw&&!decision.allowed&&decision.reason==a::BoundaryReason::VisualInput&&sinks==0,
           "TTL immediately before first target returns recoverable refusal with zero sinks");
      q.r.request_visual_land(q.s,q.t,"TEST_FIRST_SINK_EXPIRY");q.tick();
      need(q.r.attitude_core().phase()==a::Phase::Descent&&q.target_committed&&q.heartbeat_committed,
           "uncommitted task-input expiry resumes native descent without hard Safety");
    }
  }
}
// RC 摇杆介入沿原受控下降；实际离开 Offboard 或 Kill 沿原 Safety 停流，视觉不能重发解锁/入模式。
static void visual_motion_retains_native_rc_and_failsafe_priority(){
  for(int kind=0;kind<3;++kind){
    VisionFlight q;q.reach_hold();q.set_visual(input(q.t+.05));q.tick();
    need(q.o.attitude.visual_motion,"priority exercise starts from visual movement");
    if(kind==0){q.s.attitude.rc_sticks_moving=true;q.s.attitude.rc_axes_safe=false;}
    if(kind==1){q.s.feedback.offboard=false;q.s.nav_state=15;}
    if(kind==2)q.s.attitude.kill=true;
    q.tick();
    need(!q.o.attitude.visual_motion,"RC or failsafe always removes visual movement");
    if(kind==0)need(q.r.attitude_core().phase()==a::Phase::Descent&&q.target_committed&&q.heartbeat_committed,
      "RC stick intervention keeps original native controlled descent");
    else need(!q.o.attitude.publish&&q.r.attitude_core().phase()==a::Phase::Safety,
      "actual mode-out and Kill keep original ownership release");
    need(q.arm_commands==0&&q.mode_commands==0,"vision never re-enters mode after intervention");
  }
}
int main(){
  motion_runs_only_after_existing_manual_start();disabled_visual_preserves_original_level_output();
  state_and_target_change_control_calculation();lowering_target_preserves_original_descent_rate();
  expiration_requests_native_descent_and_cannot_resume();final_sink_refuses_expired_or_changed_motion();
  native_landing_keeps_last_committed_yaw();
  first_sink_expiry_is_recoverable_and_partial_expiry_is_hard();
  visual_motion_retains_native_rc_and_failsafe_priority();
  std::cout<<"VISUAL_RUNTIME_CONTROL_PASS\n";
}
