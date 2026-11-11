// 起飞斜坡早期反馈回归：高度可先触发闭环制动，正式离地仍依赖独立新检测与运动证据。
// GuardFlight坐标转换保持上正高度/速度到PX4 NED；控制证据与离地认证是两个不同条件。
#include "ramp_guard_test_support.hpp"
// 检测器landed缓存滞后时，观测已过目标也应先制动；减推受slew约束，不能直接切零。
static void above_target_brakes_with_landed_cache(){
  GuardFlight q;q.ground_ramp();
  // Continuous observations remain below the existing 0.45 m jump limit.
  q.observe(.30,.15);q.advance(.05,false);
  q.observe(.56,.15);q.advance(.05,false);
  const double prior=q.thrust();
  q.advance(.05,false);
  require_guard(q.o.attitude.publish,"healthy feedback retains output custody");
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"cached landed cannot authorize Ascend");
  require_guard(q.o.attitude.velocity_target<0,"above goal must request braking before detector confirmation");
  require_guard(q.thrust()<prior,"above goal cannot keep the open-loop lift reserve");
  require_guard(q.thrust()>0&&prior-q.thrust()<=.4*.05+1e-9,"braking preserves physical slew, never zero cut");
}
// 跨过早期高度门后闭环保持接入，不因小幅回落在开环/闭环之间来回跳。
static void feedback_starts_at_liftoff_height(){
  GuardFlight q;q.ground_ramp(3.);
  q.observe(.13,.10);q.advance(.05,false);
  require_guard(q.o.attitude.velocity_target>0,"height feedback must start before detector confirmation");
  require_guard(q.o.attitude.target_height>=.13&&q.o.attitude.target_height<=.5,"reference joins measured height and mission bound");
  const double prior=q.o.attitude.target_height;
  q.observe(.10,.02);q.advance(.05,false);
  require_guard(q.o.attitude.target_height>prior,"feedback remains latched across a small height dip");
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"height is control evidence, not certified liftoff");
}
// 单个!landed被重复缓存不能完成dwell；第二个独立检测到来才Ascend。
static void one_detector_message_never_completes_dwell(){
  GuardFlight q;q.ground_ramp();q.observe(.2,.10);
  q.s.feedback.landed=false;q.s.at_rest=false;q.advance();
  for(int i=0;i<18;++i){q.observe(.20+.005*(i+1),.10);q.advance(.05,false);}
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"replayed !landed must not count as independent samples");
  require_guard(q.o.attitude.velocity_target>0,"detector delay does not freeze the feedback controller");
  q.advance();
  require_guard(q.r.attitude_core().phase()==a::Phase::Ascend,"two distinct detector samples still admit real liftoff");
}
// 连续新local上升证据可跨到目标悬停，不能为了证明离地强求目标处继续上升。
static void hovering_after_sustained_rise_can_complete_detector_dwell(){
  GuardFlight q;q.ground_ramp(3.);
  q.s.feedback.landed=false;q.s.at_rest=false;
  for(int i=0;i<8;++i){q.observe(.13+.01*i,.20);q.advance(.05,i==0);}
  q.observe(.22,0);
  for(int i=0;i<10;++i)q.advance(.05,false);
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"same detector sample cannot certify hover");
  q.advance();
  require_guard(q.r.attitude_core().phase()==a::Phase::Ascend,
    "fresh !landed plus sustained NEW local rising samples must not demand continued climb at the goal");
}
// 只有位置偏置、没有真实新上升速度不能代替运动证据。
static void stationary_offset_cannot_replace_velocity_evidence(){
  GuardFlight q;q.ground_ramp();q.observe(.2,0);
  q.s.feedback.landed=false;q.s.at_rest=false;
  for(int i=0;i<20;++i)q.advance();
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"height offset with no observed climb cannot prove liftoff");
}
// local戳冻结时改数值也不可积累持续上升证据。
static void stale_local_samples_cannot_store_rise_evidence(){
  GuardFlight q;q.ground_ramp();q.s.feedback.landed=false;q.s.at_rest=false;
  for(int i=0;i<8;++i){q.observe(.13+.01*i,.20);q.advance(.05,i==0,false);}
  q.observe(.22,0);
  for(int i=0;i<18;++i)q.advance(.05,i%10==0,false);
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"cached local stamps cannot establish sustained rise");
}
// 正常地面斜坡仍可单调到原0.54储备，新增反馈门不能让正常起飞永远达不到推力。
static void ground_thrust_reaches_bounded_reserve(){
  GuardFlight q;double prior=0;
  for(int i=0;i<90;++i){
    q.advance();require_guard(q.thrust()>=prior-1e-9,"ground ramp stays monotonic");
    require_guard(q.thrust()-prior<=.4*.05+1e-9,"ground ramp remains slew bounded");
    require_guard(q.thrust()<=.54+1e-9,"ground thrust retains reviewed reserve ceiling");
    prior=q.thrust();
  }
  require_guard(q.thrust()>.5399,"guard cannot prevent reaching lift reserve on the ground");
  require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"no false liftoff on thrust alone");
}
// 保留建立过的升力trim便于交接，但trim不是推力下限，速度过快仍要允许制动。
static void feedback_retains_lift_reserve_but_allows_braking(){
  GuardFlight climb;climb.ground_ramp();climb.observe(.13,.10);
  climb.s.feedback.landed=false;climb.s.at_rest=false;
  for(int i=0;i<8;++i){climb.observe(.13+.005*i,.10);climb.advance();}
  require_guard(climb.o.attitude.velocity_target>.10,"positive speed error in this scenario");
  require_guard(climb.thrust()>.54,"feedback may add correction to established lift trim, not fall back to 0.50");
  GuardFlight brake;brake.ground_ramp();brake.observe(.13,.5);brake.advance();
  require_guard(brake.thrust()<.54,"retained lift trim must never become a minimum-thrust floor");
}
// reset/过期/跳变/NaN/Kill在闭环接入后仍锁存停流，不能用新TAKEOFF绕过。
static void estimator_faults_and_takeover_remain_latched(){
  for(int kind=0;kind<7;++kind){
    GuardFlight q;q.ground_ramp(3.);q.observe(.13,.1);q.advance();
    if(kind==0)q.s.attitude.z_reset++;
    if(kind==1)q.s.attitude.vz_reset++;
    if(kind==2)q.s.attitude.quat_reset++;
    if(kind==3)q.s.local_fresh=false;
    if(kind==4)q.s.attitude.z-=.46;
    if(kind==5)q.s.attitude.vz=NAN;
    if(kind==6)q.s.attitude.kill=true;
    q.advance();
    require_guard(!q.o.attitude.publish,"invalid feedback/reset/jump/Kill still revokes output");
    require_guard(q.r.attitude_core().phase()==a::Phase::Safety,"fault remains latched");
    require_guard(!q.r.request("TAKEOFF",{.5},q.s,q.t).accepted,"no automatic rearm after fault");
  }
}
// 早期高度已跨离地控制门时，即便尚未正式认证，LAND/断线/超时也不能误归类地面直接撤动力。
static void early_feedback_is_not_ground_abort_evidence(){
  for(int cause=0;cause<4;++cause){
    GuardFlight q(.13);q.ground_ramp();q.observe(.13,0);
    if(cause!=3)q.advance(); // LAND may see a new sample before the next control tick.
    // .13 is above liftoff control threshold but below the old .15 ground
    // tolerance. A pending landed cache cannot resolve this overlap.
    if(cause==0){
      for(int i=0;i<160&&q.r.attitude_core().phase()==a::Phase::ThrustRamp;++i)q.advance();
    }else if(cause==1||cause==3){
      require_guard(q.r.request("LAND",{},q.s,q.t).accepted,"LAND remains available");
      q.advance();
    }else{
      q.r.disconnect(q.s,q.t);q.advance();
    }
    require_guard(q.r.attitude_core().phase()==a::Phase::Descent,
      "timeout/LAND/console loss after early feedback must descend, not classify PoweredGround");
    require_guard(q.o.attitude.publish&&q.thrust()>0&&q.o.attitude.command==a::Command::None,
      "uncertain low-altitude recovery must retain controlled lift without a Disarm command");
    require_guard(q.o.attitude.velocity_target<=0,"recovery cannot resume upward trajectory");
  }
}
// 超时同一tick收到的新跨门样本仍须承担受控下降，推力变化按真实dt限幅。
static void threshold_crossing_on_timeout_tick_retains_control(){
  GuardFlight q;q.ground_ramp();
  while(q.t-q.ramp_start+.10<q.c.attitude.ascent_timeout)q.advance();
  q.observe(.13,0);const double prior=q.thrust();q.advance(.10);
  require_guard(q.r.attitude_core().phase()==a::Phase::Descent,
    "a newly received height crossing on the timeout tick is not ground proof");
  require_guard(q.thrust()>0&&std::abs(q.thrust()-prior)<=.4*.10+1e-9,
    "timeout crossing keeps bounded nonzero control");
}
// 低于离地证明门的任务目标不能降低认证要求；不支持的目标最终有界恢复，不能永远斜坡。
static void sub_liftoff_goal_never_weakens_certification(){
  GuardFlight q(.05);q.ground_ramp();q.observe(.05,0);q.advance();
  require_guard(q.o.attitude.target_height<=.05,"small goal still bounds feedback reference");
  for(int i=0;i<160&&q.r.attitude_core().phase()==a::Phase::ThrustRamp;++i){
    q.advance();
    require_guard(q.r.attitude_core().phase()!=a::Phase::Ascend&&
                  q.r.attitude_core().phase()!=a::Phase::HeightHold,
      "a goal below proof height cannot certify liftoff");
  }
  require_guard(q.r.attitude_core().phase()==a::Phase::Descent,
    "small unsupported goal must enter bounded recovery instead of waiting forever");
}
int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="all"||which=="brake")above_target_brakes_with_landed_cache();
  if(which=="all"||which=="early")feedback_starts_at_liftoff_height();
  if(which=="all"||which=="detector")one_detector_message_never_completes_dwell();
  if(which=="all"||which=="hover")hovering_after_sustained_rise_can_complete_detector_dwell();
  if(which=="all"||which=="offset")stationary_offset_cannot_replace_velocity_evidence();
  if(which=="all"||which=="cached")stale_local_samples_cannot_store_rise_evidence();
  if(which=="all"||which=="reserve")ground_thrust_reaches_bounded_reserve();
  if(which=="all"||which=="trim")feedback_retains_lift_reserve_but_allows_braking();
  if(which=="all"||which=="faults")estimator_faults_and_takeover_remain_latched();
  if(which=="all"||which=="abort")early_feedback_is_not_ground_abort_evidence();
  if(which=="all"||which=="timeout-crossing")threshold_crossing_on_timeout_tick_retains_control();
  if(which=="all"||which=="small-goal")sub_liftoff_goal_never_weakens_certification();
  std::cout<<"R632_RAMP_HEIGHT_GUARD_PASS "<<which<<'\n';
}
