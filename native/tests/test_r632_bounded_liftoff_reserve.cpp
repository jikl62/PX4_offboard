// R6.3.2 有界起飞储备回归：显式开启后才在悬停斜坡结束后渐加储备，不凭推力证明离地。
// 使用原Runtime合成夹具；归一化悬停0.50、储备0.04、slew0.4/s都是测试参数，FRD z输出取负。
// R6.3.2 focused synthetic test; never publishes to PX4.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main baseline_attitude_runtime_main_not_called
#include "test_attitude_runtime.cpp"
#undef main
#pragma GCC diagnostic pop

static f::Config r63_config(){
  auto c=cfg(true);c.attitude.vertical.hover=.50;
  c.attitude.bench_arm_only_enabled=false;c.validate();return c;
}
// 前2 s不得超过悬停值，后续只能按原slew爬到0.54；检测/高度证据缺失时仍ThrustRamp。
static void bounded_reserve_is_opt_in_and_only_after_hover(){
  auto cfg=r63_config();
  cfg.attitude.takeoff_reserve_thrust=.04;
  cfg.attitude.vertical.slew=.4; // real R6.3.2 candidate
  cfg.attitude.ramp_seconds=2.;
  cfg.validate();
  f::Runtime r(cfg);auto s=sample();double t=ready(r,s,true);
  assert(r.request("INTERACTIVE",{},s,t).accepted);
  assert(r.request("TAKEOFF",{.6},s,t).accepted);
  auto o=tick(r,s,t+=.05);assert(o.attitude.command==a::Command::Arm);
  auto base=r.pending(400).baseline;
  r.ack({base+1,400,0,1,191,false},t+.01);
  s.feedback.armed=true;
  o=tick(r,s,t+=.05);
  assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  for(int i=0;i<35&&r.attitude_core().phase()!=a::Phase::ThrustRamp;++i){
    assert(r.request("PING",{},s,t).accepted);
    o=tick(r,s,t+=.05);
    assert(o.attitude.publish&&o.attitude.thrust[2]==0);
  }
  assert(r.attitude_core().phase()==a::Phase::ThrustRamp);
  double prior=0.,early_max=0.,late_max=0.;bool saw_after_hover=false;
  for(int i=0;i<130;++i){
    assert(r.request("PING",{},s,t).accepted);
    o=tick(r,s,t+=.05);
    assert(r.attitude_core().phase()==a::Phase::ThrustRamp);
    assert(o.attitude.publish&&o.attitude.command==a::Command::None);
    const double mag=-o.attitude.thrust[2];
    assert(mag>=0&&mag<=.54000001);
    assert(std::abs(mag-prior)<=.4*.05+1e-6); // original slew bound preserved
    if(i<39)early_max=std::max(early_max,mag);
    if(i>90)late_max=std::max(late_max,mag);
    if(mag>.500001)saw_after_hover=true;
    prior=mag;
  }
  assert(early_max<=.5000001);
  assert(saw_after_hover && late_max>.539);
  // Explicitly: no detector/height evidence => no ascent despite thrust reserve.
  assert(s.feedback.landed&&r.attitude_core().phase()==a::Phase::ThrustRamp);
  std::cout<<"R632_BOUNDED_RESERVE_WITHOUT_FALSE_LIFTOFF_PASS\n";
}
// 持续未离地最终走已有PoweredGround减推恢复，不允许因储备已用完就伪造Ascend。
static void bounded_reserve_no_liftoff_still_recovers(){
  auto c=r63_config();c.attitude.takeoff_reserve_thrust=.04;
  c.attitude.ramp_seconds=2.;c.attitude.vertical.slew=.4;c.validate();
  f::Runtime r(c);auto s=sample();double t=ready(r,s,true);
  assert(r.request("INTERACTIVE",{},s,t).accepted);
  assert(r.request("TAKEOFF",{.6},s,t).accepted);
  auto o=tick(r,s,t+=.05);assert(o.attitude.command==a::Command::Arm);
  auto base=r.pending(400).baseline;
  r.ack({base+1,400,0,1,191,false},t+.01);
  s.feedback.armed=true;
  tick(r,s,t+=.05);
  bool saw_ramp=false, recovered=false;
  for(int i=0;i<280;++i){
    assert(r.request("PING",{},s,t).accepted);
    o=tick(r,s,t+=.05);
    auto p=r.attitude_core().phase();
    saw_ramp |= p==a::Phase::ThrustRamp;
    assert(p!=a::Phase::Ascend && p!=a::Phase::HeightHold);
    assert(o.attitude.thrust[2]<=0 && o.attitude.thrust[2]>=-.540001);
    if(p==a::Phase::PoweredGround){
      recovered=true;
      assert(o.attitude.publish);
      break;
    }
    assert(o.attitude.publish);
  }
  assert(saw_ramp && recovered);
  std::cout<<"R632_LIFTOFF_TIMEOUT_GUARDED_POWER_REMOVAL_PASS\n";
}
// 负储备、超0.06、NaN或超过物理最大推力的组合在配置校验时拒绝。
static void invalid_reserve_must_fail_closed(){
  for(double x:{-.01,.061,static_cast<double>(NAN)}){
    auto c=r63_config();c.attitude.takeoff_reserve_thrust=x;
    bool rejected=false;
    try{c.validate();}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  auto c=r63_config();c.attitude.takeoff_reserve_thrust=.04;
  c.attitude.vertical.maximum=.50;
  bool rejected=false;
  try{c.validate();}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
  std::cout<<"R632_INVALID_RESERVE_DENIAL_PASS\n";
}
// 储备默认0保持原路径；其它原流程由独立既有回归覆盖，不在此暗改默认。
static void bench_and_original_path_unchanged(){
  auto c=r63_config(); // zero reserve defaults to prior R6.3.1
  assert(c.attitude.takeoff_reserve_thrust==0.);
  // The original 0-reserve path is independently exercised by r63_offline_verify.sh.
  std::cout<<"R632_R631_ZERO_RESERVE_COMPATIBILITY_PASS\n";
}
int main(){
  bounded_reserve_is_opt_in_and_only_after_hover();
  bounded_reserve_no_liftoff_still_recovers();
  invalid_reserve_must_fail_closed();
  bench_and_original_path_unchanged();
  std::cout<<"R632_FOCUSED_OFFLINE_PASS\n";
}
