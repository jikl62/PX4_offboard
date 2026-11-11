// 早期高度反馈与推力时间表回归：闭环可制动，却不能越过既有起飞斜坡上限。
// 测试正常50 ms与抖动dt；demand由原悬停斜坡加随后储备斜坡计算，归一化无量纲。
#include "ramp_guard_test_support.hpp"
// 即便内部一致的高度提前变化，推力不得比时间表更快；临时起飞上限不应误触物理饱和超时。
static void early_height_cannot_accelerate_scheduled_ramp(bool jitter){
  auto c=guard_config();c.attitude.feedback_guard.enabled=true;
  GuardFlight q(.5,c);q.s.attitude.z_deriv=0;
  const double periods[]={.02,.10,.05,.08};double prior=0;
  for(int i=0;i<100;++i){
    const double dt=jitter?periods[i%4]:.05;
    const double age=q.t-q.ramp_start+dt;
    // An internally consistent estimate can move before the actual detector
    // confirms liftoff. It may engage braking but cannot speed up the ramp.
    q.observe(.10*age,.10);q.advance(dt);
    const double demand=.50*std::min(1.,age/2.)+.04*std::clamp((age-2.)/2.,0.,1.);
    require_guard(q.o.attitude.publish,"temporary takeoff ceiling must not trigger persistent saturation Safety");
    require_guard(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"height alone cannot certify liftoff");
    require_guard(q.thrust()<=demand+1e-9,"early height feedback must not outrun scheduled ramp demand");
    require_guard(std::abs(q.thrust()-prior)<=.4*dt+1e-9,"the scheduled ceiling preserves physical slew");
    prior=q.thrust();
  }
}
// 把速度增益放大故意打到真实物理上限，饱和看门狗仍要触发，不能被临时起飞上限掩盖。
static void physical_saturation_still_expires_under_takeoff_ceiling(){
  auto c=guard_config();c.attitude.vertical.velocity_gain=2.;
  // Isolate the unchanged physical-saturation watchdog, independently of
  // feedback-consistency tests in the existing dedicated guard suite.
  GuardFlight q(.5,c);q.observe(.13,-.5);
  bool stopped=false;
  for(int i=0;i<40;++i){
    q.advance();
    if(q.r.attitude_core().phase()==a::Phase::Safety){
      require_guard(q.r.attitude_core().reason()=="PERSISTENT_THRUST_SATURATION",
        "physical saturation must keep its explicit failure reason");
      require_guard(!q.o.attitude.publish,"physical saturation suppresses output");
      stopped=true;break;
    }
    require_guard(q.o.attitude.saturated,
      "raw physical overflow must remain visible under the takeoff ceiling");
  }
  require_guard(stopped,"active takeoff ceiling cannot mask the physical saturation timeout");
}
int main(){
  early_height_cannot_accelerate_scheduled_ramp(false);
  early_height_cannot_accelerate_scheduled_ramp(true);
  physical_saturation_still_expires_under_takeoff_ceiling();
  std::cout<<"R632_EARLY_FEEDBACK_RAMP_SCHEDULE_PASS\n";
}
