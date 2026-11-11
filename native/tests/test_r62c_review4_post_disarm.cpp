// 台架第四轮上锁后复核：区分真提交、因果新未解锁状态、后续估计漂移和独立静止终态。
// measured_tail读取17行实测，其余缺回调/静止/漂移均明确合成；本机提交不证明PX4实际执行。
#include "review_bench_support.hpp"
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <vector>

// Synthetic PX4 observations drive the real Runtime, Core and final sinks.
// Only measured_tail() consumes recorded phase-event values. Missing callbacks
// and all later at_rest/landing responses are explicitly synthetic.
// 串联零推力台架、上锁计划、假提交、真实模拟未解锁、静止窗与176释放，保留一次命令计数。
struct R4Bench : B {
  R4Bench():B(true){}
  void spinning(){armed();s.at_rest=false;s.stationary_seconds=0;}
  a::Output plan(){
    for(int i=0;i<160;++i){
      auto o=next(true,true,false);
      if(o.command==a::Command::Disarm)return o;
      commit(o);
    }
    assert(false&&"new armed ground dwell must plan Disarm");return {};
  }
  void stopped(){spinning();commit(plan());s.feedback.armed=false;next();
    assert(r.attitude_core().phase()==a::Phase::RequestDisarm);}
  void drift(double amount=.006){s.attitude.z+=amount;s.attitude.vz=.18;}
  void finish(){
    s.at_rest=true;
    for(int i=0;i<110&&r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i){drift();next();}
    assert(r.attitude_core().phase()==a::Phase::SafeDisarmed);
  }
  void release(){
    next();assert(releases==1);
    r.ack({r.pending(176).wire_stamp+1000,176,0,1,191,false},t+.001);
    s.feedback.offboard=false;s.nav_state=15;next();
    assert(r.attitude_core().phase()==a::Phase::Idle);
  }
};

// 回放实测17行未解锁尾部但不发明at_rest；故仍RequestDisarm，无state_verified与释放。
static void measured_tail(){
  R4Bench h;h.s.attitude.z=-.196438625;h.spinning();h.commit(h.plan());
  const auto p=std::filesystem::path(__FILE__).parent_path()/"data/review3_measured_post_disarm.csv";
  std::ifstream in(p);assert(in);std::string line;std::getline(in,line);
  double origin=0,first_time=0;std::uint64_t status0=0,land0=0,local0=0;
  const auto status_base=h.s.status_stamp,land_base=h.s.land_stamp,local_base=h.s.attitude.local_stamp;
  unsigned rows=0;
  while(std::getline(in,line)){
    for(auto& c:line)if(c==',')c=' ';
    std::istringstream row(line);double now,z0,z,vz;std::uint64_t status,land,local;int armed,landed,rest;
    assert(row>>now>>status>>land>>local>>armed>>landed>>rest>>z0>>z>>vz);
    if(!rows){origin=h.t+.05;first_time=now;status0=status;land0=land;local0=local;}
    h.t=origin+now-first_time;
    h.s.status_stamp=status_base+status-status0;h.s.land_stamp=land_base+land-land0;
    h.s.attitude.local_stamp=local_base+local-local0;
    h.s.feedback.armed=armed;h.s.feedback.landed=landed;h.s.at_rest=rest;
    h.s.attitude.z=z;h.s.attitude.vz=vz;
    assert(std::abs(h.r.attitude_core().reference_z()-z0)<1e-12);
    assert(h.r.request("PING",{},h.s,h.t).accepted);
    const auto out=h.r.step(h.s,h.t).attitude;h.commit(out);
    h.log("REVIEW4_MEASURED_TAIL",out);
    assert(out.publish&&h.r.attitude_core().phase()==a::Phase::RequestDisarm);
    assert(out.command==a::Command::None&&h.releases==0&&!h.r.pending(400).state_verified);
    ++rows;
  }
  assert(rows==17&&h.arms==1&&h.disarms==1);
  assert(h.r.attitude_core().reference_z()-h.s.attitude.z<-.50);
  std::cout<<"REVIEW4_MEASURED_POST_DISARM_TAIL_SURVIVES_WITHOUT_INVENTING_REST PASS\n";
}

// 因果新未解锁后连续估计漂移不能强求回到旧z0；新静止地面窗完成后只能state-only终态。
static void drift_rest_and_release(){
  R4Bench h;h.stopped();
  for(int i=0;i<180;++i){h.drift();const auto out=h.next();
    assert(out.publish&&h.r.attitude_core().phase()==a::Phase::RequestDisarm);
    assert(!h.r.pending(400).state_verified&&h.releases==0);}
  assert(h.s.attitude.z-h.r.attitude_core().reference_z()>1.);
  h.finish();
  assert(h.r.pending(400).state_verified&&!h.r.pending(400).accepted);
  assert(h.r.pending(400).stage==f::CommandStage::StateOnlyTerminal);
  assert(h.r.pending(400).generation==2);
  h.release();
  assert(h.arms==1&&h.disarms==1&&h.targets==h.heartbeats&&h.denials==0);
  std::cout<<"REVIEW4_SYNTHETIC_CONTINUOUS_DRIFT_NEW_REST_ONE_ARM_ONE_DISARM_TO_IDLE PASS\n";
}

// 重新解锁/离地/跳变/reset/过期/时钟/控制权等任一异常仍Safety，不能沿用台架宽容。
static void post_disarm_attacks(){
  const std::function<void(f::Snapshot&)> attacks[]={
    [](auto& s){s.feedback.armed=true;},[](auto& s){s.feedback.landed=false;},
    [](auto& s){s.attitude.z+=.46;},[](auto& s){s.attitude.vz+=1.51;},
    [](auto& s){++s.attitude.z_reset;},[](auto& s){++s.attitude.vz_reset;},
    [](auto& s){++s.attitude.quat_reset;},[](auto& s){s.status_fresh=false;},
    [](auto& s){s.land_fresh=false;},[](auto& s){s.local_fresh=false;},
    [](auto& s){s.attitude.attitude_fresh=false;},[](auto& s){s.attitude.failsafe_fresh=false;},
    [](auto& s){s.attitude.z=NAN;},[](auto& s){s.attitude.vz=NAN;},
    [](auto& s){s.attitude.kill=true;},[](auto& s){s.attitude.rc_link_ready=false;},
    [](auto& s){s.attitude.source_clock_valid=false;},[](auto& s){s.arming_state_known=false;},
    [](auto& s){s.feedback.offboard=false;},[](auto& s){s.feedback.failsafe=true;}
  };
  for(const auto& attack:attacks){
    R4Bench h;h.stopped();attack(h.s);const auto out=h.next();
    assert(h.r.attitude_core().phase()==a::Phase::Safety&&!out.publish);
    assert(!h.r.pending(400).state_verified&&h.arms==1&&h.disarms==1&&h.releases==0);
  }
  std::cout<<"REVIEW4_POST_DISARM_REARM_LANDED_JUMPS_RESETS_STALE_CLOCK_AND_AUTHORITY_BLOCKED PASS\n";
}

// status/land/local任一冻结不能完成上锁；恢复连续新源后才可结束。
static void frozen_sources_cannot_finish(){
  for(int which=0;which<3;++which){
    R4Bench h;h.stopped();h.s.at_rest=true;
    const auto status=h.s.status_stamp;
    for(int i=0;i<160;++i){
      if(which==0)h.s.status_stamp=status-50000;
      h.drift();h.next(true,which!=1,true,which!=2);
      assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm);
      assert(!h.r.pending(400).state_verified&&h.releases==0);
    }
    h.finish();h.release();assert(h.disarms==1);
  }
  std::cout<<"REVIEW4_POST_DISARM_FROZEN_STATUS_LAND_LOCAL_NEVER_COMPLETE_OLD_DWELL PASS\n";
}

// 只有计划未实际提交的Disarm不得获得提交后宽容，否则会伪造协议因果。
static void uncommitted_disarm_never_gets_post_disarm_grace(){
  R4Bench h;h.spinning();h.plan();assert(h.disarms==0);
  h.s.feedback.armed=false;
  for(int i=0;i<110&&h.r.attitude_core().phase()!=a::Phase::Safety;++i){h.drift();h.next();}
  assert(h.r.attitude_core().phase()==a::Phase::Safety&&h.disarms==0&&h.releases==0);
  assert(!h.r.pending(400).state_verified);
  std::cout<<"REVIEW4_UNCOMMITTED_DISARM_CANNOT_BYPASS_ARMED_ENVELOPE PASS\n";
}

// 提交后仍真实armed时保留原已解锁包络，连续漂移越界仍必须停流。
static void armed_boundary_unchanged(){
  R4Bench h;h.spinning();h.commit(h.plan());
  for(int i=0;i<110&&h.r.attitude_core().phase()!=a::Phase::Safety;++i){h.drift();h.next();}
  assert(h.r.attitude_core().phase()==a::Phase::Safety&&h.s.feedback.armed);
  assert(h.r.attitude_core().reason()=="HEIGHT_OR_SPEED_ENVELOPE_EXCEEDED");
  assert(h.disarms==1&&h.releases==0);
  std::cout<<"REVIEW4_ARMED_LOWER_HEIGHT_BOUND_UNCHANGED PASS\n";
}

// 外部未解锁可独立证明安全地面，不能捏造本机Disarm记录。
static void external_stop_stays_external(){
  R4Bench h;h.spinning();h.s.feedback.armed=false;
  h.next();
  for(int i=0;i<120;++i){h.drift();h.next();
    assert(h.r.attitude_core().phase()==a::Phase::PoweredGround&&h.releases==0);}
  h.finish();
  assert(h.r.attitude_core().reason()=="BENCH_EXTERNAL_DISARM_NEW_SAFE_GROUND_EVIDENCED");
  assert(h.disarms==0&&h.r.pending(400).command400_kind==f::Command400Kind::Arm);
  h.release();assert(h.disarms==0);
  std::cout<<"REVIEW4_EXTERNAL_DISARM_DRIFT_REST_RELEASE_NEVER_ATTRIBUTED_TO_COMPANION PASS\n";
}

// 即使已SafeDisarmed，模式释放前仍复核当前健康、reset/跳变、地面与bench范围。
static void release_sink_races(){
  R4Bench h;h.stopped();for(int i=0;i<100;++i){h.drift();h.next();}h.finish();h.next();
  const auto original=h.r.publication_context(h.s,h.t,true,true);
  assert(a::release_publication_policy(original).allowed);
  const std::function<void(a::BoundaryContext&)> attacks[]={
    [](auto& c){c.feedback.armed=true;},[](auto& c){c.feedback.landed=false;},
    [](auto& c){c.feedback.at_rest=false;},[](auto& c){c.feedback.status_fresh=false;},
    [](auto& c){c.feedback.land_fresh=false;},[](auto& c){c.feedback.local_fresh=false;},
    [](auto& c){c.feedback.z=NAN;},[](auto& c){c.feedback.vz=NAN;},
    [](auto& c){c.ground_confirmed=false;},[](auto& c){c.flight_started=true;},
    [](auto& c){c.config.takeoff_enabled=true;},[](auto& c){c.bench_arm_only_active=false;}
    ,[](auto& c){c.feedback.z+=.46;},[](auto& c){c.feedback.vz+=1.51;}
    ,[](auto& c){++c.feedback.z_reset;},[](auto& c){++c.feedback.vz_reset;}
    ,[](auto& c){++c.feedback.quat_reset;}
  };
  for(const auto& attack:attacks){auto c=original;attack(c);unsigned count=0;
    assert(!a::publish_release_at_boundary(c,[&]{++count;}).allowed&&count==0);}
  std::cout<<"REVIEW4_POST_DISARM_RELEASE_SINK_RECHECKS_NEW_STATE_AND_BENCH_SCOPE PASS\n";
}

// 从SafeDisarmed到释放的最后窗口也保留Core跳变/reset门，避免阶段尾部保护缺口。
static void post_safe_core_guards(){
  for(int which=0;which<5;++which){
    R4Bench h;h.stopped();h.finish();h.s.stationary_seconds=3;
    if(which==0)h.s.attitude.z+=.46;
    if(which==1)h.s.attitude.vz+=1.51;
    if(which==2)++h.s.attitude.z_reset;
    if(which==3)++h.s.attitude.vz_reset;
    if(which==4)++h.s.attitude.quat_reset;
    const auto out=h.next();
    assert(h.r.attitude_core().phase()==a::Phase::Safety&&!out.publish&&h.releases==0);
  }
  std::cout<<"REVIEW4_POST_SAFE_CORE_JUMPS_AND_RESETS_REMAIN_GUARDED_THROUGH_RELEASE PASS\n";
}

// 已离开Offboard的Safety只读观察可报告新安全地面，不把它标成任务成功或400已完成。
static void terminal_observation_outside_offboard(){
  R4Bench h;h.stopped();h.s.feedback.offboard=false;h.s.nav_state=15;h.next();
  assert(h.r.attitude_core().phase()==a::Phase::Safety);
  h.s.at_rest=true;
  for(int i=0;i<100;++i){const auto out=h.next();assert(!out.publish);}
  const auto current=h.r.publication_context(h.s,h.t,true,true).feedback;
  assert(std::string(h.r.attitude_core().recovery_responsibility(current))=="GROUND_DISARMED_CONFIRMED");
  assert(h.r.attitude_core().phase()==a::Phase::Safety&&h.releases==0);
  assert(!h.r.pending(400).state_verified);
  std::cout<<"REVIEW4_SAFETY_READ_ONLY_GROUND_CONFIRMATION_OUTSIDE_OFFBOARD_NO_TASK_SUCCESS PASS\n";
}

int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="recorded"||which=="all")measured_tail();
  if(which=="cycle"||which=="all")drift_rest_and_release();
  if(which=="attacks"||which=="all")post_disarm_attacks();
  if(which=="freeze"||which=="all")frozen_sources_cannot_finish();
  if(which=="uncommitted"||which=="all")uncommitted_disarm_never_gets_post_disarm_grace();
  if(which=="armed"||which=="all")armed_boundary_unchanged();
  if(which=="external"||which=="all")external_stop_stays_external();
  if(which=="release"||which=="all")release_sink_races();
  if(which=="post_safe"||which=="all")post_safe_core_guards();
  if(which=="terminal"||which=="all")terminal_observation_outside_offboard();
}
