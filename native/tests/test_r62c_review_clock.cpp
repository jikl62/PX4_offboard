// 同步源时钟因果回归：线上命令戳、同步偏差预算与新物理状态共同约束ACK和事务终态。
// 使用固定epoch的合成µs与独立单调s；允许小同步偏差不等于允许命令前ACK/状态证明命令后事件。
#include "review_clock_support.hpp"
#include "jikl_fc01_system/source_clock.hpp"
// 命令前ACK即使ACCEPTED/DENIED也不得归属本次Arm；越过因果floor的新ACK才可接受。
static void precommand_ack_cannot_match(){
  for(int result:{0,2}){
    Harness h;h.ready();assert(h.r.request("TAKEOFF",{.6},h.s,h.t).accepted);h.tick();
    const auto wire=h.r.pending(400).wire_stamp;
    h.r.ack({wire-100000,400,result,1,191,false},h.t+.01);
    assert(!h.r.pending(400).accepted&&!h.r.pending(400).rejected);
    assert(h.r.pending(400).ack_observations==0);
    h.r.ack({wire+f::source_clock_skew_budget_us+1000,400,0,1,191,false},h.t+.02);
    assert(h.r.pending(400).accepted);
  }
  std::cout<<"REVIEW_PRECOMMAND_ACK_CANNOT_MATCH_CURRENT_ARM PASS\n";
}
// 已知armed但源戳仍在命令前不能验证当前Arm，须等独立更新越过wire同步预算。
static void precommand_arm_state_cannot_verify(){
  Harness h;h.ready();assert(h.r.request("TAKEOFF",{.6},h.s,h.t).accepted);h.tick();
  const auto wire=h.r.pending(400).wire_stamp;
  h.r.ack({wire+f::source_clock_skew_budget_us+1000,400,0,1,191,false},h.t+.001);
  h.s.feedback.armed=true;h.s.status_stamp=wire-100000;
  auto o=h.r.step(h.s,h.t+.01).attitude;
  assert(!h.r.pending(400).state_verified);
  assert(h.r.attitude_core().phase()==a::Phase::RequestArm);
  assert(o.command==a::Command::None);
  h.s.status_stamp=wire+f::source_clock_skew_budget_us+2000;
  h.r.step(h.s,h.t+.02);
  assert(h.r.pending(400).state_verified);
  assert(h.r.attitude_core().phase()==a::Phase::ArmedGround);
  std::cout<<"REVIEW_PRECOMMAND_STATUS_CANNOT_VERIFY_CURRENT_ARM PASS\n";
}
// 上锁状态与land需各自越过提交因果水位，再保留台架新静止窗，不由单帧完成。
static void precommand_disarm_evidence_cannot_release(){
  Harness h(true);h.ready();assert(h.r.request("ARM_ONLY",{},h.s,h.t).accepted);h.tick();
  h.r.ack({h.r.pending(400).wire_stamp+f::source_clock_skew_budget_us+1000,400,0,1,191,false},h.t+.001);
  h.s.feedback.armed=true;
  for(int i=0;i<12&&h.r.attitude_core().phase()!=a::Phase::BenchArmedGround;++i)h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::BenchArmedGround);
  for(int i=0;i<260&&h.r.attitude_core().phase()!=a::Phase::RequestDisarm;++i)h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm&&h.r.pending(400).ack_ambiguous);
  const auto wire=h.r.pending(400).wire_stamp;
  h.s.feedback.armed=false;h.s.status_stamp=h.s.land_stamp=wire-100000;
  auto o=h.r.step(h.s,h.t+.01).attitude;
  assert(o.publish&&!h.r.pending(400).state_verified);
  assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm);
  h.s.status_stamp=wire+f::source_clock_skew_budget_us+2000;
  o=h.r.step(h.s,h.t+.02).attitude;
  assert(o.publish&&!h.r.pending(400).state_verified);
  h.s.land_stamp=wire+f::source_clock_skew_budget_us+3000;
  h.r.step(h.s,h.t+.03);
  // One new sample is no longer enough on the bench exit: retain the
  // post-Disarm stationary dwell before verifying the command transaction.
  assert(!h.r.pending(400).state_verified);
  for(int i=0;i<80&&h.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)h.tick();
  assert(h.r.pending(400).state_verified);
  assert(h.r.attitude_core().phase()==a::Phase::SafeDisarmed);
  std::cout<<"REVIEW_PRECOMMAND_DISARM_STATUS_AND_LAND_CANNOT_RELEASE PASS\n";
}
// 缺戳/启动相对域/过旧/过远未来全部拒绝，最终Arm边界亦独立检查。
static void source_clock_must_be_recent_and_shared(){
  const std::uint64_t host=1700000000000000ULL;
  assert(f::source_stamp_in_clock(host-400000,host,.5));
  for(auto stamp:std::array<std::uint64_t,4>{0,1000000,host-600000,host+200000})
    assert(!f::source_stamp_in_clock(stamp,host,.5));
  Harness h(true);h.ready();assert(h.r.request("ARM_ONLY",{},h.s,h.t).accepted);h.tick();
  h.s.attitude.source_clock_valid=false;h.s.attitude.source_clock_validated=false;
  unsigned commands=0;
  assert(!a::publish_arm_at_boundary(h.r.publication_context(h.s,h.t,true,true),[&]{++commands;}).allowed);
  assert(commands==0);
  std::cout<<"REVIEW_BOOT_CLOCK_STALE_AND_FUTURE_CLOCK_FAIL_CLOSED PASS\n";
}
// sink之后反馈过期不能抹除已提交wire floor，旧ACK仍拒绝。
static void post_commit_expiry_cannot_downgrade_wire_floor(){
  Harness h;h.ready();assert(h.r.request("TAKEOFF",{.6},h.s,h.t).accepted);
  h.t+=.05;h.s.status_stamp=h.s.land_stamp=h.wire()-200000;
  h.r.request("PING",{},h.s,h.t);
  const auto o=h.r.step(h.s,h.t).attitude;assert(o.command==a::Command::Arm);
  auto after=h.s;after.attitude.source_clock_valid=false;
  h.r.confirm_attitude_command_publication(o.command,true,h.wire(),after,h.t);
  assert(h.r.pending(400).published&&h.r.pending(400).source_clock_validated);
  h.r.ack({h.wire()-100000,400,0,1,191,false},h.t+.01);
  assert(!h.r.pending(400).accepted);
  std::cout<<"REVIEW_POST_COMMIT_CLOCK_EXPIRY_RETAINS_WIRE_FLOOR PASS\n";
}
// 允许的正偏差区间内样本仍可能来自命令前，因此不能作为当前事务新证据。
static void tolerated_future_skew_cannot_prove_precommand_event(){
  Harness h;h.ready();assert(h.r.request("TAKEOFF",{.6},h.s,h.t).accepted);h.tick();
  const auto wire=h.r.pending(400).wire_stamp;
  h.r.ack({wire+60000,400,0,1,191,false},h.t+.01);
  assert(!h.r.pending(400).accepted);
  h.s.feedback.armed=true;h.s.status_stamp=wire+50000;
  h.r.step(h.s,h.t+.02);
  assert(!h.r.pending(400).state_verified&&h.r.attitude_core().phase()==a::Phase::RequestArm);
  std::cout<<"REVIEW_POSITIVE_CLOCK_SKEW_CANNOT_PROVE_PRECOMMAND_EVENTS PASS\n";
}
// 精确零推力台架可只记prompt ACK观察，随后新状态独立终态；不得扩散到普通动力Arm。
static void prompt_ack_is_observation_only_in_exact_zero_bench(){
  Harness h(true);h.ready();assert(h.r.request("ARM_ONLY",{},h.s,h.t).accepted);h.tick();
  const auto wire=h.r.pending(400).wire_stamp;
  h.r.ack({wire+1000,400,0,1,191,false},h.t+.001);
  assert(!h.r.pending(400).accepted&&h.r.pending(400).unattributed_clock_acks==1);
  h.s.feedback.armed=true;a::Output o;
  for(int i=0;i<14&&h.r.attitude_core().phase()!=a::Phase::BenchArmedGround;++i){
    o=h.tick();assert(!a::nonzero(o.thrust));
  }
  assert(h.r.attitude_core().phase()==a::Phase::BenchArmedGround);
  assert(h.r.attitude_core().reason()=="BENCH_ARM_NEW_STATE_EVIDENCED_ACK_UNATTRIBUTED");
  assert(h.r.pending(400).state_verified&&!h.r.pending(400).accepted);
  assert(a::attitude_publication_policy(h.r.publication_context(h.s,h.t,true,true),o).allowed);
  h.r.ack({wire+1000000,400,0,1,191,false},h.t+.001);
  assert(!h.r.pending(400).accepted); // source far beyond host receipt budget
  std::cout<<"REVIEW_EARLY_ACK_OBSERVATION_ONLY_NEW_STATE_ZERO_BENCH_ONLY PASS\n";
}
// 命令时间戳必须在真实sink位置读取时钟，不能沿用早先构造消息的旧值。
static void command_stamp_is_captured_at_the_actual_sink(){
  struct Message{std::uint64_t timestamp{7};} command;
  std::uint64_t now=123456,seen=0;
  f::publish_command_with_stamp(command,[&]{return now;},[&](const Message& m){seen=m.timestamp;});
  assert(seen==123456&&command.timestamp==123456);
  std::cout<<"REVIEW_COMMAND_TIMESTAMP_CAPTURED_IN_FINAL_SINK PASS\n";
}
// 全台架往返中prompt ACK始终未归属；每步只靠独立因果新状态，所有推力严格零。
static void exact_zero_bench_round_trip_with_prompt_unattributed_acks(){
  Harness h(true);
  for(int i=0;i<60&&h.r.attitude_core().phase()!=a::Phase::RequestOffboard;++i)h.tick();
  auto wire=h.r.pending(176).wire_stamp;
  h.r.ack({wire+1000,176,0,1,191,false},h.t+.001);
  h.s.feedback.offboard=true;h.s.nav_state=14;
  for(int i=0;i<14&&h.r.attitude_core().phase()!=a::Phase::OffboardReady;++i)h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::OffboardReady&&!h.r.pending(176).accepted);
  assert(h.r.request("ARM_ONLY",{},h.s,h.t).accepted);h.tick();
  wire=h.r.pending(400).wire_stamp;h.r.ack({wire+1000,400,0,1,191,false},h.t+.001);
  h.s.feedback.armed=true;
  for(int i=0;i<14&&h.r.attitude_core().phase()!=a::Phase::BenchArmedGround;++i)h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::BenchArmedGround);
  for(int i=0;i<260&&h.r.attitude_core().phase()!=a::Phase::RequestDisarm;++i){
    const auto o=h.tick();assert(!a::nonzero(o.thrust));
  }
  assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm);
  wire=h.r.pending(400).wire_stamp;h.r.ack({wire+1000,400,0,1,191,false},h.t+.001);
  h.s.feedback.armed=false;
  for(int i=0;i<80&&h.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::SafeDisarmed);
  assert(h.tick().command==a::Command::ReleaseOffboard);
  wire=h.r.pending(176).wire_stamp;h.r.ack({wire+1000,176,0,1,191,false},h.t+.001);
  h.s.feedback.offboard=false;h.s.nav_state=15;
  for(int i=0;i<14&&h.r.attitude_core().phase()!=a::Phase::Idle;++i)h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::Idle);
  assert(h.r.pending(176).state_verified&&!h.r.pending(176).accepted);
  std::cout<<"REVIEW_ZERO_BENCH_FULL_ROUND_TRIP_PROMPT_ACKS_UNATTRIBUTED PASS\n";
}
// 缓存的命令前disarmed状态不得赋予提交后漂移宽容；只有越过因果floor的新状态才可。
static void post_disarm_grace_requires_causal_status(){
  for(bool causal:{false,true}){
    Harness h(true);h.ready();assert(h.r.request("ARM_ONLY",{},h.s,h.t).accepted);h.tick();
    h.s.feedback.armed=true;h.s.at_rest=false;h.s.stationary_seconds=0;
    for(int i=0;i<280&&h.r.attitude_core().phase()!=a::Phase::RequestDisarm;++i)h.tick();
    assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm);
    const auto wire=h.r.pending(400).wire_stamp;
    const auto status=causal?f::source_causal_floor(wire)+1:wire-100000;
    assert(status>h.r.pending(400).status_at_wire);
    const auto anchor=h.r.attitude_core().reference_z();h.s.feedback.armed=false;
    for(int i=1;i<=2;++i){
      h.t+=.05;h.s.status_stamp=status;h.s.attitude.local_stamp+=50000;
      h.s.attitude.z=anchor+.3*i;h.s.attitude.vz=.18;
      assert(h.r.request("PING",{},h.s,h.t).accepted);
      const auto out=h.r.step(h.s,h.t).attitude;
      if(i==1||causal)assert(out.publish&&h.r.attitude_core().phase()==a::Phase::RequestDisarm);
      else assert(!out.publish&&h.r.attitude_core().phase()==a::Phase::Safety&&
        h.r.attitude_core().reason()=="HEIGHT_OR_SPEED_ENVELOPE_EXCEEDED");
      assert(!h.r.pending(400).state_verified);
    }
  }
  std::cout<<"REVIEW4_PREWIRE_DISARMED_CACHE_CANNOT_GRANT_POST_DISARM_ENVELOPE_GRACE PASS\n";
}
int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="ack"||which=="all")precommand_ack_cannot_match();
  if(which=="state"||which=="all")precommand_arm_state_cannot_verify();
  if(which=="disarm"||which=="all")precommand_disarm_evidence_cannot_release();
  if(which=="domain"||which=="all")source_clock_must_be_recent_and_shared();
  if(which=="expiry"||which=="all")post_commit_expiry_cannot_downgrade_wire_floor();
  if(which=="skew"||which=="all")tolerated_future_skew_cannot_prove_precommand_event();
  if(which=="bench"||which=="all")prompt_ack_is_observation_only_in_exact_zero_bench();
  if(which=="stamp"||which=="all")command_stamp_is_captured_at_the_actual_sink();
  if(which=="roundtrip"||which=="all")exact_zero_bench_round_trip_with_prompt_unattributed_acks();
  if(which=="post_disarm"||which=="all")post_disarm_grace_requires_causal_status();
}
