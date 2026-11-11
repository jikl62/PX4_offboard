// 命令400审计缓存容量长期回归：320次真实人工RC任务取消与恢复跨越128条淘汰边界。
// 最近审计尾部可淘汰，但wire generation、因果水位、当前待决事务及ACK复用歧义必须终身保留。
// 每轮重新地面参考/预发送、人工Arm与新Offboard；命令sink合成，零真实飞控连接。
// 复用真实Runtime、RC状态机与最终零推力边界；所有发布者都是本进程内的测试替身。
#define main existing_rc_navigation_contract_main
#include "test_rc_navigation.cpp"
#undef main

static void begin_next_manual_rc_task(Flight& flight, unsigned mission) {
  flight.warm(); // 新任务必须重新完成地面参考及零推力预发送，不能继承上一轮READY。
  need(flight.r.attitude_core().phase() == a::Phase::RcWaitArm,
       "every task independently reaches RC readiness");

  flight.s.attitude.rc_arm_switch = 1; // 模拟新的人工CH5授权边沿，不调用自动Arm接口。
  ++flight.s.attitude.operator_switch_generation;
  flight.tick();
  flight.s.feedback.armed = true; // 独立的新PX4状态才能证明人工解锁已发生。
  flight.s.armed_time = mission * 10;
  flight.s.latest_arming_reason = 2;
  flight.s.at_rest = false;
  flight.tick();
  flight.navigation(); // 新CH6边沿及随后新Offboard状态共同完成任务接入。
  need(flight.r.attitude_core().phase() == a::Phase::ArmedGround,
       "every task requires a new manual arm and new Offboard observation");
}

static void complete_ground_cancel(Flight& flight, unsigned mission) {
  flight.s.at_rest = true; // 飞机仍在地面；这里只验证取消任务后的受保护上锁路径。
  need(flight.r.request("LAND", {}, flight.s, flight.t).accepted,
       "ground cancellation remains available for each RC task");
  bool disarm_committed = false;
  for (unsigned tick = 0; tick < 120 && !disarm_committed; ++tick) {
    flight.tick(); // 真实Core必须先取得新的地面证据，测试不直接制造Disarm事务。
    if (flight.o.attitude.command == a::Command::Disarm) {
      need(
          a::disarm_publication_policy(flight.r.publication_context(flight.s, flight.t, true, true))
              .allowed,
          "fresh ground evidence still guards the actual Disarm sink");
      flight.r.confirm_attitude_command_publication(a::Command::Disarm, true,
                                                    flight.s.status_stamp + 1, flight.s, flight.t);
      disarm_committed = true; // 仅记录本地测试sink已提交，不冒充PX4执行成功。
    }
  }
  need(disarm_committed, "every task reaches one real Disarm transaction");
  need(flight.r.pending(400).generation == mission,
       "global wire generation never resets when audit entries are evicted");

  if (mission > 1) {
    for (unsigned tick = 0; tick < 3; ++tick)
      flight.tick(); // 新状态先跨过原有100ms因果下界。
    const auto wire = flight.r.pending(400).wire_stamp;
    const auto floor = flight.r.pending(400).causal_floor;
    for (int result = 0; result <= 6; ++result) {
      // 400消息没有Arm/Disarm事务nonce；即使时间新鲜，各类重用ACK也只能作观察。
      flight.r.ack({flight.s.status_stamp + result + 1, 400, result, 1, 191, false},
                   flight.t + .001);
      const auto& current = flight.r.pending(400);
      need(current.ack_ambiguous && !current.accepted && !current.rejected &&
               !current.state_verified && current.generation == mission &&
               current.wire_stamp == wire && current.causal_floor == floor,
           "all reused ACK results remain unattributed after audit-history eviction");
    }
  }

  flight.s.feedback.armed = false; // 仍需新的PX4上锁及地面状态，而不是上面的ACK。
  for (unsigned tick = 0; tick < 10 && flight.r.attitude_core().phase() != a::Phase::SafeDisarmed;
       ++tick) {
    flight.tick();
  }
  need(flight.r.attitude_core().phase() == a::Phase::SafeDisarmed &&
           flight.r.pending(400).state_verified,
       "independent fresh disarmed ground evidence completes every task");

  flight.s.feedback.offboard = false; // 人工复位控制开关，并让新状态证明已退出Offboard。
  flight.s.nav_state = 15;
  flight.s.attitude.rc_mode_slot = flight.c.attitude.rc_navigation_mode_slot == 1
                                       ? 6
                                       : 1; // 复位到当前导航档以外，兼容部署配置的CH6档位。
  flight.s.attitude.rc_arm_switch = 3;
  flight.s.at_rest = true;
  flight.s.stationary_seconds = 3;
  flight.tick();
  need(flight.r.reprepare_rc(flight.s, flight.t),
       "safe completed tasks reprepare without reboot even after history reaches its limit");
}

int main() {
  constexpr unsigned missions = 320; // 超过128条容量的两倍，实际经过多次淘汰边界。
  constexpr std::size_t retained_limit = 128;
  Flight flight{
      production_no_vision_rc_config()}; // 使用1.5m、0.50/0.04/0.80和垂向监测均保留的部署候选配置。
  std::vector<f::Pending> completed;
  for (unsigned mission = 1; mission <= missions; ++mission) {
    begin_next_manual_rc_task(flight, mission);
    complete_ground_cancel(flight, mission);
    const auto& history = flight.r.command400_history();
    need(history.size() <= retained_limit,
         "Command400 audit history must retain at most 128 transactions");
    need(history.size() == std::min(completed.size(), retained_limit),
         "audit cache retains the complete recent tail without dropping current pending state");

    const auto first_expected = completed.size() - history.size();
    for (std::size_t index = 0; index < history.size(); ++index) {
      const auto& expected = completed[first_expected + index];
      const auto& actual = history[index];
      // 缓存淘汰只能改变保留长度；最近事务的顺序和重放保护证据不得变化。
      need(actual.generation == expected.generation && actual.wire_stamp == expected.wire_stamp &&
               actual.baseline == expected.baseline &&
               actual.status_at_wire == expected.status_at_wire &&
               actual.land_at_wire == expected.land_at_wire &&
               actual.causal_floor == expected.causal_floor &&
               actual.ack_ambiguous == expected.ack_ambiguous &&
               actual.command400_kind == expected.command400_kind &&
               actual.state_verified == expected.state_verified,
           "retained audit records preserve transaction order, identity and causal watermarks");
    }
    completed.push_back(flight.r.pending(400)); // 测试侧保存完整期望记录，用于独立核对保留尾部。
    need(flight.r.rc_attempts_completed() == mission && flight.arm_commands == 0 &&
             flight.mode_commands == 0,
         "bounded diagnostics do not reset task totals or emit automatic Arm/Offboard");
  }
  std::cout << "RUNTIME_HISTORY_BOUNDED_RC_RECOVERY_PASS missions=" << missions
            << " history=" << flight.r.command400_history().size()
            << " generation=" << flight.r.pending(400).generation
            << " automatic_arm=" << flight.arm_commands
            << " automatic_offboard=" << flight.mode_commands << '\n';
}
