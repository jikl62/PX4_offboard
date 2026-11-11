// Stage3 有限状态组合：真实 Runtime/Core/RC 与离散 FIFO；不运行 ROS 或输出到硬件。
// 这些断言覆盖指定组合，不能据此声称已证明所有状态空间安全。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main original_rc_contract_main
#include "test_rc_navigation.cpp"
#undef main
#pragma GCC diagnostic pop
#include "jikl_fc01_system/discrete_event_latch.hpp"

// 合成步保持每个独立源推进；检查计划和最终策略，再模拟获准配对提交。
// 此驱动不调用实际 sink；配对调用与异常完整性由独立 publication_combinations 驱动验证。
static f::Output step_snapshot(Flight& flight) {
  flight.t += .05;
  flight.s.status_stamp += 50000;
  flight.s.land_stamp += 50000;
  flight.s.attitude.local_stamp += 50000;
  flight.s.attitude.vertical_sample_stamp += 50000;
  flight.s.attitude.rc_switch_sample_stamp += 50000;
  flight.s.attitude.source_host_stamp = flight.s.status_stamp;
  auto output = flight.r.step(flight.s, flight.t);
  assert(output.attitude.command != a::Command::Arm);
  assert(output.attitude.command != a::Command::Offboard);
  if (output.attitude.publish) {
    const auto decision = a::attitude_publication_policy(
        flight.r.publication_context(flight.s, flight.t, true, true), output.attitude);
    if (decision.allowed)
      flight.r.confirm_rc_output(true, flight.t);
  }
  return output;
}

int main() {
  unsigned no_intent = 0, hazards = 0;
  // 启动前已有各种状态/开关组合，但没有本轮配对的人工边沿；不能把偶然同时为真当授权。
  for (unsigned mask = 0; mask < 256; ++mask) {
    Flight q(production_no_vision_rc_config());
    q.s.feedback.armed = mask & 1;
    q.s.feedback.offboard = mask & 2;
    q.s.nav_state = q.s.feedback.offboard ? 14 : 15;
    q.s.feedback.landed = !(mask & 4);
    q.s.land_fresh = !(mask & 8);
    q.s.at_rest = !(mask & 16);
    q.s.attitude.rc_arm_switch = mask & 32 ? 1 : 3;
    if (mask & 64)
      q.s.attitude.rc_mode_slot = q.c.attitude.rc_navigation_mode_slot;
    q.s.attitude.kill = mask & 128;
    for (unsigned tick = 0; tick < 80; ++tick) {
      const auto output = step_snapshot(q);
      assert(!a::nonzero(output.attitude.thrust));
      assert(!q.r.attitude_core().flight_started());
    }
    std::cout << "NO_INTENT " << mask << ':' << q.r.phase_name() << ':' << q.r.message() << '\n';
    ++no_intent;
  }
  // 两个危险事件之间插健康事件，随后只有普通好反馈；锁存不能被尾帧覆盖或自动恢复飞行。
  for (unsigned first = 0; first < 5; ++first)
    for (unsigned second = 0; second < 5; ++second) {
      Flight q(production_no_vision_rc_config());
      q.join();
      for (unsigned i = 0; i < 50 && q.r.attitude_core().phase() != a::Phase::ThrustRamp; ++i)
        q.tick();
      assert(q.r.attitude_core().phase() == a::Phase::ThrustRamp);
      const auto original = q.r.publication_context(q.s, q.t, true, true).feedback;
      const auto observe = [&](unsigned kind) {
        auto bad = original;
        auto source = a::Core::SafetyEvent::Status;
        if (kind == 0)
          bad.kill = true;
        if (kind == 1)
          bad.failsafe = true;
        if (kind == 2)
          bad.offboard = false;
        if (kind == 3) {
          bad.q = {0, 0, 0, 0};
          source = a::Core::SafetyEvent::Attitude;
        }
        if (kind == 4) {
          bad.z_valid = false;
          source = a::Core::SafetyEvent::Vertical;
        }
        q.r.observe_attitude_safety_event(bad, source, q.t);
      };
      observe(first);
      q.r.observe_attitude_safety_event(original, a::Core::SafetyEvent::Status, q.t);
      observe(second);
      for (unsigned tick = 0; tick < 12; ++tick) {
        const auto output = step_snapshot(q);
        assert(q.r.attitude_core().phase() == a::Phase::Safety);
        assert(!output.attitude.publish && !a::nonzero(output.attitude.thrust));
        assert(output.attitude.command == a::Command::None);
      }
      std::cout << "HAZARDS " << first << second << ':' << q.r.message() << '\n';
      ++hazards;
    }
  // 2^10 突发到达序列，逐项内容与顺序不丢；容量溢出为粘性，ack 不能清除。
  for (unsigned mask = 0; mask < 1024; ++mask) {
    f::DiscreteEventLatch<unsigned, 8> latch;
    std::vector<unsigned> expected;
    for (unsigned i = 0; i < 10; ++i) {
      const unsigned event = (mask >> i) & 1;
      const bool accepted = latch.push(event);
      assert(accepted == (i < 8));
      if (accepted)
        expected.push_back(event);
    }
    std::vector<unsigned> actual;
    latch.for_each([&](unsigned event) {
      actual.push_back(event);
    });
    assert(actual == expected && latch.overflowed());
    latch.acknowledge_all();
    assert(latch.empty() && latch.overflowed());
  }
  assert(no_intent == 256 && hazards == 25);
  std::cout << "Stage3 state invariants: 256 startup + 25 ordered hazards + 1024 FIFO PASS\n";
}
