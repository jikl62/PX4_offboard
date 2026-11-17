// Stage3：三个发布观察点的组合与异常注入；执行真实边界，所有 sink 仅在内存计数。
// 完成只表示两个本地调用均返回；目标调用过但抛错，仍不能断言飞控没有收到目标。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main original_publication_contract_main
#include "test_publication_boundary.cpp"
#undef main
#pragma GCC diagnostic pop
#include <new>

// 每个编号破坏一种真实权限/时效证据；0 为合法动力上下文。参数为本次局部副本。
static a::BoundaryContext observation(unsigned fault) {
  auto c = context(true);
  c.phase = a::Phase::HeightHold;
  c.feedback.armed = true;
  switch (fault) {
  case 1:
    c.control_fresh = false;
    break;
  case 2:
    c.feedback.offboard = false;
    break;
  case 3:
    c.feedback.kill = true;
    break;
  case 4:
    c.feedback.failsafe = true;
    break;
  case 5:
    c.feedback.status_fresh = false;
    break;
  case 6:
    c.feedback.exclusive = false;
    break;
  case 7:
    c.authority.takeover_latched = true;
    break;
  default:
    break;
  }
  return c;
}

int main() {
  a::Output output;
  output.publish = true;
  output.thrust = {0, 0, -.4};
  unsigned cases = 0;
  // 8^3 种观察组合：早先拒绝不得被后面的好观察覆盖；第三次拒绝属于部分提交。
  for (unsigned first = 0; first < 8; ++first)
    for (unsigned second = 0; second < 8; ++second)
      for (unsigned third = 0; third < 8; ++third) {
        const std::array<unsigned, 3> faults{first, second, third};
        unsigned samples = 0, targets = 0, heartbeats = 0;
        bool threw = false;
        a::PublicationAttempt evidence;
        a::Decision result;
        try {
          result = a::publish_attitude_resampled<Mode, Target>(
              [&] {
                return observation(faults.at(samples++));
              },
              output, 123,
              [&](const auto&) {
                ++targets;
              },
              [&](const auto&) {
                ++heartbeats;
              },
              &evidence);
        } catch (const std::runtime_error& e) {
          threw = true;
          assert(std::string(e.what()).find("HEARTBEAT_FINAL_CONTEXT_CHANGED_") == 0);
        }
        const bool before_target = first || second;
        const bool partial = !before_target && third;
        assert(targets == unsigned(!before_target));
        assert(heartbeats == unsigned(!before_target && !third));
        assert(evidence.target_committed == bool(targets));
        assert(evidence.heartbeat_committed == bool(heartbeats));
        assert(threw == partial && !evidence.sink_exception);
        assert((evidence.stage == a::PublicationStage::Complete) == bool(heartbeats));
        if (!threw)
          assert(result.allowed == bool(heartbeats));
        std::cout << "PAIR " << first << second << third << ':' << samples << ':' << targets
                  << heartbeats << ':' << a::name(evidence.stage) << ':'
                  << a::name(evidence.decision.reason) << ':' << threw << '\n';
        ++cases;
      }
  // Context、目标和心跳分别抛运行错误/分配错误；不得把“调用已发生”写成“配对成功”。
  for (unsigned point = 0; point < 5; ++point)
    for (bool allocation : {false, true}) {
      unsigned samples = 0, targets = 0, heartbeats = 0;
      bool caught = false;
      a::PublicationAttempt evidence;
      const auto fail = [&] {
        if (allocation)
          throw std::bad_alloc();
        throw std::runtime_error("injected resource failure");
      };
      try {
        a::publish_attitude_resampled<Mode, Target>(
            [&] {
              const auto index = samples++;
              if (point < 3 && index == point)
                fail();
              return observation(0);
            },
            output, 123,
            [&](const auto&) {
              ++targets;
              if (point == 3)
                fail();
            },
            [&](const auto&) {
              ++heartbeats;
              if (point == 4)
                fail();
            },
            &evidence);
      } catch (const std::bad_alloc&) {
        caught = allocation;
      } catch (const std::runtime_error&) {
        caught = !allocation;
      }
      assert(caught && evidence.stage != a::PublicationStage::Complete);
      assert(targets == unsigned(point >= 2));
      assert(heartbeats == unsigned(point == 4));
      assert(evidence.target_committed == (point == 2 || point == 4));
      assert(!evidence.heartbeat_committed);
      assert(evidence.sink_exception == (point >= 3));
      std::cout << "THROW " << point << allocation << ':' << samples << ':' << targets << heartbeats
                << ':' << evidence.target_committed << evidence.heartbeat_committed
                << evidence.sink_exception << ':' << a::name(evidence.stage) << '\n';
      ++cases;
    }
  assert(cases == 522);
  std::cout << "Stage3 publication combinations: " << cases << " PASS\n";
}
