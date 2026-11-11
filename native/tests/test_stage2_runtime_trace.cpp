// 第二阶段确定性事务/IPC记录：同一程序分别链接5198d33和当前头文件，输出逐字节对照。
// 复用原测试的配置/反馈工厂，不复制控制算法；所有socket均在本进程唯一临时目录，无设备操作。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main original_runtime_regression_main
#include "test_attitude_runtime.cpp"
#undef main
#pragma GCC diagnostic pop
#include "jikl_fc01_system/unix_server.hpp"
#include <filesystem>
#include <iomanip>
#include <limits>

// 输出事务所有关键归属/水位/终态事实；只读r，浮点用hexfloat，不用容差隐藏控制差异。
static void trace_pending(const f::Runtime& r, unsigned command) {
  const auto& p = r.pending(command);
  std::cout << command << ':' << p.sent << p.accepted << p.rejected << p.timed_out << p.requested
            << p.published << p.sink_denied << ':' << p.result << ':' << p.sent_at << ':'
            << p.baseline << ':' << p.last_ack << ':' << p.wire_stamp << ':' << p.status_at_wire
            << ':' << p.land_at_wire << ':' << p.armed_time_at_wire << ':' << p.rejected_at << ':'
            << p.ack_received_at << ':' << int(p.command400_kind) << ':' << int(p.stage) << ':'
            << p.ack_stamp << ':' << p.verified_status_stamp << ':' << p.state_verified << ':'
            << p.verified_at << ':' << p.generation << ':' << p.ack_observations << ':'
            << p.ack_ambiguous << p.previous400_unresolved << p.source_clock_validated << ':'
            << p.causal_floor << ':' << p.unattributed_clock_acks << ':' << p.ack_candidate
            << p.ack_confirmed_by_state << p.exclusive_at_wire << ':' << p.ack_disposition << '|';
}

// 记录一次实际step及其公开状态；输入使用同域源戳和主机steady秒，函数不额外续租或确认发布。
static void trace_step(f::Runtime& r, f::Snapshot& s, double now) {
  const auto output = tick(r, s, now);
  std::cout << "STEP " << now << ' ' << r.phase_name() << ' ' << r.message() << ' '
            << output.attitude.publish << ':' << int(output.attitude.command) << ':';
  for (double value : output.attitude.thrust)
    std::cout << value << ',';
  for (double value : output.attitude.q)
    std::cout << value << ',';
  std::cout << output.attitude.target_height << ':' << output.attitude.velocity_target << ':'
            << output.attitude.saturated << output.attitude.slew_limited << ':'
            << int(output.flight.command) << ':' << output.send_ground << '|';
  for (unsigned command : {176, 400, 21})
    trace_pending(r, command);
  std::cout << '\n';
}

// 在相同冷启动状态遍历请求、参数错误与profile优先级；每次重建Runtime，避免案例互相污染。
static void request_matrix() {
  const std::vector<std::string> operations = {"STATUS",
                                               "AUTHORITY_STATUS",
                                               "INTERACTIVE",
                                               "PING",
                                               "AUTHORITY_RC",
                                               "AUTHORITY_AUTONOMOUS",
                                               "AUTHORITY_RESET",
                                               "AUTHORITY_TAKEOVER",
                                               "ATTITUDE_READY",
                                               "ARM_ONLY",
                                               "TAKEOFF",
                                               "LAND",
                                               "STOP",
                                               "MODE",
                                               "UPDATE_CHECK",
                                               "HOLD",
                                               "VEL",
                                               "UNKNOWN"};
  for (int profile = 0; profile < 4; ++profile)
    for (const auto& op : operations)
      for (int argument = 0; argument < 4; ++argument) {
        auto c = cfg();
        if (profile == 1)
          c.profile = "ground";
        if (profile == 2)
          c.profile = "sim";
        if (profile == 3) {
          c = cfg(true);
          c.attitude.rc_navigation_enabled = true;
          c.attitude.arm_enabled = false;
          c.attitude.rc_takeover_confirmed = true;
          c.attitude.autonomous_failsafe_confirmed = false;
          c.attitude.max_flight_seconds = 30;
          c.attitude.max_tilt_deg = 10;
        }
        f::Runtime r(c);
        auto s = sample();
        std::vector<double> values;
        if (argument == 1)
          values = {1};
        if (argument == 2)
          values = {0, 0, 0, 0};
        if (argument == 3)
          values = {std::numeric_limits<double>::quiet_NaN()};
        const auto reply = r.request(op, values, s, 0);
        std::cout << "REQUEST " << profile << ':' << op << ':' << argument << ':' << reply.accepted
                  << ':' << reply.message << ':' << r.message() << ':' << r.phase_name() << '\n';
        trace_step(r, s, .05);
      }
}

// ACK覆盖正常/错误目标、重复与迟到、进度/未知结果和同步时钟上下界；不以ACK替代物理模式。
static void ack_matrix() {
  for (bool synchronized : {false, true})
    for (int variant = 0; variant < 12; ++variant)
      for (int result : {0, 1, 2, 3, 4, 5, 6, 99}) {
        f::Runtime r(cfg());
        auto s = sample();
        prepare(r, s);
        if (synchronized) {
          s.attitude.source_clock_validated = true;
          r.confirm_attitude_command_publication(a::Command::Offboard, true,
                                                 s.status_stamp + 200000, s, 1.25);
        }
        const auto pending = r.pending(176);
        auto stamp = synchronized ? pending.wire_stamp + 1 : pending.baseline + 1;
        double now = 1.3;
        if (variant == 1)
          stamp = pending.baseline;
        if (variant == 2)
          stamp = pending.status_at_wire;
        if (variant == 3)
          stamp = pending.wire_stamp;
        if (variant == 4)
          stamp += 10000000;
        if (variant == 5)
          now = 1.2;
        if (variant == 6)
          now = 6.3;
        if (variant == 7)
          now = std::numeric_limits<double>::quiet_NaN();
        const fc01::g::Ack ack{stamp, 176, result, variant == 8 ? 2 : 1, variant == 9 ? 1 : 191,
                               false};
        r.ack(ack, now);
        if (variant == 10)
          r.ack(ack, now + .01);
        if (variant == 11)
          r.ack({stamp + 1, 176, 0, 1, 191, false}, now + .01);
        std::cout << "ACK " << synchronized << ':' << variant << ':' << result << '|';
        trace_pending(r, 176);
        std::cout << '\n';
        s.feedback.offboard = true;
        s.nav_state = 14;
        trace_step(r, s, 1.35);
      }
}

// 真正的本机IPC按轮推进；记录分帧/序号/回调异常/背压的处理顺序，退出只清测试自有目录。
static void socket_trace() {
  char pattern[] = "/tmp/fc01_stage2_ipc_XXXXXX";
  const char* directory = ::mkdtemp(pattern);
  assert(directory);
  const std::string path = std::string(directory) + "/control.sock";
  {
    f::UnixServer server(path);
    const auto connect_client = [&] {
      const int client = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      assert(client >= 0);
      sockaddr_un address{};
      address.sun_family = AF_UNIX;
      std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);
      assert(::connect(client, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
      return client;
    };
    const auto poll = [&] {
      server.poll(
          [&](const f::Request& request) {
            std::cout << "IPC_HANDLE " << request.seq << ':' << request.op << '\n';
            if (request.op == "FAIL")
              throw std::runtime_error("fixture handler failure");
            return std::to_string(request.seq);
          },
          [&] {
            std::cout << "IPC_DROP " << server.connected() << '\n';
          });
    };
    int client = connect_client();
    poll();
    for (const std::string part : {"1 PING\n2 STA", "TUS\n", "2 PING\n"}) {
      assert(::send(client, part.data(), part.size(), MSG_NOSIGNAL) == ssize_t(part.size()));
      poll();
    }
    assert(!server.connected());
    ::close(client);
    client = connect_client();
    poll();
    const std::string failure = "1 FAIL\n";
    assert(::send(client, failure.data(), failure.size(), MSG_NOSIGNAL) > 0);
    poll();
    assert(!server.connected());
    ::close(client);
    client = connect_client();
    poll();
    server.publish(std::string(65537, 'x'));
    poll();
    assert(!server.connected());
    ::close(client);
  }
  assert(!std::filesystem::exists(path));
  std::filesystem::remove_all(directory);
}

// 入口只执行主机确定性矩阵和临时IPC，不连接ROS；输出本身供差分器逐字节校验。
int main() {
  std::cout << std::hexfloat;
  request_matrix();
  ack_matrix();
  socket_trace();
  return 0;
}
