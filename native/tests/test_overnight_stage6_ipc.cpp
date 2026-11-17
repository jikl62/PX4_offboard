// IPC异常合同回归：只使用自有临时Unix socket、纯解析器和时钟对象，不创建ROS或控制器。
// 候选“同一轮状态查询突发放大”另存隔离证据；这里不把已发现问题固化成正确行为。
#include "jikl_fc01_system/source_diagnostics.hpp"
#include "jikl_fc01_system/unix_server.hpp"
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
unsigned checks = 0;
void require(bool value, const char* why) {
  ++checks;
  if (!value)
    throw std::runtime_error(why);
}

// Fixture只拥有本次mkdtemp目录、socket对象与客户端fd；销毁顺序先服务端、最后目录。
struct Fixture {
  std::string directory, path;
  std::unique_ptr<fc01::UnixServer> server;
  int client{-1};
  unsigned dropped{};
  std::vector<std::uint64_t> calls;
  bool throw_handler{};
  Fixture() {
    char pattern[] = "/tmp/fc01_stage6_ipc_XXXXXX";
    const auto folder = mkdtemp(pattern);
    require(folder != nullptr, "temporary directory creation");
    directory = folder;
    path = directory + "/control.sock";
    server = std::make_unique<fc01::UnixServer>(path);
    connect_client();
  }
  ~Fixture() {
    if (client >= 0)
      ::close(client);
    server.reset();
    std::filesystem::remove_all(directory);
  }
  void connect_client() {
    require(client < 0, "only fixture-owned client may be replaced");
    client = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    require(client >= 0, "client socket");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    require(::connect(client, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
            "connect temporary local socket");
  }
  void send(const std::string& bytes) {
    require(::send(client, bytes.data(), bytes.size(), MSG_NOSIGNAL) == ssize_t(bytes.size()),
            "send small fixture request");
  }
  void poll() {
    server->poll(
        [&](const fc01::Request& request) {
          calls.push_back(request.seq);
          if (throw_handler)
            throw std::runtime_error("synthetic ordinary handler failure");
          return "reply" + std::to_string(request.seq);
        },
        [&] {
          require(!server->connected(), "disconnect callback sees already-closed connection");
          ++dropped;
        });
  }
  std::string drain() {
    std::string result;
    char bytes[4096];
    for (;;) {
      const auto count = ::recv(client, bytes, sizeof(bytes), 0);
      if (count > 0)
        result.append(bytes, std::size_t(count));
      else {
        require(count == 0 || errno == EAGAIN || errno == EWOULDBLOCK,
                "only clean EOF or nonblocking empty receive");
        return result;
      }
    }
  }
};

void split_and_coalesced_lines() {
  Fixture f;
  f.send("1 STA");
  f.poll();
  require(f.calls.empty() && f.server->connected(), "partial line is not a command");
  f.send("TUS\n2 STATUS\n3 STA");
  f.poll();
  require(f.calls == std::vector<std::uint64_t>{1, 2}, "coalesced commands retain order");
  f.send("TUS\n");
  f.poll();
  require(f.calls == std::vector<std::uint64_t>{1, 2, 3}, "partial tail completes exactly once");
  f.poll();
  require(f.drain() == "reply1\nreply2\nreply3\n", "replies preserve framing and order");
  require(f.dropped == 0, "ordinary split/coalesced traffic does not drop");
}

void malformed_and_replayed_lines() {
  // 最大整数边界、浮点非有限/溢出、残行和内嵌零分别进入实际服务器解析拒绝路径。
  const std::vector<std::string> bad{"0 STATUS\n",
                                     "18446744073709551616 STATUS\n",
                                     "-1 STATUS\n",
                                     "1 VEL nan 0 0 0\n",
                                     "1 VEL inf 0 0 0\n",
                                     "1 VEL 1e309 0 0 0\n",
                                     "1 VEL 1 2 3 4 5\n",
                                     "1 VEL 1.2junk\n",
                                     std::string("1 VEL 2\0x\n", 10),
                                     std::string(1025, 'x')};
  for (const auto& value : bad) {
    Fixture f;
    f.send(value);
    f.poll();
    require(!f.server->connected() && f.calls.empty() && f.dropped == 1,
            "malformed packet closes before handler");
  }
  Fixture f;
  f.send("1 STATUS\n2 STATUS\n2 STATUS\n3 STATUS\n");
  f.poll();
  require(f.calls == std::vector<std::uint64_t>{1, 2}, "replay stops following queued commands");
  require(!f.server->connected() && f.dropped == 1, "replay drops exactly once");
  f.poll();
  require(f.dropped == 1, "empty later poll does not repeat disconnect callback");
}

void failed_handler_and_reconnect() {
  Fixture f;
  f.throw_handler = true;
  f.send("1 STATUS\n2 STATUS\n");
  f.poll();
  require(f.calls == std::vector<std::uint64_t>{1} && f.dropped == 1,
          "handler error abandons remaining same-session input");
  require(f.drain().empty(), "failed handler emits no successful response");
  ::close(f.client);
  f.client = -1;
  f.throw_handler = false;
  f.connect_client();
  f.send("1 STATUS\n");
  f.poll();
  f.poll();
  require(f.calls == std::vector<std::uint64_t>{1, 1}, "new connection has independent sequence");
  require(f.drain() == "reply1\n" && f.server->connected(), "old buffered commands/replies absent");
  ::close(f.client);
  f.client = -1;
  f.poll();
  require(f.dropped == 2 && !f.server->connected(), "peer EOF invokes existing disconnect policy");
}

void sampled_clock_adversaries() {
  fc01::SourceSampleProgress p;
  p.observe(1'000'000, 1'000'000, 1.);
  require(p.fresh(1'000'000, 1., .5), "initial sampled clock");
  p.observe(1'400'000, 1'000'000, 1.4);
  require(p.received == 1., "new envelope does not renew frozen raw sample");
  require(!p.fresh(1'500'000, 1.5, .5), "exact receipt TTL is expired");
  p.observe(1'600'000, 1'700'000, 1.6);
  require(!p.fresh(1'600'000, 1.6, .5), "raw beyond envelope invalidates latest payload");
  p.observe(1'700'000, 1'700'000, 1.7);
  require(p.fresh(1'700'000, 1.7, .5), "new valid raw can recover malformed relation");
  p.observe(1'800'000, 1'600'000, 1.8);
  p.observe(1'900'000, 1'900'000, 1.9);
  require(p.regressed && !p.fresh(1'900'000, 1.9, 100), "raw regression survives healthy tail");
  for (double value :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -1., 0.})
    require(!fc01::source_stamp_in_clock(1, 1, value), "nonpositive/nonfinite source TTL rejected");
  const auto max = std::numeric_limits<std::uint64_t>::max();
  require(fc01::source_causal_floor(max - 100) == max, "causal floor saturates before uint64 wrap");
  require(!fc01::source_stamp_in_clock(max, 1, .5), "large future stamp cannot wrap into fresh");
}
} // namespace

int main() {
  split_and_coalesced_lines();
  malformed_and_replayed_lines();
  failed_handler_and_reconnect();
  sampled_clock_adversaries();
  std::cout << "STAGE6_IPC_CLOCK_CONTRACT_PASS checks=" << checks << '\n';
}
