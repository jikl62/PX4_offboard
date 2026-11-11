// 异步日志资源回归：空队列退出、满管道背压与断裂输出端均不能永久阻塞或杀死进程。
// 测试临时替换 stdout 文件描述符，结束后逐一恢复和 close；没有控制器或电机启动。
#include "jikl_fc01_system/async_log.hpp"
#include <cassert>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
int main() {
  // Exercise shutdown while an empty consumer is starting or entering wait.
  // The host runner also bounds process completion, so a missed wake is a failure.
  // 反复构造/析构空消费者，覆盖等待前后通知竞态；测试进程超时会暴露丢唤醒。
  for (int i = 0; i < 100; ++i) {
    fc01::AsyncLog log;
  }
  int pipefd[2];
  assert(pipe(pipefd) == 0);
  const int saved = dup(STDOUT_FILENO);
  assert(saved >= 0);
  assert(fcntl(pipefd[1], F_SETFL, O_NONBLOCK) == 0);
  // 先填满非阻塞管道，再让日志产生大量 flush，验证有界丢弃代替等待慢终端。
  char fill[4096]{};
  while (write(pipefd[1], fill, sizeof(fill)) > 0) {
  }
  assert(dup2(pipefd[1], STDOUT_FILENO) >= 0);
  close(pipefd[1]);
  const auto begin = std::chrono::steady_clock::now();
  {
    fc01::AsyncLog log;
    for (int i = 0; i < 20000; ++i)
      std::cout << "native logging must never own a control deadline " << i << std::endl;
    assert(fc01::native_log_dropped.load() > 0);
  }
  // 包含析构排空在内应在 1.5 s 以内返回，同时 native_log_dropped 必须推进。
  const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  assert(dup2(saved, STDOUT_FILENO) >= 0);
  close(saved);
  close(pipefd[0]);
  assert(elapsed < 1.5);
  // 恢复 SIGPIPE 默认后制造无读端管道，证明 AsyncLog 自身的信号保护可避免进程被管道异常终止。
  std::signal(SIGPIPE, SIG_DFL);
  int broken[2];
  assert(pipe(broken) == 0);
  close(broken[0]);
  const int original = dup(STDOUT_FILENO);
  assert(dup2(broken[1], STDOUT_FILENO) >= 0);
  close(broken[1]);
  {
    fc01::AsyncLog log;
    std::cout << "supervisor disappearance must not kill the controller" << std::endl;
  }
  assert(dup2(original, STDOUT_FILENO) >= 0);
  close(original);
}
