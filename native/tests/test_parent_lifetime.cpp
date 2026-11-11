// 父进程寿命真实进程回归：监督者退出只触发SIGTERM有序关闭请求，保留子进程记录机会。
// 事件/ready管道按R就绪、S收到停止传递证据；poll最多4 s防测试本身永久等待，结束逐一close。
#include "jikl_fc01_system/parent_lifetime.hpp"
#include <cassert>
#include <csignal>
#include <poll.h>
#include <sys/wait.h>
#include <time.h>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {
  stop_requested = 1;
}
// 等待一个字节并断言poll/read结果；返回字节用于区分已建立绑定与已收到内核停止信号。
char receive(int fd) {
  pollfd item{fd, POLLIN, 0};
  assert(poll(&item, 1, 4000) == 1);
  char value{};
  assert(read(fd, &value, 1) == 1);
  return value;
}
} // namespace
int main() {
  // A real parent exit requests graceful shutdown through the installed
  // handler; the child remains alive to record that request, not SIGKILLed.
  int events[2];
  assert(pipe(events) == 0);
  const auto parent = fork();
  assert(parent >= 0);
  if (parent == 0) {
    close(events[0]);
    int ready[2];
    if (pipe(ready))
      _exit(10);
    const auto worker = fork();
    if (worker < 0)
      _exit(11);
    // 孙进程先安装最小sig_atomic_t处理器再绑定父寿命；就绪后父进程才退出，排除未建立绑定竞态。
    if (worker == 0) {
      close(ready[0]);
      std::signal(SIGTERM, request_stop);
      if (!fc01::arm_parent_loss_signal(getppid(), SIGTERM))
        _exit(12);
      if (write(events[1], "R", 1) != 1 || write(ready[1], "R", 1) != 1)
        _exit(13);
      for (int n = 0; n < 300 && !stop_requested; ++n) {
        timespec delay{0, 10000000};
        nanosleep(&delay, nullptr);
      }
      const char result = stop_requested ? 'S' : 'X';
      if (write(events[1], &result, 1) != 1)
        _exit(14);
      _exit(stop_requested ? 0 : 15);
    }
    close(ready[1]);
    const auto result = receive(ready[0]);
    _exit(result == 'R' ? 0 : 16);
  }
  close(events[1]);
  assert(receive(events[0]) == 'R');
  assert(receive(events[0]) == 'S');
  int result{};
  assert(waitpid(parent, &result, 0) == parent);
  assert(WIFEXITED(result) && WEXITSTATUS(result) == 0);
  close(events[0]);
  // Covers the setup race: an already changed parent is not accepted as
  // the captured supervisor. The caller must latch shutdown before ticking.
  // 另验证captured parent已变化的设置竞态，调用者必须在后续tick前锁存shutdown。
  assert(!fc01::arm_parent_loss_signal(0, SIGTERM));
}
