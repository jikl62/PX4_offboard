// 日志资源故障回归：在测试链接层注入系统调用、线程创建和分配失败，不给生产代码加入测试钩子。
// 所有输出写临时 /dev/null；不启动控制节点、DDS 或硬件。等待上限只是本机测试保护，不是实时保证。
#include "jikl_fc01_system/async_log.hpp"
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <new>
#include <pthread.h>
#include <system_error>

namespace {
// 只有生产线程被当前测试主动标记后才使下一次分配失败，消费者的正常分配不受影响。
thread_local bool fail_allocation = false;
std::atomic<bool> allocation_fired{false};
std::atomic<bool> fail_thread{false};
std::atomic<int> fail_fcntl{-1}, write_error{0};
std::atomic<bool> hold_sink{false}, sink_entered{false};

void pipe_handler(int) {
}

// 有界等待确定性的工作线程观察点；没有用睡眠时长猜测队列是否已经出队。
template <typename Predicate> void await(Predicate predicate) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!predicate() && std::chrono::steady_clock::now() < end)
    std::this_thread::yield();
  assert(predicate());
}

// startup 失败须保持原 streambuf、stdout flags 和 SIGPIPE 处理器；异常仍向调用方传播。
void startup_failure(int command, bool thread) {
  auto* const cout = std::cout.rdbuf();
  auto* const cerr = std::cerr.rdbuf();
  struct sigaction original {
  }, wanted{}, after{};
  wanted.sa_handler = pipe_handler;
  sigemptyset(&wanted.sa_mask);
  assert(sigaction(SIGPIPE, &wanted, &original) == 0);
  const int flags = fcntl(STDOUT_FILENO, F_GETFL);
  assert(flags >= 0);
  fail_fcntl.store(command);
  fail_thread.store(thread);
  bool caught = false;
  try {
    fc01::AsyncLog log;
  } catch (const std::system_error& error) {
    assert(thread && error.code().value() == EAGAIN);
    caught = true;
  } catch (const std::runtime_error& error) {
    assert(!thread);
    assert(std::string(error.what()) == "cannot configure nonblocking native log sink");
    caught = true;
  }
  assert(caught);
  assert(std::cout.rdbuf() == cout && std::cerr.rdbuf() == cerr);
  assert(fcntl(STDOUT_FILENO, F_GETFL) == flags);
  assert(sigaction(SIGPIPE, nullptr, &after) == 0);
  assert(after.sa_handler == pipe_handler);
  assert(sigaction(SIGPIPE, &original, nullptr) == 0);
}

// 字符串扩容失败必须算一次整段丢弃，且 ostream 不进入永久 badbit，下一条仍可提交。
void pending_allocation_failure() {
  fc01::AsyncLog log;
  const std::string record(40000, 'p');
  const auto before = fc01::native_log_dropped.load();
  allocation_fired.store(false);
  fail_allocation = true;
  std::cout.write(record.data(), record.size());
  fail_allocation = false;
  assert(allocation_fired.load());
  assert(std::cout.good());
  assert(fc01::native_log_dropped.load() == before + 1);
  std::cout << "producer recovered after allocation failure" << std::endl;
  assert(std::cout.good());
}

// 堵住已出队记录的实际 write，让生产者独立填队；队列持锁期间没有测试等待。
void begin_held_record(const std::string& record) {
  hold_sink.store(true);
  sink_entered.store(false);
  std::cout.write(record.data(), record.size()).flush();
  await([] {
    return sink_entered.load();
  });
}

// 只在 flush 期间使下一次 new 失败，等待 deque 自身申请下一块；返回已成功排队记录数。
std::size_t fail_one_queue_allocation(const std::string& record) {
  std::size_t accepted = 0;
  allocation_fired.store(false);
  for (int attempts = 0; attempts < 128 && !allocation_fired.load(); ++attempts) {
    std::cout.write(record.data(), record.size());
    fail_allocation = true;
    std::cout.flush();
    fail_allocation = false;
    if (!allocation_fired.load())
      ++accepted;
  }
  assert(allocation_fired.load());
  assert(std::cout.good());
  return accepted;
}

// 失败队列插入不占用幽灵字节：之后仍能精确容纳 256 条 1 KiB 记录，超额一条再丢弃。
void queue_allocation_failure_keeps_exact_budget() {
  fc01::AsyncLog log;
  const std::string record(1024, 'q');
  begin_held_record(record);
  const auto before = fc01::native_log_dropped.load();
  const auto accepted = fail_one_queue_allocation(record);
  assert(fc01::native_log_dropped.load() == before + 1);
  for (std::size_t n = accepted; n < 256; ++n) {
    std::cout.write(record.data(), record.size()).flush();
    assert(fc01::native_log_dropped.load() == before + 1);
  }
  std::cout.write(record.data(), record.size()).flush();
  assert(fc01::native_log_dropped.load() == before + 2);
  assert(std::cout.good());
  hold_sink.store(false);
}

// 队列下一块申请在析构 sync 中失败也只丢一段，不得由 noexcept 析构触发 terminate。
void destructor_allocation_failure() {
  auto* const original = std::cout.rdbuf();
  auto* log = new fc01::AsyncLog;
  const std::string record(1024, 'd');
  begin_held_record(record);
  const auto before = fc01::native_log_dropped.load();
  fail_one_queue_allocation(record);
  assert(fc01::native_log_dropped.load() == before + 1);
  std::cout.write(record.data(), record.size());
  allocation_fired.store(false);
  std::thread release([] {
    await([] {
      return allocation_fired.load();
    });
    hold_sink.store(false);
  });
  fail_allocation = true;
  delete log;
  fail_allocation = false;
  release.join();
  assert(allocation_fired.load());
  assert(fc01::native_log_dropped.load() >= before + 2);
  assert(std::cout.rdbuf() == original && std::cout.good());
}

// 模拟磁盘满/永久 I/O 错误；只让消费者丢当前记录，不污染生产端 stream 状态。
void sink_failure(int code) {
  fc01::AsyncLog log;
  const auto before = fc01::native_log_dropped.load();
  write_error.store(code);
  std::cout << "synthetic sink failure" << std::endl;
  await([before] {
    return fc01::native_log_dropped.load() > before;
  });
  assert(write_error.load() == 0);
  assert(std::cout.good());
  std::cout << "sink recovered" << std::endl;
}
} // namespace

void* operator new(std::size_t size) {
  if (fail_allocation) {
    fail_allocation = false;
    allocation_fired.store(true);
    throw std::bad_alloc();
  }
  if (void* memory = std::malloc(size ? size : 1))
    return memory;
  throw std::bad_alloc();
}
// 测试替换的 new/delete 都使用 malloc/free；保留调用边界，避免 GCC 把故障注入实现
// 内联后误判为生产代码混用了 malloc 与普通 delete，严格告警仍然开启。
[[gnu::noinline]] void operator delete(void* memory) noexcept {
  std::free(memory);
}
[[gnu::noinline]] void operator delete(void* memory, std::size_t) noexcept {
  std::free(memory);
}

extern "C" int __real_fcntl(int, int, ...);
extern "C" int __wrap_fcntl(int fd, int command, ...) {
  int value = 0;
  if (command == F_SETFL) {
    va_list args;
    va_start(args, command);
    value = va_arg(args, int);
    va_end(args);
  }
  if (fd == STDOUT_FILENO && fail_fcntl.load() == command) {
    fail_fcntl.store(-1);
    errno = EIO;
    return -1;
  }
  return command == F_SETFL ? __real_fcntl(fd, command, value) : __real_fcntl(fd, command);
}

// libstdc++ 的 pthread_create 调用来自共享库，普通 --wrap 不覆盖；此处仅在测试可执行文件中
// 通过 ELF 符号插入并用 RTLD_NEXT 转交真实实现，原生头文件无条件分支或测试开关。
extern "C" int pthread_create(pthread_t* thread, const pthread_attr_t* attr, void* (*start)(void*),
                              void* arg) {
  if (fail_thread.exchange(false))
    return EAGAIN;
  using Create = int (*)(pthread_t*, const pthread_attr_t*, void* (*)(void*), void*);
  static const auto original = reinterpret_cast<Create>(dlsym(RTLD_NEXT, "pthread_create"));
  assert(original != nullptr);
  return original(thread, attr, start, arg);
}

extern "C" ssize_t __real_write(int, const void*, std::size_t);
extern "C" ssize_t __wrap_write(int fd, const void* data, std::size_t count) {
  if (fd == STDOUT_FILENO) {
    if (hold_sink.load()) {
      sink_entered.store(true);
      while (hold_sink.load())
        std::this_thread::yield();
    }
    if (const int error = write_error.exchange(0)) {
      errno = error;
      return -1;
    }
  }
  return __real_write(fd, data, count);
}

int main() {
  const int original_stdout = dup(STDOUT_FILENO);
  const int sink = open("/dev/null", O_WRONLY);
  assert(original_stdout >= 0 && sink >= 0 && dup2(sink, STDOUT_FILENO) >= 0);
  close(sink);
  for (int cycle = 0; cycle < 20; ++cycle) {
    startup_failure(F_GETFL, false);
    startup_failure(F_SETFL, false);
    startup_failure(-1, true);
    fc01::AsyncLog log;
  }
  pending_allocation_failure();
  queue_allocation_failure_keeps_exact_budget();
  destructor_allocation_failure();
  sink_failure(ENOSPC);
  sink_failure(EIO);
  assert(dup2(original_stdout, STDOUT_FILENO) >= 0);
  close(original_stdout);
  std::cout << "STAGE3_ASYNC_LOG_RESOURCE_FAILURES_PASS" << std::endl;
}
