#pragma once
// 异步原生日志：控制线程只向有界内存队列写入，后台线程处理终端/管道背压。
// 日志丢弃计数用于诊断，不能作为飞行租约或控制权限；所有时限使用 steady_clock 纳秒。
// 构造时接管 cout/cerr 的缓冲区，析构时恢复它们，并设置约 100 ms 的名义排空截止时间。
// poll 和线程调度仍可能增加退出耗时；join 没有硬实时超时保证，不能将该值当绝对上界。
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <deque>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <thread>
#include <unistd.h>
namespace fc01 {
inline std::atomic<unsigned long long> native_log_dropped{0};
// Bounded producer queue. SD/pipe/terminal backpressure is never a flight lease.
class AsyncLog final : private std::streambuf {
public:
  // 忽略 SIGPIPE 并将 stdout 置为非阻塞；终端消失不能通过信号直接杀死控制进程。
  AsyncLog() {
    struct sigaction ignore {};
    ignore.sa_handler = SIG_IGN;
    sigemptyset(&ignore.sa_mask);
    if (sigaction(SIGPIPE, &ignore, &pipe_signal_) < 0)
      throw std::runtime_error("cannot protect native log pipe");
    flags_ = fcntl(STDOUT_FILENO, F_GETFL);
    if (flags_ < 0 || fcntl(STDOUT_FILENO, F_SETFL, flags_ | O_NONBLOCK) < 0)
      throw std::runtime_error("cannot configure nonblocking native log sink");
    worker_ = std::thread([this] {
      consume();
    });
    cout_ = std::cout.rdbuf(this);
    cerr_ = std::cerr.rdbuf(this);
  }
  // 先提交生产端残留、恢复标准流，再在与条件变量相同的互斥量下发布停止条件。
  ~AsyncLog() override {
    sync();
    std::cout.rdbuf(cout_);
    std::cerr.rdbuf(cerr_);
    // Publish the wait predicate under the consumer's mutex. An atomic store
    // alone could race its predicate check and lose the shutdown notification.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      deadline_.store(clock_ns() + 100000000);
    }
    ready_.notify_one();
    if (worker_.joinable())
      worker_.join();
    fcntl(STDOUT_FILENO, F_SETFL, flags_);
    sigaction(SIGPIPE, &pipe_signal_, nullptr);
  }

private:
  static long long clock_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  // 单条累计缓冲上限 64 KiB；超限丢弃整段并记录，不等待慢速输出端。
  std::streamsize xsputn(const char* data, std::streamsize n) override {
    if (n <= 0)
      return 0;
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::size_t(n) > 65536 || pending_.size() + std::size_t(n) > 65536) {
      pending_.clear();
      ++native_log_dropped;
    } else
      pending_.append(data, std::size_t(n));
    return n;
  }
  int_type overflow(int_type ch) override {
    if (!traits_type::eq_int_type(ch, traits_type::eof())) {
      const char value = traits_type::to_char_type(ch);
      xsputn(&value, 1);
    }
    return traits_type::not_eof(ch);
  }
  // flush 只是把 pending 移入总上限 256 KiB 的队列，不同步等待 SD 卡或终端。
  int sync() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.empty()) {
      if (bytes_ + pending_.size() > 262144) {
        ++native_log_dropped;
        pending_.clear();
      } else {
        bytes_ += pending_.size();
        queue_.push_back(std::move(pending_));
        pending_.clear();
        ready_.notify_one();
      }
    }
    return 0;
  }
  // 消费者取出记录后释放锁再写文件描述符；部分写保留偏移，EAGAIN/EINTR 仅短暂 poll 后重试。
  void consume() {
    while (true) {
      std::string record;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] {
          return !queue_.empty() || deadline_.load() != 0;
        });
        if (queue_.empty())
          return;
        record = std::move(queue_.front());
        queue_.pop_front();
        bytes_ -= record.size();
      }
      std::size_t at = 0;
      while (at < record.size()) {
        // 只有析构发布了截止时间才限制排空；到期丢弃尾部，避免退出永久卡在日志。
        const auto end = deadline_.load();
        if (end && clock_ns() >= end) {
          ++native_log_dropped;
          return;
        }
        const auto sent = write(STDOUT_FILENO, record.data() + at, record.size() - at);
        if (sent > 0) {
          at += std::size_t(sent);
          continue;
        }
        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
          pollfd p{STDOUT_FILENO, POLLOUT, 0};
          poll(&p, 1, 20);
          continue;
        }
        ++native_log_dropped;
        break;
      }
    }
  }
  int flags_{};
  std::streambuf *cout_{}, *cerr_{};
  struct sigaction pipe_signal_ {};
  // mutex_ 同时保护生产缓存、队列和字节计数；deadline_ 跨线程观察，worker_ 只消费输出。
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<std::string> queue_;
  std::string pending_;
  std::size_t bytes_{};
  std::atomic<long long> deadline_{0};
  std::thread worker_;
};
} // namespace fc01
