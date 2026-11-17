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
#include <new>
#include <poll.h>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <thread>
#include <unistd.h>
namespace fc01 {
inline std::atomic<unsigned long long> native_log_dropped{0};
// 私有继承streambuf只为重载标准流写入接口，外部不能把日志对象当任意流缓冲使用。
// 本类修改进程级cout/cerr、stdout标志及SIGPIPE处理器；它的寿命必须包住这些标准流的使用。
class AsyncLog final : private std::streambuf {
public:
  // 忽略 SIGPIPE 并将 stdout 置为非阻塞；终端消失不能通过信号直接杀死控制进程。
  // 系统设置与工作线程全部取得后才接管标准流；中途失败恢复已改设置并传播原异常。
  AsyncLog() {
    // sigaction结构先清零；保存旧处理器到成员pipe_signal_，以后恢复的是原状态而非猜测默认值。
    struct sigaction ignore {};
    ignore.sa_handler = SIG_IGN;
    sigemptyset(&ignore.sa_mask);
    if (sigaction(SIGPIPE, &ignore, &pipe_signal_) < 0)
      throw std::runtime_error("cannot protect native log pipe");
    try {
      flags_ = fcntl(STDOUT_FILENO, F_GETFL);
      if (flags_ < 0 || fcntl(STDOUT_FILENO, F_SETFL, flags_ | O_NONBLOCK) < 0)
        throw std::runtime_error("cannot configure nonblocking native log sink");
      worker_ = std::thread([this] {
        // [this]借用本日志对象；构造先取得工作线程，最后才替换标准流，析构join后对象成员才销毁。
        consume();
      });
    } catch (...) {
      // 构造未完成时不会调用析构函数，必须在此恢复进程级设置。回滚不覆盖首个错误；
      // 若操作系统连恢复系统调用也拒绝，无法承诺恢复成功，不能把此路径当飞行许可。
      if (flags_ >= 0)
        fcntl(STDOUT_FILENO, F_SETFL, flags_);
      sigaction(SIGPIPE, &pipe_signal_, nullptr);
      throw;
    }
    cout_ = std::cout.rdbuf(this);
    cerr_ = std::cerr.rdbuf(this);
  }
  // 先提交生产端残留、恢复标准流，再在与条件变量相同的互斥量下发布停止条件。
  ~AsyncLog() override {
    sync();
    std::cout.rdbuf(cout_);
    std::cerr.rdbuf(cerr_);
    // 与消费者检查谓词使用同一mutex后再通知；只写atomic并notify会在检查/入睡之间漏掉唤醒。
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // lock_guard作用域结束自动解锁；notify_one放在锁外，消费者醒来后可争取锁。
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
    // data/n是本次标准流传来的只读字节段；只复制进有界pending，不保存调用者的原始地址。
    if (n <= 0)
      return 0;
    std::lock_guard<std::mutex> lock(mutex_);
    // 控制侧仍可能等待此短临界区/分配器；“异步”表示不等待慢终端写完，不是无锁或硬实时保证。
    if (std::size_t(n) > 65536 || pending_.size() + std::size_t(n) > 65536) {
      pending_.clear();
      ++native_log_dropped;
    } else {
      try {
        pending_.append(data, std::size_t(n));
      } catch (const std::bad_alloc&) {
        // 日志分配失败按整段丢弃处理；不能让 ostream 永久置 badbit 后静默停记。
        pending_.clear();
        ++native_log_dropped;
      }
    }
    return n;
  }
  int_type overflow(int_type ch) override {
    // streambuf单字符入口；traits_type处理字符与EOF的区别，避免把EOF当字节写进日志。
    if (!traits_type::eq_int_type(ch, traits_type::eof())) {
      const char value = traits_type::to_char_type(ch);
      xsputn(&value, 1);
    }
    return traits_type::not_eof(ch);
  }
  // flush 只是把 pending 移入总上限 256 KiB 的队列，不同步等待 SD 卡或终端。
  int sync() override {
    // ostream的flush调用此方法。返回0是缓冲接口成功，即使有界策略丢弃记录也靠计数披露而非阻塞。
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.empty()) {
      if (bytes_ + pending_.size() > 262144) {
        ++native_log_dropped;
        pending_.clear();
      } else {
        const auto size = pending_.size();
        try {
          queue_.push_back(std::move(pending_));
          // std::move允许移交字符串缓冲；仅push_back成功才增加bytes_，异常不能留下虚假占用。
          bytes_ += size;
          ready_.notify_one();
        } catch (const std::bad_alloc&) {
          // 析构也会调用 sync；内存不足丢弃本段并计数，不从 noexcept 析构传播异常。
          ++native_log_dropped;
        }
        pending_.clear();
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
        // 条件变量wait会原子释放锁并等待，唤醒后重新持锁且重查谓词；不靠一次通知猜队列非空。
        ready_.wait(lock, [this] {
          return !queue_.empty() || deadline_.load() != 0;
        });
        if (queue_.empty())
          return;
        record = std::move(queue_.front());
        // 先把记录所有权移到局部值，再出队并减预算；离开花括号解锁后才接触可能背压的write。
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
        // write可能只写一部分；at为本条已完成字节偏移，不允许从头重发造成日志重复。
        if (sent > 0) {
          at += std::size_t(sent);
          continue;
        }
        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
          pollfd p{STDOUT_FILENO, POLLOUT, 0};
          // 以20ms名义超时poll后再试，调度可再延后；消费者未持队列锁，生产者不陪终端一起等待。
          poll(&p, 1, 20);
          continue;
        }
        ++native_log_dropped;
        break;
      }
    }
  }
  int flags_{-1};
  // cout_/cerr_是借用的原标准流缓冲，恢复后不能delete；worker_则是本对象必须join的线程所有者。
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
