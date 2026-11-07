#pragma once
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
class AsyncLog final:private std::streambuf {
 public:
  AsyncLog(){
    struct sigaction ignore{};ignore.sa_handler=SIG_IGN;sigemptyset(&ignore.sa_mask);
    if(sigaction(SIGPIPE,&ignore,&pipe_signal_)<0)throw std::runtime_error("cannot protect native log pipe");
    flags_=fcntl(STDOUT_FILENO,F_GETFL);
    if(flags_<0||fcntl(STDOUT_FILENO,F_SETFL,flags_|O_NONBLOCK)<0)
      throw std::runtime_error("cannot configure nonblocking native log sink");
    worker_=std::thread([this]{consume();});
    cout_=std::cout.rdbuf(this);cerr_=std::cerr.rdbuf(this);
  }
  ~AsyncLog()override{
    sync();std::cout.rdbuf(cout_);std::cerr.rdbuf(cerr_);
    // Publish the wait predicate under the consumer's mutex. An atomic store
    // alone could race its predicate check and lose the shutdown notification.
    {std::lock_guard<std::mutex> lock(mutex_);deadline_.store(clock_ns()+100000000);}
    ready_.notify_one();
    if(worker_.joinable())worker_.join();
    fcntl(STDOUT_FILENO,F_SETFL,flags_);
    sigaction(SIGPIPE,&pipe_signal_,nullptr);
  }
 private:
  static long long clock_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();}
  std::streamsize xsputn(const char* data,std::streamsize n)override{
    if(n<=0)return 0;
    std::lock_guard<std::mutex> lock(mutex_);
    if(std::size_t(n)>65536||pending_.size()+std::size_t(n)>65536){pending_.clear();++native_log_dropped;}
    else pending_.append(data,std::size_t(n));
    return n;
  }
  int_type overflow(int_type ch)override{
    if(!traits_type::eq_int_type(ch,traits_type::eof())){const char value=traits_type::to_char_type(ch);xsputn(&value,1);}
    return traits_type::not_eof(ch);
  }
  int sync()override{
    std::lock_guard<std::mutex> lock(mutex_);
    if(!pending_.empty()){
      if(bytes_+pending_.size()>262144){++native_log_dropped;pending_.clear();}
      else{bytes_+=pending_.size();queue_.push_back(std::move(pending_));pending_.clear();ready_.notify_one();}
    }
    return 0;
  }
  void consume(){
    while(true){
      std::string record;
      {std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock,[this]{return !queue_.empty()||deadline_.load()!=0;});
        if(queue_.empty())return;
        record=std::move(queue_.front());queue_.pop_front();bytes_-=record.size();}
      std::size_t at=0;
      while(at<record.size()){
        const auto end=deadline_.load();if(end&&clock_ns()>=end){++native_log_dropped;return;}
        const auto sent=write(STDOUT_FILENO,record.data()+at,record.size()-at);
        if(sent>0){at+=std::size_t(sent);continue;}
        if(sent<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)){
          pollfd p{STDOUT_FILENO,POLLOUT,0};poll(&p,1,20);continue;}
        ++native_log_dropped;break;
      }
    }
  }
  int flags_{};std::streambuf *cout_{},*cerr_{};
  struct sigaction pipe_signal_{};
  std::mutex mutex_;std::condition_variable ready_;std::deque<std::string> queue_;
  std::string pending_;std::size_t bytes_{};std::atomic<long long> deadline_{0};std::thread worker_;
};
} // namespace fc01
