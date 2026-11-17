#pragma once
// Unix socket 路径租约：整个服务器寿命持有同一个 flock 锁 inode，防止双控制器竞争。
// 锁文件不能删除后重建，否则旧持有者和新持有者会锁住不同 inode，失去互斥。
// 目录/文件要求当前用户所有且其它用户不可写；拒绝符号链接和身份替换，不猜测旧对象是否可删。
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
namespace fc01 {
// 锁的身份是文件系统inode而非路径文本；整个服务器寿命使用同一对象，绝不unlink锁文件。
class SocketPathLease {
public:
  // 先验证私有父目录，再用 NOFOLLOW/CLOEXEC/非阻塞方式打开锁；任何未知情况都拒绝。
  explicit SocketPathLease(const std::string& path) : path_(path + ".lock") {
    // :path_(...)先构造成员字符串；fd_为唯一拥有的系统文件描述符，-1表示尚未取得。
    const auto parent = std::filesystem::path(path).parent_path();
    struct stat st {};
    if (lstat(parent.c_str(), &st) != 0 || !S_ISDIR(st.st_mode) || st.st_uid != getuid() ||
        (st.st_mode & 0022))
      throw std::runtime_error("SOCKET_DIRECTORY_NOT_PRIVATE_OWNED_DIRECTORY");
    fd_ = open(path_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK, 0600);
    // NOFOLLOW拒绝末级符号链接，CLOEXEC避免子进程exec继承租约；位掩码|组合系统调用选项。
    if (fd_ < 0)
      throw std::runtime_error("SOCKET_LOCK_OPEN_UNKNOWN");
    // 同一文件描述符的普通文件、所有者、权限、独占锁及路径身份须同时满足，失败先关闭 fd。
    if (fstat(fd_, &identity_) != 0 || !S_ISREG(identity_.st_mode) ||
        identity_.st_uid != getuid() || (identity_.st_mode & 0022) ||
        flock(fd_, LOCK_EX | LOCK_NB) != 0 || !identity_matches()) {
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("SOCKET_LOCK_BUSY_OR_UNKNOWN");
    }
  }
  // 析构仅关闭自己的 fd 释放锁，不 unlink 路径，避免分裂协调域。
  ~SocketPathLease() {
    if (fd_ >= 0)
      close(fd_);
  }
  SocketPathLease(const SocketPathLease&) = delete;
  // =delete禁止复制；否则两个对象会自认为拥有同一个fd，可能重复close或提前释放别人的锁。
  SocketPathLease& operator=(const SocketPathLease&) = delete;
  // 每次比较设备号+inode，确保仍是最初锁住的文件，不把路径同名当成同一对象。
  bool identity_matches() const {
    // 只读重新核对路径；不能因为文本路径相同就把被替换的文件当原锁，更不能据此删除它。
    struct stat st {};
    return lstat(path_.c_str(), &st) == 0 && S_ISREG(st.st_mode) && st.st_uid == getuid() &&
           !(st.st_mode & 0022) && st.st_dev == identity_.st_dev && st.st_ino == identity_.st_ino;
  }

private:
  // path_ 保存 .lock 路径，identity_ 是打开时身份，fd_ 在对象生命周期内独占拥有。
  std::string path_;
  int fd_{-1};
  struct stat identity_ {};
};
} // namespace fc01
