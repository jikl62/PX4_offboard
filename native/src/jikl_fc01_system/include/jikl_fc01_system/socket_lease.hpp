#pragma once
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
// Kept for the entire server lifetime. Never unlink this advisory lock pathname:
// unlink/recreate would split the coordination domain into two lock inodes.
class SocketPathLease {
 public:
  explicit SocketPathLease(const std::string& path):path_(path+".lock"){
    const auto parent=std::filesystem::path(path).parent_path();struct stat st{};
    if(lstat(parent.c_str(),&st)!=0||!S_ISDIR(st.st_mode)||st.st_uid!=getuid()||(st.st_mode&0022))
      throw std::runtime_error("SOCKET_DIRECTORY_NOT_PRIVATE_OWNED_DIRECTORY");
    fd_=open(path_.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK,0600);
    if(fd_<0)throw std::runtime_error("SOCKET_LOCK_OPEN_UNKNOWN");
    if(fstat(fd_,&identity_)!=0||!S_ISREG(identity_.st_mode)||identity_.st_uid!=getuid()||(identity_.st_mode&0022)||
       flock(fd_,LOCK_EX|LOCK_NB)!=0||!identity_matches()){
      close(fd_);fd_=-1;throw std::runtime_error("SOCKET_LOCK_BUSY_OR_UNKNOWN");
    }
  }
  ~SocketPathLease(){if(fd_>=0)close(fd_);}
  SocketPathLease(const SocketPathLease&)=delete;
  SocketPathLease& operator=(const SocketPathLease&)=delete;
  bool identity_matches() const{
    struct stat st{};return lstat(path_.c_str(),&st)==0&&S_ISREG(st.st_mode)&&st.st_uid==getuid()&&!(st.st_mode&0022)&&st.st_dev==identity_.st_dev&&st.st_ino==identity_.st_ino;
  }
 private:
  std::string path_;int fd_{-1};struct stat identity_{};
};
} // namespace fc01
