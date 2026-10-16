#pragma once
#include "jikl_fc01_system/protocol.hpp"
#include "jikl_fc01_system/socket_lease.hpp"
#include <cerrno>
#include <cstring>
#include <functional>
#include <memory>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
namespace fc01 {
class UnixServer {
 public:
  explicit UnixServer(std::string path):path_(std::move(path)){
    if(path_.size()>=sizeof(sockaddr_un::sun_path))throw std::runtime_error("socket path too long");
    lease_=std::make_unique<SocketPathLease>(path_);
    struct stat st{};
    if(lstat(path_.c_str(),&st)==0){
      if(!S_ISSOCK(st.st_mode)||st.st_uid!=getuid())throw std::runtime_error("SOCKET_PATH_NOT_OWNED_UNIX_SOCKET: object preserved");
      throw std::runtime_error("SOCKET_PRESENT_LIVENESS_UNKNOWN: "+path_+"; existing socket preserved; quiesce every starter and use tools/socket_safety.py for manual diagnosis; ECONNREFUSED never authorizes deletion");
    }else if(errno!=ENOENT)throw std::runtime_error("SOCKET_PATH_UNKNOWN: activation refused");
    listener_=::socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    sockaddr_un a=address();
    if(listener_<0||::bind(listener_,reinterpret_cast<sockaddr*>(&a),sizeof(a))!=0){if(listener_>=0)::close(listener_);throw std::runtime_error("cannot bind controller socket");}
    if(lstat(path_.c_str(),&st)!=0||!S_ISSOCK(st.st_mode)||st.st_uid!=getuid()){
      ::close(listener_);listener_=-1;throw std::runtime_error("cannot identify newly bound controller socket; object preserved");
    }
    inode_=st.st_ino;device_=st.st_dev;
    if(::chmod(path_.c_str(),0600)!=0||::listen(listener_,2)!=0){::close(listener_);listener_=-1;remove_owned_socket();throw std::runtime_error("cannot protect/listen on controller socket");}
  }
  ~UnixServer(){if(client_>=0)::close(client_);if(listener_>=0)::close(listener_);
    remove_owned_socket();}
  UnixServer(const UnixServer&)=delete;UnixServer& operator=(const UnixServer&)=delete;
  bool connected() const{return client_>=0;}
  void publish(const std::string& line){if(client_>=0)output_+=line+'\n';}
  void poll(const std::function<std::string(const Request&)>& handle,const std::function<void()>& disconnected){
    int accepted=::accept4(listener_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
    if(accepted>=0){
      if(client_>=0){const char busy[]="{\"kind\":\"error\",\"message\":\"CONSOLE_BUSY\"}\n";
        ::send(accepted,busy,sizeof(busy)-1,MSG_NOSIGNAL);::close(accepted);}
      else {client_=accepted;input_.clear();output_.clear();last_seq_=0;}
    }
    if(client_<0)return;
    if(output_.size()>65536){drop(disconnected);return;}
    if(!output_.empty()){
      auto n=::send(client_,output_.data(),output_.size(),MSG_NOSIGNAL);
      if(n>0)output_.erase(0,std::size_t(n));
      else if(n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK){drop(disconnected);return;}
    }
    char buffer[4096];auto n=::recv(client_,buffer,sizeof(buffer),0);
    if(n==0){drop(disconnected);return;}
    if(n<0){if(errno!=EAGAIN&&errno!=EWOULDBLOCK)drop(disconnected);return;}
    input_.append(buffer,std::size_t(n));
    for(std::size_t pos;(pos=input_.find('\n'))!=std::string::npos;){
      std::string line=input_.substr(0,pos);input_.erase(0,pos+1);
      try {auto r=parse_request(line);
        if(r.seq<=last_seq_)throw std::invalid_argument("SEQUENCE_NOT_NEW");
        last_seq_=r.seq;publish(handle(r));
      }catch(const std::exception&){drop(disconnected);return;}
    }
    if(input_.size()>1024)drop(disconnected);
  }
 private:
  void remove_owned_socket(){
    struct stat st{};
    if(inode_&&lease_&&lease_->identity_matches()&&lstat(path_.c_str(),&st)==0&&S_ISSOCK(st.st_mode)&&
       st.st_uid==getuid()&&st.st_dev==device_&&st.st_ino==inode_)::unlink(path_.c_str());
  }
  sockaddr_un address() const{sockaddr_un a{};a.sun_family=AF_UNIX;std::memcpy(a.sun_path,path_.c_str(),path_.size()+1);return a;}
  void drop(const std::function<void()>& fn){if(client_>=0)::close(client_);client_=-1;input_.clear();output_.clear();fn();}
  std::string path_,input_,output_;int listener_{-1},client_{-1};ino_t inode_{};dev_t device_{};std::uint64_t last_seq_{};
  std::unique_ptr<SocketPathLease> lease_;
};
} // namespace fc01
