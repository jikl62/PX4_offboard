#include "jikl_fc01_system/unix_server.hpp"
#include <cassert>
#include <filesystem>
#include <iostream>
int main(){
  char pattern[]="/tmp/g_socket_policy_XXXXXX";const char* directory=mkdtemp(pattern);assert(directory);
  const std::string path=std::string(directory)+"/control.sock";
  // A real socket filesystem inode without socket()/bind() (sandbox forbids
  // those). This proves type handling/retention, not transport or liveness.
  if(mknod(path.c_str(),S_IFSOCK|0600,0)!=0){
    std::cout<<"socket filesystem fixture: SKIP (UNKNOWN)\n";
  }else {
    struct stat before{},after{};assert(lstat(path.c_str(),&before)==0);
    bool refused=false;
    try{fc01::UnixServer server(path);}catch(const std::runtime_error& e){
      refused=std::string(e.what()).find("LIVENESS_UNKNOWN")!=std::string::npos;
    }
    assert(refused&&lstat(path.c_str(),&after)==0&&before.st_ino==after.st_ino&&S_ISSOCK(after.st_mode));
    assert(unlink(path.c_str())==0); // test owns this offline fixture only
    std::cout<<"server_stale_looking_socket_refused_without_connect_or_delete: PASS\n";
  }
  {
    fc01::SocketPathLease first(path);bool refused=false;
    try{fc01::SocketPathLease second(path);}catch(const std::runtime_error&){refused=true;}
    assert(refused&&first.identity_matches());
  }
  const auto lock=path+".lock";assert(unlink(lock.c_str())==0);
  assert(symlink("missing",lock.c_str())==0);bool refused=false;
  try{fc01::SocketPathLease bad(path);}catch(const std::runtime_error&){refused=true;}
  struct stat st{};assert(refused&&lstat(lock.c_str(),&st)==0&&S_ISLNK(st.st_mode));
  std::filesystem::remove_all(directory);
  std::cout<<"server_lifetime_lock_and_lock_symlink_refusal: PASS\n";
}
