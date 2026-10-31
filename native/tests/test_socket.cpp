#include "jikl_fc01_system/unix_server.hpp"
#include "jikl_fc01_system/runtime.hpp"
#include <cassert>
#include <filesystem>
#include <iostream>
int main(){
  char pattern[]="/tmp/fc01_socket_XXXXXX";char* directory=mkdtemp(pattern);assert(directory);
  const std::string path=std::string(directory)+"/control.sock";
  try{
    fc01::UnixServer server(path);struct stat st{};assert(lstat(path.c_str(),&st)==0);
    assert((st.st_mode&0777)==0600&&st.st_uid==getuid());
    bool refused=false;try{fc01::UnixServer duplicate(path);}catch(const std::runtime_error&){refused=true;}assert(refused);
  }catch(const std::runtime_error& error){
    if(errno!=EPERM&&errno!=EACCES)throw;
    std::cout<<"native AF_UNIX bind/listen: SKIP (execution sandbox permission; runtime UNKNOWN): "<<error.what()<<'\n';
  }
  assert(!std::filesystem::exists(path));std::filesystem::remove_all(directory); // fixture also owns the persistent .lock
  fc01::Stamp stamp;assert(stamp.accept(10,1));assert(stamp.fresh(1.1,.5));
  assert(!stamp.accept(10,1.4));assert(!stamp.fresh(1.51,.5)); // duplicate must not renew freshness
  assert(!stamp.accept(9,1.6));assert(stamp.regressed&&!stamp.fresh(1.6,10));
  char owned_pattern[]="/tmp/fc01_owned_XXXXXX";int fd=mkstemp(owned_pattern);assert(fd>=0);close(fd);
  bool refused=false;try{fc01::UnixServer bad(owned_pattern);}catch(const std::runtime_error&){refused=true;}
  assert(refused);unlink(owned_pattern);
  std::cout<<"socket regular-file protection / telemetry stamp regression: PASS\n";
}
