// 本机Unix控制入口资源回归：独占锁、权限、常规文件保护与源时间戳寿命分别验证。
// 创建唯一/tmp夹具目录并析构后删除；若平台禁止AF_UNIX，仅明确SKIP传输，不能伪装该项通过。
#include "jikl_fc01_system/unix_server.hpp"
#include "jikl_fc01_system/runtime.hpp"
#include <cassert>
#include <filesystem>
#include <iostream>
int main() {
  char pattern[] = "/tmp/fc01_socket_XXXXXX";
  char* directory = mkdtemp(pattern);
  assert(directory);
  const std::string path = std::string(directory) + "/control.sock";
  // 真实UnixServer要求socket0600且本用户拥有，第二实例拒绝；析构只移除本夹具自有入口。
  try {
    fc01::UnixServer server(path);
    struct stat st {};
    assert(lstat(path.c_str(), &st) == 0);
    assert((st.st_mode & 0777) == 0600 && st.st_uid == getuid());
    bool refused = false;
    try {
      fc01::UnixServer duplicate(path);
    } catch (const std::runtime_error&) {
      refused = true;
    }
    assert(refused);
  } catch (const std::runtime_error& error) {
    if (errno != EPERM && errno != EACCES)
      throw;
    std::cout
        << "native AF_UNIX bind/listen: SKIP (execution sandbox permission; runtime UNKNOWN): "
        << error.what() << '\n';
  }
  assert(!std::filesystem::exists(path));
  std::filesystem::remove_all(directory); // fixture also owns the persistent .lock
  // 重复时间戳不续新鲜度，回退锁存后正常时长预算也不能重新fresh。
  fc01::Stamp stamp;
  assert(stamp.accept(10, 1));
  assert(stamp.fresh(1.1, .5));
  assert(!stamp.accept(10, 1.4));
  assert(!stamp.fresh(1.51, .5)); // duplicate must not renew freshness
  assert(!stamp.accept(9, 1.6));
  assert(stamp.regressed && !stamp.fresh(1.6, 10));
  // 目标是普通文件时必须拒绝保留，不能误当陈旧socket删除用户文件。
  char owned_pattern[] = "/tmp/fc01_owned_XXXXXX";
  int fd = mkstemp(owned_pattern);
  assert(fd >= 0);
  close(fd);
  bool refused = false;
  try {
    fc01::UnixServer bad(owned_pattern);
  } catch (const std::runtime_error&) {
    refused = true;
  }
  assert(refused);
  unlink(owned_pattern);
  std::cout << "socket regular-file protection / telemetry stamp regression: PASS\n";
}
