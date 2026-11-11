# 纯主机 C++ 正向回归集合：不需要 ROS、DDS、飞控或电机；不包含历史漏洞反例。
# 每个 test_*.cpp 独立成为可执行程序，以 CTest 退出码判定，不把日志文本当作通过。
# 原生离线单元测试工程：只构建本机纯 C++ 测试，不启动 ROS 控制节点或连接飞控。
cmake_minimum_required(VERSION 3.16)
project(fc01_native_host_tests LANGUAGES CXX)
# 启用 CTest 注册；下方 CONFIGURE_DEPENDS 使新增测试触发重新生成。
enable_testing()
# 收集直属 test_*.cpp；历史反例目录和独立回放工具不会因递归 glob 被意外运行。
file(GLOB FC01_TESTS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")
# 每个源文件生成同名可执行程序并注册 CTest；包含路径仅指向本工程头文件。
foreach(source IN LISTS FC01_TESTS)
  # 文件名对应唯一测试名，保留原注册和执行顺序合同。
  get_filename_component(name "${source}" NAME_WE)
  add_executable("${name}" "${source}")
  # C++17 与严格告警确保编译期问题不会静默通过；Debug 构建保持 assert 生效。
  # 保持 C++17 与严格警告；任何新警告视为失败，便于捕获接口/类型回归。
  target_compile_features("${name}" PRIVATE cxx_std_17)
  target_compile_options("${name}" PRIVATE -Wall -Wextra -Werror -pedantic)
  # 直接包含生产纯核心头文件，验证同一份控制实现，不复制替身算法。
  target_include_directories("${name}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src/jikl_fc01_system/include")
  # CTest 运行测试入口中的 assert 合同；Release 构建需留意 NDEBUG，验证任务使用 Debug。
  add_test(NAME "${name}" COMMAND "${name}")
endforeach()
