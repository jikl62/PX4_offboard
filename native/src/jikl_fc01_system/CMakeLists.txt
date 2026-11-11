cmake_minimum_required(VERSION 3.16)
project(jikl_fc01_system)
# 可执行程序在 lib/<包名>，消息类型支持库在 lib；相对 RPATH 允许安装目录搬迁后继续找到库。
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/..")
# 构建依赖：ament 负责 ROS 包安装，rclcpp 负责节点，px4_msgs 必须来自实际选定的 PX4 SDK。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
# 自定义消息只引用标准时间类型；Python Odometry 订阅依赖在 package.xml 中声明。
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 原生控制器的日志等后台线程需要系统线程库。
find_package(Threads REQUIRED)
# 五个消息保留原字段和类型身份；生成类型不代表给视觉通道授予控制权限。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionState.msg"
  "msg/NavigationCommand.msg"
  "msg/VelocityCommand.msg"
  "msg/VisionHealth.msg"
  "msg/AttitudeThrustPreview.msg"
  DEPENDENCIES builtin_interfaces)
# C++ 节点链接本包生成的消息类型支持；不能只生成 .msg 而遗漏运行期库。
rosidl_get_typesupport_target(fc01_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
# 控制节点与参数检查器分别生成；检查配置不需要启动控制节点或连接飞控。
add_executable(fc01_system_node src/fc01_system_node.cpp)
add_executable(fc01_config_check src/fc01_config_check.cpp)
# 参数检查器使用与控制器相同的 C++17 合同和 ROS/PX4 类型依赖。
target_compile_features(fc01_config_check PUBLIC cxx_std_17)
target_compile_options(fc01_config_check PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(fc01_config_check PRIVATE include)
ament_target_dependencies(fc01_config_check rclcpp px4_msgs)
# 控制节点保持现有编译选项、头文件路径和消息链接关系，注释整理不改变它们。
target_compile_features(fc01_system_node PUBLIC cxx_std_17)
target_compile_options(fc01_system_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(fc01_system_node PRIVATE include)
ament_target_dependencies(fc01_system_node rclcpp px4_msgs)
target_link_libraries(fc01_system_node Threads::Threads "${fc01_typesupport}")
# ROS 2 在 lib/<包名> 查找可执行入口；头文件单独安装供离线校验程序复用。
install(TARGETS fc01_system_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS fc01_config_check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
# 两个独立 Python 节点需要可执行权限；原始转换器与速度诊断均不自动启动。
install(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/vision_odometry_adapter.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/velocity_command_diagnostic.py"
  DESTINATION lib/${PROJECT_NAME})
# 状态校验器是被导入的纯计算模块，与转换器放在同一目录，不注册为节点入口。
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/vision_state_validation.py"
  DESTINATION lib/${PROJECT_NAME})
# 下游使用消息时需要运行期类型支持；ament_package 最后生成包索引和环境脚本。
ament_export_dependencies(rosidl_default_runtime)
ament_package()
