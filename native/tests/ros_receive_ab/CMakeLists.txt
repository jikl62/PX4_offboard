# ROS 接收调度 A/B 台架：使用 std_msgs 测量 DDS/执行器行为，不发布 PX4 动力指令。
# 此小包独立构建，避免把台架测量入口加入生产控制节点。
# ROS 2 接收调度对照实验：构建专用小消息探针，验证接收追赶/预算，不链接生产飞控控制器。
cmake_minimum_required(VERSION 3.8)
project(fc01_receive_ab)
# 构建框架、节点 API 与台架通用消息分别声明，缺少依赖即配置失败。
# 只依赖 rclcpp 和 std_msgs；实验消息不发往 /fmu 控制话题。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# 由本文件位置定位工程头文件，不依赖调用者当前工作目录。
# 从本文件位置计算源码根目录，使独立构建目录也能包含实际 receive_schedule 头文件。
get_filename_component(FC01_SOURCE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
# 生成本机实验程序；本文件没有安装/启动服务指令，运行方式由独立验证任务约束。
add_executable(fc01_receive_ab receive_ab.cpp)
ament_target_dependencies(fc01_receive_ab rclcpp std_msgs)
# 保留 C++17 与原优化选项；A/B 对比必须使用一致编译条件。
# 保留 C++17、O2 和既有警告参数；本次只解释构建职责，不改变实验负载。
target_compile_features(fc01_receive_ab PRIVATE cxx_std_17)
target_compile_options(fc01_receive_ab PRIVATE -O2 -Wall -Wextra)
target_include_directories(fc01_receive_ab PRIVATE "${FC01_SOURCE_ROOT}/native/src/jikl_fc01_system/include")
