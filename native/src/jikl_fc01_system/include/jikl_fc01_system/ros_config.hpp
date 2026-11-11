#pragma once
// 统一 ROS 配置读取：实际节点和离线安装校验器使用完全相同的参数解释。
// 本机真实控制要求 use_sim_time=false；profile、运行模式和配置来源都进入诊断身份。
#include "jikl_fc01_system/runtime.hpp"
#include "jikl_fc01_system/attitude_ros_parameters.hpp"
namespace fc01 {
// The node and install-time validator consume precisely the same ROS parameters.
inline Config read_ros_config(rclcpp::Node& node) {
  // 禁止把可跳变/暂停的模拟时钟引入生产配置；失败时直接停止配置加载。
  if (node.get_parameter("use_sim_time").as_bool())
    throw std::runtime_error("use_sim_time must be false");
  Config config;
  config.profile = node.declare_parameter<std::string>("profile", "ground");
  config.runtime_mode = node.declare_parameter<std::string>("fc01_runtime_mode", "diagnostic");
  config.entry_version = node.declare_parameter<std::string>("fc01_entry_version", "unknown");
  config.config_path = node.declare_parameter<std::string>("fc01_config_path", "");
  config.config_source_path = node.declare_parameter<std::string>("fc01_config_source_path", "");
  // 通用控制入口保持独立动作门；max_height/max_radius 为 m，各线速度 m/s，偏航速度 rad/s。
  config.flight_enabled = node.declare_parameter<bool>("flight_enabled", false);
  config.control.max_height = node.declare_parameter<double>("max_height_m", 5.);
  config.control.max_radius = node.declare_parameter<double>("max_radius_m", 3.);
  config.control.xy_speed = node.declare_parameter<double>("max_xy_speed", .3);
  config.control.z_speed = node.declare_parameter<double>("max_z_speed", .3);
  config.control.climb_speed = config.control.z_speed;
  config.control.yaw_speed = node.declare_parameter<double>("max_yaw_rate", .5);
  // 姿态高度参数由同一入口补齐，最后整体验证；无效配置不能靠默认值继续启动。
  config.attitude = attitude_parameters(node);
  config.validate();
  return config;
}
} // namespace fc01
