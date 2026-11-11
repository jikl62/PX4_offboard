// 只读配置检查入口：使用与生产节点相同的参数读取/校验合同，要求RC导航启用且程序Arm关闭。
// 不创建飞行发布器、不spin、不发解锁或模式命令；成功0，异常打印原因返回2。
#include "jikl_fc01_system/ros_config.hpp"
#include <iostream>
// 命令行参数沿ROS惯例传入init；返回码供安装/构建预检识别配置是否满足人工解锁政策。
int main(int argc, char** argv) {
  int code = 0;
  try {
    // 这里只建参数节点，名称保持fc01_system以读取同一YAML作用域；读取失败向下方catch传播。
    rclcpp::init(argc, argv);
    rclcpp::Node node("fc01_system");
    const auto config = fc01::read_ros_config(node);
    // RC自动起飞必须保留人工解锁；这条检查只验证配置，没有绕过原Core中的权限/地面/新样本门。
    if (!config.attitude.rc_navigation_enabled || config.attitude.arm_enabled)
      throw std::runtime_error("manual-only RC navigation config required");
    std::cout << "FC01_RC_NAVIGATION_CONFIG_VALID manual_arm_only=true target="
              << config.attitude.rc_target_height << '\n';
    // 统一捕获配置与ROS初始化异常，保留具体错误文本，退出码2明确表示未通过。
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    code = 2;
  }

  // 清理ROS上下文后退出；局部节点已按作用域释放，没有留下后台控制循环。
  rclcpp::shutdown();
  return code;
}
