#include "jikl_fc01_system/ros_config.hpp"
#include <iostream>
int main(int argc,char** argv){
  int code=0;
  try{
    rclcpp::init(argc,argv);rclcpp::Node node("fc01_system");
    const auto config=fc01::read_ros_config(node);
    if(!config.attitude.rc_navigation_enabled||config.attitude.arm_enabled)
      throw std::runtime_error("manual-only RC navigation config required");
    std::cout<<"FC01_RC_NAVIGATION_CONFIG_VALID manual_arm_only=true target="
      <<config.attitude.rc_target_height<<'\n';
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';code=2;}
  rclcpp::shutdown();return code;
}
