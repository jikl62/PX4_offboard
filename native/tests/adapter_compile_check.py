#!/usr/bin/env python3
# 主机语法探针：用最小 rclcpp/PX4/视觉消息替身编译真实适配器源码。
# 只证明所用类型/API的C++语法兼容；不链接、不启动ROS，也不能证明Humble或实际DDS通信通过。
"""Compile the actual adapter against a narrow type/API fixture.

This is NOT a ROS/Humble build, DDS test, or verification of the exact vendor
message source. No executable or ROS connection is created. Real build UNKNOWN.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
# 以下原始字符串是编译夹具源码，只覆盖适配器实际调用的接口；不是完整 rclcpp 实现。
RCLCPP=r'''#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
namespace rclcpp {
struct Event {
  using SharedPtr=std::shared_ptr<Event>;
  bool value{};
  bool set(){const bool before=value;value=true;return before;}
  bool check() const{return value;}
  bool check_and_clear(){const bool before=value;value=false;return before;}
};
struct Parameter {bool as_bool() const{return false;}};
struct Time {std::int64_t nanoseconds() const{return 1000000000;}};
struct Clock {Time now() const{return {};}};
struct MessageInfo {
  struct Rmw {std::int64_t received_timestamp{};};
  const Rmw& get_rmw_message_info() const{static const Rmw value;return value;}
};
struct QoS {
  explicit QoS(std::size_t){}
  QoS keep_last(int) const{return *this;}
  QoS reliable() const{return *this;}
  QoS best_effort() const{return *this;}
  QoS durability_volatile() const{return *this;}
};
struct SensorDataQoS:QoS {
  SensorDataQoS():QoS(5){}
  SensorDataQoS keep_last(int) const{return {};}
};
template<class T>struct Subscription {using SharedPtr=std::shared_ptr<Subscription<T>>;};
template<class T>struct Publisher {using SharedPtr=std::shared_ptr<Publisher<T>>;void publish(const T&){};};
class Node {
 public:
  explicit Node(const std::string&){}
  Parameter get_parameter(const std::string&) const{return {};}
  template<class T>T declare_parameter(const std::string&,const T& value){return value;}
  template<class T,class Qos,class Callback>typename Subscription<T>::SharedPtr create_subscription(const std::string&,Qos,Callback){return std::make_shared<Subscription<T>>();}
  template<class T,class Qos>typename Publisher<T>::SharedPtr create_publisher(const std::string&,Qos){return std::make_shared<Publisher<T>>();}
  std::shared_ptr<Clock> get_clock(){return std::make_shared<Clock>();}
  Event::SharedPtr get_graph_event(){return graph_event;}
  std::size_t count_publishers(const std::string& topic) const{return graph_count?graph_count(topic,true):1;}
  std::size_t count_subscribers(const std::string& topic) const{return graph_count?graph_count(topic,false):1;}
  Event::SharedPtr graph_event{std::make_shared<Event>()};
  std::function<std::size_t(const std::string&,bool)> graph_count;
};
inline void init(int,char**){}
inline void shutdown(){}
inline bool ok(){return false;}
template<class T>void spin_some(const T&){}
namespace executors {
struct SingleThreadedExecutor {
  template<class T>void add_node(const T&){}
  template<class T>void remove_node(const T&){}
  void spin_some(){}
};
}
}
'''
# PX4字段和版本常量仅用于编译期检查；不能代替部署机上的真实厂商消息版本核验。
MESSAGES={
    'manual_control_setpoint':('ManualControlSetpoint',0,'static constexpr std::uint8_t SOURCE_UNKNOWN=0,SOURCE_RC=1,SOURCE_MAVLINK_0=2,SOURCE_MAVLINK_1=3,SOURCE_MAVLINK_2=4,SOURCE_MAVLINK_3=5,SOURCE_MAVLINK_4=6,SOURCE_MAVLINK_5=7;std::uint64_t timestamp_sample{};bool valid{};std::uint8_t data_source{};float roll{},pitch{},yaw{},throttle{},flaps{},aux1{},aux2{},aux3{},aux4{},aux5{},aux6{};bool sticks_moving{};std::uint16_t buttons{};'),
    'manual_control_switches':('ManualControlSwitches',None,'std::uint64_t timestamp_sample{};std::uint8_t mode_slot{},arm_switch{},return_switch{},loiter_switch{},offboard_switch{},kill_switch{};std::uint32_t switch_changes{};'),
    'vehicle_attitude':('VehicleAttitude',0,'std::uint64_t timestamp_sample{};std::array<float,4> q{};std::array<float,4> delta_q_reset{};std::uint8_t quat_reset_counter{};'),
    'vehicle_attitude_setpoint':('VehicleAttitudeSetpoint',0,'float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};'),
    'offboard_control_mode':('OffboardControlMode',None,'bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};'),
    'vehicle_command':('VehicleCommand',0,'static constexpr std::uint32_t VEHICLE_CMD_DO_SET_MODE=176,VEHICLE_CMD_COMPONENT_ARM_DISARM=400,VEHICLE_CMD_NAV_LAND=21;std::uint32_t command{};float param1{},param2{};std::uint8_t target_system{},target_component{},source_system{},source_component{};bool from_external{};'),
    'vehicle_command_ack':('VehicleCommandAck',0,'std::uint32_t command{};std::uint8_t result{},target_system{};std::uint16_t target_component{};bool from_external{};'),
    'vehicle_status':('VehicleStatus',1,'static constexpr int HIL_STATE_OFF=0,NAVIGATION_STATE_STAB=15,NAVIGATION_STATE_OFFBOARD=14,NAVIGATION_STATE_AUTO_LAND=18,NAVIGATION_STATE_TERMINATION=13,ARMING_STATE_DISARMED=1,ARMING_STATE_ARMED=2;bool calibration_enabled{},rc_calibration_in_progress{},pre_flight_checks_pass{},power_input_valid{},failsafe{},failsafe_and_user_took_over{};std::uint8_t system_id{},component_id{},hil_state{},nav_state{},arming_state{},latest_arming_reason{};std::uint64_t armed_time{};std::uint32_t can_set_nav_states_mask{};'),
    'vehicle_land_detected':('VehicleLandDetected',0,'bool at_rest{},landed{};'),
    'battery_status':('BatteryStatus',0,'int id{},warning{};float voltage_v{};bool connected{};'),
    'failsafe_flags':('FailsafeFlags',0,'bool manual_control_signal_lost{},offboard_control_signal_lost{},fd_critical_failure{};'),
    'vehicle_local_position':('VehicleLocalPosition',0,'std::uint64_t timestamp_sample{};float x{},y{},z{},vx{},vy{},vz{},z_deriv{},ax{},ay{},az{},heading{};bool xy_valid{},z_valid{},v_xy_valid{},v_z_valid{},heading_good_for_control{},dead_reckoning{};std::uint8_t xy_reset_counter{},z_reset_counter{},vxy_reset_counter{},vz_reset_counter{},heading_reset_counter{};'),
    'trajectory_setpoint':('TrajectorySetpoint',0,'std::array<float,3> position{},velocity{},acceleration{},jerk{};float yaw{},yawspeed{};'),
}
# 视觉消息替身保持类型形状，用来发现适配器字段名或数组长度的编译错误。
VISION_MESSAGES={
    'vision_state':('VisionState', 'std::uint16_t valid_fields{};std::uint64_t sequence{};Stamp stamp;std::uint32_t reset_counter{};std::string frame_id,body_frame_id,source_id,angular_velocity_source;bool initialized{},tracking{},calibration_valid{};std::array<double,3> position{},velocity{},angular_velocity{};std::array<double,4> attitude{};std::array<double,36> pose_covariance{},twist_covariance{};double quality{};'),
    'navigation_command':('NavigationCommand', 'std::uint64_t sequence{};Stamp stamp;std::uint32_t reset_counter{};std::string source_id,frame_id;std::uint8_t mode{};bool valid{},position_valid{},yaw_valid{},sticks_valid{};std::array<double,3> position{};double yaw{};std::array<double,4> sticks{};'),
}
# 在临时目录生成替身头文件，随后只做 -fsyntax-only 检查；退出时清理全部生成物。
def main():
    with tempfile.TemporaryDirectory(prefix='g_adapter_api_') as directory:
        root=Path(directory);(root/'rclcpp').mkdir();(root/'rclcpp/rclcpp.hpp').write_text(RCLCPP)
        msg=root/'px4_msgs/msg';msg.mkdir(parents=True)
        for filename,(name,version,fields) in MESSAGES.items():
            const='' if version is None else f'static constexpr int MESSAGE_VERSION={version};'
            (msg/(filename+'.hpp')).write_text('#pragma once\n#include <array>\n#include <cstdint>\n#include <memory>\nnamespace px4_msgs::msg {struct '+name+'{using ConstSharedPtr=std::shared_ptr<const '+name+'>;std::uint64_t timestamp{};'+const+fields+'};}\n')
        vision=root/'jikl_fc01_system/msg';vision.mkdir(parents=True)
        (vision/'stamp.hpp').write_text('#pragma once\n#include <cstdint>\nnamespace jikl_fc01_system::msg {struct Stamp{std::int32_t sec{};std::uint32_t nanosec{};};}\n')
        for filename,(name,fields) in VISION_MESSAGES.items():
            (vision/(filename+'.hpp')).write_text('#pragma once\n#include <array>\n#include <cstdint>\n#include <memory>\n#include <string>\n#include "jikl_fc01_system/msg/stamp.hpp"\nnamespace jikl_fc01_system::msg {struct '+name+'{using ConstSharedPtr=std::shared_ptr<const '+name+'>;std::uint16_t schema_version{1};'+fields+'};}\n')
        # 直接编译当前工程的两个真实入口，开启严格告警；不会生成可执行文件。
        for source in ('fc01_system_node.cpp','fc01_config_check.cpp'):
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-fsyntax-only','-I',str(root),'-I',str(ROOT/'src/jikl_fc01_system/include'),str(ROOT/'src/jikl_fc01_system/src'/source)],check=True)
    print('actual ROS adapter syntax against type/API fixture: PASS; real Humble build/link and DDS: UNKNOWN')
if __name__=='__main__':main()
