#!/usr/bin/env python3
"""Compile the actual adapter against a narrow type/API fixture.

This is NOT a ROS/Humble build, DDS test, or verification of the exact vendor
message source. No executable or ROS connection is created. Real build UNKNOWN.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
RCLCPP=r'''#pragma once
#include <cstdint>
#include <memory>
#include <string>
namespace rclcpp {
struct Parameter {bool as_bool() const{return false;}};
struct Time {std::int64_t nanoseconds() const{return 1000000000;}};
struct Clock {Time now() const{return {};}};
struct SensorDataQoS {SensorDataQoS keep_last(int) const{return {};}};
template<class T>struct Subscription {using SharedPtr=std::shared_ptr<Subscription<T>>;};
template<class T>struct Publisher {using SharedPtr=std::shared_ptr<Publisher<T>>;void publish(const T&){};};
class Node {
 public:
  explicit Node(const std::string&){}
  Parameter get_parameter(const std::string&) const{return {};}
  template<class T>T declare_parameter(const std::string&,const T& value){return value;}
  template<class T,class Callback>typename Subscription<T>::SharedPtr create_subscription(const std::string&,SensorDataQoS,Callback){return std::make_shared<Subscription<T>>();}
  template<class T>typename Publisher<T>::SharedPtr create_publisher(const std::string&,int){return std::make_shared<Publisher<T>>();}
  std::shared_ptr<Clock> get_clock(){return std::make_shared<Clock>();}
  std::size_t count_publishers(const std::string&) const{return 1;}
  std::size_t count_subscribers(const std::string&) const{return 1;}
};
inline void init(int,char**){}
inline void shutdown(){}
inline bool ok(){return false;}
template<class T>void spin_some(const T&){}
}
'''
MESSAGES={
    'manual_control_setpoint':('ManualControlSetpoint',0,'static constexpr std::uint8_t SOURCE_UNKNOWN=0,SOURCE_RC=1,SOURCE_MAVLINK_0=2,SOURCE_MAVLINK_1=3,SOURCE_MAVLINK_2=4,SOURCE_MAVLINK_3=5,SOURCE_MAVLINK_4=6,SOURCE_MAVLINK_5=7;std::uint64_t timestamp_sample{};bool valid{};std::uint8_t data_source{};float roll{},pitch{},yaw{},throttle{},flaps{},aux1{},aux2{},aux3{},aux4{},aux5{},aux6{};bool sticks_moving{};std::uint16_t buttons{};'),
    'manual_control_switches':('ManualControlSwitches',None,'std::uint64_t timestamp_sample{};std::uint8_t mode_slot{},arm_switch{},return_switch{},loiter_switch{},offboard_switch{},kill_switch{};std::uint32_t switch_changes{};'),
    'vehicle_attitude':('VehicleAttitude',0,'std::uint64_t timestamp_sample{};std::array<float,4> q{};std::array<float,4> delta_q_reset{};std::uint8_t quat_reset_counter{};'),
    'vehicle_attitude_setpoint':('VehicleAttitudeSetpoint',0,'float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};'),
    'offboard_control_mode':('OffboardControlMode',None,'bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};'),
    'vehicle_command':('VehicleCommand',0,'static constexpr std::uint32_t VEHICLE_CMD_DO_SET_MODE=176,VEHICLE_CMD_COMPONENT_ARM_DISARM=400,VEHICLE_CMD_NAV_LAND=21;std::uint32_t command{};float param1{},param2{};std::uint8_t target_system{},target_component{},source_system{},source_component{};bool from_external{};'),
    'vehicle_command_ack':('VehicleCommandAck',0,'std::uint32_t command{};std::uint8_t result{},target_system{};std::uint16_t target_component{};bool from_external{};'),
    'vehicle_status':('VehicleStatus',1,'static constexpr int HIL_STATE_OFF=0,NAVIGATION_STATE_STAB=15,NAVIGATION_STATE_OFFBOARD=14,NAVIGATION_STATE_AUTO_LAND=18,NAVIGATION_STATE_TERMINATION=13,ARMING_STATE_DISARMED=1,ARMING_STATE_ARMED=2;bool calibration_enabled{},rc_calibration_in_progress{},pre_flight_checks_pass{},power_input_valid{},failsafe{},failsafe_and_user_took_over{};std::uint8_t system_id{},component_id{},hil_state{},nav_state{},arming_state{};std::uint32_t can_set_nav_states_mask{};'),
    'vehicle_land_detected':('VehicleLandDetected',0,'bool at_rest{},landed{};'),
    'battery_status':('BatteryStatus',0,'int id{},warning{};float voltage_v{};bool connected{};'),
    'failsafe_flags':('FailsafeFlags',0,'bool manual_control_signal_lost{},offboard_control_signal_lost{},fd_critical_failure{};'),
    'vehicle_local_position':('VehicleLocalPosition',0,'std::uint64_t timestamp_sample{};float x{},y{},z{},vx{},vy{},vz{},ax{},ay{},az{},heading{};bool xy_valid{},z_valid{},v_xy_valid{},v_z_valid{},heading_good_for_control{},dead_reckoning{};std::uint8_t xy_reset_counter{},z_reset_counter{},vxy_reset_counter{},vz_reset_counter{},heading_reset_counter{};'),
    'trajectory_setpoint':('TrajectorySetpoint',0,'std::array<float,3> position{},velocity{},acceleration{},jerk{};float yaw{},yawspeed{};'),
}
def main():
    with tempfile.TemporaryDirectory(prefix='g_adapter_api_') as directory:
        root=Path(directory);(root/'rclcpp').mkdir();(root/'rclcpp/rclcpp.hpp').write_text(RCLCPP)
        msg=root/'px4_msgs/msg';msg.mkdir(parents=True)
        for filename,(name,version,fields) in MESSAGES.items():
            const='' if version is None else f'static constexpr int MESSAGE_VERSION={version};'
            (msg/(filename+'.hpp')).write_text('#pragma once\n#include <array>\n#include <cstdint>\n#include <memory>\nnamespace px4_msgs::msg {struct '+name+'{using ConstSharedPtr=std::shared_ptr<const '+name+'>;std::uint64_t timestamp{};'+const+fields+'};}\n')
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-fsyntax-only','-I',str(root),'-I',str(ROOT/'src/jikl_fc01_system/include'),str(ROOT/'src/jikl_fc01_system/src/fc01_system_node.cpp')],check=True)
    print('actual ROS adapter syntax against type/API fixture: PASS; real Humble build/link and DDS: UNKNOWN')
if __name__=='__main__':main()
