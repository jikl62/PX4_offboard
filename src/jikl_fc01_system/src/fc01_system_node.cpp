#include "jikl_fc01_system/runtime.hpp"
#include "jikl_fc01_system/unix_server.hpp"
#include "jikl_fc01_system/attitude_ros_parameters.hpp"
#include "jikl_fc01_system/attitude_wire.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include "jikl_fc01_system/publication_audit.hpp"
#include "jikl_fc01_system/authority_telemetry.hpp"
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/manual_control_switches.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

namespace f=fc01;namespace v=jikl_velocity;
using Local=px4_msgs::msg::VehicleLocalPosition;
using Status=px4_msgs::msg::VehicleStatus;
using Land=px4_msgs::msg::VehicleLandDetected;
using Battery=px4_msgs::msg::BatteryStatus;
using Flags=px4_msgs::msg::FailsafeFlags;
using Ack=px4_msgs::msg::VehicleCommandAck;
using Command=px4_msgs::msg::VehicleCommand;
using Mode=px4_msgs::msg::OffboardControlMode;
using Target=px4_msgs::msg::TrajectorySetpoint;
using Attitude=px4_msgs::msg::VehicleAttitude;
using AttitudeTarget=px4_msgs::msg::VehicleAttitudeSetpoint;
using Manual=px4_msgs::msg::ManualControlSetpoint;
using Switches=px4_msgs::msg::ManualControlSwitches;
static_assert(Status::MESSAGE_VERSION==1&&Local::MESSAGE_VERSION==0&&Land::MESSAGE_VERSION==0&&Ack::MESSAGE_VERSION==0&&Command::MESSAGE_VERSION==0&&Target::MESSAGE_VERSION==0,"FC01 requires pinned PX4 v1.16.0 message definitions");
static_assert(Attitude::MESSAGE_VERSION==0&&AttitudeTarget::MESSAGE_VERSION==0&&Manual::MESSAGE_VERSION==0,"FC01 attitude/manual interfaces require pinned PX4 v1.16 messages");
namespace {
volatile std::sig_atomic_t stop_requested=0;
void stop_handler(int){stop_requested=1;}
double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string number(double value,bool valid=true){if(!valid||!std::isfinite(value))return "null";std::ostringstream out;out<<std::setprecision(9)<<value;return out.str();}
const char* boolean(bool value){return value?"true":"false";}
std::string default_socket(){const char* env=std::getenv("FC01_SOCKET");if(env)return env;const char* home=std::getenv("HOME");if(!home)throw std::runtime_error("HOME missing");return std::string(home)+"/.cache/fc01/control.sock";}
}
class SystemNode final:public rclcpp::Node {
 public:
  SystemNode():Node("fc01_system"){
    if(get_parameter("use_sim_time").as_bool())throw std::runtime_error("use_sim_time must be false");
    f::Config config;
    config.profile=declare_parameter<std::string>("profile","ground");
    config.flight_enabled=declare_parameter<bool>("flight_enabled",false);
    config.control.max_height=declare_parameter<double>("max_height_m",5.);
    config.control.max_radius=declare_parameter<double>("max_radius_m",3.);
    config.control.xy_speed=declare_parameter<double>("max_xy_speed",.3);
    config.control.z_speed=declare_parameter<double>("max_z_speed",.3);
    config.control.climb_speed=config.control.z_speed;
    config.control.yaw_speed=declare_parameter<double>("max_yaw_rate",.5);
    config.attitude=f::attitude_parameters(*this);
    config.validate();runtime_=std::make_unique<f::Runtime>(config);runtime_->require_attitude_sink_commit();config_=config;
    const std::string socket=declare_parameter<std::string>("socket_path",default_socket());
    const auto parent=std::filesystem::path(socket).parent_path();
    std::filesystem::create_directories(parent);
    struct stat parent_stat{};
    if(lstat(parent.c_str(),&parent_stat)!=0||!S_ISDIR(parent_stat.st_mode)||parent_stat.st_uid!=getuid()||(parent_stat.st_mode&0022))throw std::runtime_error("socket directory must be owned and not writable by other users");
    server_=std::make_unique<f::UnixServer>(socket);
    const auto qos=rclcpp::SensorDataQoS().keep_last(10);
    if(config.profile=="attitude_altitude"){
      attitude_sub_=create_subscription<Attitude>("/fmu/out/vehicle_attitude",qos,[this](Attitude::ConstSharedPtr m){receive(*m,attitude_,attitude_stamp_);});
      manual_sub_=create_subscription<Manual>("/fmu/out/manual_control_setpoint",qos,[this](Manual::ConstSharedPtr m){authority_telemetry_.manual(*m,steady());});
      switches_sub_=create_subscription<Switches>("/fmu/out/manual_control_switches",qos,[this](Switches::ConstSharedPtr m){authority_telemetry_.switches(*m,steady(),config_.attitude.flags_timeout);});
    }
    local_sub_=create_subscription<Local>("/fmu/out/vehicle_local_position",qos,[this](Local::ConstSharedPtr m){receive(*m,local_,local_stamp_);});
    status_sub_=create_subscription<Status>("/fmu/out/vehicle_status_v1",qos,[this](Status::ConstSharedPtr m){receive(*m,status_,status_stamp_);});
    land_sub_=create_subscription<Land>("/fmu/out/vehicle_land_detected",qos,[this](Land::ConstSharedPtr m){receive(*m,land_,land_stamp_);});
    battery_sub_=create_subscription<Battery>("/fmu/out/battery_status",qos,[this](Battery::ConstSharedPtr m){if(m->id==1)receive(*m,battery_,battery_stamp_);});
    flags_sub_=create_subscription<Flags>("/fmu/out/failsafe_flags",qos,[this](Flags::ConstSharedPtr m){receive(*m,flags_,flags_stamp_);});
    ack_sub_=create_subscription<Ack>("/fmu/out/vehicle_command_ack",qos,[this](Ack::ConstSharedPtr m){runtime_->ack({m->timestamp,m->command,m->result,m->target_system,m->target_component,m->from_external},steady());
      std::cout<<"{\"kind\":\"ack\",\"command\":"<<m->command<<",\"result\":"<<int(m->result)<<",\"timestamp\":"<<m->timestamp<<",\"target_system\":"<<int(m->target_system)<<",\"target_component\":"<<m->target_component<<",\"from_external\":"<<boolean(m->from_external)<<"}"<<std::endl;});
    command_pub_=create_publisher<Command>("/fmu/in/vehicle_command",10);
    const auto publishers=f::attitude::publisher_plan(config.profile);
    if(publishers.mode)mode_pub_=create_publisher<Mode>("/fmu/in/offboard_control_mode",10);
    if(publishers.trajectory)target_pub_=create_publisher<Target>("/fmu/in/trajectory_setpoint",10);
    if(publishers.attitude)attitude_pub_=create_publisher<AttitudeTarget>("/fmu/in/vehicle_attitude_setpoint",10);
    const auto logpath=declare_parameter<std::string>("csv_path","");
    if(!logpath.empty()){
      csv_.open(logpath,std::ios::app);
      if(!csv_)throw std::runtime_error("cannot open CSV log");
      csv_<<"steady_s,phase,nav_state,armed,landed,xy_valid,z_valid,vxy_valid,vz_valid,heading_good,local_age_s,x_ned,y_ned,z_ned,vx_ned,vy_ned,vz_ned,ax_ned,ay_ned,az_ned,relative_height_m,reason,aa_healthy,aa_yaw_reference,aa_thrust_z,aa_height_target,aa_velocity_target,aa_saturated,aa_slew_limited\n";
    }
    std::cout<<"{\"kind\":\"startup\",\"profile\":"<<f::quote(config.profile)<<",\"flight_enabled\":"<<boolean(config.flight_enabled)<<",\"automatic_arm\":false,\"socket\":"<<f::quote(socket)<<"}"<<std::endl;
  }
  std::chrono::duration<double> loop_period() const{return std::chrono::duration<double>(1./config_.attitude.loop_hz);}
  bool tick(){
    const double now=steady();
    publication_audit_.begin(now);
    const std::string phase_before=runtime_->phase_name();
    control_started_=now;
    const auto wall_stamp=timestamp();
    if(last_wall_stamp_&&wall_stamp<last_wall_stamp_)clock_regressed_=true;
    last_wall_stamp_=wall_stamp;auto s=snapshot(now);
    bool resting=s.status_fresh&&s.land_fresh&&!s.feedback.armed&&s.feedback.landed&&s.at_rest;
    if(config_.profile=="attitude_altitude"){
      resting=resting&&s.local_fresh&&local_.z_valid&&local_.v_z_valid&&std::isfinite(local_.z)&&std::isfinite(local_.vz)&&std::abs(local_.vz)<=config_.attitude.ground_vz_tolerance;
      if(local_.z_reset_counter!=ground_z_reset_||local_.vz_reset_counter!=ground_vz_reset_)rest_since_=-1;
      ground_z_reset_=local_.z_reset_counter;ground_vz_reset_=local_.vz_reset_counter;
    }
    if(resting){if(rest_since_<0)rest_since_=now;s.stationary_seconds=now-rest_since_;}
    else rest_since_=-1;
    if(resting&&s.local_fresh&&local_.z_valid&&!reference_set_&&s.stationary_seconds>=2){reference_z_=local_.z;reference_set_=true;}
    if(config_.profile=="attitude_altitude"){
      if(runtime_->attitude_core().reference_ready()){reference_z_=runtime_->attitude_core().reference_z();reference_set_=true;}
    }else if(runtime_->active()){reference_z_=runtime_->flight_core().origin().z;reference_set_=true;}
    if(stop_requested)begin_shutdown(s,now);
    server_->poll([this,&s,now](const f::Request& req){
      if(stop_requested)begin_shutdown(s,now);
      auto reply=runtime_->request(req.op,req.values,s,now);
      if(req.op=="STOP"&&reply.accepted){stop_requested=1;begin_shutdown(s,now);}
      std::string out=state(s,now,"reply",req.seq,reply.accepted,reply.message);
      if(req.op!="PING")std::cout<<out<<std::endl;
      return out;
    },[this,&s,now]{runtime_->disconnect(s,now);});
    auto output=runtime_->step(s,now);
    if(config_.profile=="attitude_altitude"&&steady()-now>config_.attitude.stall_seconds){
      runtime_->abort_attitude("ROS_ADAPTER_CONTROL_LOOP_STALL");output={};
    }
    last_attitude_output_={};
    if(boundary_fault_latched_)output={};
    // Never put a 176/400 command on the wire before its companion final
    // setpoint sink has approved the same tick.  In particular this prevents
    // a denied attitude sample from preceding an untracked Arm/Disarm.
    const auto requested_command=output.attitude.command;
    const auto attempted_attitude=output.attitude;
    bool attitude_sent=!output.attitude.publish;
    if(output.attitude.publish){
      try{
        const auto decision=f::attitude::publish_attitude_at_boundary<Mode,AttitudeTarget>(
          boundary_context(),output.attitude,timestamp(),
          [this,&output](const AttitudeTarget& target){
            if(!f::attitude::attitude_publication_policy(boundary_context(),output.attitude).allowed)
              throw std::runtime_error("TARGET_FINAL_CONTEXT_CHANGED");
            attitude_pub_->publish(target);publication_audit_.target_committed(steady(),target.thrust_body[2]);},
          [this,&output](const Mode& mode){
            // Re-sample after target publish; it can block/throw independently.
            if(!f::attitude::attitude_publication_policy(boundary_context(),output.attitude).allowed)
              throw std::runtime_error("HEARTBEAT_FINAL_CONTEXT_CHANGED");
            mode_pub_->publish(mode);publication_audit_.mode_committed(steady());});
        if(decision.allowed){last_attitude_output_=output.attitude;attitude_sent=true;}
        else {refuse_publication("attitude",decision);output={};}
      }catch(const std::exception& e){
        publication_audit_.failed();attitude_sent=false;boundary_fault_latched_=true;
        publication_block_reason_=std::string("PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_")+e.what();
        runtime_->abort_attitude(publication_block_reason_);output={};
      }catch(...){
        publication_audit_.failed();attitude_sent=false;boundary_fault_latched_=true;
        publication_block_reason_="PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_UNKNOWN_EXCEPTION";
        runtime_->abort_attitude(publication_block_reason_);output={};
      }
    }
    bool command_sent=false;
    std::uint64_t command_stamp=0;
    if(requested_command!=f::attitude::Command::None&&attitude_sent&&!boundary_fault_latched_){
      if(requested_command==f::attitude::Command::Offboard)
        command_sent=publish_command(Command::VEHICLE_CMD_DO_SET_MODE,1,6,&command_stamp);
      else if(requested_command==f::attitude::Command::ReleaseOffboard)
        command_sent=publish_command(Command::VEHICLE_CMD_DO_SET_MODE,1,7,&command_stamp);
      else if(requested_command==f::attitude::Command::Arm)
        command_sent=publish_command(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM,1,0,&command_stamp);
      else if(requested_command==f::attitude::Command::Disarm)
        command_sent=publish_command(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM,0,0,&command_stamp);
      else if(requested_command==f::attitude::Command::Px4Land)
        command_sent=publish_command(Command::VEHICLE_CMD_NAV_LAND,0,0,&command_stamp);
    }
    if(requested_command!=f::attitude::Command::None){
      const auto commit_snapshot=snapshot(steady());
      runtime_->confirm_attitude_command_publication(requested_command,command_sent,command_stamp,commit_snapshot,steady());
      std::cout<<"{\"kind\":\"command_sink_audit\",\"requested\":true,\"published\":"<<boolean(command_sent)
        <<",\"monotonic_s\":"<<number(steady())
        <<",\"command\":"<<(requested_command==f::attitude::Command::Px4Land?21:
          requested_command==f::attitude::Command::Offboard||requested_command==f::attitude::Command::ReleaseOffboard?176:400)
        <<",\"wire_timestamp\":"<<command_stamp<<",\"status_watermark\":"<<commit_snapshot.status_stamp
        <<",\"land_watermark\":"<<commit_snapshot.land_stamp<<"}"<<std::endl;
      if(!command_sent&&!boundary_fault_latched_){
        runtime_->abort_attitude("COMMAND_FINAL_SINK_NOT_PUBLISHED");
        boundary_fault_latched_=true;publication_block_reason_="COMMAND_FINAL_SINK_NOT_PUBLISHED";
      }
    }
    if(output.send_ground)publish_command(Command::VEHICLE_CMD_DO_SET_MODE,1,7);
    if(output.flight.command==v::VehicleCommand::Offboard)publish_command(Command::VEHICLE_CMD_DO_SET_MODE,1,6);
    else if(output.flight.command==v::VehicleCommand::Arm){
      if(!publish_command(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM,1,0))output={};
    }
    else if(output.flight.command==v::VehicleCommand::Land)publish_command(Command::VEHICLE_CMD_NAV_LAND,0,0);
    if(output.flight.control!=v::Control::None){
      Mode mode{};mode.timestamp=timestamp();mode.position=output.flight.control==v::Control::Position;mode.velocity=output.flight.control==v::Control::Velocity;
      Target target{};target.timestamp=mode.timestamp;target.position.fill(NAN);target.velocity.fill(NAN);target.acceleration.fill(NAN);target.jerk.fill(NAN);
      if(mode.position)target.position={float(output.flight.position.x),float(output.flight.position.y),float(output.flight.position.z)};
      if(mode.velocity)target.velocity={float(output.flight.velocity.x),float(output.flight.velocity.y),float(output.flight.velocity.z)};
      target.yaw=float(output.flight.yaw);target.yawspeed=float(output.flight.yaw_rate);
      target_pub_->publish(target);mode_pub_->publish(mode);
    }
    if(config_.profile=="attitude_altitude")write_phase_event(phase_before,snapshot(steady()),attempted_attitude,now);
    if(now-printed_>=.5){printed_=now;auto line=state(s,now);server_->publish(line);std::cout<<line<<std::endl;write_csv(s,now);}
    if(stopping_){
      if(s.status_fresh&&s.land_fresh&&!s.feedback.armed&&s.feedback.landed&&!runtime_->active())return false;
      if(now-stop_since_>=60){
        // A healthy, still-armed PX4 Offboard powered-ground recovery must
        // NEVER lose setpoint custody merely because the shutdown timer fired.
        // Fatal faults / hard process termination remain PX4 failsafe-dependent.
        if(config_.profile=="attitude_altitude"&&s.status_fresh&&s.feedback.armed&&
           s.feedback.offboard&&runtime_->attitude_core().active()&&!boundary_fault_latched_){
          if(now-last_shutdown_warning_>=5){last_shutdown_warning_=now;
            std::cerr<<"Shutdown wait exceeded 60s: armed Offboard still active; continuing controlled outputs; operator intervention required\n";}
        }else{
          std::cerr<<"Shutdown timed out: PX4/operator must complete recovery; no re-arm or forced disarm\n";
          return false;
        }
      }
    }
    return true;
  }
 private:
  void begin_shutdown(const f::Snapshot& s,double now){
    if(stopping_)return;
    stopping_=true;stop_since_=now;runtime_->begin_shutdown(s,now);
  }
  template<class T>void receive(const T& msg,T& destination,f::Stamp& stamp){if(stamp.accept(msg.timestamp,steady()))destination=msg;}
  std::uint64_t timestamp(){return std::uint64_t(get_clock()->now().nanoseconds()/1000);}
  f::attitude::BoundaryContext boundary_context(){
    const double now=steady();auto s=snapshot(now);
    s.stationary_seconds=rest_since_>=0?std::max(0.,now-rest_since_):0;
    return runtime_->publication_context(s,now,server_->connected(),std::isfinite(now)&&now>=control_started_&&now-control_started_<=config_.attitude.stall_seconds,
      stop_requested||stopping_||boundary_fault_latched_);
  }
  void refuse_publication(const char* boundary,const f::attitude::Decision& decision){
    boundary_fault_latched_=true;
    publication_block_reason_=std::string("PUBLISHER_BOUNDARY_")+f::attitude::name(decision.reason);
    runtime_->abort_attitude(publication_block_reason_);last_attitude_output_={};
    std::cerr<<"{\"kind\":\"publication_blocked\",\"boundary\":"<<f::quote(boundary)
      <<",\"reason\":"<<f::quote(publication_block_reason_)<<",\"heartbeat_enabled\":false,\"attitude_output_enabled\":false}"<<std::endl;
  }
  bool publish_command(std::uint32_t id,float p1,float p2,std::uint64_t* actual_stamp=nullptr){
    Command cmd{};cmd.timestamp=timestamp();cmd.command=id;cmd.param1=p1;cmd.param2=p2;
    cmd.target_system=1;cmd.target_component=1;cmd.source_system=1;cmd.source_component=191;cmd.from_external=true;
    // This unconditional final sink check also covers future callers and the
    // legacy flight/sim route. Those profiles cannot bypass the Arm contract.
    if(id==Command::VEHICLE_CMD_COMPONENT_ARM_DISARM&&p1==1){
      const auto decision=f::attitude::publish_arm_at_boundary(boundary_context(),[this,&cmd]{command_pub_->publish(cmd);});
      if(!decision.allowed){refuse_publication("arm",decision);return false;}
    }else if(id==Command::VEHICLE_CMD_COMPONENT_ARM_DISARM&&p1==0){
      const auto decision=f::attitude::publish_disarm_at_boundary(boundary_context(),
        [this,&cmd]{command_pub_->publish(cmd);});
      if(!decision.allowed){refuse_publication("disarm",decision);return false;}
    }else if(id==Command::VEHICLE_CMD_DO_SET_MODE&&p1==1&&p2==7&&
             config_.profile=="attitude_altitude"){
      const auto decision=f::attitude::publish_release_at_boundary(boundary_context(),
        [this,&cmd]{command_pub_->publish(cmd);});
      if(!decision.allowed){refuse_publication("release",decision);return false;}
    }else if(id==Command::VEHICLE_CMD_NAV_LAND&&config_.profile=="attitude_altitude"){
      const auto decision=f::attitude::publish_land_fallback_at_boundary(boundary_context(),
        [this,&cmd]{command_pub_->publish(cmd);});
      if(!decision.allowed){refuse_publication("land_fallback",decision);return false;}
    }else command_pub_->publish(cmd);
    if(actual_stamp)*actual_stamp=cmd.timestamp;
    std::cout<<"{\"kind\":\"send\",\"command\":"<<id<<",\"param1\":"<<p1<<",\"param2\":"<<p2<<"}"<<std::endl;
    return true;
  }
  f::Snapshot snapshot(double now){
    f::Snapshot s;s.status_fresh=status_stamp_.fresh(now,config_.attitude.status_timeout);s.land_fresh=land_stamp_.fresh(now,config_.attitude.land_timeout);s.local_fresh=local_stamp_.fresh(now,config_.attitude.local_timeout);
    s.arming_state_known=status_.arming_state==Status::ARMING_STATE_ARMED||
      status_.arming_state==Status::ARMING_STATE_DISARMED;
    s.status_fresh=s.status_fresh&&s.arming_state_known;
    s.attitude_output_active=s.offboard_heartbeat_active=last_attitude_output_.publish;
    s.publication_fault_latched=boundary_fault_latched_;
    s.battery_fresh=battery_stamp_.fresh(now,1.);s.manual_fresh=flags_stamp_.fresh(now,config_.attitude.flags_timeout);
    s.time_regressed=status_stamp_.regressed||land_stamp_.regressed||local_stamp_.regressed||battery_stamp_.regressed||flags_stamp_.regressed||clock_regressed_||attitude_stamp_.regressed;
    s.status_stamp=status_stamp_.value;s.land_stamp=land_stamp_.value;s.system=status_.system_id;s.component=status_.component_id;
    s.at_rest=land_.at_rest;s.calibration=status_.calibration_enabled||status_.rc_calibration_in_progress;s.hil=status_.hil_state!=Status::HIL_STATE_OFF;
    s.stabilized_selectable=(status_.can_set_nav_states_mask&(1U<<Status::NAVIGATION_STATE_STAB))!=0;
    s.attitude.battery_voltage=battery_.voltage_v;
    s.battery_connected=battery_.connected;s.battery_warning=battery_.warning;s.power_valid=status_.power_input_valid;s.manual_lost=flags_.manual_control_signal_lost;s.dead_reckoning=local_.dead_reckoning;s.nav_state=status_.nav_state;
    auto& fb=s.feedback;fb.position={local_.x,local_.y,local_.z};fb.velocity={local_.vx,local_.vy,local_.vz};fb.yaw=local_.heading;
    fb.fresh=s.status_fresh&&s.land_fresh&&s.local_fresh;
    fb.valid=local_.xy_valid&&local_.z_valid&&local_.v_xy_valid&&local_.v_z_valid&&v::finite(fb.position)&&v::finite(fb.velocity)&&std::isfinite(fb.yaw);
    fb.heading_ok=local_.heading_good_for_control;fb.preflight=status_.pre_flight_checks_pass;fb.armed=status_.arming_state==Status::ARMING_STATE_ARMED;fb.landed=land_.landed;
    fb.offboard=s.nav_state==Status::NAVIGATION_STATE_OFFBOARD;fb.landing=s.nav_state==Status::NAVIGATION_STATE_AUTO_LAND;fb.failsafe=status_.failsafe;
    // Each packed counter occupies its own byte. All position/velocity/heading resets terminate the mission.
    fb.reset_id=std::uint32_t(local_.xy_reset_counter)|(std::uint32_t(local_.z_reset_counter)<<8)|(std::uint32_t(local_.vxy_reset_counter)<<16)|(std::uint32_t(local_.vz_reset_counter)<<24);
    if(config_.profile!="attitude_altitude"&&local_.heading_reset_counter!=heading_reset_){heading_reset_=local_.heading_reset_counter;if(runtime_->active())heading_changed_=true;}
    s.time_regressed=s.time_regressed||heading_changed_;
    s.traffic={count_publishers("/fmu/out/vehicle_status_v1"),count_publishers("/fmu/out/vehicle_land_detected"),count_publishers("/fmu/out/vehicle_command_ack"),count_subscribers("/fmu/in/vehicle_command"),count_publishers("/fmu/in/vehicle_command")};
    fb.exclusive=s.system==1&&s.component==1&&s.traffic.status_publishers==1&&s.traffic.land_publishers==1&&s.traffic.ack_publishers==1&&s.traffic.command_subscribers==1&&s.traffic.command_publishers==1&&count_publishers("/fmu/out/vehicle_local_position")==1;
    f::attitude::OwnershipCounts ownership;
    ownership.mode_publishers=count_publishers("/fmu/in/offboard_control_mode");ownership.mode_subscribers=count_subscribers("/fmu/in/offboard_control_mode");
    ownership.trajectory_publishers=count_publishers("/fmu/in/trajectory_setpoint");ownership.trajectory_subscribers=count_subscribers("/fmu/in/trajectory_setpoint");
    ownership.attitude_publishers=count_publishers("/fmu/in/vehicle_attitude_setpoint");ownership.attitude_subscribers=count_subscribers("/fmu/in/vehicle_attitude_setpoint");
    for(const char* topic:{"/fmu/in/vehicle_rates_setpoint","/fmu/in/vehicle_thrust_setpoint","/fmu/in/vehicle_torque_setpoint","/fmu/in/actuator_motors"})ownership.other_publishers+=count_publishers(topic);
    fb.exclusive=fb.exclusive&&f::attitude::owns_setpoints(config_.profile,ownership);
    if(config_.profile=="attitude_altitude"){
      auto& a=s.attitude;a.attitude_fresh=attitude_stamp_.fresh(now,config_.attitude.attitude_timeout);
      a.failsafe_fresh=flags_stamp_.fresh(now,config_.attitude.flags_timeout);
      for(std::size_t i=0;i<4;++i)a.q[i]=attitude_.q[i];
      a.quat_reset=attitude_.quat_reset_counter;a.z=local_.z;a.vz=local_.vz;
      a.z_reset=local_.z_reset_counter;a.vz_reset=local_.vz_reset_counter;a.z_valid=local_.z_valid;a.vz_valid=local_.v_z_valid;
      a.local_stamp=local_stamp_.value;a.xy_valid=local_.xy_valid&&!local_.dead_reckoning&&
        std::isfinite(local_.x)&&std::isfinite(local_.y);
      a.vxy_valid=local_.v_xy_valid&&std::isfinite(local_.vx)&&std::isfinite(local_.vy);
      a.land_selectable=(status_.can_set_nav_states_mask&(1U<<Status::NAVIGATION_STATE_AUTO_LAND))!=0;
      const auto authority=authority_telemetry_.facts(now,config_.attitude.flags_timeout,
        a.failsafe_fresh,flags_.manual_control_signal_lost,
        count_publishers("/fmu/out/manual_control_setpoint"),count_publishers("/fmu/out/manual_control_switches"));
      a.rc_link_ready=authority.rc_link_ready;a.rc_switches_ready=authority.rc_switches_ready;
      a.rc_source_seen=authority.manual_seen;a.rc_source_valid=authority.manual_valid;
      a.rc_source_selected=authority.rc_source_selected;a.rc_data_source=authority.data_source;
      a.rc_sticks_moving=authority.sticks_moving;a.rc_switches_seen=authority.switches_seen;
      a.rc_switches_fresh=authority.switches_fresh;a.rc_mode_slot=authority.mode_slot;
      a.rc_arm_switch=authority.arm_switch;a.rc_kill_switch=authority.kill_switch;
      a.operator_switch_generation=authority.switch_generation;
      a.kill=authority.kill||(s.status_fresh&&status_.nav_state==Status::NAVIGATION_STATE_TERMINATION)||
        (a.failsafe_fresh&&flags_.fd_critical_failure);
      a.operator_takeover=s.status_fresh&&status_.failsafe_and_user_took_over;
      a.offboard_signal_lost=a.failsafe_fresh&&flags_.offboard_control_signal_lost;
      fb.exclusive=fb.exclusive&&count_publishers("/fmu/out/vehicle_attitude")==1;
    }
    if(config_.profile!="sim"&&(count_publishers("/fmu/out/battery_status")!=1||count_publishers("/fmu/out/failsafe_flags")!=1))fb.exclusive=false;
    return s;
  }
  void write_phase_event(const std::string& before,const f::Snapshot& s,
      const f::attitude::Output& planned,double now)const{
    const auto& core=runtime_->attitude_core();
    const auto fb=runtime_->publication_context(s,now,false,true).feedback;
    const auto& p=runtime_->pending(400);
    std::cout<<"{\"kind\":\"phase_event\",\"monotonic_s\":"<<number(now)
      <<",\"phase_before\":"<<f::quote(before)<<",\"phase_after\":"<<f::quote(runtime_->phase_name())
      <<",\"reason\":"<<f::quote(core.reason())<<",\"ownership\":"<<f::quote(core.recovery_responsibility(fb))
      <<",\"armed\":"<<boolean(s.feedback.armed)<<",\"nav_state\":"<<s.nav_state
      <<",\"status_stamp\":"<<s.status_stamp<<",\"land_stamp\":"<<s.land_stamp
      <<",\"local_stamp\":"<<s.attitude.local_stamp<<",\"landed\":"<<boolean(s.feedback.landed)
      <<",\"attitude_fresh\":"<<boolean(s.attitude.attitude_fresh)
      <<",\"local_fresh\":"<<boolean(s.local_fresh)<<",\"status_fresh\":"<<boolean(s.status_fresh)
      <<",\"land_fresh\":"<<boolean(s.land_fresh)<<",\"failsafe_fresh\":"<<boolean(s.attitude.failsafe_fresh)
      <<",\"z_valid\":"<<boolean(s.attitude.z_valid)<<",\"vz_valid\":"<<boolean(s.attitude.vz_valid)
      <<",\"fault_class\":"<<f::quote(f::attitude::name(core.fault_class()))
      <<",\"z0\":"<<number(core.reference_z(),core.reference_ready())
      <<",\"z\":"<<number(s.attitude.z,s.local_fresh)<<",\"vz\":"<<number(s.attitude.vz,s.local_fresh)
      <<",\"height\":"<<number(core.reference_z()-s.attitude.z,core.reference_ready()&&s.local_fresh)
      <<",\"thrust_planned_z\":"<<number(planned.thrust[2],planned.publish)
      <<",\"thrust_last_target_local_commit_z\":"<<number(publication_audit_.last_target_thrust_z)
      <<",\"target_local_commit\":"<<boolean(publication_audit_.target_this_tick)
      <<",\"heartbeat_local_commit\":"<<boolean(publication_audit_.mode_this_tick)
      <<",\"sink_exception\":"<<boolean(publication_audit_.exception_this_tick)
      <<",\"target_commits\":"<<publication_audit_.target_count<<",\"heartbeat_commits\":"<<publication_audit_.mode_count
      <<",\"last_target_commit_monotonic_s\":"<<number(publication_audit_.target_last,publication_audit_.target_last>=0)
      <<",\"last_heartbeat_commit_monotonic_s\":"<<number(publication_audit_.mode_last,publication_audit_.mode_last>=0)
      <<",\"max_heartbeat_gap_s\":"<<number(publication_audit_.max_mode_gap)
      <<",\"heartbeat_gap_violations\":"<<publication_audit_.mode_gap_violations
      <<",\"recovery_deadline_monotonic_s\":"<<number(core.recovery_deadline(),core.recovery_deadline()>=0)
      <<",\"recovery_budget_exhausted\":"<<boolean(core.recovery_budget_exhausted())
      <<",\"recovery_progress_observed\":"<<boolean(core.recovery_progress_observed())
      <<",\"land21_local_commit\":"<<boolean(core.px4_land_command_committed())
      <<",\"land_mode_observed\":"<<boolean(core.px4_land_mode_observed())
      <<",\"command400_generation\":"<<p.generation<<",\"command400_ack_ambiguous\":"<<boolean(p.ack_ambiguous)
      <<",\"command400_kind\":"<<f::quote(f::command400_name(p.command400_kind))
      <<",\"command400_stage\":"<<f::quote(f::command_stage_name(p.stage))
      <<",\"command400_local_commit\":"<<boolean(p.published)<<",\"command400_wire_stamp\":"<<p.wire_stamp
      <<",\"command400_ack_result\":"<<p.result
      <<",\"px4_receipt_proven\":false}"<<std::endl;
  }
  std::string state(const f::Snapshot& s,double now,const char* kind="state",std::uint64_t seq=0,bool accepted=true,const std::string& message="")const{
    std::ostringstream out;out<<"{\"kind\":"<<f::quote(kind)<<",\"seq\":"<<seq<<",\"accepted\":"<<boolean(accepted)<<",\"message\":"<<f::quote(message)
      <<",\"phase\":"<<f::quote(runtime_->phase_name())<<",\"active\":"<<boolean(runtime_->active())<<",\"profile\":"<<f::quote(config_.profile)<<",\"flight_enabled\":"<<boolean(config_.flight_enabled)
      <<','<<f::attitude::flight_gates_json(config_.attitude)
      <<','<<runtime_->authority_json(s)
      <<",\"heartbeat_enabled\":"<<boolean(last_attitude_output_.publish)<<",\"attitude_output_enabled\":"<<boolean(last_attitude_output_.publish)
      <<",\"publication_block_reason\":"<<f::quote(publication_block_reason_)<<",\"publication_fault_latched\":"<<boolean(boundary_fault_latched_)
      <<",\"attitude_fresh\":"<<boolean(s.attitude.attitude_fresh)<<",\"attitude_healthy_output\":"<<boolean(last_attitude_output_.publish)
      <<",\"yaw_reference_rad\":"<<number(runtime_->attitude_core().yaw_reference(),runtime_->attitude_core().reference_ready())
      <<",\"attitude_reset_counter\":"<<int(attitude_.quat_reset_counter)<<",\"attitude_age_s\":"<<number(now-attitude_stamp_.received,attitude_stamp_.received>=0)
      <<",\"attitude_q\":["<<number(attitude_.q[0],s.attitude.attitude_fresh)<<','<<number(attitude_.q[1],s.attitude.attitude_fresh)<<','<<number(attitude_.q[2],s.attitude.attitude_fresh)<<','<<number(attitude_.q[3],s.attitude.attitude_fresh)<<"]"
      <<",\"attitude_event\":"<<f::quote(runtime_->attitude_core().event())
      <<",\"m1_fault_class\":"<<f::quote(f::attitude::name(runtime_->attitude_core().fault_class()))
      <<",\"m1_recovery_budget_exhausted\":"<<boolean(runtime_->attitude_core().recovery_budget_exhausted())
      <<",\"m1_recovery_responsibility\":"<<f::quote(runtime_->attitude_core().recovery_responsibility(runtime_->publication_context(s,now,server_->connected(),true).feedback))
      <<",\"m1_operator_release_requested\":"<<boolean(runtime_->attitude_core().operator_release_requested())
      <<",\"m1_400_requested\":"<<boolean(runtime_->pending(400).requested)
      <<",\"m1_400_intent\":"<<f::quote(f::command400_name(runtime_->pending(400).command400_kind))
      <<",\"m1_400_transaction_stage\":"<<f::quote(f::command_stage_name(runtime_->pending(400).stage))
      <<",\"m1_400_wire_stamp\":"<<runtime_->pending(400).wire_stamp
      <<",\"m1_400_status_at_wire\":"<<runtime_->pending(400).status_at_wire
      <<",\"m1_400_ack_stamp\":"<<runtime_->pending(400).ack_stamp
      <<",\"m1_400_ack_accepted\":"<<boolean(runtime_->pending(400).accepted)
      <<",\"m1_400_ack_rejected\":"<<boolean(runtime_->pending(400).rejected)
      <<",\"m1_400_ack_ambiguous\":"<<boolean(runtime_->pending(400).ack_ambiguous)
      <<",\"m1_400_previous_unresolved\":"<<boolean(runtime_->pending(400).previous400_unresolved)
      <<",\"m1_400_local_generation\":"<<runtime_->pending(400).generation
      <<",\"m1_400_ack_observations\":"<<runtime_->pending(400).ack_observations
      <<",\"m1_400_observed_result\":"<<runtime_->pending(400).result
      <<",\"m1_400_observed_result_name\":"<<f::quote(f::ack_result_name(runtime_->pending(400).result))
      <<",\"m1_400_retained_history_count\":"<<runtime_->command400_history().size()
      <<",\"m1_400_external_review_required\":"<<boolean(runtime_->pending(400).sent&&
        runtime_->pending(400).ack_ambiguous&&!runtime_->pending(400).state_verified)
      <<",\"m1_400_state_verified\":"<<boolean(runtime_->pending(400).state_verified)
      <<",\"m1_400_verified_status_stamp\":"<<runtime_->pending(400).verified_status_stamp
      <<",\"m1_400_published\":"<<boolean(runtime_->pending(400).published)
      <<",\"m1_400_sink_denied\":"<<boolean(runtime_->pending(400).sink_denied)
      <<",\"m1_176_requested\":"<<boolean(runtime_->pending(176).requested)
      <<",\"m1_176_published\":"<<boolean(runtime_->pending(176).published)
      <<",\"attitude_setpoint_q\":["<<number(last_attitude_output_.q[0])<<','<<number(last_attitude_output_.q[1])<<','<<number(last_attitude_output_.q[2])<<','<<number(last_attitude_output_.q[3])<<"]"
      <<",\"thrust_body\":["<<number(last_attitude_output_.thrust[0])<<','<<number(last_attitude_output_.thrust[1])<<','<<number(last_attitude_output_.thrust[2])<<"]"
      <<",\"nav_state\":"<<s.nav_state<<",\"armed\":"<<boolean(s.feedback.armed)<<",\"landed\":"<<boolean(s.feedback.landed)<<",\"at_rest\":"<<boolean(s.at_rest)
      <<",\"status_fresh\":"<<boolean(s.status_fresh)<<",\"estimate_valid\":"<<boolean(s.feedback.fresh&&s.feedback.valid&&s.feedback.heading_ok&&!s.dead_reckoning)
      <<",\"arming_state_known\":"<<boolean(s.arming_state_known)<<",\"arming_state_raw\":"<<int(status_.arming_state)
      <<",\"xy_valid\":"<<boolean(s.local_fresh&&local_.xy_valid)<<",\"z_valid\":"<<boolean(s.local_fresh&&local_.z_valid)<<",\"vxy_valid\":"<<boolean(s.local_fresh&&local_.v_xy_valid)<<",\"vz_valid\":"<<boolean(s.local_fresh&&local_.v_z_valid)
      <<",\"heading_good\":"<<boolean(s.local_fresh&&local_.heading_good_for_control)<<",\"dead_reckoning\":"<<boolean(s.dead_reckoning)<<",\"preflight\":"<<boolean(s.status_fresh&&s.feedback.preflight)
      <<",\"manual_signal\":"<<boolean(s.manual_fresh&&!s.manual_lost)<<",\"battery_connected\":"<<boolean(s.battery_fresh&&s.battery_connected)<<",\"voltage_v\":"<<number(battery_.voltage_v,s.battery_fresh&&s.battery_connected)
      <<",\"local_age_s\":"<<number(now-local_stamp_.received,local_stamp_.received>=0)<<",\"position_ned\":["<<number(local_.x,s.local_fresh&&local_.xy_valid)<<','<<number(local_.y,s.local_fresh&&local_.xy_valid)<<','<<number(local_.z,s.local_fresh&&local_.z_valid)<<"]"
      <<",\"velocity_ned\":["<<number(local_.vx,s.local_fresh&&local_.v_xy_valid)<<','<<number(local_.vy,s.local_fresh&&local_.v_xy_valid)<<','<<number(local_.vz,s.local_fresh&&local_.v_z_valid)<<"]"
      <<",\"acceleration_ned_estimate\":["<<number(local_.ax,s.local_fresh&&local_.v_xy_valid)<<','<<number(local_.ay,s.local_fresh&&local_.v_xy_valid)<<','<<number(local_.az,s.local_fresh&&local_.v_z_valid)<<"]"
      <<",\"relative_height_m\":"<<number(reference_z_-local_.z,reference_set_&&s.local_fresh&&local_.z_valid)
      <<",\"ground_test\":"<<f::quote(runtime_->ground_name())<<",\"ground_reason\":"<<f::quote(runtime_->ground_reason())<<",\"reason\":"<<f::quote(runtime_->message())
      <<",\"source_counts\":["<<s.traffic.status_publishers<<','<<s.traffic.land_publishers<<','<<s.traffic.ack_publishers<<"]"
      <<",\"offboard_ack\":"<<runtime_->pending(176).result<<",\"arm_ack\":"<<runtime_->pending(400).result<<",\"land_ack\":"<<runtime_->pending(21).result<<'}';return out.str();
  }
  void write_csv(const f::Snapshot& s,double now){if(!csv_)return;csv_<<std::setprecision(9)<<now<<','<<runtime_->phase_name()<<','<<s.nav_state<<','<<s.feedback.armed<<','<<s.feedback.landed<<','<<local_.xy_valid<<','<<local_.z_valid<<','<<local_.v_xy_valid<<','<<local_.v_z_valid<<','<<local_.heading_good_for_control<<','<<number(now-local_stamp_.received,local_stamp_.received>=0)<<','<<local_.x<<','<<local_.y<<','<<local_.z<<','<<local_.vx<<','<<local_.vy<<','<<local_.vz<<','<<local_.ax<<','<<local_.ay<<','<<local_.az<<','<<number(reference_z_-local_.z,reference_set_&&s.local_fresh&&local_.z_valid)<<','<<f::quote(runtime_->message())<<','<<last_attitude_output_.publish<<','<<number(runtime_->attitude_core().yaw_reference())<<','<<last_attitude_output_.thrust[2]<<','<<last_attitude_output_.target_height<<','<<last_attitude_output_.velocity_target<<','<<last_attitude_output_.saturated<<','<<last_attitude_output_.slew_limited<<'\n';csv_.flush();}
  f::Config config_;std::unique_ptr<f::Runtime> runtime_;std::unique_ptr<f::UnixServer> server_;
  bool boundary_fault_latched_{};double control_started_{};std::string publication_block_reason_;
  Attitude attitude_{};f::attitude::Output last_attitude_output_;
  f::AuthorityTelemetry authority_telemetry_;
  Local local_{};Status status_{};Land land_{};Battery battery_{};Flags flags_{};
  f::Stamp local_stamp_,status_stamp_,land_stamp_,battery_stamp_,flags_stamp_,attitude_stamp_;
  bool clock_regressed_{};std::uint64_t last_wall_stamp_{};std::uint8_t ground_z_reset_{},ground_vz_reset_{};
  double printed_{},rest_since_{-1},reference_z_{},stop_since_{},last_shutdown_warning_{};bool reference_set_{},stopping_{},heading_changed_{};std::uint8_t heading_reset_{};
  std::ofstream csv_;
  f::attitude::PublicationAudit publication_audit_;
  rclcpp::Subscription<Attitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<Manual>::SharedPtr manual_sub_;
  rclcpp::Subscription<Switches>::SharedPtr switches_sub_;
  rclcpp::Publisher<AttitudeTarget>::SharedPtr attitude_pub_;
  rclcpp::Subscription<Local>::SharedPtr local_sub_;rclcpp::Subscription<Status>::SharedPtr status_sub_;rclcpp::Subscription<Land>::SharedPtr land_sub_;rclcpp::Subscription<Battery>::SharedPtr battery_sub_;rclcpp::Subscription<Flags>::SharedPtr flags_sub_;rclcpp::Subscription<Ack>::SharedPtr ack_sub_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;rclcpp::Publisher<Mode>::SharedPtr mode_pub_;rclcpp::Publisher<Target>::SharedPtr target_pub_;
};
int main(int argc,char** argv){
  rclcpp::init(argc,argv);std::signal(SIGINT,stop_handler);std::signal(SIGTERM,stop_handler);
  int code=0;
  try{auto node=std::make_shared<SystemNode>();while(rclcpp::ok()){const auto start=std::chrono::steady_clock::now();rclcpp::spin_some(node);if(!node->tick())break;std::this_thread::sleep_until(start+node->loop_period());}}
  catch(const std::exception& error){std::cerr<<"[ERROR] "<<error.what()<<'\n';code=2;}
  rclcpp::shutdown();return code;
}
