#include "jikl_fc01_system/runtime.hpp"
#include "jikl_fc01_system/unix_server.hpp"
#include "jikl_fc01_system/attitude_ros_parameters.hpp"
#include "jikl_fc01_system/ros_config.hpp"
#include "jikl_fc01_system/async_log.hpp"
#include "jikl_fc01_system/attitude_wire.hpp"
#include "jikl_fc01_system/attitude_diagnostics.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include "jikl_fc01_system/publication_audit.hpp"
#include "jikl_fc01_system/authority_telemetry.hpp"
#include "jikl_fc01_system/source_clock.hpp"
#include "jikl_fc01_system/source_diagnostics.hpp"
#include "jikl_fc01_system/vision_ros_adapter.hpp"
#include "jikl_fc01_system/control_diagnostics.hpp"
#include <jikl_fc01_system/msg/vision_state.hpp>
#include <jikl_fc01_system/msg/navigation_command.hpp>
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
using VisualState=jikl_fc01_system::msg::VisionState;
using VisualCommand=jikl_fc01_system::msg::NavigationCommand;
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
    f::Config config=f::read_ros_config(*this);
    config.validate();runtime_=std::make_unique<f::Runtime>(config);runtime_->require_attitude_sink_commit();config_=config;
    const std::string socket=declare_parameter<std::string>("socket_path",default_socket());
    const auto parent=std::filesystem::path(socket).parent_path();
    std::filesystem::create_directories(parent);
    struct stat parent_stat{};
    if(lstat(parent.c_str(),&parent_stat)!=0||!S_ISDIR(parent_stat.st_mode)||parent_stat.st_uid!=getuid()||(parent_stat.st_mode&0022))throw std::runtime_error("socket directory must be owned and not writable by other users");
    server_=std::make_unique<f::UnixServer>(socket);
    const auto qos=rclcpp::SensorDataQoS().keep_last(10);
    if(config.attitude.visual.enabled){
      visual_state_topic_=declare_parameter<std::string>("visual_state_topic","/fc01/vision/state");
      visual_command_topic_=declare_parameter<std::string>("visual_command_topic","/fc01/navigation/command");
      const auto depth=declare_parameter<std::int64_t>("visual_qos_depth",5);
      const bool state_reliable=declare_parameter<bool>("visual_state_reliable",false);
      const bool command_reliable=declare_parameter<bool>("visual_command_reliable",true);
      if(depth<1||depth>20||visual_state_topic_.empty()||visual_command_topic_.empty()||
         visual_state_topic_[0]!='/'||visual_command_topic_[0]!='/'||visual_state_topic_==visual_command_topic_||
         visual_state_topic_.rfind("/fmu/",0)==0||visual_command_topic_.rfind("/fmu/",0)==0)
        throw std::invalid_argument("INVALID_VISUAL_INPUT_TOPICS_OR_QOS");
      auto state_qos=rclcpp::QoS(std::size_t(depth)).durability_volatile();
      auto command_qos=rclcpp::QoS(std::size_t(depth)).durability_volatile();
      if(state_reliable)state_qos.reliable();else state_qos.best_effort();
      if(command_reliable)command_qos.reliable();else command_qos.best_effort();
      visual_state_sub_=create_subscription<VisualState>(visual_state_topic_,state_qos,
        [this](VisualState::ConstSharedPtr m){visual_inbox_.state(f::vision::state_from_message(*m),steady());});
      visual_command_sub_=create_subscription<VisualCommand>(visual_command_topic_,command_qos,
        [this](VisualCommand::ConstSharedPtr m){visual_inbox_.command(f::vision::command_from_message(*m),steady());});
      std::cout<<"{\"kind\":\"visual_inputs\",\"state_topic\":"<<f::quote(visual_state_topic_)
        <<",\"command_topic\":"<<f::quote(visual_command_topic_)<<",\"state_reliable\":"<<boolean(state_reliable)
        <<",\"command_reliable\":"<<boolean(command_reliable)<<",\"qos_depth\":"<<depth<<"}"<<std::endl;
    }
    if(config.profile=="attitude_altitude"){
      attitude_sub_=create_subscription<Attitude>("/fmu/out/vehicle_attitude",qos,[this](Attitude::ConstSharedPtr m,const rclcpp::MessageInfo& info){receive_sampled(*m,attitude_,attitude_stamp_,attitude_sample_,f::SourceSlot::Attitude,info);});
      manual_sub_=create_subscription<Manual>("/fmu/out/manual_control_setpoint",qos,[this](Manual::ConstSharedPtr m){authority_telemetry_.manual(*m,steady(),timestamp(),config_.attitude.flags_timeout);});
      switches_sub_=create_subscription<Switches>("/fmu/out/manual_control_switches",qos,[this](Switches::ConstSharedPtr m){authority_telemetry_.switches(*m,steady(),config_.attitude.flags_timeout,timestamp());});
    }
    local_sub_=create_subscription<Local>("/fmu/out/vehicle_local_position",qos,[this](Local::ConstSharedPtr m,const rclcpp::MessageInfo& info){receive_sampled(*m,local_,local_stamp_,local_sample_,f::SourceSlot::Local,info);});
    status_sub_=create_subscription<Status>("/fmu/out/vehicle_status_v1",qos,[this](Status::ConstSharedPtr m,const rclcpp::MessageInfo& info){receive(*m,status_,status_stamp_,f::SourceSlot::Status,info);});
    land_sub_=create_subscription<Land>("/fmu/out/vehicle_land_detected",qos,[this](Land::ConstSharedPtr m,const rclcpp::MessageInfo& info){receive(*m,land_,land_stamp_,f::SourceSlot::Land,info);});
    battery_sub_=create_subscription<Battery>("/fmu/out/battery_status",qos,[this](Battery::ConstSharedPtr m,const rclcpp::MessageInfo& info){if(m->id==1)receive(*m,battery_,battery_stamp_,f::SourceSlot::Battery,info);});
    flags_sub_=create_subscription<Flags>("/fmu/out/failsafe_flags",qos,[this](Flags::ConstSharedPtr m,const rclcpp::MessageInfo& info){receive(*m,flags_,flags_stamp_,f::SourceSlot::Flags,info);});
    ack_sub_=create_subscription<Ack>("/fmu/out/vehicle_command_ack",qos,[this](Ack::ConstSharedPtr m){
      const auto received=steady();
      runtime_->ack({m->timestamp,m->command,m->result,m->target_system,m->target_component,m->from_external},received);
      std::cout<<"{\"kind\":\"ack\",\"command\":"<<m->command<<",\"result\":"<<int(m->result)<<",\"timestamp\":"<<m->timestamp<<",\"target_system\":"<<int(m->target_system)<<",\"target_component\":"<<m->target_component<<",\"from_external\":"<<boolean(m->from_external)
        <<",\"monotonic_s\":"<<number(received);
      if(m->command==176){const auto& p=runtime_->pending(176);
        std::cout<<",\"ack176_disposition\":"<<f::quote(p.ack_disposition)
          <<",\"ack176_candidate\":"<<boolean(p.ack_candidate)
          <<",\"ack176_accepted\":"<<boolean(p.accepted)
          <<",\"ack176_stage\":"<<f::quote(f::command_stage_name(p.stage))
          <<",\"ack176_generation\":"<<p.generation
          <<",\"ack176_wire_stamp\":"<<p.wire_stamp
          <<",\"ack176_baseline\":"<<p.baseline
          <<",\"ack176_status_at_wire\":"<<p.status_at_wire;
      }
      std::cout<<'}'<<std::endl;});
    command_pub_=create_publisher<Command>("/fmu/in/vehicle_command",10);
    const auto publishers=f::attitude::publisher_plan(config.profile);
    if(publishers.mode)mode_pub_=create_publisher<Mode>("/fmu/in/offboard_control_mode",10);
    if(publishers.trajectory)target_pub_=create_publisher<Target>("/fmu/in/trajectory_setpoint",10);
    if(publishers.attitude)attitude_pub_=create_publisher<AttitudeTarget>("/fmu/in/vehicle_attitude_setpoint",10);
    const auto logpath=declare_parameter<std::string>("csv_path","");
    if(!logpath.empty()){
      csv_.open(logpath,std::ios::app);
      if(!csv_)throw std::runtime_error("cannot open CSV log");
      csv_<<"steady_s,phase,nav_state,armed,landed,xy_valid,z_valid,vxy_valid,vz_valid,heading_good,local_age_s,x_ned,y_ned,z_ned,vx_ned,vy_ned,vz_ned,ax_ned,ay_ned,az_ned,relative_height_m,reason,aa_healthy,aa_yaw_reference,aa_thrust_z,aa_height_target,aa_velocity_target,aa_saturated,aa_slew_limited,aa_z_deriv_ned,aa_feedback_checked,aa_feedback_suspect,aa_feedback_rate_error_mps,aa_feedback_position_rate_error_mps,aa_feedback_disagreement_s,aa_feedback_reason,aa_vertical_closed_loop,aa_takeoff_thrust_limited,aa_p_thrust,aa_i_thrust,aa_raw_thrust,aa_thrust_ceiling\n";
    }
    std::cout<<"{\"kind\":\"startup\",\"profile\":"<<f::quote(config.profile)<<",\"flight_enabled\":"<<boolean(config.flight_enabled)<<",\"automatic_arm\":false,\"rc_navigation_policy\":\"RC_NAVIGATION_MANUAL_ARM_V1\",\"takeoff_policy\":\"NAV_TAKEOFF_EVIDENCE_V2_20261103\",\"vertical_control_policy\":\"R632_SCHEDULED_TAKEOFF_CEILING_V1\",\"socket\":"<<f::quote(socket)<<"}"<<std::endl;
  }
  std::chrono::duration<double> loop_period() const{return std::chrono::duration<double>(1./config_.attitude.loop_hz);}
  void begin_cycle(double start){
    cycle_.begin(start);work_={};work_.started=start;
    pair_context_count_=0;pair_context_seconds_.fill(0);
  }
  void callbacks_done(double now){
    cycle_.callbacks_done(now);work_.callbacks=now-work_.started;
  }
  void observe_cycle(double start,double callbacks_done,double done){
    timing_.observe(start,callbacks_done,done,config_.attitude.stall_seconds);
    completed_work_=work_;
    if(done-start>config_.attitude.stall_seconds&&done-last_timing_warning_>=1.){
      last_timing_warning_=done;
      std::cerr<<"{\"kind\":\"control_cycle_overrun\",\"cycle_seconds\":"<<number(done-start)
        <<work_fields(work_)<<",\"previous_start_period_s\":"<<number(timing_.period_seconds,timing_.period_valid)
        <<",\"heartbeat_this_tick\":"<<boolean(publication_audit_.mode_this_tick)<<"}"<<std::endl;
    }
  }
  bool tick(){
    const double now=steady();
    publication_audit_.begin(now);
    const std::string phase_before=runtime_->phase_name();
    auto s=snapshot(now);
    bool resting=s.status_fresh&&s.land_fresh&&!s.feedback.armed&&s.feedback.landed&&s.at_rest;
    if(config_.profile=="attitude_altitude"){
      resting=resting&&s.local_fresh&&local_.z_valid&&local_.v_z_valid&&std::isfinite(local_.z)&&std::isfinite(local_.vz)&&std::abs(local_.vz)<=config_.attitude.ground_vz_tolerance;
      if(local_.z_reset_counter!=ground_z_reset_||local_.vz_reset_counter!=ground_vz_reset_)rest_since_=-1;
      ground_z_reset_=local_.z_reset_counter;ground_vz_reset_=local_.vz_reset_counter;
    }
    if(resting){if(rest_since_<0)rest_since_=now;s.stationary_seconds=now-rest_since_;}
    else {rest_since_=-1;s.stationary_seconds=0;}
    if(config_.profile!="attitude_altitude"){
      if(resting&&s.local_fresh&&local_.z_valid&&!velocity_reference_set_&&s.stationary_seconds>=2){
        velocity_reference_z_=local_.z;velocity_reference_set_=true;
      }
      if(runtime_->active()){velocity_reference_z_=runtime_->flight_core().origin().z;velocity_reference_set_=true;}
    }
    if(stop_requested)begin_shutdown(s,now);
    const double ipc_started=steady();
    server_->poll([this,&s,now](const f::Request& req){
      if(stop_requested)begin_shutdown(s,now);
      auto reply=runtime_->request(req.op,req.values,s,now);
      if(req.op=="STOP"&&reply.accepted){stop_requested=1;begin_shutdown(s,now);}
      std::string out=state(s,now,"reply",req.seq,reply.accepted,reply.message);
      if(req.op!="PING")std::cout<<out<<std::endl;
      return out;
    },[this,&s,now]{runtime_->disconnect(s,now);});
    work_.ipc=steady()-ipc_started;
    const double runtime_started=steady();
    auto output=runtime_->step(s,now);
    work_.runtime=steady()-runtime_started;
    // A task-input TTL may expire between calculation and the first sink.
    // Re-enter the original descent controller before any motion is committed.
    if(output.attitude.visual_motion){
      const auto actual_time=steady();const auto actual=snapshot(actual_time);
      if(!runtime_->visual_output_current(actual,actual_time)){
        runtime_->request_visual_land(actual,actual_time,"INPUT_EXPIRED_BEFORE_SINK");
        output=runtime_->step(actual,steady());
      }
    }
    if(config_.profile=="attitude_altitude"&&!cycle_.current(steady(),config_.attitude.stall_seconds)){
      runtime_->abort_attitude("ROS_ADAPTER_CONTROL_LOOP_STALL");output={};
    }
    const auto previous_attitude_output=last_attitude_output_;
    const bool previous_heartbeat_committed=previous_attitude_output.publish;
    const double previous_heartbeat_stamp=publication_audit_.mode_last;
    last_attitude_output_={};
    if(boundary_fault_latched_)output={};
    // PX4 status can change after Core::step but before publication. A
    // confirmed, disarmed external departure during stage-one release makes
    // both the zero heartbeat and any release 176 unnecessary. Do not weaken
    // the ordinary final guard or apply this to a flight/armed vehicle.
    if(config_.profile=="attitude_altitude"&&
       (output.attitude.publish||output.attitude.command==f::attitude::Command::ReleaseOffboard)&&
       runtime_->attitude_core().phase()==f::attitude::Phase::ReleaseOffboard&&
       f::attitude::external_ground_mode_departure(boundary_context())){
      if(output.attitude.command==f::attitude::Command::ReleaseOffboard){
        const auto actual=snapshot(steady());
        runtime_->confirm_attitude_command_publication(
          f::attitude::Command::ReleaseOffboard,false,0,actual,steady());
      }
      std::cout<<"{\"kind\":\"external_ground_release_cancelled\","
        <<"\"reason\":\"PX4_ALREADY_LEFT_OFFBOARD_DISARMED_NO_176_NO_HEARTBEAT\"}"
        <<std::endl;
      output.attitude={};
    }
    // Never put a 176/400 command on the wire before its companion final
    // setpoint sink has approved the same tick.  In particular this prevents
    // a denied attitude sample from preceding an untracked Arm/Disarm.
    const auto requested_command=output.attitude.command;
    const auto attempted_attitude=output.attitude;
    bool attitude_sent=!output.attitude.publish;
    if(output.attitude.publish){
      try{
        pair_context_count_=0;pair_context_seconds_.fill(0);sampling_pair_=true;
        const auto decision=f::attitude::publish_attitude_resampled<Mode,AttitudeTarget>(
          [this]{return boundary_context();},output.attitude,timestamp(),
          [this](const AttitudeTarget& target){
            attitude_pub_->publish(target);publication_audit_.target_committed(steady(),target.thrust_body[2]);},
          [this](const Mode& mode){
            mode_pub_->publish(mode);publication_audit_.mode_committed(steady());});
        sampling_pair_=false;
        if(decision.allowed){
          last_attitude_output_=output.attitude;attitude_sent=true;runtime_->confirm_rc_output(true,steady());
          if(publication_block_reason_==preparation_age_wait_)publication_block_reason_.clear();
        }
        else if(decision.reason==f::attitude::BoundaryReason::VisualInput){
          // No sink ran: the motion input or detector agreement changed. A partial
          // target/heartbeat failure still takes the hard exception path below.
          const auto actual_time=steady();const auto actual=snapshot(actual_time);
          runtime_->request_visual_land(actual,actual_time,actual.feedback.landed?
            "FLIGHT_DETECTOR_CONTRADICTION_AT_FIRST_SINK":"INPUT_EXPIRED_AT_FIRST_SINK");
          publication_block_reason_="VISUAL_INPUT_RECOVERY_REQUESTED";output={};
        }else if(defer_unarmed_source_expiry(decision,output.attitude)){output={};}
        else {refuse_publication("attitude",decision);output={};}
      }catch(const std::exception& e){
        sampling_pair_=false;
        publication_audit_.failed();attitude_sent=false;boundary_fault_latched_=true;
        publication_block_reason_=std::string("PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_")+e.what();
        runtime_->abort_attitude(publication_block_reason_);output={};report_publication_failure("attitude");
      }catch(...){
        sampling_pair_=false;
        publication_audit_.failed();attitude_sent=false;boundary_fault_latched_=true;
        publication_block_reason_="PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_UNKNOWN_EXCEPTION";
        runtime_->abort_attitude(publication_block_reason_);output={};report_publication_failure("attitude");
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
        <<",\"intent\":"<<f::quote(requested_command==f::attitude::Command::Arm?"ARM":
          requested_command==f::attitude::Command::Disarm?"DISARM":
          requested_command==f::attitude::Command::Offboard?"OFFBOARD":
          requested_command==f::attitude::Command::ReleaseOffboard?"RELEASE_OFFBOARD":"PX4_LAND")
        <<",\"command400_generation\":"<<((requested_command==f::attitude::Command::Arm||
          requested_command==f::attitude::Command::Disarm)?runtime_->pending(400).generation:0)
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
    if(config_.profile=="attitude_altitude"&&previous_heartbeat_committed&&
       !last_attitude_output_.publish){
      const auto evidence=runtime_->attitude_core().tilt_fault_evidence();
      const auto& fb=s.attitude;
      const auto att=f::attitude::attitude_diagnostics(fb.q,previous_attitude_output.q);
      // PREVIOUS committed target vs CURRENT observed pose. Never claim that
      // a locally committed ROS publish was received/applied by PX4.
      std::cout<<"{\"kind\":\"offboard_stream_edge\",\"edge\":\"LOCAL_HEARTBEAT_COMMIT_STOPPED\""
        <<",\"monotonic_s\":"<<number(now)
        <<",\"previous_heartbeat_commit_monotonic_s\":"<<number(previous_heartbeat_stamp,previous_heartbeat_stamp>=0)
        <<",\"phase_before\":"<<f::quote(phase_before)
        <<",\"phase_after\":"<<f::quote(runtime_->phase_name())
        <<",\"reason\":"<<f::quote(runtime_->attitude_core().reason())
        <<",\"publication_block_reason\":"<<f::quote(publication_block_reason_)
        <<",\"publication_fault_latched\":"<<boolean(boundary_fault_latched_)
        <<",\"operator_takeover\":"<<boolean(fb.operator_takeover)
        <<",\"armed\":"<<boolean(fb.armed)
        <<",\"offboard\":"<<boolean(fb.offboard)
        <<",\"landed\":"<<boolean(fb.landed)
        <<",\"xy_valid\":"<<boolean(fb.xy_valid)
        <<",\"vxy_valid\":"<<boolean(fb.vxy_valid)
        <<",\"local_stamp\":"<<fb.local_stamp
        <<",\"status_stamp\":"<<fb.status_stamp
        <<",\"land_stamp\":"<<fb.land_stamp
        <<",\"last_committed_target_thrust_z\":"<<number(previous_attitude_output.thrust[2])
        <<",\"attitude_audit_valid\":"<<boolean(att.valid)
        <<",\"attitude_roll_deg\":"<<number(att.roll_deg,att.valid)
        <<",\"attitude_pitch_deg\":"<<number(att.pitch_deg,att.valid)
        <<",\"attitude_tilt_deg\":"<<number(att.tilt_deg,att.valid)
        <<",\"attitude_error_deg\":"<<number(att.error_deg,att.valid)
        <<",\"tilt_fault_trigger_sample_available\":"<<boolean(evidence.valid)
        <<",\"local_target_this_tick\":"<<boolean(publication_audit_.target_this_tick)
        <<",\"local_heartbeat_this_tick\":"<<boolean(publication_audit_.mode_this_tick)
        <<"}"<<std::endl;
    }
    // Keep every transition; ordinary thrust samples need only the existing
    // 2 Hz reporting cadence. Avoid a second full DDS graph snapshot and a
    // large diagnostic record on all 20 control ticks.
    const double reporting_started=steady();
    if(config_.profile=="attitude_altitude"&&
       (phase_before!=runtime_->phase_name()||now-printed_>=.5))
      write_phase_event(phase_before,s,attempted_attitude,now);
    if(now-printed_>=.5){printed_=now;auto line=state(s,now);server_->publish(line);std::cout<<line<<std::endl;write_csv(s,now);}
    work_.reporting=steady()-reporting_started;
    if(!stopping_&&runtime_->reprepare_rc(s,now)){
      std::cout<<"{\"kind\":\"rc_safe_reprepare\",\"reason\":\"NEW_DISARMED_GROUND_AND_CONTROLS_RESET\"}"<<std::endl;
    }
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
  struct Work {
    double started{},callbacks{},ipc{},runtime{},snapshot{},graph{},boundary{},reporting{};
    std::uint64_t snapshots{},graph_queries{},contexts{};
  };
  struct GraphCounts {
    jikl_fc01_ground_link::Traffic traffic;
    f::attitude::OwnershipCounts ownership;
    std::size_t local{},attitude{},manual{},switches{},battery{},flags{},visual_state{},visual_command{};
  };
  GraphCounts query_graph(){
    const double begin=steady();GraphCounts graph;
    const auto publishers=[this](const std::string& topic){++work_.graph_queries;return count_publishers(topic);};
    const auto subscribers=[this](const std::string& topic){++work_.graph_queries;return count_subscribers(topic);};
    graph.traffic={publishers("/fmu/out/vehicle_status_v1"),publishers("/fmu/out/vehicle_land_detected"),
      publishers("/fmu/out/vehicle_command_ack"),subscribers("/fmu/in/vehicle_command"),publishers("/fmu/in/vehicle_command")};
    graph.local=publishers("/fmu/out/vehicle_local_position");
    auto& ownership=graph.ownership;
    ownership.mode_publishers=publishers("/fmu/in/offboard_control_mode");ownership.mode_subscribers=subscribers("/fmu/in/offboard_control_mode");
    ownership.trajectory_publishers=publishers("/fmu/in/trajectory_setpoint");ownership.trajectory_subscribers=subscribers("/fmu/in/trajectory_setpoint");
    ownership.attitude_publishers=publishers("/fmu/in/vehicle_attitude_setpoint");ownership.attitude_subscribers=subscribers("/fmu/in/vehicle_attitude_setpoint");
    for(const char* topic:{"/fmu/in/vehicle_rates_setpoint","/fmu/in/vehicle_thrust_setpoint","/fmu/in/vehicle_torque_setpoint","/fmu/in/actuator_motors"})ownership.other_publishers+=publishers(topic);
    if(config_.profile=="attitude_altitude"){
      graph.manual=publishers("/fmu/out/manual_control_setpoint");graph.switches=publishers("/fmu/out/manual_control_switches");
      graph.attitude=publishers("/fmu/out/vehicle_attitude");
    }
    if(config_.profile!="sim"){
      graph.battery=publishers("/fmu/out/battery_status");graph.flags=publishers("/fmu/out/failsafe_flags");
    }
    if(config_.attitude.visual.enabled){graph.visual_state=publishers(visual_state_topic_);graph.visual_command=publishers(visual_command_topic_);}
    work_.graph+=steady()-begin;return graph;
  }
  bool reference_ready()const{
    return config_.profile=="attitude_altitude"?runtime_->attitude_core().reference_ready():velocity_reference_set_;
  }
  double reference_z()const{
    return config_.profile=="attitude_altitude"?runtime_->attitude_core().reference_z():velocity_reference_z_;
  }
  void begin_shutdown(const f::Snapshot& s,double now){
    if(stopping_)return;
    stopping_=true;stop_since_=now;runtime_->begin_shutdown(s,now);
  }
  void record_receipt(f::SourceSlot slot,std::uint64_t envelope,std::uint64_t sample,
      bool sampled,double received,const rclcpp::MessageInfo& info){
    receipt_timing_[std::size_t(slot)]=f::observe_source_receipt(envelope,sample,sampled,
      timestamp(),received,info.get_rmw_message_info().received_timestamp);
  }
  template<class T>void receive(const T& msg,T& destination,f::Stamp& stamp,
      f::SourceSlot slot,const rclcpp::MessageInfo& info){
    const double received=steady();record_receipt(slot,msg.timestamp,0,false,received,info);
    const bool accepted=stamp.accept(msg.timestamp,received);
    receipt_accepted_[std::size_t(slot)]=accepted;
    if(accepted)destination=msg;
  }
  template<class T>void receive_sampled(const T& msg,T& destination,f::Stamp& stamp,
      f::SourceSampleProgress& sample,f::SourceSlot slot,const rclcpp::MessageInfo& info){
    const double received=steady();
    record_receipt(slot,msg.timestamp,msg.timestamp_sample,true,received,info);
    receipt_accepted_[std::size_t(slot)]=f::receive_sampled_source(msg,destination,stamp,sample,received);
  }
  std::uint64_t timestamp(){
    const auto ns=get_clock()->now().nanoseconds();
    const auto stamp=ns>0?std::uint64_t(ns/1000):0;
    clock_observation_=host_clock_.observe(stamp,steady());
    if(!stamp||(last_wall_stamp_&&stamp<last_wall_stamp_))clock_regressed_=true;
    last_wall_stamp_=stamp;
    if(clock_observation_.change==f::HostClockChange::ForwardJump||
       clock_observation_.change==f::HostClockChange::BackwardJump||
       clock_observation_.change==f::HostClockChange::SteadyRegression){
      last_clock_event_=clock_observation_;last_clock_event_stamp_=stamp;++clock_event_count_;
    }
    return stamp;
  }
  f::attitude::BoundaryContext boundary_context(){
    const double begin=steady();auto s=snapshot(begin);
    // The snapshot performs graph lookups. Its elapsed time belongs to this
    // publication's budget, including the final check after target commit.
    const double now=steady();
    s.stationary_seconds=rest_since_>=0?std::max(0.,now-rest_since_):0;
    auto context=runtime_->publication_context(s,now,server_->connected(),true,
      stop_requested||stopping_||boundary_fault_latched_);
    context.control_fresh=cycle_.current(steady(),config_.attitude.stall_seconds);
    const double elapsed=steady()-begin;++work_.contexts;work_.boundary+=elapsed;
    if(sampling_pair_){
      if(pair_context_count_<pair_context_seconds_.size())pair_context_seconds_[pair_context_count_]=elapsed;
      ++pair_context_count_;
    }
    return context;
  }
  void refuse_publication(const char* boundary,const f::attitude::Decision& decision){
    boundary_fault_latched_=true;
    publication_block_reason_=std::string("PUBLISHER_BOUNDARY_")+f::attitude::name(decision.reason);
    runtime_->abort_attitude(publication_block_reason_);last_attitude_output_={};
    report_publication_failure(boundary);
  }
  bool defer_unarmed_source_expiry(const f::attitude::Decision& decision,
      const f::attitude::Output& planned){
    // Only an age crossing BEFORE the first sink of an untouched RC ground
    // preparation is recoverable here. A partial pair always throws and
    // bypasses this path; command/arming/flight histories remain immutable.
    if(decision.reason!=f::attitude::BoundaryReason::SourceClock||
       stop_requested||stopping_||boundary_fault_latched_||clock_event_count_||
       planned.command!=f::attitude::Command::None||f::attitude::nonzero(planned.thrust)||
       publication_audit_.target_this_tick||publication_audit_.mode_this_tick)return false;
    auto actual=snapshot(steady());const double now=steady();
    if(clock_event_count_||!cycle_.current(now,config_.attitude.stall_seconds))return false;
    const auto host=actual.attitude.source_host_stamp;
    // Require recent reception AND advancing raw samples. Future/missing
    // stamps, clock jumps and stalled raw data cannot enter this retry path.
    if(!f::only_preparation_source_age_expired(source_timings(now,host),{
       f::receipt_time_reason(local_sample_.received,now,config_.attitude.local_timeout),
       f::receipt_time_reason(attitude_sample_.received,now,config_.attitude.attitude_timeout)}))return false;
    actual.stationary_seconds=rest_since_>=0?std::max(0.,now-rest_since_):0;
    // snapshot() ordinarily reports the previous tick's stream. Admission
    // here uses the actual sinks in this attempt, verified above as empty.
    actual.attitude_output_active=publication_audit_.target_this_tick;
    actual.offboard_heartbeat_active=publication_audit_.mode_this_tick;
    if(!runtime_->defer_rc_preparation(actual,now,preparation_age_wait_))return false;
    publication_block_reason_=preparation_age_wait_;
    if(now-last_defer_report_>=1.){last_defer_report_=now;report_publication_failure("unarmed_preparation_deferred");}
    return true;
  }
  void report_publication_failure(const char* boundary){
    const auto host=timestamp();const double now=steady();
    std::cerr<<"{\"kind\":\"publication_blocked\",\"boundary\":"<<f::quote(boundary)
      <<",\"reason\":"<<f::quote(publication_block_reason_)
      <<",\"target_this_tick\":"<<boolean(publication_audit_.target_this_tick)
      <<",\"heartbeat_this_tick\":"<<boolean(publication_audit_.mode_this_tick)
      <<",\"publication_fault_latched\":"<<boolean(boundary_fault_latched_)
      <<",\"heartbeat_enabled\":false,\"attitude_output_enabled\":false"
      <<timing_evidence(now,host)<<work_fields(work_)<<"}"<<std::endl;
  }
  bool publish_command(std::uint32_t id,float p1,float p2,std::uint64_t* actual_stamp=nullptr){
    Command cmd{};cmd.timestamp=timestamp();cmd.command=id;cmd.param1=p1;cmd.param2=p2;
    cmd.target_system=1;cmd.target_component=1;cmd.source_system=1;cmd.source_component=191;cmd.from_external=true;
    auto sink=[this,&cmd]{f::publish_command_with_stamp(cmd,[this]{return timestamp();},
      [this](const Command& stamped){command_pub_->publish(stamped);});};
    // This unconditional final sink check also covers future callers and the
    // legacy flight/sim route. Those profiles cannot bypass the Arm contract.
    if(id==Command::VEHICLE_CMD_COMPONENT_ARM_DISARM&&p1==1){
      const auto decision=f::attitude::publish_arm_at_boundary(boundary_context(),sink);
      if(!decision.allowed){refuse_publication("arm",decision);return false;}
    }else if(id==Command::VEHICLE_CMD_COMPONENT_ARM_DISARM&&p1==0){
      const auto decision=f::attitude::publish_disarm_at_boundary(boundary_context(),
        sink);
      if(!decision.allowed){refuse_publication("disarm",decision);return false;}
    }else if(id==Command::VEHICLE_CMD_DO_SET_MODE&&p1==1&&p2==7&&
             config_.profile=="attitude_altitude"){
      const auto decision=f::attitude::publish_release_at_boundary(boundary_context(),
        sink);
      if(!decision.allowed){refuse_publication("release",decision);return false;}
    }else if(id==Command::VEHICLE_CMD_NAV_LAND&&config_.profile=="attitude_altitude"){
      const auto decision=f::attitude::publish_land_fallback_at_boundary(boundary_context(),
        sink);
      if(!decision.allowed){refuse_publication("land_fallback",decision);return false;}
    }else sink();
    if(actual_stamp)*actual_stamp=cmd.timestamp;
    std::cout<<"{\"kind\":\"send\",\"command\":"<<id<<",\"param1\":"<<p1<<",\"param2\":"<<p2<<"}"<<std::endl;
    return true;
  }
  f::Snapshot snapshot(double now){
    const double begin=steady();
    // Sample age AFTER graph lookups, rather than letting graph latency make
    // a stale receipt/source look current. No graph result is reused across
    // snapshots: each of the three publication checks still observes counts.
    const auto graph=query_graph();now=steady();const auto host_stamp=timestamp();
    source_observed_steady_=now;source_observed_host_=host_stamp;
    f::Snapshot s;s.status_fresh=status_stamp_.fresh(now,config_.attitude.status_timeout);s.land_fresh=land_stamp_.fresh(now,config_.attitude.land_timeout);s.local_fresh=local_stamp_.fresh(now,config_.attitude.local_timeout);
    s.arming_state_known=status_.arming_state==Status::ARMING_STATE_ARMED||
      status_.arming_state==Status::ARMING_STATE_DISARMED;
    s.status_fresh=s.status_fresh&&s.arming_state_known;
    s.attitude_output_active=s.offboard_heartbeat_active=last_attitude_output_.publish;
    s.publication_fault_latched=boundary_fault_latched_;
    s.battery_fresh=battery_stamp_.fresh(now,1.);s.manual_fresh=flags_stamp_.fresh(now,config_.attitude.flags_timeout);
    s.time_regressed=status_stamp_.regressed||land_stamp_.regressed||local_stamp_.regressed||battery_stamp_.regressed||flags_stamp_.regressed||clock_regressed_||attitude_stamp_.regressed||local_sample_.regressed||attitude_sample_.regressed;
    s.status_stamp=status_stamp_.value;s.land_stamp=land_stamp_.value;s.system=status_.system_id;s.component=status_.component_id;
    s.armed_time=status_.armed_time;s.latest_arming_reason=status_.latest_arming_reason;
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
    s.traffic=graph.traffic;
    fb.exclusive=s.system==1&&s.component==1&&s.traffic.status_publishers==1&&s.traffic.land_publishers==1&&s.traffic.ack_publishers==1&&s.traffic.command_subscribers==1&&s.traffic.command_publishers==1&&graph.local==1;
    fb.exclusive=fb.exclusive&&f::attitude::owns_setpoints(config_.profile,graph.ownership);
    if(config_.profile=="attitude_altitude"){
      auto& a=s.attitude;a.attitude_fresh=attitude_stamp_.fresh(now,config_.attitude.attitude_timeout);
      a.failsafe_fresh=flags_stamp_.fresh(now,config_.attitude.flags_timeout);
      a.source_host_stamp=host_stamp;
      a.local_sample_fresh=local_sample_.fresh(host_stamp,now,config_.attitude.local_timeout);
      a.attitude_sample_fresh=attitude_sample_.fresh(host_stamp,now,config_.attitude.attitude_timeout);
      s.local_fresh=s.local_fresh&&a.local_sample_fresh;
      a.attitude_fresh=a.attitude_fresh&&a.attitude_sample_fresh;
      s.battery_fresh=s.battery_fresh&&f::source_stamp_in_clock(battery_stamp_.value,host_stamp,1.);
      fb.fresh=s.status_fresh&&s.land_fresh&&s.local_fresh;
      a.source_clock_valid=f::source_stamp_in_clock(status_stamp_.value,host_stamp,config_.attitude.status_timeout)&&
        f::source_stamp_in_clock(land_stamp_.value,host_stamp,config_.attitude.land_timeout)&&
        f::source_sample_in_clock(local_stamp_.value,local_.timestamp_sample,host_stamp,config_.attitude.local_timeout)&&
        f::source_sample_in_clock(attitude_stamp_.value,attitude_.timestamp_sample,host_stamp,config_.attitude.attitude_timeout)&&
        f::source_stamp_in_clock(flags_stamp_.value,host_stamp,config_.attitude.flags_timeout)&&
        f::source_stamp_in_clock(battery_stamp_.value,host_stamp,1.);
      a.source_clock_validated=true; // validation performed; validity may expire after a sink commit
      for(std::size_t i=0;i<4;++i)a.q[i]=attitude_.q[i];
      a.quat_reset=attitude_.quat_reset_counter;a.z=local_.z;a.vz=local_.vz;
      a.z_deriv=local_.z_deriv;
      a.vertical_sample_stamp=local_.timestamp_sample;
      a.z_reset=local_.z_reset_counter;a.vz_reset=local_.vz_reset_counter;a.z_valid=local_.z_valid;a.vz_valid=local_.v_z_valid;
      a.local_stamp=local_stamp_.value;a.xy_valid=local_.xy_valid&&!local_.dead_reckoning&&
        std::isfinite(local_.x)&&std::isfinite(local_.y);
      a.vxy_valid=local_.v_xy_valid&&std::isfinite(local_.vx)&&std::isfinite(local_.vy);
      a.land_selectable=(status_.can_set_nav_states_mask&(1U<<Status::NAVIGATION_STATE_AUTO_LAND))!=0;
      const auto authority=authority_telemetry_.facts(now,config_.attitude.flags_timeout,
        a.failsafe_fresh,flags_.manual_control_signal_lost,
        graph.manual,graph.switches,host_stamp);
      a.rc_link_ready=authority.rc_link_ready;a.rc_switches_ready=authority.rc_switches_ready;
      a.rc_source_clock_valid=authority.rc_source_clock_valid;a.rc_switch_clock_valid=authority.rc_switch_clock_valid;
      a.rc_source_seen=authority.manual_seen;a.rc_source_valid=authority.manual_valid;
      a.rc_source_selected=authority.rc_source_selected;a.rc_data_source=authority.data_source;
      a.rc_sticks_moving=authority.sticks_moving;a.rc_switches_seen=authority.switches_seen;
      a.rc_switches_fresh=authority.switches_fresh;a.rc_mode_slot=authority.mode_slot;
      a.rc_arm_switch=authority.arm_switch;a.rc_kill_switch=authority.kill_switch;
      a.rc_axes_safe=authority.rc_axes_safe;a.rc_throttle=authority.rc_throttle;
      a.rc_aux_switches_safe=authority.rc_aux_switches_safe;a.rc_switch_sample_stamp=authority.switch_sample_stamp;
      a.operator_switch_generation=authority.switch_generation;
      a.kill=authority.kill||(s.status_fresh&&status_.nav_state==Status::NAVIGATION_STATE_TERMINATION)||
        (a.failsafe_fresh&&flags_.fd_critical_failure);
      a.operator_takeover=s.status_fresh&&status_.failsafe_and_user_took_over;
      a.offboard_signal_lost=a.failsafe_fresh&&flags_.offboard_control_signal_lost;
      fb.exclusive=fb.exclusive&&graph.attitude==1;
    }
    if(config_.profile!="sim"&&(graph.battery!=1||graph.flags!=1))fb.exclusive=false;
    if(config_.attitude.visual.enabled){
      const auto clock_ns=get_clock()->now().nanoseconds();
      s.attitude.visual_input=visual_inbox_.snapshot(clock_ns>0?std::uint64_t(clock_ns):0,
        graph.visual_state,graph.visual_command);
    }
    ++work_.snapshots;work_.snapshot+=steady()-begin;
    return s;
  }
  std::string vertical_feedback_fields() const{
    const auto& e=runtime_->attitude_core().vertical_feedback_evidence();
    std::ostringstream out;
    out<<",\"vertical_feedback_guard_enabled\":"<<boolean(config_.attitude.feedback_guard.enabled)
       <<",\"z_deriv_ned\":"<<number(local_.z_deriv,local_stamp_.received>=0)
       <<",\"vertical_feedback_checked\":"<<boolean(e.checked)
       <<",\"vertical_feedback_healthy\":"<<boolean(e.healthy)
       <<",\"vertical_feedback_suspect\":"<<boolean(e.suspect)
       <<",\"vertical_feedback_rate_error_mps\":"<<number(e.rate_error_mps,e.checked)
       <<",\"vertical_feedback_position_rate_error_mps\":"<<number(e.position_rate_error_mps,e.position_window_checked)
       <<",\"vertical_feedback_disagreement_s\":"<<number(e.disagreement_seconds,e.checked)
       <<",\"vertical_feedback_disagreement_samples\":"<<e.disagreement_samples
       <<",\"vertical_feedback_sample_stamp\":"<<e.sample_stamp
       <<",\"vertical_feedback_reason\":"<<f::quote(e.reason);
    return out.str();
  }
  std::string vertical_control_fields(const f::attitude::Output& value,bool planned=false) const{
    const bool valid=value.publish&&value.vertical_closed_loop;
    const char* prefix=planned?"planned_":"";
    std::ostringstream out;
    out<<",\""<<prefix<<"vertical_closed_loop\":"<<boolean(valid)
       <<",\""<<prefix<<"takeoff_thrust_limited\":"<<boolean(valid&&value.takeoff_thrust_limited)
       <<",\""<<prefix<<"vertical_proportional_thrust\":"<<number(value.proportional_thrust,valid)
       <<",\""<<prefix<<"vertical_integral_thrust\":"<<number(value.integral_thrust,valid)
       <<",\""<<prefix<<"vertical_raw_thrust\":"<<number(value.raw_thrust,valid)
       <<",\""<<prefix<<"vertical_thrust_ceiling\":"<<number(value.thrust_ceiling,valid);
    return out.str();
  }
  void write_phase_event(const std::string& before,const f::Snapshot& s,
      const f::attitude::Output& planned,double now)const{
    const auto& core=runtime_->attitude_core();
    const auto fb=runtime_->publication_context(s,now,false,true).feedback;
    const auto& p=runtime_->pending(400);
    const auto& ag=core.armed_ground_fault_evidence();
    const auto& tilt=core.tilt_fault_evidence();
    const auto tilt_att=f::attitude::attitude_diagnostics(tilt.actual_q,tilt.requested_q);
    std::cout<<"{\"kind\":\"phase_event\",\"monotonic_s\":"<<number(now)
      <<",\"phase_before\":"<<f::quote(before)<<",\"phase_after\":"<<f::quote(runtime_->phase_name())
      <<",\"reason\":"<<f::quote(core.reason())<<progress_fields()<<vertical_feedback_fields()<<vertical_control_fields(planned,true)<<",\"ownership\":"<<f::quote(core.recovery_responsibility(fb))
      <<",\"armed\":"<<boolean(s.feedback.armed)<<",\"nav_state\":"<<s.nav_state
      <<",\"status_stamp\":"<<s.status_stamp<<",\"land_stamp\":"<<s.land_stamp
      <<",\"local_stamp\":"<<s.attitude.local_stamp<<",\"landed\":"<<boolean(s.feedback.landed)
      <<",\"at_rest\":"<<boolean(s.at_rest)
      <<",\"bench_ground_down_tolerance_m\":"<<number(config_.attitude.bench_ground_down_tolerance_m)
      <<",\"bench_ground_vz_tolerance_mps\":"<<number(config_.attitude.bench_ground_vz_tolerance_mps)
      <<",\"bench_ground_dwell_s\":"<<number(core.bench_ground_dwell_seconds())
      <<",\"bench_ground_first_land_stamp\":"<<core.bench_ground_first_land_stamp()
      <<",\"bench_ground_last_land_stamp\":"<<core.bench_ground_last_land_stamp()
      <<",\"attitude_fresh\":"<<boolean(s.attitude.attitude_fresh)
      <<",\"local_fresh\":"<<boolean(s.local_fresh)<<",\"status_fresh\":"<<boolean(s.status_fresh)
      <<",\"land_fresh\":"<<boolean(s.land_fresh)<<",\"failsafe_fresh\":"<<boolean(s.attitude.failsafe_fresh)
      <<",\"z_valid\":"<<boolean(s.attitude.z_valid)<<",\"vz_valid\":"<<boolean(s.attitude.vz_valid)
      <<",\"fault_class\":"<<f::quote(f::attitude::name(core.fault_class()))
      <<",\"tilt_fault_trigger_valid\":"<<boolean(tilt.valid)
      <<",\"tilt_fault_trigger_deg\":"<<number(tilt.measured_tilt_deg,tilt.valid)
      <<",\"tilt_fault_trigger_limit_deg\":"<<number(tilt.limit_deg,tilt.valid)
      <<",\"tilt_fault_trigger_roll_deg\":"<<number(tilt_att.roll_deg,tilt.valid&&tilt_att.valid)
      <<",\"tilt_fault_trigger_pitch_deg\":"<<number(tilt_att.pitch_deg,tilt.valid&&tilt_att.valid)
      <<",\"tilt_fault_trigger_error_deg\":"<<number(tilt_att.error_deg,tilt.valid&&tilt_att.valid)
      <<",\"tilt_fault_trigger_height\":"<<number(tilt.height,tilt.valid)
      <<",\"tilt_fault_trigger_up_speed\":"<<number(tilt.up_speed,tilt.valid)
      <<",\"tilt_fault_trigger_armed\":"<<boolean(tilt.armed)
      <<",\"tilt_fault_trigger_landed\":"<<boolean(tilt.landed)
      <<",\"tilt_fault_trigger_offboard\":"<<boolean(tilt.offboard)
      <<",\"tilt_fault_trigger_quat_reset\":"<<int(tilt.quat_reset)
      <<",\"tilt_fault_trigger_status_stamp\":"<<tilt.status_stamp
      <<",\"tilt_fault_trigger_land_stamp\":"<<tilt.land_stamp
      <<",\"tilt_fault_trigger_local_stamp\":"<<tilt.local_stamp
      <<",\"armed_ground_evidence_valid\":"<<boolean(ag.valid)
      <<",\"armed_ground_failed_mask\":"<<ag.failed_mask
      <<",\"armed_ground_sample_armed\":"<<boolean(ag.armed)
      <<",\"armed_ground_sample_offboard\":"<<boolean(ag.offboard)
      <<",\"armed_ground_sample_landed\":"<<boolean(ag.landed)
      <<",\"armed_ground_sample_z0\":"<<number(ag.z0,ag.valid)
      <<",\"armed_ground_sample_z\":"<<number(ag.z,ag.valid)
      <<",\"armed_ground_sample_vz\":"<<number(ag.vz,ag.valid)
      <<",\"armed_ground_sample_height\":"<<number(ag.height,ag.valid)
      <<",\"armed_ground_sample_up_speed\":"<<number(ag.up_speed,ag.valid)
      <<",\"armed_ground_height_limit\":"<<number(ag.height_limit,ag.valid)
      <<",\"armed_ground_vz_limit\":"<<number(ag.vz_limit,ag.valid)
      <<",\"armed_ground_down_height_limit\":"<<number(ag.downward_height_limit,ag.valid&&core.bench_arm_only_active())
      <<",\"armed_ground_status_stamp\":"<<ag.status_stamp
      <<",\"armed_ground_land_stamp\":"<<ag.land_stamp
      <<",\"armed_ground_local_stamp\":"<<ag.local_stamp
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
  std::string progress_fields()const{
    const auto& core=runtime_->attitude_core();const auto& e=core.liftoff_evidence();
    const char* pending=e.detector_samples<2?"WAIT_NEW_AIRBORNE_DETECTOR_SAMPLES":
      e.local_samples<2?"WAIT_NEW_LOCAL_MOTION_SAMPLES":
      !e.rise_confirmed?"WAIT_SUSTAINED_HEIGHT_AND_VELOCITY_AGREEMENT":"WAIT_JOINT_EVIDENCE";
    std::ostringstream out;
    out<<",\"takeoff_stage\":"<<f::quote(core.takeoff_stage())
      <<",\"takeoff_request_accepted\":"<<boolean(!core.bench_arm_only_active()&&(core.flight_started()||core.arm_confirmed()))
      <<",\"liftoff_confirmed\":"<<boolean(core.liftoff_confirmed())
      <<",\"target_height_settled\":"<<boolean(core.target_height_settled())
      <<",\"navigation_allowed\":"<<boolean(core.navigation_allowed())
      <<",\"liftoff_unconfirmed_reason\":"<<f::quote(core.liftoff_confirmed()?"":pending)
      <<",\"liftoff_detector_samples\":"<<e.detector_samples
      <<",\"liftoff_local_samples\":"<<e.local_samples
      <<",\"liftoff_detector_confirmed\":"<<boolean(e.detector_confirmed)
      <<",\"liftoff_rise_confirmed\":"<<boolean(e.rise_confirmed)
      <<",\"liftoff_first_land_stamp\":"<<e.first_land_stamp
      <<",\"liftoff_last_land_stamp\":"<<e.last_land_stamp
      <<",\"liftoff_first_local_sample_stamp\":"<<e.first_local_stamp
      <<",\"liftoff_last_local_sample_stamp\":"<<e.last_local_stamp;
    return out.str();
  }
  std::string work_fields(const Work& value,const char* field="cycle_work")const{
    std::ostringstream out;
    out<<",\""<<field<<"\":{\"callbacks_s\":"<<number(value.callbacks)
      <<",\"ipc_s\":"<<number(value.ipc)<<",\"runtime_s\":"<<number(value.runtime)
      <<",\"snapshot_s\":"<<number(value.snapshot)<<",\"graph_s\":"<<number(value.graph)
      <<",\"snapshot_calls\":"<<value.snapshots<<",\"graph_queries\":"<<value.graph_queries
      <<",\"boundary_s\":"<<number(value.boundary)<<",\"boundary_calls\":"<<value.contexts
      <<",\"reporting_s\":"<<number(value.reporting)<<"}";
    return out.str();
  }
  f::SourceTimingSet source_timings(double now,std::uint64_t host)const{
    const std::array<const f::Stamp*,6> stamps={&status_stamp_,&land_stamp_,&local_stamp_,&attitude_stamp_,&flags_stamp_,&battery_stamp_};
    const std::array<std::uint64_t,6> samples={0,0,local_.timestamp_sample,attitude_.timestamp_sample,0,0};
    const std::array<double,6> timeouts={config_.attitude.status_timeout,config_.attitude.land_timeout,
      config_.attitude.local_timeout,config_.attitude.attitude_timeout,config_.attitude.flags_timeout,1.};
    f::SourceTimingSet result;
    for(std::size_t i=0;i<stamps.size();++i){
      const bool sampled=i==std::size_t(f::SourceSlot::Local)||i==std::size_t(f::SourceSlot::Attitude);
      result[i]=f::diagnose_source_timing(stamps[i]->value,samples[i],sampled,host,now,stamps[i]->received,timeouts[i]);
    }
    return result;
  }
  std::string timing_evidence(double now,std::uint64_t host)const{
    std::ostringstream out;
    const auto age=[](f::AgeMeasurement value){return number(value.seconds,value.available);};
    out<<",\"controller_pid\":"<<getpid()<<",\"diagnostic_host_us\":"<<host
      <<",\"diagnostic_monotonic_s\":"<<number(now)
      <<",\"host_clock_anchor_error_s\":"<<age(clock_observation_.anchor_error)
      <<",\"host_clock_last_increment_error_s\":"<<age(clock_observation_.delta_error)
      <<",\"host_clock_event_count\":"<<clock_event_count_
      <<",\"host_clock_last_event\":"<<f::quote(f::host_clock_change_name(last_clock_event_.change))
      <<",\"host_clock_last_event_us\":"<<last_clock_event_stamp_
      <<",\"host_clock_last_event_delta_error_s\":"<<age(last_clock_event_.delta_error)
      <<",\"source_timing\":{";
    const std::array<const f::Stamp*,6> stamps={&status_stamp_,&land_stamp_,&local_stamp_,&attitude_stamp_,&flags_stamp_,&battery_stamp_};
    const std::array<std::uint64_t,6> samples={0,0,local_.timestamp_sample,attitude_.timestamp_sample,0,0};
    const std::array<double,6> timeouts={config_.attitude.status_timeout,config_.attitude.land_timeout,
      config_.attitude.local_timeout,config_.attitude.attitude_timeout,config_.attitude.flags_timeout,1.};
    const auto sources=source_timings(now,host);
    for(std::size_t i=0;i<stamps.size();++i){
      if(i)out<<',';
      const auto& stamp=*stamps[i];const auto& receipt=receipt_timing_[i];
      const bool sampled=i==std::size_t(f::SourceSlot::Local)||i==std::size_t(f::SourceSlot::Attitude);
      const auto& item=sources[i];
      out<<f::quote(f::source_slot_name(f::SourceSlot(i)))<<":{\"envelope_us\":"<<stamp.value
        <<",\"sample_us\":"<<samples[i]<<",\"timeout_s\":"<<number(timeouts[i])
        <<",\"envelope_age_s\":"<<age(item.envelope_age)<<",\"sample_age_s\":"<<age(item.sample_age)
        <<",\"receipt_age_s\":"<<age(item.receipt_age)<<",\"envelope_sample_gap_s\":"<<age(item.envelope_sample_gap)
        <<",\"source_reason\":"<<f::quote(f::source_time_reason_name(item.source_reason))
        <<",\"receipt_reason\":"<<f::quote(f::receipt_time_reason_name(item.receipt_reason))
        <<",\"stamp_regressed\":"<<boolean(stamp.regressed)
        <<",\"last_callback_accepted\":"<<boolean(receipt_accepted_[i])
        <<",\"last_callback_envelope_us\":"<<receipt.envelope_stamp
        <<",\"last_callback_monotonic_s\":"<<number(receipt.steady_stamp,receipt.steady_stamp>=0)
        <<",\"callback_envelope_age_s\":"<<age(receipt.envelope_host_age)
        <<",\"callback_sample_age_s\":"<<age(receipt.sample_host_age)
        <<",\"rmw_to_callback_s\":"<<age(receipt.middleware_callback_age);
      if(sampled){
        const auto& progress=i==std::size_t(f::SourceSlot::Local)?local_sample_:attitude_sample_;
        out<<",\"sample_progress_age_s\":"<<number(now-progress.received,progress.received>=0)
          <<",\"sample_regressed\":"<<boolean(progress.regressed);
      }
      out<<'}';
    }
    out<<"},\"pair_context_calls\":"<<pair_context_count_<<",\"pair_context_seconds\":["
      <<number(pair_context_seconds_[0],pair_context_count_>0)<<','
      <<number(pair_context_seconds_[1],pair_context_count_>1)<<','
      <<number(pair_context_seconds_[2],pair_context_count_>2)<<']';
    return out.str();
  }
  std::string feedback_timing_fields(const f::Snapshot& s,double now)const{
    std::ostringstream out;
    const auto host=s.attitude.source_host_stamp;
    const auto age=[host](std::uint64_t stamp){return number((double(host)-double(stamp))*1e-6,host&&stamp);};
    out<<",\"land_age_s\":"<<number(now-land_stamp_.received,land_stamp_.received>=0)
      <<",\"flags_age_s\":"<<number(now-flags_stamp_.received,flags_stamp_.received>=0)
      <<",\"battery_age_s\":"<<number(now-battery_stamp_.received,battery_stamp_.received>=0)
      <<",\"status_source_age_s\":"<<age(status_stamp_.value)
      <<",\"land_source_age_s\":"<<age(land_stamp_.value)
      <<",\"local_source_age_s\":"<<age(local_stamp_.value)
      <<",\"local_sample_age_s\":"<<age(local_.timestamp_sample)
      <<",\"attitude_source_age_s\":"<<age(attitude_stamp_.value)
      <<",\"attitude_sample_age_s\":"<<age(attitude_.timestamp_sample)
      <<",\"flags_source_age_s\":"<<age(flags_stamp_.value)
      <<",\"battery_source_age_s\":"<<age(battery_stamp_.value)
      <<",\"last_cycle_callbacks_s\":"<<number(timing_.callback_seconds,timing_.valid)
      <<",\"last_cycle_control_s\":"<<number(timing_.control_seconds,timing_.valid)
      <<",\"last_cycle_period_s\":"<<number(timing_.period_seconds,timing_.period_valid)
      <<",\"max_cycle_period_s\":"<<number(timing_.max_period_seconds,timing_.period_valid)
      <<",\"cycle_stall_count\":"<<timing_.stall_count
      <<timing_evidence(source_observed_steady_,source_observed_host_)
      <<work_fields(completed_work_,"last_completed_cycle_work");
    return out.str();
  }
  std::string state(const f::Snapshot& s,double now,const char* kind="state",std::uint64_t seq=0,bool accepted=true,const std::string& message="")const{
    std::ostringstream out;out<<"{\"kind\":"<<f::quote(kind)<<",\"seq\":"<<seq<<",\"accepted\":"<<boolean(accepted)<<",\"message\":"<<f::quote(message)
      <<",\"phase\":"<<f::quote(runtime_->phase_name())<<",\"active\":"<<boolean(runtime_->active())<<",\"profile\":"<<f::quote(config_.profile)<<",\"flight_enabled\":"<<boolean(config_.flight_enabled)
      <<','<<f::attitude::flight_gates_json(config_.attitude)
      <<','<<runtime_->authority_json(s)
      <<",\"rc_navigation_enabled\":"<<boolean(config_.attitude.rc_navigation_enabled)
      <<",\"visual_navigation_enabled\":"<<boolean(config_.attitude.visual.enabled)
      <<",\"visual_motion\":"<<boolean(last_attitude_output_.visual_motion)
      <<",\"visual_reason\":"<<f::quote(runtime_->attitude_core().visual_result().reason)
      <<",\"visual_mode\":"<<int(runtime_->attitude_core().visual_result().mode)
      <<",\"visual_state_sequence\":"<<s.attitude.visual_input.state.sequence
      <<",\"visual_command_sequence\":"<<s.attitude.visual_input.command.sequence
      <<",\"visual_state_valid_fields\":"<<s.attitude.visual_input.state.valid_fields
      <<",\"visual_reset_counter\":"<<s.attitude.visual_input.state.reset_counter
      <<",\"visual_state_age_s\":"<<number(now-s.attitude.visual_input.state.received_at,s.attitude.visual_input.state.received_at>=0)
      <<",\"visual_command_age_s\":"<<number(now-s.attitude.visual_input.command.received_at,s.attitude.visual_input.command.received_at>=0)
      <<",\"rc_target_height\":"<<number(config_.attitude.rc_target_height)
      <<progress_fields()<<feedback_timing_fields(s,now)
      <<",\"rc_navigation_mode_slot\":"<<config_.attitude.rc_navigation_mode_slot
      <<",\"rc_prestream_commits\":"<<runtime_->attitude_core().rc_prestream_commits()
      <<",\"rc_manual_arm_proven\":"<<boolean(runtime_->attitude_core().rc_manual_arm_proven())
      <<",\"rc_axes_safe\":"<<boolean(s.attitude.rc_axes_safe)
      <<",\"rc_throttle\":"<<number(s.attitude.rc_throttle)
      <<",\"native_log_dropped\":"<<f::native_log_dropped.load()
      <<",\"target_local_commits\":"<<publication_audit_.target_count
      <<",\"heartbeat_local_commits\":"<<publication_audit_.mode_count
      <<",\"last_target_local_thrust_z\":"<<number(publication_audit_.last_target_thrust_z,publication_audit_.target_count>0)
      <<",\"last_target_local_commit_monotonic_s\":"<<number(publication_audit_.target_last,publication_audit_.target_last>=0)
      <<",\"last_heartbeat_local_commit_monotonic_s\":"<<number(publication_audit_.mode_last,publication_audit_.mode_last>=0)
      <<",\"px4_thrust_adoption\":\"UNOBSERVED\",\"physical_liftoff\":\"EXTERNAL_OBSERVATION_REQUIRED\""
      <<",\"heartbeat_enabled\":"<<boolean(last_attitude_output_.publish)<<",\"attitude_output_enabled\":"<<boolean(last_attitude_output_.publish)
      <<",\"publication_block_reason\":"<<f::quote(publication_block_reason_)<<",\"publication_fault_latched\":"<<boolean(boundary_fault_latched_)
      <<",\"attitude_fresh\":"<<boolean(s.attitude.attitude_fresh)<<",\"attitude_healthy_output\":"<<boolean(last_attitude_output_.publish)
      <<",\"yaw_reference_rad\":"<<number(runtime_->attitude_core().yaw_reference(),runtime_->attitude_core().reference_ready())
      <<",\"attitude_reset_counter\":"<<int(attitude_.quat_reset_counter)<<",\"attitude_age_s\":"<<number(now-attitude_stamp_.received,attitude_stamp_.received>=0)
      <<",\"attitude_q\":["<<number(attitude_.q[0],s.attitude.attitude_fresh)<<','<<number(attitude_.q[1],s.attitude.attitude_fresh)<<','<<number(attitude_.q[2],s.attitude.attitude_fresh)<<','<<number(attitude_.q[3],s.attitude.attitude_fresh)<<"]"
      <<",\"attitude_event\":"<<f::quote(runtime_->attitude_core().event())
      <<",\"m1_fault_class\":"<<f::quote(f::attitude::name(runtime_->attitude_core().fault_class()))
      <<",\"armed_ground_evidence_valid\":"<<boolean(runtime_->attitude_core().armed_ground_fault_evidence().valid)
      <<",\"armed_ground_failed_mask\":"<<runtime_->attitude_core().armed_ground_fault_evidence().failed_mask
      <<",\"m1_recovery_budget_exhausted\":"<<boolean(runtime_->attitude_core().recovery_budget_exhausted())
      <<",\"bench_operator_required\":"<<boolean(runtime_->attitude_core().bench_wait_overdue(now)&&s.feedback.armed)
      <<",\"m1_recovery_responsibility\":"<<f::quote(runtime_->attitude_core().recovery_responsibility(runtime_->publication_context(s,now,server_->connected(),true).feedback))
      <<",\"m1_operator_release_requested\":"<<boolean(runtime_->attitude_core().operator_release_requested())
      <<",\"m1_400_requested\":"<<boolean(runtime_->pending(400).requested)
      <<",\"m1_400_intent\":"<<f::quote(f::command400_name(runtime_->pending(400).command400_kind))
      <<",\"m1_400_transaction_stage\":"<<f::quote(f::command_stage_name(runtime_->pending(400).stage))
      <<",\"m1_400_wire_stamp\":"<<runtime_->pending(400).wire_stamp
      <<",\"m1_400_causal_floor\":"<<runtime_->pending(400).causal_floor
      <<",\"m1_400_clock_unattributed_acks\":"<<runtime_->pending(400).unattributed_clock_acks
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
      <<",\"m1_176_ack_policy\":\"R632_ACK176_STATE_CANDIDATE_V1\""
      <<",\"m1_176_ack_candidate\":"<<boolean(runtime_->pending(176).ack_candidate)
      <<",\"m1_176_ack_accepted\":"<<boolean(runtime_->pending(176).accepted)
      <<",\"m1_176_ack_confirmed_by_state\":"<<boolean(runtime_->pending(176).ack_confirmed_by_state)
      <<",\"m1_176_ack_disposition\":"<<f::quote(runtime_->pending(176).ack_disposition)
      <<",\"m1_176_transaction_stage\":"<<f::quote(f::command_stage_name(runtime_->pending(176).stage))
      <<",\"m1_176_local_generation\":"<<runtime_->pending(176).generation
      <<",\"m1_176_wire_stamp\":"<<runtime_->pending(176).wire_stamp
      <<",\"m1_176_causal_floor\":"<<runtime_->pending(176).causal_floor
      <<",\"m1_176_ack_stamp\":"<<runtime_->pending(176).ack_stamp
      <<",\"m1_176_clock_unattributed_acks\":"<<runtime_->pending(176).unattributed_clock_acks
      <<",\"m1_176_verified_status_stamp\":"<<runtime_->pending(176).verified_status_stamp
      <<",\"attitude_setpoint_q\":["<<number(last_attitude_output_.q[0])<<','<<number(last_attitude_output_.q[1])<<','<<number(last_attitude_output_.q[2])<<','<<number(last_attitude_output_.q[3])<<"]"
      <<",\"thrust_body\":["<<number(last_attitude_output_.thrust[0])<<','<<number(last_attitude_output_.thrust[1])<<','<<number(last_attitude_output_.thrust[2])<<"]"
      <<",\"nav_state\":"<<s.nav_state<<",\"armed\":"<<boolean(s.feedback.armed)<<",\"landed\":"<<boolean(s.feedback.landed)<<",\"at_rest\":"<<boolean(s.at_rest)
      <<",\"status_fresh\":"<<boolean(s.status_fresh)<<",\"local_fresh\":"<<boolean(s.local_fresh)
      <<",\"land_fresh\":"<<boolean(s.land_fresh)<<",\"failsafe_fresh\":"<<boolean(s.attitude.failsafe_fresh)
      <<",\"battery_fresh\":"<<boolean(s.battery_fresh)
      <<",\"source_clock_valid\":"<<boolean(s.attitude.source_clock_valid)
      <<",\"source_clock_validated\":"<<boolean(s.attitude.source_clock_validated)
      <<",\"local_timestamp_sample\":"<<local_.timestamp_sample
      <<",\"attitude_timestamp_sample\":"<<attitude_.timestamp_sample
      <<",\"local_sample_fresh\":"<<boolean(s.attitude.local_sample_fresh)
      <<",\"attitude_sample_fresh\":"<<boolean(s.attitude.attitude_sample_fresh)
      <<",\"rc_source_clock_valid\":"<<boolean(s.attitude.rc_source_clock_valid)
      <<",\"rc_switch_clock_valid\":"<<boolean(s.attitude.rc_switch_clock_valid)
      <<",\"estimate_valid\":"<<boolean(s.feedback.fresh&&s.feedback.valid&&s.feedback.heading_ok&&!s.dead_reckoning)
      <<",\"arming_state_known\":"<<boolean(s.arming_state_known)<<",\"arming_state_raw\":"<<int(status_.arming_state)
      <<",\"xy_valid\":"<<boolean(s.local_fresh&&local_.xy_valid)<<",\"z_valid\":"<<boolean(s.local_fresh&&local_.z_valid)<<",\"vxy_valid\":"<<boolean(s.local_fresh&&local_.v_xy_valid)<<",\"vz_valid\":"<<boolean(s.local_fresh&&local_.v_z_valid)
      <<",\"heading_good\":"<<boolean(s.local_fresh&&local_.heading_good_for_control)<<",\"dead_reckoning\":"<<boolean(s.dead_reckoning)<<",\"preflight\":"<<boolean(s.status_fresh&&s.feedback.preflight)
      <<",\"manual_signal\":"<<boolean(s.manual_fresh&&!s.manual_lost)<<",\"battery_connected\":"<<boolean(s.battery_fresh&&s.battery_connected)<<",\"voltage_v\":"<<number(battery_.voltage_v,s.battery_fresh&&s.battery_connected)
      <<",\"status_age_s\":"<<number(now-status_stamp_.received,status_stamp_.received>=0)
      <<",\"local_age_s\":"<<number(now-local_stamp_.received,local_stamp_.received>=0)<<",\"position_ned\":["<<number(local_.x,s.local_fresh&&local_.xy_valid)<<','<<number(local_.y,s.local_fresh&&local_.xy_valid)<<','<<number(local_.z,s.local_fresh&&local_.z_valid)<<"]"
      <<",\"velocity_ned\":["<<number(local_.vx,s.local_fresh&&local_.v_xy_valid)<<','<<number(local_.vy,s.local_fresh&&local_.v_xy_valid)<<','<<number(local_.vz,s.local_fresh&&local_.v_z_valid)<<"]"
      <<",\"acceleration_ned_estimate\":["<<number(local_.ax,s.local_fresh&&local_.v_xy_valid)<<','<<number(local_.ay,s.local_fresh&&local_.v_xy_valid)<<','<<number(local_.az,s.local_fresh&&local_.v_z_valid)<<"]"
      <<",\"relative_height_m\":"<<number(reference_z()-local_.z,reference_ready()&&s.local_fresh&&local_.z_valid)
      <<",\"height_reference_ready\":"<<boolean(reference_ready())
      <<",\"height_unavailable_reason\":"<<f::quote(!reference_ready()?"REFERENCE_NOT_READY":
         !s.local_fresh?"LOCAL_POSITION_STALE":
         !local_.z_valid||!std::isfinite(local_.z)||!std::isfinite(reference_z())?"Z_ESTIMATE_INVALID":"")
      <<vertical_feedback_fields()<<vertical_control_fields(last_attitude_output_)
      <<",\"ground_test\":"<<f::quote(runtime_->ground_name())<<",\"ground_reason\":"<<f::quote(runtime_->ground_reason())<<",\"reason\":"<<f::quote(runtime_->message())
      <<",\"source_counts\":["<<s.traffic.status_publishers<<','<<s.traffic.land_publishers<<','<<s.traffic.ack_publishers<<"]"
      <<",\"offboard_ack\":"<<runtime_->pending(176).result<<",\"arm_ack\":"<<runtime_->pending(400).result<<",\"land_ack\":"<<runtime_->pending(21).result<<'}';return out.str();
  }
  void write_csv(const f::Snapshot& s,double now){if(!csv_)return;csv_<<std::setprecision(9)<<now<<','<<runtime_->phase_name()<<','<<s.nav_state<<','<<s.feedback.armed<<','<<s.feedback.landed<<','<<local_.xy_valid<<','<<local_.z_valid<<','<<local_.v_xy_valid<<','<<local_.v_z_valid<<','<<local_.heading_good_for_control<<','<<number(now-local_stamp_.received,local_stamp_.received>=0)<<','<<local_.x<<','<<local_.y<<','<<local_.z<<','<<local_.vx<<','<<local_.vy<<','<<local_.vz<<','<<local_.ax<<','<<local_.ay<<','<<local_.az<<','<<number(reference_z()-local_.z,reference_ready()&&s.local_fresh&&local_.z_valid)<<','<<f::quote(runtime_->message())<<','<<last_attitude_output_.publish<<','<<number(runtime_->attitude_core().yaw_reference())<<','<<last_attitude_output_.thrust[2]<<','<<last_attitude_output_.target_height<<','<<last_attitude_output_.velocity_target<<','<<last_attitude_output_.saturated<<','<<last_attitude_output_.slew_limited<<','<<number(local_.z_deriv,s.local_fresh)<<','<<runtime_->attitude_core().vertical_feedback_evidence().checked<<','<<runtime_->attitude_core().vertical_feedback_evidence().suspect<<','<<number(runtime_->attitude_core().vertical_feedback_evidence().rate_error_mps,runtime_->attitude_core().vertical_feedback_evidence().checked)<<','<<number(runtime_->attitude_core().vertical_feedback_evidence().position_rate_error_mps,runtime_->attitude_core().vertical_feedback_evidence().position_window_checked)<<','<<runtime_->attitude_core().vertical_feedback_evidence().disagreement_seconds<<','<<f::quote(runtime_->attitude_core().vertical_feedback_evidence().reason)
      <<','<<last_attitude_output_.vertical_closed_loop<<','<<last_attitude_output_.takeoff_thrust_limited
      <<','<<number(last_attitude_output_.proportional_thrust,last_attitude_output_.vertical_closed_loop)
      <<','<<number(last_attitude_output_.integral_thrust,last_attitude_output_.vertical_closed_loop)
      <<','<<number(last_attitude_output_.raw_thrust,last_attitude_output_.vertical_closed_loop)
      <<','<<number(last_attitude_output_.thrust_ceiling,last_attitude_output_.vertical_closed_loop)<<'\n';csv_.flush();}
  f::Config config_;std::unique_ptr<f::Runtime> runtime_;std::unique_ptr<f::UnixServer> server_;
  bool boundary_fault_latched_{};std::string publication_block_reason_;
  static constexpr const char* preparation_age_wait_="RC_PREPARATION_SOURCE_AGE_EXPIRED_WAIT_NEW_DATA";
  Attitude attitude_{};f::attitude::Output last_attitude_output_;
  f::AuthorityTelemetry authority_telemetry_;
  f::vision::Inbox visual_inbox_;std::string visual_state_topic_,visual_command_topic_;
  rclcpp::Subscription<VisualState>::SharedPtr visual_state_sub_;
  rclcpp::Subscription<VisualCommand>::SharedPtr visual_command_sub_;
  Local local_{};Status status_{};Land land_{};Battery battery_{};Flags flags_{};
  f::Stamp local_stamp_,status_stamp_,land_stamp_,battery_stamp_,flags_stamp_,attitude_stamp_;
  f::SourceSampleProgress local_sample_,attitude_sample_;
  bool clock_regressed_{};std::uint64_t last_wall_stamp_{};std::uint8_t ground_z_reset_{},ground_vz_reset_{};
  f::HostClockMonitor host_clock_;
  f::HostClockObservation clock_observation_,last_clock_event_;
  std::uint64_t clock_event_count_{},last_clock_event_stamp_{};
  double source_observed_steady_{};std::uint64_t source_observed_host_{};
  std::array<f::SourceReceiptTiming,std::size_t(f::SourceSlot::Count)> receipt_timing_;
  std::array<bool,std::size_t(f::SourceSlot::Count)> receipt_accepted_{};
  f::CurrentControlCycle cycle_;Work work_,completed_work_;
  bool sampling_pair_{};std::size_t pair_context_count_{};
  std::array<double,3> pair_context_seconds_{};double last_timing_warning_{-1},last_defer_report_{-1};
  f::ControlTiming timing_;
  double printed_{},rest_since_{-1},velocity_reference_z_{},stop_since_{},last_shutdown_warning_{};
  bool velocity_reference_set_{},stopping_{},heading_changed_{};std::uint8_t heading_reset_{};
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
  int code=0;std::unique_ptr<f::AsyncLog> logs;
  try{logs=std::make_unique<f::AsyncLog>();rclcpp::init(argc,argv);std::signal(SIGINT,stop_handler);std::signal(SIGTERM,stop_handler);
    auto node=std::make_shared<SystemNode>();
    // Keep one executor and the original single-threaded callback/tick order.
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    while(rclcpp::ok()){
      const auto deadline=std::chrono::steady_clock::now()+node->loop_period();
      const double start=steady();
      node->begin_cycle(start);
      executor.spin_some();
      const double callbacks_done=steady();
      node->callbacks_done(callbacks_done);
      const bool running=node->tick();
      node->observe_cycle(start,callbacks_done,steady());
      if(!running)break;
      std::this_thread::sleep_until(deadline);
    }
    executor.remove_node(node);}
  catch(const std::exception& error){std::cerr<<"[ERROR] "<<error.what()<<'\n';code=2;}
  rclcpp::shutdown();return code;
}
