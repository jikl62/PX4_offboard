// ROS 2 适配主节点：把 PX4 反馈与离散安全事件交给纯计算 Runtime，再经最终边界发布获准的控制计划。
// main 使用单线程执行器，回调、快照、Core 和发布串行执行；只有 AsyncLog 在后台写盘。
// 本文件同时包含诊断序列化；日志里的本地 publish 成功不等于 PX4 收到、采用或飞机真实离地。
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
#include "jikl_fc01_system/receive_schedule.hpp"
#include "jikl_fc01_system/discrete_event_latch.hpp"
#include "jikl_fc01_system/receive_events.hpp"
#include "jikl_fc01_system/parent_lifetime.hpp"
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

// 本地别名缩短协议类型；实际 PX4 消息身份与版本在下方编译期断言中固定。
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
// 信号处理器只写 sig_atomic_t 标志；不能在异步信号上下文发布 ROS、拿锁或运行降落控制。
volatile std::sig_atomic_t stop_requested=0;
void stop_handler(int){stop_requested=1;}
// 单调时钟秒数只用于本机周期、租约和耗时；不能直接填入 PX4 的同步微秒时间戳。
double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
// JSON 数值序列化：无效或非有限值输出 null，防止把缺失测量伪装成数值零。
std::string number(double value,bool valid=true){if(!valid||!std::isfinite(value))return "null";std::ostringstream out;out<<std::setprecision(9)<<value;return out.str();}
const char* boolean(bool value){return value?"true":"false";}
// IPC 路径优先使用环境显式设置，否则放入当前用户缓存；缺 HOME 时拒绝猜测目录。
std::string default_socket(){const char* env=std::getenv("FC01_SOCKET");if(env)return env;const char* home=std::getenv("HOME");if(!home)throw std::runtime_error("HOME missing");return std::string(home)+"/.cache/fc01/control.sock";}
}
// 该类独占 Runtime 和 UnixServer；构造只建立端点，实际控制由 main 每轮接收后调用 tick。
class SystemNode final:public rclcpp::Node {
 public:
  // 初始化失败直接抛到 main；所有参数验证先于创建飞行发布链。真实 sink 提交模式禁止用计划值代替已发送。
  SystemNode():Node("fc01_system"){
    f::Config config=f::read_ros_config(*this);
    config.validate();runtime_=std::make_unique<f::Runtime>(config);runtime_->require_attitude_sink_commit();config_=config;
    const std::string socket=declare_parameter<std::string>("socket_path",default_socket());
    // socket 父目录必须属于当前用户且不可被其他用户写入；避免外部替换控制入口。
    const auto parent=std::filesystem::path(socket).parent_path();
    std::filesystem::create_directories(parent);
    struct stat parent_stat{};
    if(lstat(parent.c_str(),&parent_stat)!=0||!S_ISDIR(parent_stat.st_mode)||parent_stat.st_uid!=getuid()||(parent_stat.st_mode&0022))throw std::runtime_error("socket directory must be owned and not writable by other users");
    server_=std::make_unique<f::UnixServer>(socket);
    // PX4 使用传感器 QoS、深度 10；队列深度只决定保留量，不替代源时间和测量时间校验。
    const auto qos=rclcpp::SensorDataQoS().keep_last(10);
    // 只有显式启用既有视觉导航时才订阅 VisionState/NavigationCommand。
    // 新 VelocityCommand 由独立诊断工具只读订阅，此处不授予它飞行权限。
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
        [this](VisualState::ConstSharedPtr m){++received_callbacks_;++receive_events_;visual_inbox_.state(f::vision::state_from_message(*m),steady());});
      visual_command_sub_=create_subscription<VisualCommand>(visual_command_topic_,command_qos,
        [this](VisualCommand::ConstSharedPtr m){++received_callbacks_;++receive_events_;visual_inbox_.command(f::vision::command_from_message(*m),steady());});
      std::cout<<"{\"kind\":\"visual_inputs\",\"state_topic\":"<<f::quote(visual_state_topic_)
        <<",\"command_topic\":"<<f::quote(visual_command_topic_)<<",\"state_reliable\":"<<boolean(state_reliable)
        <<",\"command_reliable\":"<<boolean(command_reliable)<<",\"qos_depth\":"<<depth<<"}"<<std::endl;
    }
    if(config.profile=="attitude_altitude"){
      // 姿态原始测量必须同时推进 envelope 与 timestamp_sample；无效四元数或重置计数变化另存队列；普通有效姿态更新仍走连续缓存。
      attitude_sub_=create_subscription<Attitude>("/fmu/out/vehicle_attitude",qos,[this](Attitude::ConstSharedPtr m,const rclcpp::MessageInfo& info){const auto previous=attitude_;receive_sampled(*m,attitude_,attitude_stamp_,attitude_sample_,f::SourceSlot::Attitude,info);
        if(f::attitude_receive_event(previous,*m,config_.attitude)){latch_event(attitude_events_,*m);++receive_events_;}});
      manual_sub_=create_subscription<Manual>("/fmu/out/manual_control_setpoint",qos,[this](Manual::ConstSharedPtr m){receive_manual(*m);});
      switches_sub_=create_subscription<Switches>("/fmu/out/manual_control_switches",qos,[this](Switches::ConstSharedPtr m){receive_switches(*m);});
    }
    // 本地位置/速度采用 PX4 NED：z、vz 向下为正，单位分别 m、m/s；离散有效性/重置变化单独留证。
    local_sub_=create_subscription<Local>("/fmu/out/vehicle_local_position",qos,[this](Local::ConstSharedPtr m,const rclcpp::MessageInfo& info){const auto previous=local_;receive_sampled(*m,local_,local_stamp_,local_sample_,f::SourceSlot::Local,info);
      const auto& core=runtime_->attitude_core();
      const bool vertical_only=config_.profile=="attitude_altitude"&&
        !config_.attitude.visual.enabled&&!config_.attitude.px4_land_fallback_confirmed;
      if(f::local_receive_event(previous,*m,config_.attitude,vertical_only)||
         core.local_requires_control_step(m->timestamp_sample,m->z,m->vz,m->z_deriv)){latch_event(local_events_,*m);++receive_events_;}});
    // 状态反馈固定使用 PX4 v1.16 的版本话题；模式、解锁、接管等边沿不能被末尾正常帧覆盖。
    status_sub_=create_subscription<Status>("/fmu/out/vehicle_status_v1",qos,[this](Status::ConstSharedPtr m,const rclcpp::MessageInfo& info){receive_event(*m,status_,status_stamp_,f::SourceSlot::Status,info,f::status_receive_event<Status>);});
    land_sub_=create_subscription<Land>("/fmu/out/vehicle_land_detected",qos,[this](Land::ConstSharedPtr m,const rclcpp::MessageInfo& info){receive_event(*m,land_,land_stamp_,f::SourceSlot::Land,info,f::land_receive_event<Land>);});
    battery_sub_=create_subscription<Battery>("/fmu/out/battery_status",qos,[this](Battery::ConstSharedPtr m,const rclcpp::MessageInfo& info){
      if(m->id==1)receive_event(*m,battery_,battery_stamp_,f::SourceSlot::Battery,info,f::battery_receive_event<Battery>);
      else ++received_callbacks_;});
    flags_sub_=create_subscription<Flags>("/fmu/out/failsafe_flags",qos,[this](Flags::ConstSharedPtr m,const rclcpp::MessageInfo& info){receive_event(*m,flags_,flags_stamp_,f::SourceSlot::Flags,info,f::flags_receive_event<Flags>);});
    // ACK 交给 Runtime 的真实提交/时间/编号归属校验；收到 ACK 本身并不证明动作已物理完成。
    ack_sub_=create_subscription<Ack>("/fmu/out/vehicle_command_ack",qos,[this](Ack::ConstSharedPtr m){
      ++received_callbacks_;++receive_events_;
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
    // 按 profile 只创建允许的控制目标发布器；姿态高度路线不同时拥有 trajectory 输出。
    const auto publishers=f::attitude::publisher_plan(config.profile);
    if(publishers.mode)mode_pub_=create_publisher<Mode>("/fmu/in/offboard_control_mode",10);
    if(publishers.trajectory)target_pub_=create_publisher<Target>("/fmu/in/trajectory_setpoint",10);
    if(publishers.attitude)attitude_pub_=create_publisher<AttitudeTarget>("/fmu/in/vehicle_attitude_setpoint",10);
    graph_event_=get_graph_event();
    // CSV 为可选同步写盘出口；打开后每条会 flush，耗时计入周期，生产默认配置保持关闭。
    const auto logpath=declare_parameter<std::string>("csv_path","");
    if(!logpath.empty()){
      csv_.open(logpath,std::ios::app);
      if(!csv_)throw std::runtime_error("cannot open CSV log");
      csv_<<"steady_s,phase,nav_state,armed,landed,xy_valid,z_valid,vxy_valid,vz_valid,heading_good,local_age_s,x_ned,y_ned,z_ned,vx_ned,vy_ned,vz_ned,ax_ned,ay_ned,az_ned,relative_height_m,reason,aa_healthy,aa_yaw_reference,aa_thrust_z,aa_height_target,aa_velocity_target,aa_saturated,aa_slew_limited,aa_z_deriv_ned,aa_feedback_checked,aa_feedback_suspect,aa_feedback_rate_error_mps,aa_feedback_position_rate_error_mps,aa_feedback_disagreement_s,aa_feedback_reason,aa_vertical_closed_loop,aa_takeoff_thrust_limited,aa_p_thrust,aa_i_thrust,aa_raw_thrust,aa_thrust_ceiling\n";
    }
    std::cout<<"{\"kind\":\"startup\",\"profile\":"<<f::quote(config.profile)
      <<",\"controller_build\":\"FC01_RECOVERY_DIAGNOSTIC_V5_20261110\""
      <<",\"lifecycle_policy\":\"FC01_SAFE_LIFECYCLE_V1_20261109\""
      <<",\"safety_event_policy\":\"R2_DISCRETE_EVENT_PREFLIGHT_V2_20261108\""
      <<",\"runtime_mode\":"<<f::quote(config.runtime_mode.empty()?"unspecified":config.runtime_mode)
      <<",\"entry_version\":"<<f::quote(config.entry_version)
      <<",\"config_path\":"<<f::quote(config.config_path)
      <<",\"config_source_path\":"<<f::quote(config.config_source_path)
      <<",\"flight_enabled\":"<<boolean(config.flight_enabled)
      <<",\"flight_actions_enabled\":"<<boolean(config.attitude.flight_actions_enabled)
      <<",\"arm_enabled\":"<<boolean(config.attitude.arm_enabled)
      <<",\"takeoff_enabled\":"<<boolean(config.attitude.takeoff_enabled)
      <<",\"rc_navigation_enabled\":"<<boolean(config.attitude.rc_navigation_enabled)
      <<",\"visual_navigation_enabled\":"<<boolean(config.attitude.visual.enabled)
      <<",\"aa_hover_thrust\":"<<number(config.attitude.vertical.hover)
      <<",\"aa_takeoff_reserve_thrust\":"<<number(config.attitude.takeoff_reserve_thrust)
      <<",\"aa_thrust_max\":"<<number(config.attitude.vertical.maximum)
      <<",\"rc_target_height\":"<<number(config.attitude.rc_target_height)
      <<",\"automatic_arm\":false,\"rc_navigation_policy\":\"RC_NAVIGATION_MANUAL_ARM_V1\",\"takeoff_policy\":\"NAV_TAKEOFF_EVIDENCE_V2_20261103\",\"vertical_control_policy\":\"R632_SCHEDULED_TAKEOFF_CEILING_V1\",\"socket\":"<<f::quote(socket)<<"}"<<std::endl;
  }
  // 控制周期来自已校验 loop_hz；秒用于 steady deadline，不与 ROS 时间混算。
  std::chrono::duration<double> loop_period() const{return std::chrono::duration<double>(1./config_.attitude.loop_hz);}
  // 每轮重置瞬时耗时、图缓存与成对发布样本；进程级时钟故障、源水位和事务历史继续保留。
  void begin_cycle(double start){
    cycle_.begin(start);work_={};work_.started=start;
    ++cycle_id_;cycle_graph_ready_=cycle_graph_invalidated_=false;
    pair_attempt_={};pair_samples_={};
    pair_context_count_=0;pair_context_seconds_.fill(0);
    source_cycle_callbacks_.fill(0);
  }
  std::uint64_t received_callbacks()const{return received_callbacks_;}
  std::uint64_t receive_events()const{return receive_events_;}
  // 额外接收轮次最多使用周期十分之一的合作预算；无法抢占正在执行的单次回调。
  double receive_budget_seconds()const{return loop_period().count()/10.;}
  // 可输出任务遇离散事件尽快进入安全计算；完全静默的 Safety 可有界追赶队列，但危险事件仍全部保留。
  bool stop_receive_on_events()const{
    // A stopped task still retains every payload for the same-cycle recovery
    // veto. Extra bounded passes let its cached DDS samples catch up instead
    // of repeatedly stopping on an ended flight's reference or invalid XY.
    return config_.profile!="attitude_altitude"||
      runtime_->attitude_core().phase()!=f::attitude::Phase::Safety||
      last_attitude_output_.publish;
  }
  void receive_pass_done(){++work_.receive_passes;}
  void receive_stopped(const f::ReceivePassSchedule& schedule){receive_stop_reason_=schedule.stop_reason();}
  // 关闭本轮接收后标记控制周期新鲜度；后续所有最终 sink 都再核对整轮 deadline。
  void callbacks_done(double now){
    cycle_.callbacks_done(now);work_.callbacks=now-work_.started;
  }
  // 统一记录回调/控制耗时和超时计数；告警限频不代表忽略实际超时。
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
  // 单轮顺序：同轮快照→静止证据→逐个危险事件→IPC→Runtime计划→最终sink→报告→安全重新准备。
  // 返回 false 表示原停机条件已结束；true 表示 main 继续调度，不能把活跃任务遗弃。
  bool tick(){
    const double now=steady();
    // The continuous snapshot stays newest. Every discrete safety payload
    // is checked separately before planning, then acknowledged after sinks.
    publication_audit_.begin(now);
    const std::string phase_before=runtime_->phase_name();
    auto s=snapshot(now);
    // 地面停留必须有当前新鲜 status/land、未解锁和静止；姿态路线还要求有效 Z/Vz 与不变估计器参考。
    bool resting=s.status_fresh&&s.land_fresh&&!s.feedback.armed&&s.feedback.landed&&s.at_rest;
    if(config_.profile=="attitude_altitude"){
      resting=resting&&s.local_fresh&&local_.z_valid&&local_.v_z_valid&&std::isfinite(local_.z)&&std::isfinite(local_.vz)&&std::abs(local_.vz)<=config_.attitude.ground_vz_tolerance;
      if(local_.z_reset_counter!=ground_z_reset_||local_.vz_reset_counter!=ground_vz_reset_)rest_since_=-1;
      ground_z_reset_=local_.z_reset_counter;ground_vz_reset_=local_.vz_reset_counter;
    }
    if(resting){if(rest_since_<0)rest_since_=now;s.stationary_seconds=now-rest_since_;}
    else {rest_since_=-1;s.stationary_seconds=0;}
    // Queued events must use the same current ground dwell as step(). A
    // default-zero snapshot here would reset recovery even on healthy events.
    // Their individual unsafe payloads still veto recovery during the scan.
    // 危险队列使用本轮真实地面停留时间；任意一条不安全 payload 都能否决本轮恢复。
    observe_pending_safety(s,now);
    if(config_.profile!="attitude_altitude"){
      if(resting&&s.local_fresh&&local_.z_valid&&!velocity_reference_set_&&s.stationary_seconds>=2){
        velocity_reference_z_=local_.z;velocity_reference_set_=true;
      }
      if(runtime_->active()){velocity_reference_z_=runtime_->flight_core().origin().z;velocity_reference_set_=true;}
    }
    if(stop_requested)begin_shutdown(s,now);
    const double ipc_started=steady();
    // 控制台请求和断连均在本线程交给 Runtime 权限仲裁；PING 不发飞行命令，但会按原协议续操作员租约；STOP 成功才锁存停机请求。
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
    // Core 只产生计划，没有直接 ROS 发布权限；后续仍可能因为过期、控制权或部分提交而拒绝。
    auto output=runtime_->step(s,now);
    work_.runtime=steady()-runtime_started;
    // A task-input TTL may expire between calculation and the first sink.
    // Re-enter the original descent controller before any motion is committed.
    // 计算结束后视觉 TTL 可能已到期；首个 sink 前重新采样，过期便重新进入既有下降链。
    if(output.attitude.visual_motion){
      const auto actual_time=steady();const auto actual=snapshot(actual_time);
      if(!runtime_->visual_output_current(actual,actual_time)){
        runtime_->request_visual_land(actual,actual_time,"INPUT_EXPIRED_BEFORE_SINK");
        output=runtime_->step(actual,steady());
      }
    }
    // 整轮超时清空计划；仅零输出、无动作的未解锁准备允许原窄恢复，其余沿故障链。
    if(config_.profile=="attitude_altitude"&&!cycle_.current(steady(),config_.attitude.stall_seconds)){
      if(runtime_->attitude_core().phase()!=f::attitude::Phase::Safety&&
         !runtime_->abort_rc_preparation_timing("ROS_ADAPTER_CONTROL_LOOP_STALL",s))
        runtime_->abort_attitude("ROS_ADAPTER_CONTROL_LOOP_STALL");
      output={};
    }
    // 保留上轮真正通过配对提交的目标；清空本轮 last，不能把计划被拒仍记成正在输出。
    const auto previous_attitude_output=last_attitude_output_;
    const bool previous_heartbeat_committed=previous_attitude_output.publish;
    const double previous_heartbeat_stamp=publication_audit_.mode_last;
    last_attitude_output_={};
    if(boundary_fault_latched_)output={};
    // PX4 status can change after Core::step but before publication. A
    // confirmed, disarmed external departure during stage-one release makes
    // both the zero heartbeat and any release 176 unnecessary. Do not weaken
    // the ordinary final guard or apply this to a flight/armed vehicle.
    // 释放Offboard阶段若已新鲜证明未解锁地面且外部已离开模式，就取消多余176和零心跳。
    // 此窄分支不能用于仍解锁的飞机，也不能替代普通发布边界。
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
    // 先记动作意图；只有配套目标/心跳整体批准，才能继续对应176/400动作提交。
    const auto requested_command=output.attitude.command;
    const auto attempted_attitude=output.attitude;
    bool attitude_sent=!output.attitude.publish;
    if(output.attitude.publish){
      try{
        pair_context_count_=0;pair_context_seconds_.fill(0);
        sampling_pair_=true;
        // 每个 sink 前重建上下文，目标与心跳各自计真实提交；第二个 sink 失败属于部分提交完整性故障。
        const auto decision=f::attitude::publish_attitude_resampled<Mode,AttitudeTarget>(
          [this]{return boundary_context();},output.attitude,timestamp(),
          [this](const AttitudeTarget& target){
            attitude_pub_->publish(target);publication_audit_.target_committed(steady(),target.thrust_body[2]);},
          [this](const Mode& mode){
            mode_pub_->publish(mode);publication_audit_.mode_committed(steady());},&pair_attempt_);
        sampling_pair_=false;
        // 只有配对成功才更新 last_attitude_output 与 RC 零流提交次数；本地成功仍不等于飞控采用。
        if(decision.allowed){
          last_attitude_output_=output.attitude;attitude_sent=true;runtime_->confirm_rc_output(true,steady());
          if(publication_block_reason_==preparation_age_wait_)publication_block_reason_.clear();
        }
        // 视觉首次sink拒绝时尚无输出，可走原降落请求；已经部分提交的异常不会进入此分支。
        else if(decision.reason==f::attitude::BoundaryReason::VisualInput){
          // No sink ran: the motion input or detector agreement changed. A partial
          // target/heartbeat failure still takes the hard exception path below.
          const auto actual_time=steady();const auto actual=snapshot(actual_time);
          runtime_->request_visual_land(actual,actual_time,actual.feedback.landed?
            "FLIGHT_DETECTOR_CONTRADICTION_AT_FIRST_SINK":"INPUT_EXPIRED_AT_FIRST_SINK");
          publication_block_reason_="VISUAL_INPUT_RECOVERY_REQUESTED";output={};
        }else if(defer_unarmed_source_expiry(decision,output.attitude)){output={};}
        else {refuse_publication("attitude",decision,&attempted_attitude);output={};}
      // 成对发布抛异常时无法假定未发送：保留审计并锁存边界故障，禁止继续发关联动作。
      }catch(const std::exception& e){
        sampling_pair_=false;
        publication_audit_.failed();attitude_sent=false;boundary_fault_latched_=true;
        publication_block_reason_=std::string("PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_")+e.what();
        if(!abort_preparation_timing(publication_block_reason_,attempted_attitude))
          runtime_->abort_attitude(publication_block_reason_);
        output={};report_publication_failure("attitude");
      }catch(...){
        sampling_pair_=false;
        publication_audit_.failed();attitude_sent=false;boundary_fault_latched_=true;
        publication_block_reason_="PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_UNKNOWN_EXCEPTION";
        runtime_->abort_attitude(publication_block_reason_);output={};report_publication_failure("attitude");
      }
    }
    // 动作是否发送及真实 wire 微秒戳独立记录；默认未发，不能使用 Core 请求时间冒充。
    bool command_sent=false;
    std::uint64_t command_stamp=0;
    // 动作参数保持 PX4 合同：176 模式、400 解/上锁、21 降落；本轮输出未获准则禁止动作。
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
    // 用真实发布结果回填事务，再记录提交水位；未发但未已有明确边界故障时同样拒绝继续控制。
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
    // 以下为原通用/仿真路线，与姿态高度路线按 profile 互斥；保留其既有发送顺序。
    if(output.send_ground)publish_command(Command::VEHICLE_CMD_DO_SET_MODE,1,7);
    if(output.flight.command==v::VehicleCommand::Offboard)publish_command(Command::VEHICLE_CMD_DO_SET_MODE,1,6);
    else if(output.flight.command==v::VehicleCommand::Arm){
      if(!publish_command(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM,1,0))output={};
    }
    else if(output.flight.command==v::VehicleCommand::Land)publish_command(Command::VEHICLE_CMD_NAV_LAND,0,0);
    // 未使用的轨迹向量填 NaN，避免零值被 PX4 当成有效位置或速度目标；NED/yaw单位保持原合同。
    if(output.flight.control!=v::Control::None){
      Mode mode{};mode.timestamp=timestamp();mode.position=output.flight.control==v::Control::Position;mode.velocity=output.flight.control==v::Control::Velocity;
      Target target{};target.timestamp=mode.timestamp;target.position.fill(NAN);target.velocity.fill(NAN);target.acceleration.fill(NAN);target.jerk.fill(NAN);
      if(mode.position)target.position={float(output.flight.position.x),float(output.flight.position.y),float(output.flight.position.z)};
      if(mode.velocity)target.velocity={float(output.flight.velocity.x),float(output.flight.velocity.y),float(output.flight.velocity.z)};
      target.yaw=float(output.flight.yaw);target.yawspeed=float(output.flight.yaw_rate);
      target_pub_->publish(target);mode_pub_->publish(mode);
    }
    // 心跳停止沿同时报告上次真实目标和当前观测姿态；必须用合并 status/land 的反馈，不能读默认 armed。
    if(config_.profile=="attitude_altitude"&&previous_heartbeat_committed&&
       !last_attitude_output_.publish){
      // The adapter stores status/land facts in Snapshot, then Runtime folds
      // them into attitude feedback. Raw s.attitude leaves those fields at
      // defaults and can falsely report a disarmed mode departure on this edge.
      // Reuse this tick's snapshot; this conversion performs no graph query.
      const auto fb=runtime_->publication_context(s,now,server_->connected(),true).feedback;
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
        <<",\"local_target_this_tick\":"<<boolean(publication_audit_.target_this_tick)
        <<",\"local_heartbeat_this_tick\":"<<boolean(publication_audit_.mode_this_tick)
        <<"}"<<std::endl;
    }
    // Keep every transition; ordinary thrust samples need only the existing
    // 2 Hz reporting cadence. Avoid a second full DDS graph snapshot and a
    // large diagnostic record on all 20 control ticks.
    // 每个阶段变化保留一条事件，普通状态约2Hz；报告耗时计入预算，不在每个控制tick重复大记录。
    const double reporting_started=steady();
    if(config_.profile=="attitude_altitude"&&
       (phase_before!=runtime_->phase_name()||now-printed_>=.5))
      write_phase_event(phase_before,s,attempted_attitude,now);
    if(now-printed_>=.5){printed_=now;auto line=state(s,now);server_->publish(line);std::cout<<line<<std::endl;write_csv(s,now);}
    work_.reporting=steady()-reporting_started;
    // Reporting can take time. Recovery must use a freshly aged snapshot,
    // not the original tick sample after logging has consumed its lifetime.
    // 报告可能耗时：重新准备前重龄反馈、静止秒数与整轮 deadline，不复用tick开始时的寿命。
    auto reprepare_snapshot=snapshot(steady());const double reprepare_now=steady();
    reprepare_snapshot.stationary_seconds=rest_since_>=0?std::max(0.,reprepare_now-rest_since_):0;
    reprepare_snapshot.control_cycle_fresh=cycle_.current(reprepare_now,config_.attitude.stall_seconds);
    // 新安全地面和开关复位全部通过才重建任务并清本地边界锁；停止期间绝不启动下一轮。
    if(!stopping_&&runtime_->reprepare_rc(reprepare_snapshot,reprepare_now)){
      boundary_fault_latched_=false;publication_block_reason_.clear();
      std::cout<<"{\"kind\":\"rc_safe_reprepare\",\"reason\":\"NEW_DISARMED_GROUND_AND_CONTROLS_RESET\""
        <<",\"completed_attempt\":"<<runtime_->rc_attempts_completed()
        <<",\"previous_reason\":"<<f::quote(runtime_->last_rc_attempt_reason())<<"}"<<std::endl;
    }
    // 只有已上锁着地且任务结束才正常退出；60秒不能强行截断健康armed+Offboard的受控恢复输出。
    if(stopping_){
      if(s.status_fresh&&s.land_fresh&&!s.feedback.armed&&s.feedback.landed&&!runtime_->active()){
        end_latched_events();return false;
      }
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
          end_latched_events();return false;
        }
      }
    }
    end_latched_events();
    return true;
  }
 private:
  // 单周期工作量诊断：时间为单调秒，计数区分快照、图查询、最终上下文与接收轮次。
  struct Work {
    double started{},callbacks{},ipc{},runtime{},snapshot{},graph{},boundary{},reporting{};
    std::uint64_t snapshots{},graph_queries{},contexts{},receive_passes{};
  };
  // DDS图中的来源/发布者/订阅者数；唯一性用于控制权合同，话题存在不等于来源正确。
  struct GraphCounts {
    jikl_fc01_ground_link::Traffic traffic;
    f::attitude::OwnershipCounts ownership;
    std::size_t local{},attitude{},manual{},switches{},battery{},flags{},visual_state{},visual_command{};
  };
  // 记录目标、心跳及最终复核使用的证据；只供审计，不会在下轮作为新的飞控测量。
  struct BoundarySample {
    std::uint64_t cycle{},graph_generation{},graph_queries{},callbacks{},events{},status{},land{},local{},local_sample{},host{};
    double observed{},height{};
    f::attitude::Phase phase{f::attitude::Phase::Idle};
    bool armed{},offboard{},landed{},at_rest{},control_fresh{},graph_valid{},reference_ready{};
  };
  // 仅本周期且 discovery 事件未变时复用图计数；查询过程中又变化则本轮拒绝，不无界循环等稳定。
  GraphCounts current_graph(){
    if(cycle_graph_invalidated_)return {};
    if(!cycle_graph_ready_||(graph_event_&&graph_event_->check())){
      // Clear BEFORE counting: a discovery change during the query invalidates
      // the entire result. A pending change is cleared only with a full query.
      if(graph_event_)graph_event_->check_and_clear();
      cycle_graph_=query_graph();cycle_graph_ready_=true;++graph_generation_;
    }
    if(!graph_event_||graph_event_->check())cycle_graph_invalidated_=true;
    // Stable discovery reuses this cycle's counts. A real discovery event
    // triggers one refresh, including unrelated changes with identical counts.
    // Further churn DURING that refresh refuses this cycle, never a wait loop.
    return cycle_graph_invalidated_?GraphCounts{}:cycle_graph_;
  }
  // 检查全部竞争 setpoint/command 发布者及必需飞控来源/接收者，不能只数本节点创建的端点。
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
  // 参考是否可用按当前profile读取，未建立地面零点时诊断不可把默认0当有效参考。
  bool reference_ready()const{
    return config_.profile=="attitude_altitude"?runtime_->attitude_core().reference_ready():velocity_reference_set_;
  }
  // 返回 PX4 NED 地面z，单位m；只有reference_ready为真时可用 z0-z 求向上高度。
  double reference_z()const{
    return config_.profile=="attitude_altitude"?runtime_->attitude_core().reference_z():velocity_reference_z_;
  }
  // 第一次停止请求才记录单调起始秒并通知Runtime沿原降落链；重复信号不重启停机计时。
  void begin_shutdown(const f::Snapshot& s,double now){
    if(stopping_)return;
    stopping_=true;stop_since_=now;runtime_->begin_shutdown(s,now);
  }
  // 每种反馈使用自己的寿命上限；电池为1秒，不能用低频来源阈值放宽姿态或本地位置。
  double source_timeout(f::SourceSlot slot)const{
    switch(slot){
      case f::SourceSlot::Status:return config_.attitude.status_timeout;
      case f::SourceSlot::Land:return config_.attitude.land_timeout;
      case f::SourceSlot::Local:return config_.attitude.local_timeout;
      case f::SourceSlot::Attitude:return config_.attitude.attitude_timeout;
      case f::SourceSlot::Flags:return config_.attitude.flags_timeout;
      case f::SourceSlot::Battery:return 1.;
      case f::SourceSlot::Count:break;
    }
    return 0;
  }
  // 同时记录源envelope/原始sample、ROS主机微秒、steady接收秒及RMW时间；多种时钟证据不混用。
  void record_receipt(f::SourceSlot slot,std::uint64_t envelope,std::uint64_t sample,
      bool sampled,double received,const rclcpp::MessageInfo& info){
    ++received_callbacks_;++source_callbacks_[std::size_t(slot)];++source_cycle_callbacks_[std::size_t(slot)];
    receipt_timing_[std::size_t(slot)]=f::observe_source_receipt(envelope,sample,sampled,
      timestamp(),received,info.get_rmw_message_info().received_timestamp);
    const auto host=receipt_timing_[std::size_t(slot)].host_stamp;
    // A newer healthy callback must not erase an unsafe timing observation
    // while stopped Safety drains queued samples. Preserve the first cause
    // through the same acknowledgement boundary as discrete payload events.
    // 本轮首个坏时序保持到末尾ack边界；后到的健康帧不能抹掉过期或未同步事件。
    if(receive_timing_block_source_==f::SourceSlot::Count){
      const auto timeout=source_timeout(slot);
      const auto source_reason=f::source_time_reason(envelope,sample,sampled,host,timeout);
      const auto receipt_reason=f::receipt_time_reason(received,steady(),timeout);
      if(source_reason!=f::SourceTimeReason::Valid||receipt_reason!=f::ReceiptTimeReason::Valid){
        receive_timing_block_source_=slot;
        receive_timing_block_reason_=source_reason!=f::SourceTimeReason::Valid?
          f::source_time_reason_name(source_reason):f::receipt_time_reason_name(receipt_reason);
      }
    }
    if(!envelope||(sampled&&(!sample||sample>envelope))||
       (envelope>host&&envelope-host>f::source_clock_skew_budget_us))++receive_events_;
  }
  // 连续缓存只接受严格更新的envelope；拒收仍增加事件计数，避免新鲜尾帧掩盖乱序。
  template<class T>void receive(const T& msg,T& destination,f::Stamp& stamp,
      f::SourceSlot slot,const rclcpp::MessageInfo& info){
    const double received=steady();record_receipt(slot,msg.timestamp,0,false,received,info);
    const bool accepted=stamp.accept(msg.timestamp,received);
    receipt_accepted_[std::size_t(slot)]=accepted;
    if(accepted){destination=msg;++source_accepted_[std::size_t(slot)];}
    else ++receive_events_;
  }
  // 姿态/本地位置还需原始sample推进；仅envelope刷新不能延长同一旧测量的使用时间。
  template<class T>void receive_sampled(const T& msg,T& destination,f::Stamp& stamp,
      f::SourceSampleProgress& sample,f::SourceSlot slot,const rclcpp::MessageInfo& info){
    const double received=steady();
    record_receipt(slot,msg.timestamp,msg.timestamp_sample,true,received,info);
    const bool accepted=f::receive_sampled_source(msg,destination,stamp,sample,received);
    receipt_accepted_[std::size_t(slot)]=accepted;
    if(accepted)++source_accepted_[std::size_t(slot)];else ++receive_events_;
  }
  // 固定容量队列满则锁存传输故障，不能静默丢危险证据后继续声称安全恢复。
  template<class T>void latch_event(f::DiscreteEventLatch<T>& latch,const T& value){
    if(!latch.push(value)) receive_event_overflow_=true;
  }
  // 每类事件覆盖快照里的对应事实逐条给Core观察；连续缓存仍是最新值，二者用途分离。
  void observe_pending_safety(const f::Snapshot& snapshot,double now){
    if(config_.profile!="attitude_altitude")return;
    using Event=f::attitude::Core::SafetyEvent;
    runtime_->begin_attitude_safety_observation();
    const auto base=runtime_->publication_context(snapshot,now,true,true).feedback;
    const auto observe=[this,now](const auto& sample,Event kind){
      runtime_->observe_attitude_safety_event(sample,kind,now);
    };
    // Check every payload, including an unsafe event behind a benign head.
    // This bounded scan performs no DDS query, output or receive wait.
    attitude_events_.for_each([&](const auto& value){auto sample=base;
      for(std::size_t i=0;i<4;++i)sample.q[i]=value.q[i];
      sample.quat_reset=value.quat_reset_counter;observe(sample,Event::Attitude);});
    local_events_.for_each([&](const auto& value){auto sample=base;
      sample.z=value.z;sample.vz=value.vz;sample.z_deriv=value.z_deriv;
      sample.z_valid=value.z_valid;sample.vz_valid=value.v_z_valid;
      sample.z_reset=value.z_reset_counter;sample.vz_reset=value.vz_reset_counter;
      sample.local_stamp=value.timestamp;sample.vertical_sample_stamp=value.timestamp_sample;
      observe(sample,Event::Vertical);});
    status_events_.for_each([&](const auto& value){auto sample=base;
      sample.status_stamp=value.timestamp;
      sample.status_fresh=base.status_fresh&&(value.arming_state==Status::ARMING_STATE_ARMED||
        value.arming_state==Status::ARMING_STATE_DISARMED);sample.armed=value.arming_state==Status::ARMING_STATE_ARMED;
      sample.offboard=value.nav_state==Status::NAVIGATION_STATE_OFFBOARD;sample.failsafe=value.failsafe;
      sample.auto_land_selected=value.nav_state==Status::NAVIGATION_STATE_AUTO_LAND;
      sample.stabilized_selected=value.nav_state==Status::NAVIGATION_STATE_STAB;
      sample.stabilized_selectable=(value.can_set_nav_states_mask&(1U<<Status::NAVIGATION_STATE_STAB))!=0;
      sample.exclusive=base.exclusive&&value.system_id==1&&value.component_id==1;
      sample.kill=base.kill||value.nav_state==Status::NAVIGATION_STATE_TERMINATION;
      sample.operator_takeover=value.failsafe_and_user_took_over;
      sample.calibration=value.calibration_enabled||value.rc_calibration_in_progress;
      sample.hil=value.hil_state!=Status::HIL_STATE_OFF;sample.preflight=value.pre_flight_checks_pass;
      sample.battery_ready=base.battery_ready&&value.power_input_valid;observe(sample,Event::Status);});
    battery_events_.for_each([&](const auto& value){auto sample=base;
      sample.battery_voltage=value.voltage_v;
      sample.battery_ready=value.connected&&value.warning==0&&std::isfinite(value.voltage_v)&&
        value.voltage_v>0&&snapshot.power_valid;observe(sample,Event::Battery);});
    land_events_.for_each([&](const auto& value){auto sample=base;
      sample.landed=value.landed;sample.at_rest=value.at_rest;observe(sample,Event::Land);});
    flags_events_.for_each([&](const auto& value){auto sample=base;
      sample.kill=base.kill||value.fd_critical_failure;
      sample.offboard_signal_lost=value.offboard_control_signal_lost;
      if(value.manual_control_signal_lost)sample.rc_link_ready=false;
      observe(sample,Event::Failsafe);});
    authority_events_.for_each([&](const auto& value){auto sample=base;
      sample.kill=base.kill||value.kill;sample.rc_link_ready=value.rc_link_ready;
      sample.rc_switches_ready=value.rc_switches_ready;
      sample.rc_sticks_moving=value.sticks_moving;sample.rc_axes_safe=value.rc_axes_safe;
      sample.rc_source_clock_valid=value.rc_source_clock_valid;sample.rc_switch_clock_valid=value.rc_switch_clock_valid;
      sample.rc_source_seen=value.manual_seen;sample.rc_source_valid=value.manual_valid;
      sample.rc_source_selected=value.rc_source_selected;sample.rc_data_source=value.data_source;
      sample.rc_switches_seen=value.switches_seen;sample.rc_switches_fresh=value.switches_fresh;
      sample.rc_mode_slot=value.mode_slot;sample.rc_arm_switch=value.arm_switch;sample.rc_kill_switch=value.kill_switch;
      sample.rc_aux_switches_safe=value.rc_aux_switches_safe;sample.rc_throttle=value.rc_throttle;
      sample.rc_switch_sample_stamp=value.switch_sample_stamp;sample.operator_switch_generation=value.switch_generation;
      observe(sample,Event::Authority);});
    // Scan payloads first: one may enter Safety this cycle. The timing veto
    // also applies then, even when the newest continuous snapshot is healthy.
    if(receive_timing_block_source_!=f::SourceSlot::Count&&
       runtime_->attitude_core().phase()==f::attitude::Phase::Safety){
      auto sample=base;sample.source_clock_valid=false;observe(sample,Event::Status);
    }
  }
  void latch_event_for_slot(f::SourceSlot slot,const Status& value){
    if(slot==f::SourceSlot::Status)latch_event(status_events_,value);
  }
  void latch_event_for_slot(f::SourceSlot slot,const Land& value){
    if(slot==f::SourceSlot::Land)latch_event(land_events_,value);
  }
  void latch_event_for_slot(f::SourceSlot slot,const Battery& value){
    if(slot==f::SourceSlot::Battery)latch_event(battery_events_,value);
  }
  void latch_event_for_slot(f::SourceSlot slot,const Flags& value){
    if(slot==f::SourceSlot::Flags)latch_event(flags_events_,value);
  }
  // 完成控制与所有sink后才ack整轮事件；overflow保持粘性，普通本轮时序否决在此收尾。
  void end_latched_events(){
    // Acknowledge only after the full control/publication boundary. Overflow
    // remains sticky; a healthy later payload cannot clear an observed fault.
    status_events_.acknowledge_all();land_events_.acknowledge_all();
    battery_events_.acknowledge_all();flags_events_.acknowledge_all();
    local_events_.acknowledge_all();attitude_events_.acknowledge_all();
    authority_events_.acknowledge_all();
    receive_timing_block_source_=f::SourceSlot::Count;receive_timing_block_reason_="";
  }
  // 离散变化比较previous与收到payload，即使该帧没获连续缓存接受，也保留其中的危险变化。
  template<class T,class Event>void receive_event(const T& msg,T& destination,f::Stamp& stamp,
      f::SourceSlot slot,const rclcpp::MessageInfo& info,Event event){
    const auto previous=destination;receive(msg,destination,stamp,slot,info);
    if(event(previous,msg)){
      latch_event_for_slot(slot,msg);
      ++receive_events_;
    }
  }
  // 手动输入按选中数据源、时钟、摇杆和开关事实形成权限key；key变化才额外锁存离散事件。
  void receive_manual(const Manual& message){
    ++received_callbacks_;const double now=steady();const auto host=timestamp();
    const auto before=authority_telemetry_.facts(now,config_.attitude.flags_timeout,true,false,1,1,host);
    authority_telemetry_.manual(message,now,host,config_.attitude.flags_timeout);
    const auto after=authority_telemetry_.facts(now,config_.attitude.flags_timeout,true,false,1,1,host);
    if(f::authority_receive_key(before)!=f::authority_receive_key(after)){latch_event(authority_events_,after);++receive_events_;}
  }
  // 开关与手动源分别验证，再把Kill/模式/解锁等变化保留到本轮控制；不能只看最新开关尾值。
  void receive_switches(const Switches& message){
    ++received_callbacks_;const double now=steady();const auto host=timestamp();
    const auto before=authority_telemetry_.facts(now,config_.attitude.flags_timeout,true,false,1,1,host);
    authority_telemetry_.switches(message,now,config_.attitude.flags_timeout,host);
    const auto after=authority_telemetry_.facts(now,config_.attitude.flags_timeout,true,false,1,1,host);
    if(f::authority_receive_key(before)!=f::authority_receive_key(after)){latch_event(authority_events_,after);++receive_events_;}
  }
  // ROS纳秒转正的同步微秒供PX4接口；HostClockMonitor同时用steady对照检测跳变，回退锁存。
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
  // 每个sink前重新快照和重龄；图查询耗时也计入完整周期，返回合同及三次配对取证。
  f::attitude::BoundaryContext boundary_context(){
    const double begin=steady();auto s=snapshot(begin);
    // The snapshot performs graph lookups. Its elapsed time belongs to this
    // publication's budget, including the final check after target commit.
    const double now=steady();
    s.stationary_seconds=rest_since_>=0?std::max(0.,now-rest_since_):0;
    auto context=runtime_->publication_context(s,now,server_->connected(),true,
      stop_requested||stopping_||boundary_fault_latched_);
    if(cycle_graph_invalidated_)context.feedback.exclusive=false;
    context.control_fresh=cycle_.current(steady(),config_.attitude.stall_seconds);
    const double elapsed=steady()-begin;++work_.contexts;work_.boundary+=elapsed;
    if(sampling_pair_){
      if(pair_context_count_<pair_context_seconds_.size()){
        pair_context_seconds_[pair_context_count_]=elapsed;
        auto& sample=pair_samples_[pair_context_count_];
        sample.graph_generation=graph_generation_;sample.graph_queries=work_.graph_queries;
        sample.cycle=cycle_id_;sample.callbacks=received_callbacks_;sample.events=receive_events_;
        sample.status=s.status_stamp;sample.land=s.land_stamp;sample.local=s.attitude.local_stamp;
        sample.local_sample=s.attitude.vertical_sample_stamp;sample.host=s.attitude.source_host_stamp;
        sample.observed=now;sample.phase=context.phase;sample.armed=s.feedback.armed;
        sample.offboard=s.feedback.offboard;sample.landed=s.feedback.landed;sample.at_rest=s.at_rest;
        sample.control_fresh=context.control_fresh;sample.graph_valid=!cycle_graph_invalidated_;
        sample.reference_ready=context.reference_ready;sample.height=context.reference_z-s.attitude.z;
      }
      ++pair_context_count_;
    }
    return context;
  }
  // 只接受typed ControlStall、无sink异常、无动作且精确零推力的未解锁准备；不能把未知部分发送降级为等待。
  bool abort_preparation_timing(const std::string& reason,const f::attitude::Output& planned){
    if(pair_attempt_.decision.reason!=f::attitude::BoundaryReason::ControlStall||
       pair_attempt_.sink_exception||planned.command!=f::attitude::Command::None||
       f::attitude::nonzero(planned.thrust))return false;
    const auto actual=snapshot(steady());
    return runtime_->abort_rc_preparation_timing(reason,actual);
  }
  // 拒绝完整性故障会锁边界、转交Core并清当前输出证据；诊断不直接更改飞控armed。
  void refuse_publication(const char* boundary,const f::attitude::Decision& decision,
      const f::attitude::Output* planned=nullptr){
    boundary_fault_latched_=true;
    publication_block_reason_=std::string("PUBLISHER_BOUNDARY_")+f::attitude::name(decision.reason);
    if(!planned||!abort_preparation_timing(publication_block_reason_,*planned))
      runtime_->abort_attitude(publication_block_reason_);
    last_attitude_output_={};
    report_publication_failure(boundary);
  }
  // 仅首个sink前的普通源年龄跨界可安全延期；必须无历史时钟事件、无命令、无非零推力、无实际sink。
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
  // 拒绝日志保留同轮sink与时钟证据，明确心跳/姿态输出关闭；它是审计而非恢复命令。
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
  // 唯一VehicleCommand出口；参数来自原状态机，目标1/1与来源1/191保持现有协议。
  // 发送前重打真实时间戳，Arm/Disarm/Release/Land各自执行最终权限合同。
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
  // 单线程tick内无回调，但时间与DDS discovery仍可变化；每次snapshot都重新核对源/接收年龄和权限。
  f::Snapshot snapshot(double now){
    const double begin=steady();
    // SingleThreadedExecutor performs no callbacks during tick. Reuse only
    // this cycle's graph counts while its discovery event stays unchanged;
    // every snapshot still ages source/receipt/RC/clock evidence anew.
    const auto graph=current_graph();now=steady();const auto host_stamp=timestamp();
    source_observed_steady_=now;source_observed_host_=host_stamp;
    f::Snapshot s;s.status_fresh=status_stamp_.fresh(now,config_.attitude.status_timeout);s.land_fresh=land_stamp_.fresh(now,config_.attitude.land_timeout);s.local_fresh=local_stamp_.fresh(now,config_.attitude.local_timeout);
    // arming枚举必须明确是ARMED或DISARMED；未知枚举不能因默认false而被解释为已经上锁。
    s.arming_state_known=status_.arming_state==Status::ARMING_STATE_ARMED||
      status_.arming_state==Status::ARMING_STATE_DISARMED;
    s.status_fresh=s.status_fresh&&s.arming_state_known;
    s.attitude_output_active=s.offboard_heartbeat_active=last_attitude_output_.publish;
    s.publication_fault_latched=boundary_fault_latched_;
    s.control_cycle_fresh=cycle_.current(now,config_.attitude.stall_seconds);
    s.battery_fresh=battery_stamp_.fresh(now,1.);s.manual_fresh=flags_stamp_.fresh(now,config_.attitude.flags_timeout);
    // A lost discrete observation is a sticky transport fault. Include it in
    // the final assignment so normal source clocks cannot overwrite it.
    // 时钟回退、原始样本回退和离散事件丢失都保留为否决，正常的其他来源不能覆盖它们。
    s.time_regressed=receive_event_overflow_||status_stamp_.regressed||land_stamp_.regressed||local_stamp_.regressed||battery_stamp_.regressed||flags_stamp_.regressed||clock_regressed_||attitude_stamp_.regressed||local_sample_.regressed||attitude_sample_.regressed;
    s.status_stamp=status_stamp_.value;s.land_stamp=land_stamp_.value;s.system=status_.system_id;s.component=status_.component_id;
    s.armed_time=status_.armed_time;s.latest_arming_reason=status_.latest_arming_reason;
    s.at_rest=land_.at_rest;s.calibration=status_.calibration_enabled||status_.rc_calibration_in_progress;s.hil=status_.hil_state!=Status::HIL_STATE_OFF;
    s.stabilized_selectable=(status_.can_set_nav_states_mask&(1U<<Status::NAVIGATION_STATE_STAB))!=0;
    s.attitude.battery_voltage=battery_.voltage_v;
    s.battery_connected=battery_.connected;s.battery_warning=battery_.warning;s.power_valid=status_.power_input_valid;s.manual_lost=flags_.manual_control_signal_lost;s.dead_reckoning=local_.dead_reckoning;s.nav_state=status_.nav_state;
    // 通用反馈位置/速度均为NED，m与m/s；yaw来自PX4 heading，单位rad，valid还需有限值和估计器有效位。
    auto& fb=s.feedback;fb.position={local_.x,local_.y,local_.z};fb.velocity={local_.vx,local_.vy,local_.vz};fb.yaw=local_.heading;
    fb.fresh=s.status_fresh&&s.land_fresh&&s.local_fresh;
    fb.valid=local_.xy_valid&&local_.z_valid&&local_.v_xy_valid&&local_.v_z_valid&&v::finite(fb.position)&&v::finite(fb.velocity)&&std::isfinite(fb.yaw);
    fb.heading_ok=local_.heading_good_for_control;fb.preflight=status_.pre_flight_checks_pass;fb.armed=status_.arming_state==Status::ARMING_STATE_ARMED;fb.landed=land_.landed;
    fb.offboard=s.nav_state==Status::NAVIGATION_STATE_OFFBOARD;fb.landing=s.nav_state==Status::NAVIGATION_STATE_AUTO_LAND;fb.failsafe=status_.failsafe;
    // Each packed counter occupies its own byte. All position/velocity/heading resets terminate the mission.
    // 四个重置计数各占一个byte；该通用路线把坐标重置视为旧任务参考失效。
    fb.reset_id=std::uint32_t(local_.xy_reset_counter)|(std::uint32_t(local_.z_reset_counter)<<8)|(std::uint32_t(local_.vxy_reset_counter)<<16)|(std::uint32_t(local_.vz_reset_counter)<<24);
    if(config_.profile!="attitude_altitude"&&local_.heading_reset_counter!=heading_reset_){heading_reset_=local_.heading_reset_counter;if(runtime_->active())heading_changed_=true;}
    s.time_regressed=s.time_regressed||heading_changed_;
    s.traffic=graph.traffic;
    fb.exclusive=s.system==1&&s.component==1&&s.traffic.status_publishers==1&&s.traffic.land_publishers==1&&s.traffic.ack_publishers==1&&s.traffic.command_subscribers==1&&s.traffic.command_publishers==1&&graph.local==1;
    fb.exclusive=fb.exclusive&&f::attitude::owns_setpoints(config_.profile,graph.ownership);
    if(config_.profile=="attitude_altitude"){
      // 姿态路线允许无XY定高，但Z/Vz、姿态、来源/权限都按各自时限验证，不能因不用水平位置省略安全链。
      auto& a=s.attitude;a.attitude_fresh=attitude_stamp_.fresh(now,config_.attitude.attitude_timeout);
      a.failsafe_fresh=flags_stamp_.fresh(now,config_.attitude.flags_timeout);
      a.source_host_stamp=host_stamp;
      a.local_sample_fresh=local_sample_.fresh(host_stamp,now,config_.attitude.local_timeout);
      a.attitude_sample_fresh=attitude_sample_.fresh(host_stamp,now,config_.attitude.attitude_timeout);
      s.local_fresh=s.local_fresh&&a.local_sample_fresh;
      a.attitude_fresh=a.attitude_fresh&&a.attitude_sample_fresh;
      s.battery_fresh=s.battery_fresh&&f::source_stamp_in_clock(battery_stamp_.value,host_stamp,1.);
      fb.fresh=s.status_fresh&&s.land_fresh&&s.local_fresh;
      // 逐源同步微秒域校验；validated表示执行了校验，不等于valid，真实sink时仍可能已过期。
      a.source_clock_valid=f::source_stamp_in_clock(status_stamp_.value,host_stamp,config_.attitude.status_timeout)&&
        f::source_stamp_in_clock(land_stamp_.value,host_stamp,config_.attitude.land_timeout)&&
        f::source_sample_in_clock(local_stamp_.value,local_.timestamp_sample,host_stamp,config_.attitude.local_timeout)&&
        f::source_sample_in_clock(attitude_stamp_.value,attitude_.timestamp_sample,host_stamp,config_.attitude.attitude_timeout)&&
        f::source_stamp_in_clock(flags_stamp_.value,host_stamp,config_.attitude.flags_timeout)&&
        f::source_stamp_in_clock(battery_stamp_.value,host_stamp,1.);
      a.source_clock_validated=true; // validation performed; validity may expire after a sink commit
      // PX4四元数顺序w,x,y,z；z/vz保持NED向下符号，垂向Core之后转换为向上高度/速度。
      for(std::size_t i=0;i<4;++i)a.q[i]=attitude_.q[i];
      a.quat_reset=attitude_.quat_reset_counter;a.z=local_.z;a.vz=local_.vz;
      a.z_deriv=local_.z_deriv;
      a.vertical_sample_stamp=local_.timestamp_sample;
      a.z_reset=local_.z_reset_counter;a.vz_reset=local_.vz_reset_counter;a.z_valid=local_.z_valid;a.vz_valid=local_.v_z_valid;
      a.local_stamp=local_stamp_.value;a.xy_valid=local_.xy_valid&&!local_.dead_reckoning&&
        std::isfinite(local_.x)&&std::isfinite(local_.y);
      a.vxy_valid=local_.v_xy_valid&&std::isfinite(local_.vx)&&std::isfinite(local_.vy);
      a.land_selectable=(status_.can_set_nav_states_mask&(1U<<Status::NAVIGATION_STATE_AUTO_LAND))!=0;
      // 把手动源与开关来源/时钟/新鲜度一起仲裁；RC可用不等于当前取得自动控制权限。
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
      // Kill还包括当前TERMINATION及有效failsafe中的critical failure；人工接管与Offboard丢失单独保留。
      a.kill=authority.kill||(s.status_fresh&&status_.nav_state==Status::NAVIGATION_STATE_TERMINATION)||
        (a.failsafe_fresh&&flags_.fd_critical_failure);
      a.operator_takeover=s.status_fresh&&status_.failsafe_and_user_took_over;
      a.offboard_signal_lost=a.failsafe_fresh&&flags_.offboard_control_signal_lost;
      fb.exclusive=fb.exclusive&&graph.attitude==1;
    }
    if(config_.profile!="sim"&&(graph.battery!=1||graph.flags!=1))fb.exclusive=false;
    // 视觉开关false时不取Inbox、不构成新准入条件；保留原无视觉1.5m路线。
    if(config_.attitude.visual.enabled){
      const auto clock_ns=get_clock()->now().nanoseconds();
      s.attitude.visual_input=visual_inbox_.snapshot(clock_ns>0?std::uint64_t(clock_ns):0,
        graph.visual_state,graph.visual_command);
    }
    ++work_.snapshots;work_.snapshot+=steady()-begin;
    return s;
  }
  // 导数差为独立监测诊断；位置积分一致性另有证据与故障条件，不能把两个误差混为独立停机。
  std::string vertical_feedback_fields() const{
    const auto& e=runtime_->attitude_core().vertical_feedback_evidence();
    std::ostringstream out;
    out<<",\"vertical_feedback_guard_enabled\":"<<boolean(config_.attitude.feedback_guard.enabled)
       <<",\"z_deriv_ned\":"<<number(local_.z_deriv,local_stamp_.received>=0)
       <<",\"vertical_feedback_checked\":"<<boolean(e.checked)
       <<",\"vertical_feedback_healthy\":"<<boolean(e.healthy)
       <<",\"vertical_feedback_suspect\":"<<boolean(e.suspect)
       <<",\"vertical_feedback_velocity_policy\":\"diagnostic_only\""
       <<",\"vertical_feedback_rate_warning\":"<<boolean(e.rate_warning)
       <<",\"vertical_feedback_rate_error_mps\":"<<number(e.rate_error_mps,e.checked)
       <<",\"vertical_feedback_position_rate_error_mps\":"<<number(e.position_rate_error_mps,e.position_window_checked)
       <<",\"vertical_feedback_disagreement_s\":"<<number(e.disagreement_seconds,e.checked)
       <<",\"vertical_feedback_disagreement_samples\":"<<e.disagreement_samples
       <<",\"vertical_feedback_sample_stamp\":"<<e.sample_stamp
       <<",\"vertical_feedback_reason\":"<<f::quote(e.reason);
    return out.str();
  }
  // 区分planned与实际提交推力；比例/积分/原始推力/上限/限斜率仅表示本地计算与边界结果。
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
  // 阶段事件聚合同轮参考、姿态、恢复和事务证据；JSON英文键保留给现有工具，不因中文注释改协议。
  void write_phase_event(const std::string& before,const f::Snapshot& s,
      const f::attitude::Output& planned,double now)const{
    const auto& core=runtime_->attitude_core();
    const auto fb=runtime_->publication_context(s,now,false,true).feedback;
    const auto& p=runtime_->pending(400);
    const auto& ag=core.armed_ground_fault_evidence();
    std::cout<<"{\"kind\":\"phase_event\",\"monotonic_s\":"<<number(now)
      <<",\"phase_before\":"<<f::quote(before)<<",\"phase_after\":"<<f::quote(runtime_->phase_name())
      <<",\"reason\":"<<f::quote(core.reason())<<progress_fields()<<rc_recovery_fields(s,now)<<reference_fields(s)<<vertical_feedback_fields()<<vertical_control_fields(planned,true)<<",\"ownership\":"<<f::quote(core.recovery_responsibility(fb))
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
  // 恢复观察给出具体block_reason、新样本和地面停留；只读展示不清锁、不递增恢复计数。
  std::string rc_recovery_fields(const f::Snapshot& s,double now)const{
    const auto& core=runtime_->attitude_core();const auto d=core.rc_recovery_diagnostics();
    std::ostringstream out;
    out<<",\"rc_first_fault_reason\":"<<f::quote(d.first_fault_reason)
      <<",\"rc_latest_fault_reason\":"<<f::quote(d.latest_fault_reason)
      <<",\"rc_fault_recoverable\":"<<boolean(d.recoverable)
      <<",\"rc_restart_required\":"<<boolean(core.restart_required())
      <<",\"rc_recovery_ready\":"<<boolean(d.ready)
      <<",\"rc_recovery_event_blocked\":"<<boolean(d.event_blocked)
      <<",\"rc_recovery_window_active\":"<<boolean(d.window_active)
      <<",\"rc_recovery_elapsed_s\":"<<number(d.elapsed_s)
      <<",\"rc_recovery_required_s\":"<<number(d.required_s)
      <<",\"rc_recovery_status_samples\":"<<d.status_samples
      <<",\"rc_recovery_land_samples\":"<<d.land_samples
      <<",\"rc_recovery_local_samples\":"<<d.local_samples
      <<",\"rc_recovery_status_stamp\":"<<d.status_stamp
      <<",\"rc_recovery_land_stamp\":"<<d.land_stamp
      <<",\"rc_recovery_local_stamp\":"<<d.local_stamp
      <<",\"rc_recovery_block_reason\":"<<f::quote(d.block_reason)
      <<",\"rc_reprepare_block_reason\":"<<f::quote(runtime_->rc_reprepare_block_reason(s,now));
    return out.str();
  }
  // 任务开始、独立确认离地、稳定到高、导航获准分别记录；实际离地仍要外部日志/现场证据核对。
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
  // 地面参考z0单位m、NED向下；reset计数绑定建立时的坐标系，未建立参考输出null。
  std::string reference_fields(const f::Snapshot& s)const{
    const auto& core=runtime_->attitude_core();std::ostringstream out;
    out<<",\"height_reference_z0\":"<<number(core.reference_z(),core.reference_ready())
      <<",\"height_reference_z_reset\":"<<unsigned(core.reference_z_reset())
      <<",\"height_reference_vz_reset\":"<<unsigned(core.reference_vz_reset())
      <<",\"height_reference_quat_reset\":"<<unsigned(core.reference_quat_reset())
      <<",\"height_sample_z\":"<<number(s.attitude.z,s.local_fresh&&s.attitude.z_valid)
      <<",\"height_sample_z_reset\":"<<unsigned(s.attitude.z_reset)
      <<",\"height_sample_vz_reset\":"<<unsigned(s.attitude.vz_reset)
      <<",\"height_sample_quat_reset\":"<<unsigned(s.attitude.quat_reset)
      <<",\"height_local_sample_stamp\":"<<s.attitude.vertical_sample_stamp
      <<",\"rc_ground_reference_qualified\":"<<boolean(core.rc_ground_reference_qualified())
      <<",\"rc_ground_reference_samples\":"<<core.rc_ground_reference_samples()
      <<",\"rc_ground_reference_seconds\":"<<number(core.rc_ground_reference_seconds());
    return out.str();
  }
  // 输出本轮/上轮耗时统计，时间单位秒；计数只解释调度负载，不代表PX4端处理时间。
  std::string work_fields(const Work& value,const char* field="cycle_work")const{
    std::ostringstream out;
    out<<",\""<<field<<"\":{\"callbacks_s\":"<<number(value.callbacks)
      <<",\"ipc_s\":"<<number(value.ipc)<<",\"runtime_s\":"<<number(value.runtime)
      <<",\"snapshot_s\":"<<number(value.snapshot)<<",\"graph_s\":"<<number(value.graph)
      <<",\"receive_passes\":"<<value.receive_passes
      <<",\"snapshot_calls\":"<<value.snapshots<<",\"graph_queries\":"<<value.graph_queries
      <<",\"boundary_s\":"<<number(value.boundary)<<",\"boundary_calls\":"<<value.contexts
      <<",\"reporting_s\":"<<number(value.reporting)<<"}";
    return out.str();
  }
  // 聚合各源envelope、sample与接收寿命供最终边界检查，不用最新回调时间替代原始测量时间。
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
  // 分层显示本机时钟变化、每源时间证据与配对上下文，便于区分未同步、原始样本不推进和控制超时。
  std::string timing_evidence(double now,std::uint64_t host)const{
    std::ostringstream out;
    const auto age=[](f::AgeMeasurement value){return number(value.seconds,value.available);};
    out<<",\"controller_pid\":"<<getpid()<<",\"diagnostic_host_us\":"<<host
      <<",\"diagnostic_monotonic_s\":"<<number(now)
      <<",\"host_clock_anchor_error_s\":"<<age(clock_observation_.anchor_error)
      <<",\"host_clock_last_increment_error_s\":"<<age(clock_observation_.delta_error)
      <<",\"receive_policy\":\"BOUNDED_SINGLE_THREAD_PASSES_V1\",\"receive_budget_s\":"<<number(receive_budget_seconds())
      <<",\"receive_stop_reason\":"<<f::quote(receive_stop_reason_)
      <<",\"receive_callbacks_total\":"<<received_callbacks_<<",\"receive_events_total\":"<<receive_events_
      <<",\"receive_timing_block_source\":"<<f::quote(receive_timing_block_source_==f::SourceSlot::Count?
        "":f::source_slot_name(receive_timing_block_source_))
      <<",\"receive_timing_block_reason\":"<<f::quote(receive_timing_block_reason_)
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
        <<",\"callbacks_total\":"<<source_callbacks_[i]<<",\"accepted_total\":"<<source_accepted_[i]
        <<",\"callbacks_this_cycle\":"<<source_cycle_callbacks_[i]
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
    out<<"},\"control_cycle_id\":"<<cycle_id_
      <<",\"graph_snapshot_cycle\":"<<(cycle_graph_ready_?cycle_id_:0)
      <<",\"graph_snapshot_generation\":"<<graph_generation_
      <<",\"graph_snapshot_invalidated\":"<<boolean(cycle_graph_invalidated_)
      <<",\"publication_stage\":"<<f::quote(f::attitude::name(pair_attempt_.stage))
      <<",\"publication_decision\":"<<f::quote(f::attitude::name(pair_attempt_.decision.reason))
      <<",\"publication_target_committed\":"<<boolean(pair_attempt_.target_committed)
      <<",\"publication_heartbeat_committed\":"<<boolean(pair_attempt_.heartbeat_committed)
      <<",\"publication_sink_exception\":"<<boolean(pair_attempt_.sink_exception)
      <<",\"pair_context_calls\":"<<pair_context_count_<<",\"pair_context_seconds\":["
      <<number(pair_context_seconds_[0],pair_context_count_>0)<<','
      <<number(pair_context_seconds_[1],pair_context_count_>1)<<','
      <<number(pair_context_seconds_[2],pair_context_count_>2)<<"],\"pair_context_snapshots\":[";
    for(std::size_t i=0;i<std::min(pair_context_count_,pair_samples_.size());++i){
      if(i)out<<',';
      const auto& sample=pair_samples_[i];
      out<<"{\"context_index\":"<<i+1<<",\"cycle_id\":"<<sample.cycle
        <<",\"graph_generation\":"<<sample.graph_generation<<",\"graph_queries\":"<<sample.graph_queries
        <<",\"monotonic_s\":"<<number(sample.observed)
        <<",\"phase\":"<<f::quote(f::attitude::name(sample.phase))
        <<",\"callback_watermark\":"<<sample.callbacks<<",\"event_watermark\":"<<sample.events
        <<",\"status_stamp\":"<<sample.status<<",\"land_stamp\":"<<sample.land
        <<",\"local_stamp\":"<<sample.local<<",\"local_sample_stamp\":"<<sample.local_sample
        <<",\"host_stamp\":"<<sample.host<<",\"armed\":"<<boolean(sample.armed)
        <<",\"offboard\":"<<boolean(sample.offboard)<<",\"landed\":"<<boolean(sample.landed)
        <<",\"at_rest\":"<<boolean(sample.at_rest)<<",\"control_fresh\":"<<boolean(sample.control_fresh)
        <<",\"graph_valid\":"<<boolean(sample.graph_valid)
        <<",\"relative_height\":"<<number(sample.height,sample.reference_ready)<<'}';
    }
    out<<']';
    return out.str();
  }
  // 反馈新鲜必须同时有源域与本机接收证据；历史可显示但不能据此解锁或给导航准入。
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
  // 状态JSON是观察接口；保留原字段名、消息与null约定，任何客户端展示都不参与控制权授予。
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
      <<progress_fields()<<rc_recovery_fields(s,now)<<reference_fields(s)<<feedback_timing_fields(s,now)
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
      <<",\"local_payload_z_valid\":"<<boolean(local_.z_valid)
      <<",\"local_payload_vz_valid\":"<<boolean(local_.v_z_valid)
      <<",\"px4_user_takeover_reported\":"<<boolean(status_.failsafe_and_user_took_over)
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
  // 可选CSV逐条同步flush保持原行为；主服务关闭该出口，耗时风险列为独立技术债。
  void write_csv(const f::Snapshot& s,double now){if(!csv_)return;csv_<<std::setprecision(9)<<now<<','<<runtime_->phase_name()<<','<<s.nav_state<<','<<s.feedback.armed<<','<<s.feedback.landed<<','<<local_.xy_valid<<','<<local_.z_valid<<','<<local_.v_xy_valid<<','<<local_.v_z_valid<<','<<local_.heading_good_for_control<<','<<number(now-local_stamp_.received,local_stamp_.received>=0)<<','<<local_.x<<','<<local_.y<<','<<local_.z<<','<<local_.vx<<','<<local_.vy<<','<<local_.vz<<','<<local_.ax<<','<<local_.ay<<','<<local_.az<<','<<number(reference_z()-local_.z,reference_ready()&&s.local_fresh&&local_.z_valid)<<','<<f::quote(runtime_->message())<<','<<last_attitude_output_.publish<<','<<number(runtime_->attitude_core().yaw_reference())<<','<<last_attitude_output_.thrust[2]<<','<<last_attitude_output_.target_height<<','<<last_attitude_output_.velocity_target<<','<<last_attitude_output_.saturated<<','<<last_attitude_output_.slew_limited<<','<<number(local_.z_deriv,s.local_fresh)<<','<<runtime_->attitude_core().vertical_feedback_evidence().checked<<','<<runtime_->attitude_core().vertical_feedback_evidence().suspect<<','<<number(runtime_->attitude_core().vertical_feedback_evidence().rate_error_mps,runtime_->attitude_core().vertical_feedback_evidence().checked)<<','<<number(runtime_->attitude_core().vertical_feedback_evidence().position_rate_error_mps,runtime_->attitude_core().vertical_feedback_evidence().position_window_checked)<<','<<runtime_->attitude_core().vertical_feedback_evidence().disagreement_seconds<<','<<f::quote(runtime_->attitude_core().vertical_feedback_evidence().reason)
      <<','<<last_attitude_output_.vertical_closed_loop<<','<<last_attitude_output_.takeoff_thrust_limited
      <<','<<number(last_attitude_output_.proportional_thrust,last_attitude_output_.vertical_closed_loop)
      <<','<<number(last_attitude_output_.integral_thrust,last_attitude_output_.vertical_closed_loop)
      <<','<<number(last_attitude_output_.raw_thrust,last_attitude_output_.vertical_closed_loop)
      <<','<<number(last_attitude_output_.thrust_ceiling,last_attitude_output_.vertical_closed_loop)<<'\n';csv_.flush();}
  // 以下成员由单一控制线程持有：配置、控制核心与IPC资源同节点生命周期释放。
  f::Config config_;std::unique_ptr<f::Runtime> runtime_;std::unique_ptr<f::UnixServer> server_;
  // 发布完整性锁独立于Core故障；只有受校验的安全重新准备路径才清除，不靠UI刷新清除。
  bool boundary_fault_latched_{};std::string publication_block_reason_;
  static constexpr const char* preparation_age_wait_="RC_PREPARATION_SOURCE_AGE_EXPIRED_WAIT_NEW_DATA";
  // last_attitude_output只保存已配对提交的目标；原始PX4最新payload缓存并不等于已通过控制准入。
  Attitude attitude_{};f::attitude::Output last_attitude_output_;
  f::AuthorityTelemetry authority_telemetry_;
  // 视觉Inbox保留估计状态和目标命令两条证据；订阅SharedPtr随节点销毁，不创建额外控制线程。
  f::vision::Inbox visual_inbox_;std::string visual_state_topic_,visual_command_topic_;
  rclcpp::Subscription<VisualState>::SharedPtr visual_state_sub_;
  rclcpp::Subscription<VisualCommand>::SharedPtr visual_command_sub_;
  Local local_{};Status status_{};Land land_{};Battery battery_{};Flags flags_{};
  // envelope接收水位与raw sample推进证据分开存；重复发布同一测量不能刷新raw寿命。
  f::Stamp local_stamp_,status_stamp_,land_stamp_,battery_stamp_,flags_stamp_,attitude_stamp_;
  f::SourceSampleProgress local_sample_,attitude_sample_;
  bool clock_regressed_{};std::uint64_t last_wall_stamp_{};std::uint8_t ground_z_reset_{},ground_vz_reset_{};
  // 主机同步域、steady域、累计跳变与最近观察分别保留；诊断不能重置进程级时钟锁。
  f::HostClockMonitor host_clock_;
  f::HostClockObservation clock_observation_,last_clock_event_;
  std::uint64_t clock_event_count_{},last_clock_event_stamp_{};
  double source_observed_steady_{};std::uint64_t source_observed_host_{};
  // 各源最新接收时序与累计/本轮计数；首个坏时序另存block字段，避免被连续缓存覆盖。
  std::array<f::SourceReceiptTiming,std::size_t(f::SourceSlot::Count)> receipt_timing_;
  std::array<bool,std::size_t(f::SourceSlot::Count)> receipt_accepted_{};
  std::array<std::uint64_t,std::size_t(f::SourceSlot::Count)> source_callbacks_{},source_accepted_{},source_cycle_callbacks_{};
  std::uint64_t received_callbacks_{},receive_events_{};
  f::SourceSlot receive_timing_block_source_{f::SourceSlot::Count};
  const char* receive_timing_block_reason_{""};
  // 离散payload在控制与所有sink完成后统一ack；溢出独立粘性，不能把队列清空当安全事实。
  f::DiscreteEventLatch<Status> status_events_;
  f::DiscreteEventLatch<Land> land_events_;
  f::DiscreteEventLatch<Battery> battery_events_;
  f::DiscreteEventLatch<Flags> flags_events_;
  f::DiscreteEventLatch<Local> local_events_;
  f::DiscreteEventLatch<Attitude> attitude_events_;
  f::DiscreteEventLatch<f::AuthorityTelemetry::Facts> authority_events_;
  bool receive_event_overflow_{};
  std::string receive_stop_reason_;
  // 周期/图/配对审计仅用于当前调度，单调计数和graph generation用于关联证据。
  f::CurrentControlCycle cycle_;Work work_,completed_work_;
  std::uint64_t cycle_id_{},graph_generation_{};
  rclcpp::Event::SharedPtr graph_event_;
  GraphCounts cycle_graph_{};bool cycle_graph_ready_{},cycle_graph_invalidated_{};
  f::attitude::PublicationAttempt pair_attempt_;
  std::array<BoundarySample,3> pair_samples_{};
  bool sampling_pair_{};std::size_t pair_context_count_{};
  std::array<double,3> pair_context_seconds_{};double last_timing_warning_{-1},last_defer_report_{-1};
  f::ControlTiming timing_;
  // 报告节流、静止停留、停机期限等使用steady秒；通用参考z仍是PX4 NED米。
  double printed_{},rest_since_{-1},velocity_reference_z_{},stop_since_{},last_shutdown_warning_{};
  bool velocity_reference_set_{},stopping_{},heading_changed_{};std::uint8_t heading_reset_{};
  // 可选同步CSV及异步stdout审计分离；下列ROS SharedPtr保持端点存活直到节点释放。
  std::ofstream csv_;
  f::attitude::PublicationAudit publication_audit_;
  rclcpp::Subscription<Attitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<Manual>::SharedPtr manual_sub_;
  rclcpp::Subscription<Switches>::SharedPtr switches_sub_;
  rclcpp::Publisher<AttitudeTarget>::SharedPtr attitude_pub_;
  rclcpp::Subscription<Local>::SharedPtr local_sub_;rclcpp::Subscription<Status>::SharedPtr status_sub_;rclcpp::Subscription<Land>::SharedPtr land_sub_;rclcpp::Subscription<Battery>::SharedPtr battery_sub_;rclcpp::Subscription<Flags>::SharedPtr flags_sub_;rclcpp::Subscription<Ack>::SharedPtr ack_sub_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;rclcpp::Publisher<Mode>::SharedPtr mode_pub_;rclcpp::Publisher<Target>::SharedPtr target_pub_;
};
// 主线程先保存supervisor PID，日志RAII早于ROS创建；信号只置位，周期里执行已有停机链。
int main(int argc,char** argv){
  const auto supervisor_pid=getppid();
  int code=0;std::unique_ptr<f::AsyncLog> logs;
  try{logs=std::make_unique<f::AsyncLog>();rclcpp::init(argc,argv);std::signal(SIGINT,stop_handler);std::signal(SIGTERM,stop_handler);
    // 父进程消失信号与之后getppid复核共同封堵注册竞态；只请求优雅停机，不自动重启/强制上锁。
    bool supervisor_lost=!f::arm_parent_loss_signal(supervisor_pid,SIGTERM);
    if(supervisor_lost)stop_requested=1;
    auto node=std::make_shared<SystemNode>();
    // One executor/thread; complete ready passes may catch up within a small
    // cooperative budget. Discrete events end extra passes while a task can
    // publish. An output-silent Safety task retains every event and scans it
    // after bounded catch-up, so an unsafe payload still vetoes recovery.
    // 一个执行器线程：回调与tick不并发，有限接收追赶保留危险事件；预算属于合作调度而非抢占。
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    while(rclcpp::ok()){
      if(!supervisor_lost&&getppid()!=supervisor_pid){
        supervisor_lost=true;stop_requested=1;
        std::cerr<<"Supervisor exited: requesting existing graceful shutdown; no task restart or forced disarm\n";
      }
      // steady绝对deadline控制节拍；本轮先接收再计算/发布，sleep_until只等待剩余时间。
      const auto deadline=std::chrono::steady_clock::now()+node->loop_period();
      const double start=steady();
      node->begin_cycle(start);
      f::ReceivePassSchedule receive;
      receive.begin(start,node->receive_budget_seconds(),node->received_callbacks(),node->receive_events(),
        node->stop_receive_on_events());
      while(receive.next_pass(steady(),node->received_callbacks(),node->receive_events())){
        executor.spin_some();node->receive_pass_done();
      }
      node->receive_stopped(receive);
      const double callbacks_done=steady();
      node->callbacks_done(callbacks_done);
      // tick之后汇总完整周期耗时，结束便退出；异常由main记录返回2，最后shutdown并由RAII释放日志。
      const bool running=node->tick();
      node->observe_cycle(start,callbacks_done,steady());
      if(!running)break;
      std::this_thread::sleep_until(deadline);
    }
    executor.remove_node(node);}
  catch(const std::exception& error){std::cerr<<"[ERROR] "<<error.what()<<'\n';code=2;}
  rclcpp::shutdown();return code;
}
