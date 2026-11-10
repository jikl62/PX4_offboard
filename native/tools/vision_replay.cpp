// Host-only replay. This executable has no ROS includes or hardware transports.
#include "vision_test_support.hpp"
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

namespace vt=vision_test;namespace v=fc01::vision;namespace a=fc01::attitude;
static bool strict_bool(const std::string& value){
  if(value!="true"&&value!="false")throw std::invalid_argument("INVALID_CONFIG_BOOL");
  return value=="true";
}
static bool parameter(fc01::Config& cfg,const std::string& key,const std::string& value){
  if(key=="profile"){cfg.profile=value;return true;}
  if(key=="flight_enabled"){cfg.flight_enabled=strict_bool(value);return true;}
#include "vision_replay_parameters.inc"
  return false; // Remaining baseline YAML entries belong to other ROS adapters.
}
static void candidate(v::Config& cfg,const std::string& key,double value){
  const std::map<std::string,double*> values{
    {"state_timeout",&cfg.state_timeout},{"command_timeout",&cfg.command_timeout},
    {"max_measurement_delay",&cfg.max_measurement_delay},{"future_tolerance",&cfg.future_tolerance},
    {"position_gain",&cfg.position_gain},{"velocity_gain",&cfg.velocity_gain},
    {"max_speed",&cfg.max_speed},{"max_acceleration",&cfg.max_acceleration},
    {"max_tilt_deg",&cfg.max_tilt_deg},{"max_yaw_rate",&cfg.max_yaw_rate},
    {"max_vertical_speed",&cfg.max_vertical_speed},{"max_radius",&cfg.max_radius},
    {"max_height",&cfg.max_height},{"min_quality",&cfg.min_quality},
    {"max_position_jump",&cfg.max_position_jump},{"max_velocity_jump",&cfg.max_velocity_jump}};
  const auto it=values.find(key);if(it==values.end())throw std::invalid_argument("UNKNOWN_VISUAL_CANDIDATE");
  *it->second=value;
}
static fc01::Config simulation_config(fc01::Config cfg){
  // These process-local flags model the existing RC route. No parameter file,
  // service, socket, ARM transaction or Offboard-entry transaction is changed.
  cfg.profile="attitude_altitude";cfg.flight_enabled=false;
  auto& p=cfg.attitude;p.flight_actions_enabled=p.takeoff_enabled=p.parameters_confirmed=true;
  p.arm_enabled=p.bench_arm_only_enabled=p.autonomous_failsafe_confirmed=false;
  p.rc_takeover_confirmed=p.rc_navigation_enabled=true;p.rc_navigation_mode_slot=1;
  p.visual.enabled=true;p.vertical.ground_lower_margin=p.height_lower_margin_m;
  cfg.validate();return cfg;
}
static double real(std::istream& in){
  std::string text;if(!(in>>text))throw std::invalid_argument("TRUNCATED_NUMBER");
  std::size_t end{};const double value=std::stod(text,&end);
  if(end!=text.size())throw std::invalid_argument("INVALID_NUMBER");
  return value;
}
template<std::size_t N>static void array(std::istream& in,std::array<double,N>& value){for(double& x:value)x=real(in);}
static v::State state(std::istream& in,double now){
  v::State s;double age;unsigned mask;
  in>>s.sequence>>s.stamp_ns>>s.reset_counter;age=real(in);
  in>>std::quoted(s.frame_id)>>std::quoted(s.body_frame_id)>>std::quoted(s.source_id)>>std::quoted(s.angular_velocity_source);
  in>>s.initialized>>s.tracking>>s.calibration_valid>>mask;s.valid_fields=static_cast<std::uint16_t>(mask);
  array(in,s.position);array(in,s.velocity);array(in,s.attitude);array(in,s.angular_velocity);
  array(in,s.pose_covariance);array(in,s.twist_covariance);s.quality=real(in);s.received_at=now-age;return s;
}
static v::Command command(std::istream& in,double now){
  v::Command c;double age;unsigned mode;
  in>>c.sequence>>c.stamp_ns>>c.reset_counter;age=real(in);
  in>>std::quoted(c.source_id)>>std::quoted(c.frame_id)>>mode;c.mode=static_cast<v::Mode>(mode);
  in>>c.valid>>c.position_valid>>c.yaw_valid>>c.sticks_valid;
  array(in,c.position);c.yaw=real(in);array(in,c.sticks);c.received_at=now-age;return c;
}
static void number(double value){if(std::isfinite(value))std::cout<<value;else std::cout<<"null";}
template<class Number,std::size_t N>static void json_array(const std::array<Number,N>& values){
  std::cout<<'[';for(std::size_t i=0;i<N;++i){if(i)std::cout<<',';number(values[i]);}std::cout<<']';
}
static void json_string(const std::string& value){
  std::cout<<'"';for(unsigned char ch:value){
    if(ch=='"'||ch=='\\')std::cout<<'\\'<<ch;
    else if(ch<32)std::cout<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(ch)<<std::dec<<std::setfill(' ');
    else std::cout<<ch;
  }std::cout<<'"';
}
static void emit(const vt::VisionFlight& q,const std::string& dataset,double replay_time,double error=NAN){
  const auto& result=q.r.attitude_core().visual_result();const auto& in=q.s.attitude.visual_input;
  std::cout<<std::setprecision(12)<<"{\"schema_version\":1,\"simulation\":true,\"dataset_kind\":";
  json_string(dataset);std::cout<<",\"runtime_path\":\"Runtime/RC/Core/dual_sink\",\"time\":"<<replay_time
    <<",\"virtual_steady_time\":"<<q.t<<",\"ros_now_ns\":"<<in.ros_now_ns
    <<",\"state_sequence\":"<<in.state.sequence<<",\"command_sequence\":"<<in.command.sequence
    <<",\"accepted_state_sequence\":"<<result.state_sequence<<",\"accepted_command_sequence\":"<<result.command_sequence
    <<",\"active\":"<<std::boolalpha<<result.active<<",\"request_land\":"<<result.request_land
    <<",\"mode\":"<<static_cast<int>(result.mode)<<",\"reason\":";json_string(result.reason);
  std::cout<<",\"native_reason\":";json_string(q.r.attitude_core().reason());
  std::cout<<",\"phase\":";json_string(a::name(q.r.attitude_core().phase()));
  std::cout<<",\"velocity_target\":";json_array(result.velocity_target);
  std::cout<<",\"acceleration_target\":";json_array(result.acceleration_target);
  std::cout<<",\"q\":";json_array(q.o.attitude.q);
  std::cout<<",\"thrust\":";json_array(q.o.attitude.thrust);
  std::cout<<",\"submitted_q\":";if(q.target_committed)json_array(q.last_target.q_d);else std::cout<<"null";
  std::cout<<",\"submitted_thrust\":";if(q.target_committed)json_array(q.last_target.thrust_body);else std::cout<<"null";
  std::cout<<",\"target_height\":";number(q.o.attitude.target_height);
  std::cout<<",\"height\":";number(q.o.attitude.height);
  std::cout<<",\"yaw\":";number(result.yaw);
  std::cout<<",\"position\":";json_array(in.state.position);
  std::cout<<",\"velocity\":";json_array(in.state.velocity);
  std::cout<<",\"position_error\":";number(error);
  std::cout<<",\"publish_requested\":"<<q.o.attitude.publish<<",\"visual_motion\":"<<q.o.attitude.visual_motion
    <<",\"allowed\":"<<(q.o.attitude.publish&&q.last_decision.allowed)<<",\"boundary_reason\":";
  json_string(a::name(q.last_decision.reason));
  std::cout<<",\"target_committed\":"<<q.target_committed<<",\"heartbeat_committed\":"<<q.heartbeat_committed
    <<",\"target_commits\":"<<q.target_commits<<",\"heartbeat_commits\":"<<q.heartbeat_commits
    <<",\"arm_commands\":"<<q.arm_commands<<",\"mode_commands\":"<<q.mode_commands
    <<",\"baseline_hover_thrust\":"<<q.c.attitude.vertical.hover
    <<",\"baseline_height_gain\":"<<q.c.attitude.vertical.height_gain<<"}\n";
}
static v::State demo_state(std::uint64_t seq,std::uint64_t stamp,const v::Vec3& p,const v::Vec3& velocity,const a::Quaternion& q){
  v::State s;s.sequence=seq;s.stamp_ns=stamp;s.frame_id="fc01_vision_ned";s.body_frame_id="base_link_frd";s.source_id="ideal_plant";
  s.initialized=s.tracking=s.calibration_valid=true;s.valid_fields=v::Position|v::Velocity|v::Attitude|v::Quality;
  s.position=p;s.velocity=velocity;s.attitude=q;s.quality=1;return s;
}
static void demo(fc01::Config cfg,const std::string& scenario){
  if(scenario!="cruise"&&scenario!="height")throw std::invalid_argument("UNKNOWN_DEMO");
  vt::VisionFlight q(simulation_config(cfg));q.reach_hold();v::Inbox inbox;
  const double platform=q.c.attitude.rc_target_height;
  v::Vec3 p{0,0,-platform},velocity{0,0,0};a::Quaternion actual{1,0,0,0};
  const v::Vec3 target=scenario=="height"?v::Vec3{0,0,-platform-.4}:v::Vec3{.8,.4,-platform};
  constexpr double dt=.05,g=9.80665;double elapsed=0;
  for(std::uint64_t seq=1;seq<=180;++seq){
    elapsed+=dt;const auto stamp=1000000000ULL+static_cast<std::uint64_t>(std::llround(elapsed*1e9));
    const double now=q.t+dt;auto s=demo_state(seq,stamp,p,velocity,actual);inbox.state(s,now);
    v::Command c;c.sequence=seq;c.stamp_ns=stamp;c.source_id="ideal_scenario";c.frame_id="fc01_vision_ned";
    c.mode=v::Mode::Cruise;c.valid=c.position_valid=true;c.position=target;inbox.command(c,now);
    q.set_visual(inbox.snapshot(stamp,1,1));q.observe(-p[2],-velocity[2]);q.s.attitude.q=actual;q.tick(true,dt);
    emit(q,"synthetic",elapsed,std::hypot(target[0]-p[0],target[1]-p[1]));
    if(!q.target_committed||!q.heartbeat_committed)continue;
    // Ideal acceleration uses only the actual final submitted q and thrust.
    // It is deliberately simple and is not an FC01 vehicle dynamics model.
    for(unsigned i=0;i<4;++i)actual[i]=q.last_target.q_d[i];
    const double w=actual[0],x=actual[1],y=actual[2],z=actual[3];
    const v::Vec3 down{2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)};
    const double scale=(-q.last_target.thrust_body[2])/q.c.attitude.vertical.hover*g;
    v::Vec3 accel{-scale*down[0]-.20*velocity[0],-scale*down[1]-.20*velocity[1],g-scale*down[2]-.8*velocity[2]};
    for(unsigned i=0;i<3;++i){velocity[i]+=accel[i]*dt;p[i]+=velocity[i]*dt;}
  }
}
int main(){
  try{
    fc01::Config cfg;v::Inbox inbox;std::unique_ptr<vt::VisionFlight> flight;std::string dataset="synthetic",line;double elapsed=0;
    while(std::getline(std::cin,line)){
      if(line.empty())continue;
      std::istringstream in(line);std::string op;in>>op;
      if(op=="P"){
        vt::need(!flight,"CONFIG_AFTER_TICKS");std::string key,value;in>>std::quoted(key)>>std::quoted(value);parameter(cfg,key,value);
      }else if(op=="V"){
        vt::need(!flight,"CANDIDATE_AFTER_TICKS");std::string key;in>>std::quoted(key);candidate(cfg.attitude.visual,key,real(in));
      }else if(op=="DATA"){
        vt::need(!flight,"DATASET_AFTER_TICKS");in>>dataset;
        vt::need(dataset=="synthetic"||dataset=="recorded","INVALID_DATASET_KIND");
      }else if(op=="DEMO"){
        std::string name;in>>name;demo(cfg,name);return 0;
      }else if(op=="T"){
        if(!flight){flight=std::make_unique<vt::VisionFlight>(simulation_config(cfg));flight->reach_hold();}
        auto& q=*flight;const double dt=real(in),now=q.t+dt;std::uint64_t ros_now;std::size_t states,commands;bool has;
        in>>ros_now>>states>>commands>>has;
        if(has){double height=real(in),speed=real(in),yaw=real(in);q.observe(height,speed);q.s.attitude.q={std::cos(yaw/2),0,0,std::sin(yaw/2)};}
        in>>has;if(has){auto s=state(in,now);inbox.state(s,s.received_at);}
        in>>has;if(has){auto c=command(in,now);inbox.command(c,c.received_at);}
        vt::need(!in.fail(),"MALFORMED_NATIVE_TICK");std::string extra;vt::need(!(in>>extra),"TRAILING_NATIVE_TICK_FIELDS");
        q.set_visual(inbox.snapshot(ros_now,states,commands));q.tick(true,dt);elapsed+=dt;
        emit(q,dataset,elapsed);
      }else throw std::invalid_argument("UNKNOWN_NATIVE_RECORD");
    }
    return 0;
  }catch(const std::exception& e){std::cerr<<"native replay: "<<e.what()<<'\n';return 2;}
}
