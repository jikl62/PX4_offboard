// ROS 接收调度 A/B 探针：比较 spin_some/持久executor/重复补读/spin_all 的消费时延。
// 仅用 /fc01_offline_repro/source 的 Int64MultiArray 测试消息，不发布任何 /fmu/in 控制。
// 数据载荷为序号、外壳 wall ns、raw wall ns；源判定前转 µs，CSV 记录 ms 时延与 50 ms 周期。
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include "jikl_fc01_system/source_clock.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

using Clock=std::chrono::steady_clock;
double steady(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
std::int64_t wall(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}
using Msg=std_msgs::msg::Int64MultiArray;
int main(int argc,char**argv){
  if(argc<6){std::cerr<<"mode hz seconds budget_ms csv [raw_lag_ms] [callback_ms]\n";return 2;}
  const std::string mode=argv[1];const double hz=std::stod(argv[2]),seconds=std::stod(argv[3]);
  const double budget_ms=std::stod(argv[4]);const std::string path=argv[5];
  const double raw_lag_ms=argc>6?std::stod(argv[6]):0;
  const double callback_ms=argc>7?std::stod(argv[7]):0;
  rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("fc01_executor_ab_"+mode);
  const auto qos=rclcpp::SensorDataQoS().keep_last(10);
  // 发布端先等唯一订阅者，按 hz 生成 wall stamp，可显式加入 raw_lag_ms 检验“外壳新而 raw 旧”。
  if(mode=="pub"){
    auto publisher=node->create_publisher<Msg>("/fc01_offline_repro/source",qos);
    const double wait_until=steady()+4.;
    while(publisher->get_subscription_count()!=1&&steady()<wait_until)std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto deadline=Clock::now();const auto period=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1./hz));
    const double end=steady()+seconds;std::int64_t sequence=0;
    while(rclcpp::ok()&&steady()<end){Msg msg;const auto t=wall();msg.data={++sequence,t,t-std::int64_t(raw_lag_ms*1e6)};
      publisher->publish(msg);deadline+=period;std::this_thread::sleep_until(deadline);}
    std::cout<<"{\"mode\":\"pub\",\"produced\":"<<sequence<<"}\n";
    rclcpp::shutdown();return 0;
  }
  std::uint64_t count=0,envelope_valid=0,raw_valid=0;std::int64_t sequence=0;
  double callback_age_ms=-1,rmw_ms=-1,raw_age_ms=-1,first=-1;
  // 回调记录源/原始年龄和 RMW 到回调差；callback_ms 可人为阻塞，不能把本探针当生产实时保证。
  auto subscription=node->create_subscription<Msg>("/fc01_offline_repro/source",qos,[&](Msg::ConstSharedPtr msg,const rclcpp::MessageInfo& info){
    if(first<0)first=steady();
    ++count;sequence=msg->data[0];const auto now=wall();
    envelope_valid+=fc01::source_stamp_in_clock(msg->data[1]/1000,now/1000,.5);
    raw_valid+=fc01::source_sample_in_clock(msg->data[1]/1000,msg->data[2]/1000,now/1000,.5);
    callback_age_ms=double(now-msg->data[1])*1e-6;raw_age_ms=double(now-msg->data[2])*1e-6;
    const auto rmw=info.get_rmw_message_info().received_timestamp;rmw_ms=rmw?double(now-rmw)*1e-6:-1;
    if(callback_ms>0)std::this_thread::sleep_for(std::chrono::duration<double,std::milli>(callback_ms));
  });
  rclcpp::executors::SingleThreadedExecutor executor; if(mode!="some")executor.add_node(node);
  std::ofstream csv(path);csv<<"t_s,callbacks,seq,callback_age_ms,rmw_to_callback_ms,raw_age_ms,spin_ms,cycle_period_ms\n";
  double max_spin_ms=0,max_period_ms=0,total_spin_ms=0,first_age=-1,last_age=-1,max_age=-1;
  std::size_t cycles=0,total_cycles=0,max_callbacks=0,zero_cycles=0,max_passes=0,pass_cap_hits=0,budget_hits=0;double previous=-1;
  const double process_start=steady();auto deadline=Clock::now();
  while(rclcpp::ok()&&(first<0?steady()-process_start<4:steady()-first<seconds)){
    const double start=steady();const auto before=count;
    std::size_t passes=1;
    if(mode=="some")rclcpp::spin_some(node);
    else if(mode=="persistent_some")executor.spin_some(std::chrono::milliseconds(std::int64_t(budget_ms)));
    // 重复补读最多十轮，只有一轮结束后才能检查预算；单回调仍不可抢占。
    else if(mode=="repeated"){
      const double budget_end=start+budget_ms*1e-3;
      for(passes=0;passes<10;){
        const auto before_pass=count;
        executor.spin_some();++passes;
        if(!rclcpp::ok()||count==before_pass)break;
        if(steady()>=budget_end){++budget_hits;break;}
      }
      pass_cap_hits+=passes==10;
    }
    else executor.spin_all(std::chrono::milliseconds(std::int64_t(budget_ms)));
    max_passes=std::max(max_passes,passes);
    // 分别记录 spin 耗时、实际周期间隔、单周期回调数和队列年龄，避免只看平均吞吐漏掉抖动。
    const double spin_ms=(steady()-start)*1e3;max_spin_ms=std::max(max_spin_ms,spin_ms);total_spin_ms+=spin_ms;
    ++total_cycles;
    if(first>=0){++cycles;const auto n=count-before;max_callbacks=std::max(max_callbacks,std::size_t(n));zero_cycles+=n==0;
      const double period=previous<0?0:(start-previous)*1e3;max_period_ms=std::max(max_period_ms,period);
      if(n){if(first_age<0)first_age=callback_age_ms;last_age=callback_age_ms;max_age=std::max(max_age,callback_age_ms);}
      csv<<std::setprecision(10)<<start-first<<','<<n<<','<<sequence<<','<<callback_age_ms<<','<<rmw_ms<<','<<raw_age_ms<<','<<spin_ms<<','<<period<<'\n';
    }
    previous=start;deadline+=std::chrono::milliseconds(50);std::this_thread::sleep_until(deadline);
  }
  // 输出统计值用于比较观察，缺首帧时返回失败；不推断 PX4 链路或 ARM64 性能。
  const double elapsed=first<0?0:steady()-first;
  std::cout<<std::setprecision(10)<<"{\"mode\":\""<<mode<<"\",\"producing_hz\":"<<hz<<",\"count\":"<<count
    <<",\"elapsed_s\":"<<elapsed<<",\"consumed_hz\":"<<(elapsed?count/elapsed:0)<<",\"cycles\":"<<cycles
    <<",\"envelope_valid_count\":"<<envelope_valid<<",\"raw_sample_valid_count\":"<<raw_valid
    <<",\"max_callbacks_per_cycle\":"<<max_callbacks<<",\"zero_cycles\":"<<zero_cycles
    <<",\"max_passes\":"<<max_passes<<",\"pass_cap_hits\":"<<pass_cap_hits<<",\"budget_hits\":"<<budget_hits
    <<",\"callback_age_first_ms\":"<<first_age<<",\"callback_age_last_ms\":"<<last_age<<",\"callback_age_max_ms\":"<<max_age
    <<",\"rmw_last_ms\":"<<rmw_ms<<",\"raw_age_last_ms\":"<<raw_age_ms<<",\"budget_ms\":"<<budget_ms
    <<",\"spin_max_ms\":"<<max_spin_ms<<",\"spin_mean_ms\":"<<(total_cycles?total_spin_ms/total_cycles:0)
    <<",\"cycle_period_max_ms\":"<<max_period_ms<<",\"final_seq\":"<<sequence<<"}\n";
  rclcpp::shutdown();return first<0&&mode!="empty"?3:0;
}
