// Safety 追赶缓存调度回归：可不因每条事件停止补读，但预算、轮数与时间/计数保护仍完整。
// stopped模式只改变额外补读停止条件，事件payload仍由锁存器保留，不授予任何飞行输出。
#include "jikl_fc01_system/receive_schedule.hpp"
#include <cassert>
#include <iostream>

namespace f=fc01;

int main(){
  f::ReceivePassSchedule schedule;
  schedule.begin(1.,.005,0,0);
  assert(schedule.next_pass(1.,0,0));
  assert(!schedule.next_pass(1.0001,1,1));
  assert(schedule.reason()==f::ReceivePassStop::Event&&schedule.pass_count()==1);

  // A stopped task may retain events while catching up, within the same
  // complete-pass limit and cooperative budget as the normal receiver.
  // Safety追赶最多十完整轮或5 ms；忽略事件停止不意味着忽略调度资源界限。
  schedule.begin(2.,.005,0,0,false);
  unsigned passes=0;
  while(schedule.next_pass(2.+passes*.0001,passes,passes))++passes;
  assert(passes==10&&schedule.reason()==f::ReceivePassStop::PassLimit);
  schedule.begin(3.,.005,0,0,false);
  assert(schedule.next_pass(3.,0,0));
  assert(!schedule.next_pass(3.006,1,1));
  assert(schedule.reason()==f::ReceivePassStop::Budget);

  // Ignoring event-triggered stops cannot ignore a counter or clock rewind.
  // 即使追赶模式，计数回退和时钟回退仍立即停止，不能把坏时序当队列繁忙。
  schedule.begin(4.,.005,2,2,false);
  assert(schedule.next_pass(4.,2,2));
  assert(schedule.next_pass(4.0001,4,7));
  assert(!schedule.next_pass(4.0002,5,6));
  assert(schedule.reason()==f::ReceivePassStop::InvalidCounter);
  schedule.begin(5.,.005,0,0,false);
  assert(schedule.next_pass(5.,0,0));
  assert(!schedule.next_pass(4.9999,1,1));
  assert(schedule.reason()==f::ReceivePassStop::InvalidClock);

  // The optional mode belongs to one cycle; ordinary callers keep their
  // existing event visibility even after a previous catch-up cycle.
  // 下一普通周期begin恢复默认事件可见性，上一追赶模式不可泄漏到正常控制。
  schedule.begin(6.,.005,0,0);
  assert(schedule.next_pass(6.,0,0));
  assert(!schedule.next_pass(6.0001,1,1));
  assert(schedule.reason()==f::ReceivePassStop::Event);
  std::cout<<"Safety catch-up retains receive budgets, limits and clock/counter guards: PASS\n";
}
