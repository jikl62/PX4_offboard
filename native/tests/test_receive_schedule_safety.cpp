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
  schedule.begin(2.,.005,0,0,false);
  unsigned passes=0;
  while(schedule.next_pass(2.+passes*.0001,passes,passes))++passes;
  assert(passes==10&&schedule.reason()==f::ReceivePassStop::PassLimit);
  schedule.begin(3.,.005,0,0,false);
  assert(schedule.next_pass(3.,0,0));
  assert(!schedule.next_pass(3.006,1,1));
  assert(schedule.reason()==f::ReceivePassStop::Budget);

  // Ignoring event-triggered stops cannot ignore a counter or clock rewind.
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
  schedule.begin(6.,.005,0,0);
  assert(schedule.next_pass(6.,0,0));
  assert(!schedule.next_pass(6.0001,1,1));
  assert(schedule.reason()==f::ReceivePassStop::Event);
  std::cout<<"Safety catch-up retains receive budgets, limits and clock/counter guards: PASS\n";
}
