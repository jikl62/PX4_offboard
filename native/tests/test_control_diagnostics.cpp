#include "jikl_fc01_system/control_diagnostics.hpp"
#include <cmath>
#include <stdexcept>

int main(){
  fc01::ControlTiming timing;
  if(timing.valid)throw std::runtime_error("no cycle may be invented before observation");
  timing.observe(100.,100.01,100.03,.3);
  if(!timing.valid||std::abs(timing.callback_seconds-.01)>1e-9||
     std::abs(timing.control_seconds-.02)>1e-9||timing.period_valid)
    throw std::runtime_error("callbacks and tick must have separate measured durations");
  timing.observe(100.4,100.41,100.45,.3);
  if(!timing.period_valid||std::abs(timing.period_seconds-.4)>1e-9||timing.stall_count!=1)
    throw std::runtime_error("loop stall must include callbacks and scheduling, not only tick");
  timing.observe(99.,99.01,99.02,.3);
  if(timing.valid)throw std::runtime_error("clock regression must not appear healthy");
}
