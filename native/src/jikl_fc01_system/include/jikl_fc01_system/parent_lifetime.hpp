#pragma once
#include <stdexcept>
#include <sys/prctl.h>
#include <unistd.h>

namespace fc01 {
// A supervisor crash requests the SAME graceful shutdown as an explicit
// SIGTERM. It must never kill an armed controller or reset its flight state.
// Capture expected_parent before ROS initialization; check again after prctl
// so losing the parent in that interval cannot leave an unnoticed orphan.
inline bool arm_parent_loss_signal(pid_t expected_parent,int signal_number){
  if(prctl(PR_SET_PDEATHSIG,signal_number)!=0)
    throw std::runtime_error("cannot establish supervisor lifetime signal");
  return expected_parent>1&&getppid()==expected_parent;
}
}
